"""ROS 2 node that connects Twist/direct speed messages to Mariola motors."""

from math import isfinite
import sys
from time import monotonic

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int16MultiArray

from .hardware import HardwareSettings, MariolaHardware, SimulatedController
from .kinematics import (
    sequence_to_motor_speeds,
    twist_to_motor_speeds,
    WHEEL_NAMES,
)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('cbr_motor_control')
        self._declare_parameters()
        self._validate_parameters()

        self._dry_run = self._bool_parameter('dry_run')
        self._command_mode = self._str_parameter('command_mode')
        self._watchdog_timeout = self._float_parameter('watchdog_timeout')
        self._last_command_time = None
        self._last_logged_speeds = None
        self._moving = False
        self._closed = False

        self.get_logger().info(f'Interpretador Python: {sys.executable}')
        self.get_logger().info(f'pyserial carregado de: {serial.__file__}')

        if self._dry_run:
            self._controller = SimulatedController()
            self.get_logger().warning(
                'Modo simulado ativo: nenhuma porta serial será aberta')
        else:
            self.get_logger().warning(
                'HARDWARE REAL ativo; inicializando placas com velocidade zero')
            settings = HardwareSettings(
                expansion_port=self._int_parameter('expansion_port'),
                baud_rate=self._int_parameter('baud_rate'),
                serial_timeout=self._float_parameter('serial_timeout'),
                front_left_id=self._int_parameter('front_left_id'),
                front_right_id=self._int_parameter('front_right_id'),
                front_left_inverted=self._bool_parameter(
                    'front_left_inverted'),
                front_right_inverted=self._bool_parameter(
                    'front_right_inverted'),
                rear_left_inverted=self._bool_parameter(
                    'rear_left_inverted'),
                rear_right_inverted=self._bool_parameter(
                    'rear_right_inverted'),
                configure_on_start=self._bool_parameter(
                    'configure_on_start'),
                calibration_clockwise=self._int_parameter(
                    'calibration_clockwise'),
                calibration_counterclockwise=self._int_parameter(
                    'calibration_counterclockwise'),
                brake_kp=self._float_parameter('brake_kp'),
                brake_kd=self._float_parameter('brake_kd'),
                brake_delta=self._int_parameter('brake_delta'),
                motor_kp=self._float_parameter('motor_kp'),
                motor_ki=self._float_parameter('motor_ki'),
                motor_kd=self._float_parameter('motor_kd'),
            )
            self._controller = MariolaHardware(settings, self.get_logger())

        self._state_publisher = self.create_publisher(
            Int16MultiArray, 'motor_speeds_applied', 10)
        if self._command_mode == 'cmd_vel':
            self._subscription = self.create_subscription(
                Twist, 'cmd_vel', self._on_twist, 10)
        else:
            self._subscription = self.create_subscription(
                Int16MultiArray, 'motor_speeds', self._on_direct_speeds, 10)
        self._watchdog = self.create_timer(0.05, self._check_watchdog)
        self._publish_state()
        self.get_logger().info(
            f'Controle pronto em command_mode={self._command_mode}; '
            f'watchdog={self._watchdog_timeout:.3f}s')

    def _declare_parameters(self):
        parameters = (
            ('dry_run', False),
            ('command_mode', 'cmd_vel'),
            ('watchdog_timeout', 0.5),
            ('max_linear_speed', 0.5),
            ('max_angular_speed', 1.5),
            ('max_motor_speed', 40.0),
            ('deadband', 0.02),
            ('expansion_port', 3),
            ('baud_rate', 250000),
            ('serial_timeout', 0.005),
            ('front_left_id', 0),
            ('front_right_id', 7),
            ('front_left_inverted', True),
            ('front_right_inverted', False),
            ('rear_left_inverted', False),
            ('rear_right_inverted', True),
            ('configure_on_start', True),
            ('calibration_clockwise', 80),
            ('calibration_counterclockwise', -80),
            ('brake_kp', 3.0),
            ('brake_kd', 10.0),
            ('brake_delta', 20),
            ('motor_kp', 2.0),
            ('motor_ki', 2.0),
            ('motor_kd', 2.0),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

    def _validate_parameters(self):
        command_mode = self._str_parameter('command_mode')
        if command_mode not in ('cmd_vel', 'motor_speeds'):
            raise ValueError(
                "command_mode deve ser 'cmd_vel' ou 'motor_speeds'")
        if self._float_parameter('watchdog_timeout') <= 0.0:
            raise ValueError('watchdog_timeout deve ser maior que zero')
        # Exercise all kinematic limit validation before hardware is opened.
        twist_to_motor_speeds(
            0.0,
            0.0,
            0.0,
            max_linear_speed=self._float_parameter('max_linear_speed'),
            max_angular_speed=self._float_parameter('max_angular_speed'),
            max_motor_speed=self._float_parameter('max_motor_speed'),
            deadband=self._float_parameter('deadband'),
        )
        if not 0 <= self._int_parameter('expansion_port') <= 6:
            raise ValueError('expansion_port deve estar entre 0 e 6')
        if self._int_parameter('baud_rate') <= 0:
            raise ValueError('baud_rate deve ser maior que zero')
        timeout = self._float_parameter('serial_timeout')
        if not isfinite(timeout) or timeout <= 0.0:
            raise ValueError('serial_timeout deve ser finito e maior que zero')
        for name in ('front_left_id', 'front_right_id'):
            value = self._int_parameter(name)
            if not 0 <= value <= 255:
                raise ValueError(f'{name} deve estar entre 0 e 255')

    def _on_twist(self, message):
        try:
            speeds = twist_to_motor_speeds(
                message.linear.x,
                message.linear.y,
                message.angular.z,
                max_linear_speed=self._float_parameter('max_linear_speed'),
                max_angular_speed=self._float_parameter('max_angular_speed'),
                max_motor_speed=self._float_parameter('max_motor_speed'),
                deadband=self._float_parameter('deadband'),
            )
            self._apply_command(speeds)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'cmd_vel inválido: {exc}')
            self._emergency_stop()

    def _on_direct_speeds(self, message):
        try:
            self._apply_command(sequence_to_motor_speeds(
                message.data,
                max_motor_speed=self._float_parameter('max_motor_speed'),
            ))
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'motor_speeds inválido: {exc}')
            self._emergency_stop()

    def _apply_command(self, speeds):
        ordered_speeds = tuple(speeds[name] for name in WHEEL_NAMES)
        if ordered_speeds != self._last_logged_speeds:
            self.get_logger().info(
                'Velocidades lógicas [DE, DD, TE, TD]: '
                + str(list(ordered_speeds)))
            self._last_logged_speeds = ordered_speeds
        try:
            self._controller.definir_velocidades(**speeds)
        except Exception as exc:
            self.get_logger().error(f'Falha ao aplicar velocidade: {exc}')
            self._emergency_stop()
            return
        self._last_command_time = monotonic()
        self._moving = any(speeds.values())
        self._publish_state()

    def _check_watchdog(self):
        if not self._moving or self._last_command_time is None:
            return
        if monotonic() - self._last_command_time > self._watchdog_timeout:
            self.get_logger().warning(
                'Watchdog: comandos pararam de chegar; freando motores')
            self._emergency_stop()

    def _emergency_stop(self):
        self._moving = False
        try:
            self._controller.frear()
        except Exception as exc:
            self.get_logger().error(f'Falha também ao frear: {exc}')
        self._publish_state()

    def _publish_state(self):
        message = Int16MultiArray()
        state = self._controller.velocidades_atuais
        message.data = [state[name] for name in WHEEL_NAMES]
        self._state_publisher.publish(message)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._moving = False
        self._controller.fechar()

    def destroy_node(self):
        try:
            self.close()
        except Exception as exc:
            self.get_logger().error(f'Erro no desligamento dos motores: {exc}')
        return super().destroy_node()

    def _bool_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _int_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _float_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _str_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().string_value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
