"""ROS 2 boundary for the physical Mariola mecanum base."""

from __future__ import annotations

import math
import threading
import time
import traceback

from interfaces.msg import WheelCommand
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .mariola_adapter import (
    MariolaBase,
    MariolaConfig,
    WHEEL_NAMES,
    radians_per_second_to_command,
    validate_complete_command,
)


class BaseHardwareNode(Node):
    def __init__(self, backend=None):
        super().__init__('base_hardware_node')
        self.declare_parameter('io_rate_hz', 30.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('deduplicate_commands', True)
        self.declare_parameter('command_heartbeat_hz', 5.0)
        self.declare_parameter('max_consecutive_io_failures', 3)
        self.declare_parameter('state_topic', '/base_hardware/raw_joint_states')
        self.declare_parameter('command_topic', '/base_hardware/command_velocities')
        self.declare_parameter('hardware.expansion_serial_port', 3)
        self.declare_parameter('hardware.serial_baud_rate', 250000)
        self.declare_parameter('hardware.expansion_timeout_sec', 0.005)
        self.declare_parameter('hardware.max_wheel_velocity_rad_s', 7.0)
        self.declare_parameter('hardware.min_effective_wheel_command', 2)
        self.declare_parameter('hardware.brick_ticks_per_revolution', 1644)
        self.declare_parameter('hardware.expansion_ticks_per_revolution', 3288)
        self.declare_parameter('hardware.front_left.motor_id', 0)
        self.declare_parameter('hardware.front_left.inverted', True)
        self.declare_parameter('hardware.front_left.calibration_clockwise', 88)
        self.declare_parameter('hardware.front_left.calibration_counterclockwise', -88)
        self.declare_parameter('hardware.front_right.motor_id', 7)
        self.declare_parameter('hardware.front_right.inverted', False)
        self.declare_parameter('hardware.front_right.calibration_clockwise', 88)
        self.declare_parameter('hardware.front_right.calibration_counterclockwise', -88)
        self.declare_parameter('hardware.rear_left.inverted', False)
        self.declare_parameter('hardware.rear_right.inverted', True)

        rate = float(self.get_parameter('io_rate_hz').value)
        timeout = float(self.get_parameter('command_timeout_sec').value)
        heartbeat_rate = float(
            self.get_parameter('command_heartbeat_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('io_rate_hz deve ser positivo e finito.')
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('command_timeout_sec deve ser positivo e finito.')
        if not math.isfinite(heartbeat_rate) or heartbeat_rate <= 0.0:
            raise ValueError(
                'command_heartbeat_hz deve ser positivo e finito.')
        self._command_timeout = timeout
        self._deduplicate_commands = bool(
            self.get_parameter('deduplicate_commands').value)
        self._command_heartbeat_period = 1.0 / heartbeat_rate
        self._failure_limit = max(
            1, int(self.get_parameter('max_consecutive_io_failures').value))
        self._failures = 0
        self._last_command_time = None
        self._latest_command = {name: 0.0 for name in WHEEL_NAMES}
        self._lock = threading.Lock()
        self._last_written_signature = None
        self._last_command_write_time = float('-inf')
        hardware_config = MariolaConfig(
            expansion_serial_port=int(
                self.get_parameter('hardware.expansion_serial_port').value),
            serial_baud_rate=int(
                self.get_parameter('hardware.serial_baud_rate').value),
            expansion_timeout_sec=float(
                self.get_parameter('hardware.expansion_timeout_sec').value),
            front_left_motor_id=int(
                self.get_parameter('hardware.front_left.motor_id').value),
            front_right_motor_id=int(
                self.get_parameter('hardware.front_right.motor_id').value),
            front_left_inverted=bool(
                self.get_parameter('hardware.front_left.inverted').value),
            front_right_inverted=bool(
                self.get_parameter('hardware.front_right.inverted').value),
            front_left_calibration_clockwise=int(
                self.get_parameter(
                    'hardware.front_left.calibration_clockwise').value),
            front_left_calibration_counterclockwise=int(
                self.get_parameter(
                    'hardware.front_left.calibration_counterclockwise').value),
            front_right_calibration_clockwise=int(
                self.get_parameter(
                    'hardware.front_right.calibration_clockwise').value),
            front_right_calibration_counterclockwise=int(
                self.get_parameter(
                    'hardware.front_right.calibration_counterclockwise').value),
            rear_left_inverted=bool(
                self.get_parameter('hardware.rear_left.inverted').value),
            rear_right_inverted=bool(
                self.get_parameter('hardware.rear_right.inverted').value),
            brick_ticks_per_revolution=int(
                self.get_parameter('hardware.brick_ticks_per_revolution').value),
            expansion_ticks_per_revolution=int(
                self.get_parameter('hardware.expansion_ticks_per_revolution').value),
            max_wheel_velocity_rad_s=float(
                self.get_parameter('hardware.max_wheel_velocity_rad_s').value),
            min_effective_wheel_command=int(
                self.get_parameter(
                    'hardware.min_effective_wheel_command').value),
        )
        self._max_wheel_velocity = hardware_config.max_wheel_velocity_rad_s
        self._min_effective_command = (
            hardware_config.min_effective_wheel_command)
        self._backend = backend or MariolaBase(config=hardware_config)
        for name, calibration in getattr(
                self._backend, 'expansion_calibrations', {}).items():
            action = 'corrigida' if calibration['updated'] else 'validada'
            self.get_logger().info(
                f'Calibração {action} para {name}: '
                f"horário={calibration['giro_max_horario']}, "
                f"anti-horário={calibration['giro_max_antihorario']}.")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # This bridge carries only the newest sample.  RELIABLE can block
            # the ros2_control write loop when the SBC is temporarily busy.
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._state_publisher = self.create_publisher(
            JointState, self.get_parameter('state_topic').value, qos)
        self._command_subscription = self.create_subscription(
            WheelCommand,
            self.get_parameter('command_topic').value,
            self._command_callback,
            qos,
        )
        self._timer = self.create_timer(1.0 / rate, self._io_cycle)

    def _command_callback(self, message: WheelCommand):
        try:
            if len(message.name) != len(message.velocity):
                raise ValueError('Listas de nomes e velocidades têm tamanhos diferentes.')
            values = validate_complete_command(
                dict(zip(message.name, message.velocity)),
                self._max_wheel_velocity,
                self._min_effective_command,
            )
            if len(set(message.name)) != len(WHEEL_NAMES):
                raise ValueError('O comando contém nomes de rodas duplicados.')
        except ValueError as error:
            self.get_logger().error(f'Comando de rodas rejeitado: {error}')
            with self._lock:
                self._latest_command = {name: 0.0 for name in WHEEL_NAMES}
                self._last_command_time = None
            try:
                self._backend.stop()
            except Exception as stop_error:
                self.get_logger().error(f'Falha ao parar após comando inválido: {stop_error}')
            finally:
                self._invalidate_write_cache()
            return
        with self._lock:
            self._latest_command = values
            self._last_command_time = time.monotonic()

    def _command_signature(self, command):
        return tuple(
            radians_per_second_to_command(
                command[name],
                self._max_wheel_velocity,
                self._min_effective_command,
            )
            for name in WHEEL_NAMES
        )

    def _should_write_command(self, signature, now):
        if not self._deduplicate_commands:
            return True
        with self._lock:
            return (
                signature != self._last_written_signature
                or now - self._last_command_write_time
                >= self._command_heartbeat_period
            )

    def _record_command_write(self, signature, now):
        with self._lock:
            self._last_written_signature = signature
            self._last_command_write_time = now

    def _invalidate_write_cache(self):
        with self._lock:
            self._last_written_signature = None
            self._last_command_write_time = float('-inf')

    def _io_cycle(self):
        try:
            now = time.monotonic()
            states = self._backend.read(now=now)
            with self._lock:
                command_is_fresh = (
                    self._last_command_time is not None
                    and now - self._last_command_time <= self._command_timeout
                )
                command = dict(self._latest_command) if command_is_fresh else {
                    name: 0.0 for name in WHEEL_NAMES}
            signature = self._command_signature(command)
            if self._should_write_command(signature, now):
                self._backend.write(command)
                self._record_command_write(signature, now)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(WHEEL_NAMES)
            message.position = [states[name].position for name in WHEEL_NAMES]
            message.velocity = [states[name].velocity for name in WHEEL_NAMES]
            self._state_publisher.publish(message)
            self._failures = 0
        except Exception as error:
            self._failures += 1
            try:
                self._backend.stop()
            except Exception:
                pass
            finally:
                self._invalidate_write_cache()
            detail = f'Falha de I/O da base ({self._failures}/{self._failure_limit}): {error}'
            if self._failures >= self._failure_limit:
                self.get_logger().fatal(detail)
                raise RuntimeError(detail) from error
            self.get_logger().error(detail)

    def destroy_node(self):
        self._backend.close(stop=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = BaseHardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Driver da base encerrado: {error}')
        else:
            # Hardware construction happens before ``node`` is assigned.  Do
            # not hide serial/permission/protocol errors from ros2 launch.
            get_logger('base_hardware_node').fatal(
                f'Falha ao inicializar o hardware da base: {error}')
        traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
