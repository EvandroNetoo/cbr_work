"""Low-level LeRobot driver used by the ros2_control SystemInterface."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .lerobot_adapter import (
    LEROBOT_TO_ROS,
    make_follower,
    observation_to_ros,
    ros_to_action,
)


ROS_JOINT_ORDER = tuple(LEROBOT_TO_ROS.values())


def resolve_calibration_file(value: str) -> str:
    """Resolve nomes relativos da configuração sem fixar o workspace."""
    if not value:
        return ''
    path = Path(value)
    if path.is_absolute():
        return str(path)
    return str(
        Path(get_package_share_directory('so_arm_101_hardware'))
        / 'config' / path)


class SO101HardwareNode(Node):
    """Expose only the physical driver boundary to the ros2_control plugin."""

    def __init__(self):
        super().__init__('so101_hardware')
        self.declare_parameter('port', '')
        self.declare_parameter('robot_id', 'so101_follower')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('use_degrees', False)
        self.declare_parameter('read_rate_hz', 30.0)
        self.declare_parameter('buffer_commands', True)
        self.declare_parameter('deduplicate_commands', True)
        self.declare_parameter('command_heartbeat_hz', 5.0)
        self.declare_parameter('max_consecutive_io_failures', 5)
        self.declare_parameter('state_topic', '/so101_hardware/raw_joint_states')
        self.declare_parameter('command_topic', '/so101_hardware/command_positions')
        rate = float(self.get_parameter('read_rate_hz').value)
        heartbeat_rate = float(
            self.get_parameter('command_heartbeat_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('read_rate_hz deve ser positivo e finito.')
        if not math.isfinite(heartbeat_rate) or heartbeat_rate <= 0.0:
            raise ValueError(
                'command_heartbeat_hz deve ser positivo e finito.')
        self._serial_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._connected = False
        self._use_degrees = bool(self.get_parameter('use_degrees').value)
        self._buffer_commands = bool(
            self.get_parameter('buffer_commands').value)
        self._deduplicate_commands = bool(
            self.get_parameter('deduplicate_commands').value)
        self._command_heartbeat_period = 1.0 / heartbeat_rate
        self._latest_command = None
        self._last_sent_command = None
        self._last_command_write_time = float('-inf')
        self._last_positions = {}
        self._previous_positions = {}
        self._previous_read_time = None
        self._max_consecutive_io_failures = max(
            1, int(self.get_parameter('max_consecutive_io_failures').value))
        self._read_failures = 0
        self._write_failures = 0

        latest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # Commands and states supersede older samples.  BEST_EFFORT keeps
            # DDS backpressure out of the ros2_control real-time loop.
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.state_pub = self.create_publisher(
            JointState, self.get_parameter('state_topic').value, latest_qos)
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('command_topic').value,
            self._command_callback,
            latest_qos,
        )
        self.calibrate_srv = self.create_service(Trigger, '~/calibrate', self._calibrate)
        self.follower = None

        try:
            self.follower = make_follower(
                self.get_parameter('port').value,
                self.get_parameter('robot_id').value,
                use_degrees=self._use_degrees,
                calibration_file=resolve_calibration_file(
                    self.get_parameter('calibration_file').value),
            )
            self.follower.connect(calibrate=False)
            self._connected = True
            self.get_logger().info('SO-101 conectado via LeRobot/Feetech.')
        except Exception as error:
            self.get_logger().fatal(f'Falha ao conectar o SO-101: {error}')
            raise RuntimeError(f'Falha ao conectar o SO-101: {error}') from error

        self.timer = self.create_timer(1.0 / rate, self._io_cycle)

    @staticmethod
    def _command_signature(values):
        return tuple(values[name] for name in ROS_JOINT_ORDER)

    def _should_send_command(self, values, now):
        if not self._deduplicate_commands:
            return True
        signature = self._command_signature(values)
        return (
            signature != self._last_sent_command
            or now - self._last_command_write_time
            >= self._command_heartbeat_period
        )

    def _send_command(self, values, now):
        with self._serial_lock:
            self.follower.send_action(
                ros_to_action(values, use_degrees=self._use_degrees))
        self._last_sent_command = self._command_signature(values)
        self._last_command_write_time = now
        self._reset_io_failure('_write_failures')

    def _io_cycle(self):
        if not self._connected:
            return
        if self._buffer_commands:
            with self._command_lock:
                command = (
                    dict(self._latest_command)
                    if self._latest_command is not None else None)
            now = time.monotonic()
            if command is not None and self._should_send_command(command, now):
                try:
                    self._send_command(command, now)
                except Exception as error:
                    self._handle_io_failure(
                        'comandar', error, '_write_failures')
        self._read_observation()

    def _read_observation(self):
        if not self._connected:
            return
        try:
            with self._serial_lock:
                observation = self.follower.get_observation()
            positions = observation_to_ros(
                observation, use_degrees=self._use_degrees)
            self._last_positions.update(positions)
            if not all(name in self._last_positions for name in ROS_JOINT_ORDER):
                return
            read_time = time.monotonic()
            if self._previous_read_time is None:
                velocities = [0.0 for _ in ROS_JOINT_ORDER]
            else:
                period = read_time - self._previous_read_time
                velocities = [
                    (self._last_positions[name] - self._previous_positions[name]) / period
                    if period > 0.0 else 0.0
                    for name in ROS_JOINT_ORDER
                ]
            self._previous_positions = {
                name: self._last_positions[name] for name in ROS_JOINT_ORDER}
            self._previous_read_time = read_time
            self._reset_io_failure('_read_failures')
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(ROS_JOINT_ORDER)
            message.position = [self._last_positions[name] for name in ROS_JOINT_ORDER]
            message.velocity = velocities
            self.state_pub.publish(message)
        except Exception as error:
            self._handle_io_failure('ler', error, '_read_failures')

    def _command_callback(self, message: Float64MultiArray):
        if not self._connected:
            return
        if len(message.data) != len(ROS_JOINT_ORDER):
            self.get_logger().error(
                f'Comando inválido: esperadas {len(ROS_JOINT_ORDER)} posições, '
                f'recebidas {len(message.data)}.', throttle_duration_sec=2.0)
            return
        if not all(math.isfinite(value) for value in message.data):
            self.get_logger().error(
                'Comando inválido: todas as posições devem ser finitas.',
                throttle_duration_sec=2.0)
            return
        values = dict(zip(ROS_JOINT_ORDER, message.data))
        if self._buffer_commands:
            with self._command_lock:
                self._latest_command = values
            return
        try:
            self._send_command(values, time.monotonic())
        except Exception as error:
            self._handle_io_failure('comandar', error, '_write_failures')

    def _handle_io_failure(self, operation, error, counter_name):
        count = getattr(self, counter_name) + 1
        setattr(self, counter_name, count)
        message = (
            f'Falha ao {operation} os motores ({count}/'
            f'{self._max_consecutive_io_failures}): {error}')
        if count >= self._max_consecutive_io_failures:
            self.get_logger().fatal(message)
            raise RuntimeError(message) from error
        self.get_logger().error(message, throttle_duration_sec=2.0)

    def _reset_io_failure(self, counter_name):
        setattr(self, counter_name, 0)

    def _calibrate(self, request, response):
        del request
        if not self._connected:
            response.success = False
            response.message = 'Braço não conectado.'
            return response
        try:
            with self._serial_lock:
                self.follower.calibrate()
            response.success = True
            response.message = 'Calibração concluída.'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def destroy_node(self):
        if self._connected:
            try:
                with self._serial_lock:
                    self.follower.disconnect()
            except Exception as error:
                self.get_logger().error(f'Falha ao desconectar o braço: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SO101HardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Nó de hardware encerrado: {error}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
