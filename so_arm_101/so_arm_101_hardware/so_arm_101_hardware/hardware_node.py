"""Low-level LeRobot driver used by the ros2_control SystemInterface."""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
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


class SO101HardwareNode(Node):
    """Expose only the physical driver boundary to the ros2_control plugin."""

    def __init__(self):
        super().__init__('so101_hardware')
        self.declare_parameter('port', '')
        self.declare_parameter('robot_id', 'so101_follower')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('use_degrees', False)
        self.declare_parameter('read_rate_hz', 30.0)
        self.declare_parameter('state_topic', '/so101_hardware/raw_joint_states')
        self.declare_parameter('command_topic', '/so101_hardware/command_positions')
        self._lock = threading.Lock()
        self._connected = False
        self._use_degrees = bool(self.get_parameter('use_degrees').value)
        self._last_positions = {}
        self._previous_positions = {}
        self._previous_read_time = None

        self.state_pub = self.create_publisher(
            JointState, self.get_parameter('state_topic').value, 10)
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('command_topic').value,
            self._command_callback,
            10,
        )
        self.calibrate_srv = self.create_service(Trigger, '~/calibrate', self._calibrate)
        self.follower = None

        try:
            self.follower = make_follower(
                self.get_parameter('port').value,
                self.get_parameter('robot_id').value,
                use_degrees=self._use_degrees,
                calibration_file=self.get_parameter('calibration_file').value,
            )
            self.follower.connect(calibrate=False)
            self._connected = True
            self.get_logger().info('SO-101 conectado via LeRobot/Feetech.')
        except Exception as error:
            self.get_logger().fatal(f'Falha ao conectar o SO-101: {error}')

        period = 1.0 / float(self.get_parameter('read_rate_hz').value)
        self.timer = self.create_timer(period, self._read_observation)

    def _read_observation(self):
        if not self._connected:
            return
        try:
            with self._lock:
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
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(ROS_JOINT_ORDER)
            message.position = [self._last_positions[name] for name in ROS_JOINT_ORDER]
            message.velocity = velocities
            self.state_pub.publish(message)
        except Exception as error:
            self.get_logger().error(
                f'Falha ao ler os motores: {error}', throttle_duration_sec=2.0)

    def _command_callback(self, message: Float64MultiArray):
        if not self._connected:
            return
        if len(message.data) != len(ROS_JOINT_ORDER):
            self.get_logger().error(
                f'Comando inválido: esperadas {len(ROS_JOINT_ORDER)} posições, '
                f'recebidas {len(message.data)}.', throttle_duration_sec=2.0)
            return
        values = dict(zip(ROS_JOINT_ORDER, message.data))
        try:
            with self._lock:
                self.follower.send_action(
                    ros_to_action(values, use_degrees=self._use_degrees))
        except Exception as error:
            self.get_logger().error(
                f'Falha ao comandar os motores: {error}', throttle_duration_sec=2.0)

    def _calibrate(self, request, response):
        del request
        if not self._connected:
            response.success = False
            response.message = 'Braço não conectado.'
            return response
        try:
            with self._lock:
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
                with self._lock:
                    self.follower.disconnect()
            except Exception as error:
                self.get_logger().error(f'Falha ao desconectar o braço: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
