"""Wait for a complete, finite hardware state before starting ros2_control."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


EXPECTED_JOINTS = (
    'base_link_to_link1',
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
    'right_clamp',
)


class JointStateReadiness(Node):
    """Exit successfully after receiving one complete physical observation."""

    def __init__(self) -> None:
        super().__init__('wait_for_so101_joint_states')
        self.declare_parameter('state_topic', '/so101_hardware/raw_joint_states')
        self.declare_parameter('timeout_sec', 10.0)
        self._exit_code = 1
        self._deadline = self.get_clock().now().nanoseconds + int(
            float(self.get_parameter('timeout_sec').value) * 1e9)
        self._subscription = self.create_subscription(
            JointState,
            self.get_parameter('state_topic').value,
            self._state_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.1, self._check_timeout)

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def _state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(
            name in positions and math.isfinite(float(positions[name]))
            for name in EXPECTED_JOINTS
        ):
            return
        self._exit_code = 0
        self.get_logger().info('Estado físico completo recebido.')
        rclpy.shutdown()

    def _check_timeout(self) -> None:
        if self.get_clock().now().nanoseconds >= self._deadline:
            self.get_logger().fatal(
                'Timeout aguardando estado físico completo do SO-101.')
            rclpy.shutdown()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = JointStateReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
