"""Gate the shared controller manager on complete arm and wheel feedback."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = {
    'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
    'link3_to_link4', 'link4_to_link5', 'right_clamp',
}
WHEEL_JOINTS = {
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
}


class HardwareReadiness(Node):
    def __init__(self):
        super().__init__('wait_for_cbr_hardware_states')
        self.declare_parameter('timeout_sec', 45.0)
        self._arm_ready = False
        self._base_ready = False
        self._exit_code = 1
        self._deadline = self.get_clock().now().nanoseconds + int(
            float(self.get_parameter('timeout_sec').value) * 1e9)
        self.create_subscription(
            JointState, '/so101_hardware/raw_joint_states',
            lambda message: self._check(message, ARM_JOINTS, 'arm'), 1)
        self.create_subscription(
            JointState, '/cbr_base_hardware/raw_joint_states',
            lambda message: self._check(message, WHEEL_JOINTS, 'base'), 1)
        self.create_timer(0.1, self._timeout)

    def _check(self, message, expected, source):
        positions = dict(zip(message.name, message.position))
        if expected.issubset(positions) and all(
            math.isfinite(float(positions[name])) for name in expected
        ):
            if source == 'arm':
                self._arm_ready = True
            else:
                self._base_ready = True
        if self._arm_ready and self._base_ready:
            self._exit_code = 0
            self.get_logger().info('Braço e base forneceram estados completos.')
            rclpy.shutdown()

    def _timeout(self):
        if self.get_clock().now().nanoseconds >= self._deadline:
            self.get_logger().fatal('Timeout aguardando braço e base móvel.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        code = node._exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
