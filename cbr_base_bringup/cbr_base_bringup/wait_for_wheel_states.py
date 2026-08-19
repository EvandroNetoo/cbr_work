"""Wait for one finite, complete wheel state message."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEELS = {
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
}


class WheelReadiness(Node):
    def __init__(self):
        super().__init__('wait_for_cbr_wheel_states')
        self._exit_code = 1
        self._deadline = self.get_clock().now().nanoseconds + int(15.0e9)
        self.create_subscription(
            JointState, '/cbr_base_hardware/raw_joint_states', self._state, 1)
        self.create_timer(0.1, self._timeout)

    def _state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if WHEELS == set(positions) == set(velocities) and all(
            math.isfinite(positions[name]) and math.isfinite(velocities[name]) for name in WHEELS
        ):
            self._exit_code = 0
            rclpy.shutdown()

    def _timeout(self):
        if self.get_clock().now().nanoseconds >= self._deadline:
            self.get_logger().fatal('Timeout aguardando estados das rodas.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WheelReadiness()
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
