"""Wait for one finite, complete wheel state message."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState


WHEELS = {
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
}


class WheelReadiness(Node):
    def __init__(self):
        super().__init__('wait_for_wheel_states')
        self._exit_code = 1
        self._wheels_ready = False
        self._imu_ready = False
        self._deadline = self.get_clock().now().nanoseconds + int(20.0e9)
        self.create_subscription(
            JointState, '/base_hardware/raw_joint_states', self._state, 1)
        self.create_subscription(
            Imu, '/imu/data', self._imu, qos_profile_sensor_data)
        self.create_timer(0.1, self._timeout)

    def _state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if WHEELS == set(positions) == set(velocities) and all(
            math.isfinite(positions[name]) and math.isfinite(velocities[name]) for name in WHEELS
        ):
            self._wheels_ready = True
            self._finish_if_ready()

    def _imu(self, message):
        values = (
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w,
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if message.header.frame_id == 'imu_link' and all(map(math.isfinite, values)):
            self._imu_ready = True
            self._finish_if_ready()

    def _finish_if_ready(self):
        if self._wheels_ready and self._imu_ready:
            self._exit_code = 0
            self.get_logger().info('Rodas e IMU forneceram estados válidos.')
            rclpy.shutdown()

    def _timeout(self):
        if self.get_clock().now().nanoseconds >= self._deadline:
            self.get_logger().fatal('Timeout aguardando estados das rodas e IMU.')
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
