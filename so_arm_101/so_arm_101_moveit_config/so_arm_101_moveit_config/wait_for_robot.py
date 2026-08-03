"""Wait until Gazebo publishes the robot state and accepts arm trajectories."""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = {
    'base_link_to_link1',
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
}


class RobotReadinessWaiter(Node):
    """Exit successfully once state feedback and arm execution are available."""

    def __init__(self):
        super().__init__('wait_for_so_arm_101')
        self.ready = False
        self._state_ready = False
        self._last_report = None
        self._joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self._timer = self.create_timer(0.25, self._check_readiness)
        self.get_logger().info(
            'Waiting for complete /joint_states and arm_controller action...')

    def _on_joint_state(self, message):
        self._state_ready = ARM_JOINTS.issubset(message.name)

    def _check_readiness(self):
        action_ready = self._trajectory_client.server_is_ready()
        status = (self._state_ready, action_ready)
        if status != self._last_report:
            self.get_logger().info(
                'Readiness: joint_states=%s, arm_controller=%s'
                % status)
            self._last_report = status

        if all(status):
            self.get_logger().info(
                'SO-ARM-101 is ready; starting MoveIt and RViz.')
            self.ready = True
            self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = RobotReadinessWaiter()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
