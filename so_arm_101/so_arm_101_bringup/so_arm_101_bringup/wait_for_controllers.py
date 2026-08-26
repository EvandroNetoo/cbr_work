"""Wait until the required ros2_control controllers are active."""

from __future__ import annotations

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


REQUIRED_CONTROLLERS = (
    'joint_state_broadcaster',
    'arm_controller',
    'gripper_controller',
)


class ControllerReadiness(Node):
    def __init__(self) -> None:
        super().__init__('wait_for_so101_controllers')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('timeout_sec', 10.0)
        self._client = self.create_client(
            ListControllers,
            self.get_parameter('controller_manager').value + '/list_controllers',
        )
        # The control manager starts only after the physical state readiness
        # gate.  Do not spend the controller timeout while waiting for that
        # gate; start the deadline when the service first appears.
        self._deadline = None
        self._future = None
        self._exit_code = 1
        self._timer = self.create_timer(0.1, self._poll)

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def _poll(self) -> None:
        if self._deadline is not None and self.get_clock().now().nanoseconds >= self._deadline:
            self.get_logger().fatal('Timeout aguardando controllers ativos.')
            rclpy.shutdown()
            return
        if self._future is not None:
            if not self._future.done():
                return
            try:
                result = self._future.result()
            except Exception as error:
                self.get_logger().warning(f'Falha consultando controllers: {error}')
                self._future = None
                return
            active = {
                controller.name for controller in result.controller
                if controller.state == 'active'
            }
            if set(REQUIRED_CONTROLLERS).issubset(active):
                self._exit_code = 0
                self.get_logger().info('Controllers do SO-101 estão ativos.')
                rclpy.shutdown()
            else:
                self._future = None
            return
        if self._client.service_is_ready():
            if self._deadline is None:
                self._deadline = self.get_clock().now().nanoseconds + int(
                    float(self.get_parameter('timeout_sec').value) * 1e9)
            self._future = self._client.call_async(ListControllers.Request())


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ControllerReadiness()
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
