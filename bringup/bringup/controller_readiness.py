"""Exit successfully only after the requested ros2_control controllers are active."""

from __future__ import annotations

import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ControllerReadiness(Node):
    """One-shot, non-blocking readiness probe for controller_manager."""

    def __init__(self) -> None:
        super().__init__('controller_readiness')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('controllers', ['joint_state_broadcaster'])
        self.declare_parameter('timeout_sec', 120.0)

        self._expected = set(self.get_parameter('controllers').value)
        timeout = float(self.get_parameter('timeout_sec').value)
        if not self._expected or timeout <= 0.0:
            raise ValueError('controllers e timeout_sec devem ser válidos.')

        service = (
            str(self.get_parameter('controller_manager').value).rstrip('/')
            + '/list_controllers'
        )
        self._client = self.create_client(ListControllers, service)
        self._deadline = time.monotonic() + timeout
        self._request_pending = False
        self._exit_code = 1
        self.create_timer(0.2, self._poll)

    def _poll(self) -> None:
        if time.monotonic() >= self._deadline:
            self.get_logger().fatal(
                'Timeout aguardando controllers ativos: '
                + ', '.join(sorted(self._expected)))
            rclpy.shutdown()
            return
        if self._request_pending or not self._client.service_is_ready():
            return
        self._request_pending = True
        future = self._client.call_async(ListControllers.Request())
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future) -> None:
        self._request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f'Falha transitória consultando controller_manager: {error}',
                throttle_duration_sec=2.0)
            return
        active = {
            controller.name for controller in response.controller
            if controller.state == 'active'
        }
        if self._expected.issubset(active):
            self._exit_code = 0
            self.get_logger().info(
                'Controllers ativos: ' + ', '.join(sorted(self._expected)))
            rclpy.shutdown()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ControllerReadiness()
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
