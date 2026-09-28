"""State-owning facade for the usb_cam capture service."""

from __future__ import annotations

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


def capture_succeeded(response, enabled: bool) -> bool:
    if response is None:
        return False
    if response.success:
        return True
    expected = 'start capturing' if enabled else 'stop capturing'
    return response.message.strip().casefold() == expected


class CaptureControl(Node):
    def __init__(self):
        super().__init__('capture_control')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._state: bool | None = None
        self._initializing = False
        self._driver = self.create_client(
            SetBool, '/camera/driver/set_capture', callback_group=self._group)
        self._initialize_timer = self.create_timer(
            0.2, self._initialize, callback_group=self._group)
        # The public endpoints appear only after the driver has acknowledged
        # the initial off state.
        self._set_service = None
        self._get_service = None

    def _initialize(self):
        with self._lock:
            if self._initializing or not self._driver.service_is_ready():
                return
            self._initializing = True
        request = SetBool.Request()
        request.data = False
        future = self._driver.call_async(request)
        future.add_done_callback(self._initialized)

    def _initialized(self, future):
        try:
            response = future.result()
            ok = capture_succeeded(response, False)
        except Exception as error:
            self.get_logger().warning(f'Falha ao inicializar câmera: {error}')
            ok = False
        with self._lock:
            self._initializing = False
            if not ok:
                return
            self._state = False
            self._initialize_timer.cancel()
            self._set_service = self.create_service(
                SetBool, '/camera/set_capture', self._set_capture,
                callback_group=self._group)
            self._get_service = self.create_service(
                Trigger, '/camera/get_capture', self._get_capture,
                callback_group=self._group)
        self.get_logger().info('Controle de captura pronto; câmera desligada.')

    def _get_capture(self, _request, response):
        with self._lock:
            response.success = self._state is not None
            response.message = 'on' if self._state else 'off'
        return response

    def _set_capture(self, request, response):
        enabled = bool(request.data)
        with self._lock:
            if self._state is enabled:
                response.success = True
                response.message = 'already on' if enabled else 'already off'
                return response
            if not self._driver.service_is_ready():
                response.success = False
                response.message = 'Serviço do driver indisponível.'
                return response
            future = self._driver.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(timeout=5.0):
                self._state = None
                response.success = False
                response.message = 'Tempo limite ao controlar a câmera.'
                return response
            try:
                driver_response = future.result()
                response.success = capture_succeeded(driver_response, enabled)
                response.message = driver_response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
            if response.success:
                self._state = enabled
            else:
                self._state = None
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CaptureControl()
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
