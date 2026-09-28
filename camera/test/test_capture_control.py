import threading
from types import SimpleNamespace

from camera.capture_control import CaptureControl, capture_succeeded
from std_srvs.srv import SetBool, Trigger


class _Future:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class _Driver:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request.data)
        return _Future(SetBool.Response(
            success=False,
            message='Start Capturing' if request.data else 'Stop Capturing'))


def test_capture_control_reports_state_and_skips_repeated_commands():
    control = CaptureControl.__new__(CaptureControl)
    control._lock = threading.RLock()
    control._state = False
    control._driver = _Driver()

    state = control._get_capture(Trigger.Request(), Trigger.Response())
    assert state.success and state.message == 'off'
    started = control._set_capture(
        SetBool.Request(data=True), SetBool.Response())
    repeated = control._set_capture(
        SetBool.Request(data=True), SetBool.Response())
    state = control._get_capture(Trigger.Request(), Trigger.Response())
    stopped = control._set_capture(
        SetBool.Request(data=False), SetBool.Response())

    assert started.success and repeated.success and stopped.success
    assert state.success and state.message == 'on'
    assert control._driver.requests == [True, False]


def test_usb_cam_nonstandard_success_response_is_recognized():
    assert capture_succeeded(SimpleNamespace(
        success=False, message='Start Capturing'), True)
    assert not capture_succeeded(SimpleNamespace(
        success=False, message='unrelated failure'), True)
