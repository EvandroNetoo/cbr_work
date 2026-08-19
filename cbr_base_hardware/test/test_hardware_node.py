import os
import time

from cbr_interfaces.msg import WheelCommand
import pytest
import rclpy

from cbr_base_hardware.hardware_node import BaseHardwareNode
from cbr_base_hardware.mariola_adapter import WHEEL_NAMES, WheelState


class FakeBackend:
    def __init__(self):
        self.writes = []
        self.stop_calls = 0
        self.close_calls = 0
        self.read_error = None

    def read(self, now=None):
        if self.read_error is not None:
            raise self.read_error
        return {name: WheelState(0.0, 0.0) for name in WHEEL_NAMES}

    def write(self, command):
        self.writes.append(dict(command))

    def stop(self):
        self.stop_calls += 1

    def close(self, *, stop=True):
        self.close_calls += 1
        if stop:
            self.stop()


@pytest.fixture
def node_and_backend():
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/cbr_ros_test_logs')
    if not rclpy.ok():
        rclpy.init()
    backend = FakeBackend()
    node = BaseHardwareNode(backend=backend)
    yield node, backend
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def command_message(names=WHEEL_NAMES, velocities=None):
    message = WheelCommand()
    message.name = list(names)
    message.velocity = list(velocities or [1.0] * len(names))
    return message


def test_valid_command_is_written_while_fresh(node_and_backend):
    node, backend = node_and_backend
    node._command_callback(command_message())
    node._io_cycle()
    assert backend.writes[-1] == {name: 1.0 for name in WHEEL_NAMES}


def test_watchdog_writes_four_zero_commands(node_and_backend):
    node, backend = node_and_backend
    node._command_callback(command_message())
    node._last_command_time = time.monotonic() - 0.31
    node._io_cycle()
    assert backend.writes[-1] == {name: 0.0 for name in WHEEL_NAMES}


@pytest.mark.parametrize('message', [
    command_message(WHEEL_NAMES[:-1]),
    command_message(WHEEL_NAMES, [0.0, 0.0, float('nan'), 0.0]),
    command_message(WHEEL_NAMES, [0.0, 0.0, 7.01, 0.0]),
])
def test_invalid_command_is_rejected_and_stops(message, node_and_backend):
    node, backend = node_and_backend
    node._command_callback(message)
    assert node._last_command_time is None
    assert backend.stop_calls == 1


def test_third_consecutive_io_failure_stops_and_raises(node_and_backend):
    node, backend = node_and_backend
    backend.read_error = OSError('falha simulada')
    node._io_cycle()
    node._io_cycle()
    with pytest.raises(RuntimeError):
        node._io_cycle()
    assert backend.stop_calls == 3
