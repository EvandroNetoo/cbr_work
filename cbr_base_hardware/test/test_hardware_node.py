import os
from pathlib import Path
import time

from cbr_interfaces.msg import WheelCommand
import pytest
import rclpy
import yaml

from cbr_base_hardware.hardware_node import BaseHardwareNode
from cbr_base_hardware.mariola_adapter import WHEEL_NAMES, WheelState


PACKAGE_ROOT = Path(__file__).parents[1]


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


def test_performance_defaults_are_explicit():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'hardware.yaml').read_text())[
            'cbr_base_hardware_node']['ros__parameters']
    assert parameters['io_rate_hz'] == 30.0
    assert parameters['deduplicate_commands'] is True
    assert parameters['command_heartbeat_hz'] == 5.0

    launch_source = (PACKAGE_ROOT / 'launch' / 'driver.launch.py').read_text()
    assert "'deduplicate_commands', default_value='true'" in launch_source
    assert "'command_heartbeat_hz', default_value='5.0'" in launch_source


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


def test_repeated_quantized_command_waits_for_heartbeat(node_and_backend):
    node, backend = node_and_backend
    node._command_callback(command_message(velocities=[1.0] * 4))
    node._io_cycle()
    node._command_callback(command_message(velocities=[1.001] * 4))
    node._io_cycle()
    assert len(backend.writes) == 1

    node._last_command_write_time = (
        time.monotonic() - node._command_heartbeat_period - 0.01)
    node._io_cycle()
    assert len(backend.writes) == 2


def test_changed_quantized_command_is_written_immediately(node_and_backend):
    node, backend = node_and_backend
    node._command_callback(command_message(velocities=[1.0] * 4))
    node._io_cycle()
    node._command_callback(command_message(velocities=[2.0] * 4))
    node._io_cycle()
    assert len(backend.writes) == 2
    assert backend.writes[-1] == {name: 2.0 for name in WHEEL_NAMES}


def test_io_failure_invalidates_write_cache(node_and_backend):
    node, backend = node_and_backend
    node._command_callback(command_message())
    node._io_cycle()
    assert node._last_written_signature is not None

    backend.read_error = OSError('falha simulada')
    node._io_cycle()
    assert node._last_written_signature is None
    assert backend.stop_calls == 1


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
