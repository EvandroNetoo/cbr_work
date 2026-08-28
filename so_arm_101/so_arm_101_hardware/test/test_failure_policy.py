"""Unit tests for the physical driver's fail-fast communication policy."""

from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from std_msgs.msg import Float64MultiArray
import yaml

from so_arm_101_hardware.hardware_node import (
    ROS_JOINT_ORDER,
    SO101HardwareNode,
    resolve_calibration_file,
)


PACKAGE_ROOT = Path(__file__).parents[1]


class _Logger:
    def __init__(self):
        self.fatal_messages = []
        self.error_messages = []

    def fatal(self, message):
        self.fatal_messages.append(message)

    def error(self, message, **_kwargs):
        self.error_messages.append(message)


class _Follower:
    def __init__(self):
        self.actions = []

    def send_action(self, action):
        self.actions.append(dict(action))


def _node(limit=5):
    node = SO101HardwareNode.__new__(SO101HardwareNode)
    node._max_consecutive_io_failures = limit
    node._read_failures = 0
    node._write_failures = 0
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def _io_node(*, buffered=True, deduplicate=True):
    node = _node()
    node._connected = True
    node._use_degrees = False
    node._buffer_commands = buffered
    node._deduplicate_commands = deduplicate
    node._command_heartbeat_period = 0.2
    node._latest_command = None
    node._last_sent_command = None
    node._last_command_write_time = float('-inf')
    node._serial_lock = threading.Lock()
    node._command_lock = threading.Lock()
    node.follower = _Follower()
    node.read_calls = 0
    node._read_observation = lambda: setattr(
        node, 'read_calls', node.read_calls + 1)
    return node


def _command(value):
    message = Float64MultiArray()
    message.data = [value for _ in ROS_JOINT_ORDER]
    return message


def test_io_failure_is_tolerated_before_limit():
    node = _node()
    for _ in range(4):
        node._handle_io_failure('ler', OSError('temporário'), '_read_failures')
    assert node._read_failures == 4
    assert not node._logger.fatal_messages


def test_fifth_consecutive_io_failure_raises():
    node = _node()
    for _ in range(4):
        node._handle_io_failure('ler', OSError('temporário'), '_read_failures')
    with pytest.raises(RuntimeError):
        node._handle_io_failure('ler', OSError('permanente'), '_read_failures')
    assert node._read_failures == 5
    assert node._logger.fatal_messages


def test_successful_operation_resets_counter_contract():
    node = _node()
    node._read_failures = 4
    node._reset_io_failure('_read_failures')
    assert node._read_failures == 0


def test_io_failure_releases_busy_flag_leaked_by_feetech_sdk():
    node = _node()
    node._serial_lock = threading.Lock()
    port_handler = SimpleNamespace(is_using=True)
    node.follower = SimpleNamespace(
        bus=SimpleNamespace(port_handler=port_handler))

    node._handle_io_failure(
        'ler', OSError(5, 'Input/output error'), '_read_failures')

    assert port_handler.is_using is False


def test_internal_hardware_topics_keep_only_latest_sample():
    source = (PACKAGE_ROOT / 'so_arm_101_hardware' / 'hardware_node.py').read_text()
    plugin = (
        PACKAGE_ROOT.parent / 'so_arm_101_hardware_interface'
        / 'src' / 'so101_system.cpp'
    ).read_text()
    assert 'depth=1' in source
    assert 'KeepLast(1)' in plugin
    assert 'ReliabilityPolicy.BEST_EFFORT' in source
    assert 'best_effort()' in plugin


def test_buffering_performance_defaults_are_explicit():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'real.yaml').read_text())[
            'so101_hardware_node']['ros__parameters']
    assert parameters['read_rate_hz'] == 30.0
    assert parameters['buffer_commands'] is True
    assert parameters['deduplicate_commands'] is True
    assert parameters['command_heartbeat_hz'] == 5.0
    assert parameters['max_consecutive_io_failures'] == 30

    launch_source = (PACKAGE_ROOT / 'launch' / 'driver.launch.py').read_text()
    assert "'buffer_commands', default_value=CONFIG_DEFAULT" in launch_source
    assert "'deduplicate_commands', default_value=CONFIG_DEFAULT" in launch_source
    assert "'command_heartbeat_hz', default_value=CONFIG_DEFAULT" in launch_source
    assert 'OpaqueFunction' in launch_source


def test_relative_calibration_file_is_resolved_from_package_config(monkeypatch):
    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.get_package_share_directory',
        lambda _package: '/tmp/so_arm_share')
    assert resolve_calibration_file('so101_follower.json') == (
        '/tmp/so_arm_share/config/so101_follower.json')
    assert resolve_calibration_file('/dados/calibracao.json') == (
        '/dados/calibracao.json')


def test_buffered_callback_does_not_touch_serial_and_keeps_latest():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._command_callback(_command(0.2))
    assert node.follower.actions == []

    node._io_cycle()
    assert len(node.follower.actions) == 1
    assert node._last_sent_command == tuple(0.2 for _ in ROS_JOINT_ORDER)
    assert node.read_calls == 1


def test_buffered_command_is_deduplicated_until_heartbeat():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._io_cycle()
    node._io_cycle()
    assert len(node.follower.actions) == 1

    node._last_command_write_time -= node._command_heartbeat_period + 0.01
    node._io_cycle()
    assert len(node.follower.actions) == 2


def test_changed_buffered_command_is_sent_immediately():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._io_cycle()
    node._command_callback(_command(0.2))
    node._io_cycle()
    assert len(node.follower.actions) == 2


def test_buffered_mode_without_deduplication_writes_every_cycle():
    node = _io_node(deduplicate=False)
    node._command_callback(_command(0.1))
    node._io_cycle()
    node._io_cycle()
    assert len(node.follower.actions) == 2


def test_direct_mode_preserves_immediate_write():
    node = _io_node(buffered=False)
    node._command_callback(_command(0.1))
    assert len(node.follower.actions) == 1
