"""Unit tests for the physical driver's I/O and reconnection policy."""

from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from so_arm_101_hardware.hardware_node import (
    resolve_calibration_file,
    ROS_JOINT_ORDER,
    SO101HardwareNode,
)
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


class _Logger:
    def __init__(self):
        self.fatal_messages = []
        self.error_messages = []
        self.info_messages = []

    def fatal(self, message):
        self.fatal_messages.append(message)

    def error(self, message, **_kwargs):
        self.error_messages.append(message)

    def info(self, message):
        self.info_messages.append(message)


class _Follower:
    def __init__(self):
        self.actions = []
        self.bus = SimpleNamespace(
            enable_torque_calls=[],
            disable_torque_calls=[],
        )
        self.bus.enable_torque = lambda **kwargs: (
            self.bus.enable_torque_calls.append(kwargs))
        self.bus.disable_torque = lambda **kwargs: (
            self.bus.disable_torque_calls.append(kwargs))

    def send_action(self, action):
        self.actions.append(dict(action))

    def get_observation(self):
        return {
            f'{name}.pos': 0.1
            for name in (
                'shoulder_pan', 'shoulder_lift', 'elbow_flex',
                'wrist_flex', 'wrist_roll', 'gripper')
        }


class _Timer:
    def __init__(self, period_ns):
        self.timer_period_ns = period_ns
        self.reset_calls = 0
        self.cancel_calls = 0
        self._canceled = False

    def reset(self):
        self.reset_calls += 1
        self._canceled = False

    def cancel(self):
        self.cancel_calls += 1
        self._canceled = True

    def is_canceled(self):
        return self._canceled


def _node(timeout=5.0):
    node = SO101HardwareNode.__new__(SO101HardwareNode)
    node._reconnect_timeout = timeout
    node._communication_failure_started_at = None
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def _io_node():
    node = _node()
    node._port = '/dev/test-so101'
    node._robot_id = 'test_follower'
    node._calibration_file = ''
    node._use_degrees = False
    node._torque_enabled = True
    node._write_rate_hz = 30.0
    node._active_read_rate_hz = 10.0
    node._idle_read_rate_hz = 2.0
    node._read_idle_timeout = 1.0
    node._idle_velocity_threshold = 0.02
    node._write_idle_timeout = 2.0 / node._write_rate_hz
    node._latest_command = None
    node._last_received_command = None
    node._last_sent_command = None
    node._last_command_change_time = time.monotonic()
    node._read_is_idle = False
    node._reconnect_interval = 1.0
    node._next_reconnect_time = 0.0
    node._serial_lock = threading.Lock()
    node._command_lock = threading.Lock()
    node.write_timer = _Timer(
        SO101HardwareNode._period_ns(node._write_rate_hz))
    node.write_timer.cancel()
    node.read_timer = _Timer(
        SO101HardwareNode._period_ns(node._active_read_rate_hz))
    node.follower = _Follower()
    return node


def _command(value):
    message = Float64MultiArray()
    message.data = [value for _ in ROS_JOINT_ORDER]
    return message


def test_io_failure_is_tolerated_before_timeout(monkeypatch):
    node = _node()
    clock = iter((100.0, 104.9))
    monkeypatch.setattr(time, 'monotonic', lambda: next(clock))

    node._handle_io_failure('ler', OSError('temporário'))
    node._handle_io_failure('ler', OSError('temporário'))

    assert node._communication_failure_started_at == 100.0
    assert not node._logger.fatal_messages
    assert '4.9/5.0 s' in node._logger.error_messages[-1]


def test_io_failure_at_timeout_raises(monkeypatch):
    node = _node()
    node._communication_failure_started_at = 100.0
    monkeypatch.setattr(time, 'monotonic', lambda: 105.0)

    with pytest.raises(RuntimeError):
        node._handle_io_failure('ler', OSError('permanente'))

    assert node._logger.fatal_messages


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
    assert 'command_message_.data == last_published_command_' in plugin
    assert 'get_subscription_count() > 0' in plugin


def test_io_performance_defaults_are_explicit():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'real.yaml').read_text())[
            'so101_hardware_node']['ros__parameters']
    assert parameters['write_rate_hz'] == 60.0
    assert parameters['read_rate_hz'] == 30.0
    assert parameters['idle_read_rate_hz'] == 2.0
    assert parameters['read_idle_timeout_sec'] == 1.0
    assert parameters['idle_velocity_threshold'] == 0.02
    assert 'buffer_commands' not in parameters
    assert 'deduplicate_commands' not in parameters
    assert 'command_heartbeat_hz' not in parameters
    assert parameters['reconnect_interval_sec'] == 1.0
    assert parameters['reconnect_timeout_sec'] == 5.0
    assert 'disable_torque' not in parameters
    assert 'max_consecutive_io_failures' not in parameters

    launch_source = (PACKAGE_ROOT / 'launch' / 'driver.launch.py').read_text()
    assert 'buffer_commands' not in launch_source
    assert 'deduplicate_commands' not in launch_source
    assert 'command_heartbeat_hz' not in launch_source
    assert 'OpaqueFunction' in launch_source


def test_relative_calibration_file_is_resolved_from_package_config(monkeypatch):
    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.get_package_share_directory',
        lambda _package: '/tmp/so_arm_share')
    assert resolve_calibration_file('so101_follower.json') == (
        '/tmp/so_arm_share/config/so101_follower.json')
    assert resolve_calibration_file('/dados/calibracao.json') == (
        '/dados/calibracao.json')


def test_command_callback_does_not_touch_serial_and_keeps_latest():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._command_callback(_command(0.2))
    assert node.follower.actions == []
    assert node.write_timer.is_canceled() is False

    node._write_cycle()
    assert len(node.follower.actions) == 1
    assert node._last_sent_command == tuple(0.2 for _ in ROS_JOINT_ORDER)
    assert node.write_timer.is_canceled() is False


def test_manual_positioning_mode_ignores_commands():
    node = _io_node()
    node._torque_enabled = False

    node._command_callback(_command(0.2))

    assert node._latest_command is None
    assert node.write_timer.is_canceled() is True


def test_service_disables_torque_and_discards_pending_command():
    node = _io_node()
    node._command_callback(_command(0.2))
    request = SetBool.Request(data=False)

    response = node._set_torque(request, SetBool.Response())

    assert response.success is True
    assert node._torque_enabled is False
    assert node.follower.bus.disable_torque_calls == [{'num_retry': 5}]
    assert node._latest_command is None
    assert node.write_timer.is_canceled() is True


def test_service_holds_current_pose_before_enabling_torque():
    node = _io_node()
    node._torque_enabled = False
    request = SetBool.Request(data=True)

    response = node._set_torque(request, SetBool.Response())

    assert response.success is True
    assert node._torque_enabled is True
    assert len(node.follower.actions) == 1
    assert node.follower.bus.enable_torque_calls == [{'num_retry': 5}]
    assert node._latest_command is None


def test_command_is_not_resent_while_unchanged():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._write_cycle()
    for _ in range(10):
        node._write_cycle()
    assert len(node.follower.actions) == 1


def test_changed_command_wakes_writer_and_is_sent():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._write_cycle()
    node._command_callback(_command(0.2))
    assert node.write_timer.is_canceled() is False
    node._write_cycle()
    assert len(node.follower.actions) == 2
    assert node.write_timer.is_canceled() is False


def test_identical_command_does_not_wake_idle_write_timer():
    node = _io_node()
    node._command_callback(_command(0.1))
    node._write_cycle()
    node._last_command_change_time -= node._write_idle_timeout + 0.01
    node._write_cycle()
    resets_after_send = node.write_timer.reset_calls

    node._command_callback(_command(0.1))

    assert node.write_timer.is_canceled() is True
    assert node.write_timer.reset_calls == resets_after_send


def test_stationary_arm_reduces_read_rate_after_timeout():
    node = _io_node()
    node._last_command_change_time = time.monotonic() - 2.0
    node._read_observation = lambda: [0.0 for _ in ROS_JOINT_ORDER]

    node._read_cycle()

    assert node._read_is_idle is True
    assert node.read_timer.timer_period_ns == 500_000_000
    assert node.read_timer.reset_calls == 1


def test_measured_motion_keeps_active_read_rate():
    node = _io_node()
    node._last_command_change_time = time.monotonic() - 2.0
    velocities = [0.0 for _ in ROS_JOINT_ORDER]
    velocities[0] = node._idle_velocity_threshold + 0.01
    node._read_observation = lambda: velocities

    node._read_cycle()

    assert node._read_is_idle is False
    assert node.read_timer.timer_period_ns == 100_000_000
    assert node.read_timer.reset_calls == 0


def test_new_command_immediately_restores_active_read_rate():
    node = _io_node()
    node._read_is_idle = True
    node.read_timer.timer_period_ns = 500_000_000

    node._command_callback(_command(0.1))

    assert node._read_is_idle is False
    assert node.read_timer.timer_period_ns == 100_000_000
    assert node.read_timer.reset_calls == 1


def test_read_failure_restores_active_rate_for_fast_retry():
    node = _io_node()
    node._read_is_idle = True
    node.read_timer.timer_period_ns = 500_000_000
    node._read_observation = lambda: None

    node._read_cycle()

    assert node._read_is_idle is False
    assert node.read_timer.timer_period_ns == 100_000_000
    assert node.read_timer.reset_calls == 1


def test_read_failure_replaces_stale_follower_and_resets_timeout(monkeypatch):
    node = _io_node()
    node._communication_failure_started_at = time.monotonic() - 3.0
    node._last_positions = dict.fromkeys(ROS_JOINT_ORDER, 0.1)
    node._last_sent_command = tuple(0.1 for _ in ROS_JOINT_ORDER)
    node._latest_command = dict.fromkeys(ROS_JOINT_ORDER, 0.1)
    node.write_timer.cancel()

    old_port_handler = SimpleNamespace(
        is_open=True,
        is_using=True,
        closePort=lambda: setattr(old_port_handler, 'is_open', False),
    )
    node.follower.bus = SimpleNamespace(port_handler=old_port_handler)
    candidate = SimpleNamespace(
        bus=SimpleNamespace(
            port_handler=SimpleNamespace(is_open=True, is_using=False)),
        connect=lambda *, calibrate: None,
    )
    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.make_follower',
        lambda *_args, **_kwargs: candidate,
    )

    assert node._try_reconnect() is True
    assert node.follower is candidate
    assert old_port_handler.is_open is False
    assert old_port_handler.is_using is False
    assert node._communication_failure_started_at is None
    assert node._last_positions == {}
    assert node._last_sent_command is None
    assert node.write_timer.is_canceled() is False
    assert node._logger.info_messages


def test_reconnect_while_torque_is_disabled_never_enables_it(monkeypatch):
    node = _io_node()
    node._torque_enabled = False
    old_port_handler = SimpleNamespace(
        is_open=True,
        is_using=True,
        closePort=lambda: setattr(old_port_handler, 'is_open', False),
    )
    node.follower.bus.port_handler = old_port_handler
    bus = SimpleNamespace(
        connect_calls=0,
        disable_calls=[],
        port_handler=SimpleNamespace(is_open=True, is_using=False),
    )
    bus.connect = lambda: setattr(bus, 'connect_calls', bus.connect_calls + 1)
    bus.disable_torque = lambda **kwargs: bus.disable_calls.append(kwargs)
    candidate = SimpleNamespace(
        bus=bus,
        cameras={},
        connect=lambda *, calibrate: pytest.fail(
            'connect() não pode ser usado com torque desabilitado'),
    )
    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.make_follower',
        lambda *_args, **_kwargs: candidate,
    )

    assert node._try_reconnect() is True
    assert bus.connect_calls == 1
    assert bus.disable_calls == [{'num_retry': 5}]


def test_failed_reconnect_keeps_timeout_and_is_rate_limited(monkeypatch):
    node = _io_node()
    failure_started_at = time.monotonic() - 2.0
    node._communication_failure_started_at = failure_started_at
    node._reconnect_interval = 60.0
    attempts = 0

    def unavailable_follower(*_args, **_kwargs):
        nonlocal attempts
        attempts += 1
        raise OSError('porta ausente')

    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.make_follower',
        unavailable_follower,
    )

    assert node._try_reconnect() is False
    assert node._try_reconnect() is False
    assert attempts == 1
    assert node._communication_failure_started_at == failure_started_at
    assert 'porta ausente' in node._logger.error_messages[-1]


def test_read_exception_enters_reconnection_path(monkeypatch):
    node = _io_node()
    node.follower.get_observation = lambda: (_ for _ in ()).throw(
        OSError(5, 'Input/output error'))
    candidate = SimpleNamespace(
        bus=SimpleNamespace(
            port_handler=SimpleNamespace(is_open=True, is_using=False)),
        connect=lambda *, calibrate: None,
    )
    monkeypatch.setattr(
        'so_arm_101_hardware.hardware_node.make_follower',
        lambda *_args, **_kwargs: candidate,
    )

    assert node._read_observation() is None
    assert node.follower is candidate
    assert node._communication_failure_started_at is None
