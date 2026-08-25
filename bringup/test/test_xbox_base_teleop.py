"""Contracts for Xbox mecanum teleoperation."""

import math
from pathlib import Path
from types import SimpleNamespace

from bringup.xbox_base_teleop import (
    button_pressed, limit_mecanum_command, limit_planar_velocity, shaped_axis)
import pytest
import yaml


PACKAGE = Path(__file__).parents[1]


def test_axis_deadzone_is_zero_and_remaining_range_is_rescaled():
    assert shaped_axis([0.09], 0, 0.10) == 0.0
    assert shaped_axis([0.55], 0, 0.10) == pytest.approx(0.5)
    assert shaped_axis([-1.0], 0, 0.10) == -1.0


def test_invalid_or_nonfinite_input_is_safe():
    assert shaped_axis([], 0, 0.10) == 0.0
    assert shaped_axis([math.nan], 0, 0.10) == 0.0
    assert button_pressed([], 5) is False
    assert button_pressed([0, 1], 1) is True


def test_diagonal_velocity_is_scaled_without_changing_direction():
    x, y = limit_planar_velocity(0.2, 0.2, 0.238)
    assert x == pytest.approx(y)
    assert abs(x) + abs(y) == pytest.approx(0.238)


def test_turbo_combination_keeps_translation_and_rotation():
    x, y, yaw = limit_mecanum_command(0.2, 0.2, 1.2, 0.238, 0.2225)
    assert x > 0.0 and y > 0.0 and yaw > 0.0
    assert abs(x) + abs(y) + 0.2225 * abs(yaw) == pytest.approx(0.238)


def test_default_mapping_requires_deadman_and_has_conservative_speeds():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'controllers.yaml').read_text())
    params = config['xbox_base_teleop']['ros__parameters']
    assert params['enable_button'] == 5
    assert params['turbo_button'] == 4
    assert params['cancel_button'] == 1
    assert params['cmd_vel_topic'] == '/cmd_vel'
    assert params['joy_timeout_sec'] <= 0.30
    assert params['max_linear_x'] < params['turbo_linear_x']
    assert params['max_linear_speed'] == pytest.approx(0.238)


def test_workstation_starts_joy_only_when_explicitly_enabled():
    source = (PACKAGE / 'launch' / 'workstation.launch.py').read_text()
    assert "'enable_xbox_teleop', default_value='false'" in source
    assert "package='joy', executable='joy_node'" in source
    assert "executable='xbox_base_teleop'" in source
    assert source.count("LaunchConfiguration('enable_xbox_teleop')") == 2


def test_node_publishes_controller_native_twist_stamped_and_has_watchdog():
    source = (PACKAGE / 'bringup' / 'xbox_base_teleop.py').read_text()
    assert 'TwistStamped' in source
    assert "declare_parameter('joy_timeout_sec', 0.30)" in source
    assert 'self._publish_stop()' in source


def test_cancel_button_uses_rising_edge_and_stops_before_requesting_cancel():
    class FakeTeleop:
        _cancel_button = 1
        _cancel_was_pressed = False
        _enable_button = 5
        _enabled = True
        _command = (0.1, 0.0, 0.0)

        def __init__(self):
            self.events = []

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=123))

        def _publish_stop(self):
            self.events.append('stop')

        def _cancel_navigation(self):
            self.events.append('cancel')

    teleop = FakeTeleop()
    message = SimpleNamespace(buttons=[0, 1, 0, 0, 0, 1], axes=[])

    from bringup.xbox_base_teleop import XboxBaseTeleop
    XboxBaseTeleop._joy_callback(teleop, message)
    XboxBaseTeleop._joy_callback(teleop, message)

    assert teleop.events == ['stop', 'cancel']
    assert teleop._enabled is False
    assert teleop._command == (0.0, 0.0, 0.0)


def test_cancel_is_nonblocking_when_nav2_services_are_absent():
    class FakeClient:
        def __init__(self):
            self.call_count = 0

        def service_is_ready(self):
            return False

        def call_async(self, _request):
            self.call_count += 1

    class FakeLogger:
        def __init__(self):
            self.warnings = []

        def warning(self, message):
            self.warnings.append(message)

        def info(self, _message):
            raise AssertionError('Não deve informar solicitação sem serviço ativo')

    clients = {
        'navigate_to_pose': FakeClient(),
        'navigate_through_poses': FakeClient(),
    }
    logger = FakeLogger()
    teleop = SimpleNamespace(
        _cancel_clients=clients,
        get_logger=lambda: logger,
    )

    from bringup.xbox_base_teleop import XboxBaseTeleop
    XboxBaseTeleop._cancel_navigation(teleop)

    assert all(client.call_count == 0 for client in clients.values())
    assert len(logger.warnings) == 2


def test_cancel_request_targets_all_active_goals_for_both_nav2_actions():
    class FakeFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    class FakeClient:
        def __init__(self):
            self.requests = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return FakeFuture()

    class FakeLogger:
        def __init__(self):
            self.infos = []

        def warning(self, _message):
            raise AssertionError('Serviços disponíveis não devem gerar aviso')

        def info(self, message):
            self.infos.append(message)

    clients = {
        'navigate_to_pose': FakeClient(),
        'navigate_through_poses': FakeClient(),
    }
    logger = FakeLogger()
    teleop = SimpleNamespace(
        _cancel_clients=clients,
        _cancel_done=lambda *_args: None,
        get_logger=lambda: logger,
    )

    from bringup.xbox_base_teleop import XboxBaseTeleop
    XboxBaseTeleop._cancel_navigation(teleop)

    requests = [client.requests[0] for client in clients.values()]
    assert len(requests) == 2
    assert all(not any(request.goal_info.goal_id.uuid) for request in requests)
    assert all(request.goal_info.stamp.sec == 0 for request in requests)
    assert all(request.goal_info.stamp.nanosec == 0 for request in requests)
    assert len(logger.infos) == 1
