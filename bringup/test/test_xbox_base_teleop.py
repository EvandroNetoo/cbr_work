"""Contracts for Xbox mecanum teleoperation."""

import math
from pathlib import Path

import pytest
import yaml

from bringup.xbox_base_teleop import (
    button_pressed, limit_mecanum_command, limit_planar_velocity, shaped_axis)


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
