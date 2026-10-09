"""Tests for the ROS/LeRobot joint and adapted-gripper conversion."""

import pytest
import sys
from types import ModuleType, SimpleNamespace

from so_arm_101_hardware.lerobot_adapter import (
    GRIPPER_CLOSED_ANGLE_RAD,
    GRIPPER_CLOSED_POSITION_M,
    GRIPPER_OPEN_ANGLE_RAD,
    GRIPPER_OPEN_POSITION_M,
    _gripper_angle_to_position,
    _gripper_position_to_angle,
    observation_to_ros,
    ros_to_action,
    make_follower,
)


def test_gripper_open_endpoint_is_consistent_in_both_directions():
    observation = observation_to_ros(
        {'gripper.pos': GRIPPER_OPEN_ANGLE_RAD}, use_degrees=False)
    action = ros_to_action(
        {'right_clamp': GRIPPER_OPEN_POSITION_M}, use_degrees=False)
    assert observation['right_clamp'] == pytest.approx(GRIPPER_OPEN_POSITION_M)
    assert action['gripper.pos'] == pytest.approx(GRIPPER_OPEN_ANGLE_RAD)


def test_gripper_closed_endpoint_is_consistent_in_both_directions():
    assert _gripper_angle_to_position(GRIPPER_CLOSED_ANGLE_RAD) == pytest.approx(
        GRIPPER_CLOSED_POSITION_M)
    assert _gripper_position_to_angle(GRIPPER_CLOSED_POSITION_M) == pytest.approx(
        GRIPPER_CLOSED_ANGLE_RAD)


def test_follower_receives_configured_pid(monkeypatch):
    module = ModuleType('lerobot.robots.so_follower')
    module.SO101FollowerConfig = lambda **kwargs: SimpleNamespace(**kwargs)
    module.SO101Follower = lambda config: SimpleNamespace(config=config)
    monkeypatch.setitem(sys.modules, 'lerobot.robots.so_follower', module)

    follower = make_follower(
        '/dev/test', 'test', position_p_coefficient=24,
        position_i_coefficient=1, position_d_coefficient=40)

    assert follower.config.position_p_coefficient == 24
    assert follower.config.position_i_coefficient == 1
    assert follower.config.position_d_coefficient == 40


@pytest.mark.parametrize('name', [
    'position_p_coefficient', 'position_i_coefficient', 'position_d_coefficient',
])
@pytest.mark.parametrize('value', [-1, 256, 16.5, True])
def test_invalid_pid_is_rejected_before_accessing_hardware(name, value):
    with pytest.raises(ValueError, match=name):
        make_follower('/dev/test', 'test', **{name: value})
