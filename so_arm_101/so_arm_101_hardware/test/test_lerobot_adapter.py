"""Tests for the ROS/LeRobot joint and adapted-gripper conversion."""

import pytest

from so_arm_101_hardware.lerobot_adapter import (
    GRIPPER_CLOSED_ANGLE_RAD,
    GRIPPER_CLOSED_POSITION_M,
    GRIPPER_OPEN_ANGLE_RAD,
    GRIPPER_OPEN_POSITION_M,
    _gripper_angle_to_position,
    _gripper_position_to_angle,
    connect_follower,
    observation_to_ros,
    ros_to_action,
)


class _Bus:
    def __init__(self):
        self.connect_calls = 0
        self.disable_torque_calls = []

    def connect(self):
        self.connect_calls += 1

    def disable_torque(self, **kwargs):
        self.disable_torque_calls.append(kwargs)


class _Camera:
    def __init__(self):
        self.connect_calls = 0

    def connect(self):
        self.connect_calls += 1


class _Follower:
    def __init__(self):
        self.bus = _Bus()
        self.cameras = {'wrist': _Camera()}
        self.connect_calls = []

    def connect(self, *, calibrate):
        self.connect_calls.append(calibrate)


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


def test_normal_connection_uses_lerobot_configuration_path():
    follower = _Follower()

    connect_follower(follower, disable_torque=False)

    assert follower.connect_calls == [False]
    assert follower.bus.connect_calls == 0
    assert follower.bus.disable_torque_calls == []


def test_manual_connection_never_uses_path_that_reenables_torque():
    follower = _Follower()

    connect_follower(follower, disable_torque=True)

    assert follower.connect_calls == []
    assert follower.bus.connect_calls == 1
    assert follower.bus.disable_torque_calls == [{'num_retry': 5}]
    assert follower.cameras['wrist'].connect_calls == 1
