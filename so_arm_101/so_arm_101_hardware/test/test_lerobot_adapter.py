"""Tests for the ROS/LeRobot joint and adapted-gripper conversion."""

import pytest

from so_arm_101_hardware.lerobot_adapter import (
    GRIPPER_CLOSED_ANGLE_RAD,
    GRIPPER_CLOSED_POSITION_M,
    GRIPPER_OPEN_ANGLE_RAD,
    GRIPPER_OPEN_POSITION_M,
    _gripper_angle_to_position,
    _gripper_position_to_angle,
    observation_to_ros,
    ros_to_action,
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
