"""Unit tests for keyboard teleoperation constants and limit handling."""

from so_arm_101_teleop.keyboard_teleop import (
    ARM_JOINTS,
    GRIPPER_CLOSED_POSITION,
    GRIPPER_JOINT,
    GRIPPER_OPEN_POSITION,
    GRIPPER_STEP,
    JOINT_LIMITS,
    KEY_BINDINGS,
    clamp,
    gripper_target,
)


def test_all_requested_keys_are_bound():
    """The requested twelve characters must be the complete key map."""
    assert set(KEY_BINDINGS) == set('qawsedrftgyh')


def test_each_actuator_has_a_positive_and_negative_key():
    """Every actuator must have one key in each direction."""
    expected_joints = set(ARM_JOINTS) | {GRIPPER_JOINT}
    for joint in expected_joints:
        directions = {
            direction for bound_joint, direction in KEY_BINDINGS.values()
            if bound_joint == joint
        }
        assert directions == {-1, 1}


def test_all_bound_joints_have_limits():
    """Every controlled joint must be protected by limits."""
    assert {joint for joint, _ in KEY_BINDINGS.values()} == set(JOINT_LIMITS)


def test_gripper_keys_move_the_gripper_gradually():
    """Each key press must advance the gripper by one bounded step."""
    assert KEY_BINDINGS['y'] == (GRIPPER_JOINT, -1)
    assert KEY_BINDINGS['h'] == (GRIPPER_JOINT, 1)
    assert JOINT_LIMITS[GRIPPER_JOINT] == (
        GRIPPER_CLOSED_POSITION,
        GRIPPER_OPEN_POSITION,
    )
    assert GRIPPER_OPEN_POSITION == 0.037
    assert GRIPPER_CLOSED_POSITION == 0.0
    assert gripper_target(GRIPPER_OPEN_POSITION, KEY_BINDINGS['y'][1]) == (
        GRIPPER_OPEN_POSITION - GRIPPER_STEP)
    assert gripper_target(GRIPPER_CLOSED_POSITION, KEY_BINDINGS['h'][1]) == (
        GRIPPER_CLOSED_POSITION + GRIPPER_STEP)
    assert gripper_target(
        GRIPPER_CLOSED_POSITION, -1,
        GRIPPER_STEP) == GRIPPER_CLOSED_POSITION
    assert gripper_target(
        GRIPPER_OPEN_POSITION, 1,
        GRIPPER_STEP) == GRIPPER_OPEN_POSITION


def test_clamp_respects_both_limits():
    """Clamping must constrain extremes and preserve in-range values."""
    limits = (-1.0, 2.0)
    assert clamp(-3.0, limits) == -1.0
    assert clamp(3.0, limits) == 2.0
    assert clamp(0.5, limits) == 0.5
