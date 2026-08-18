from cbr_motor_control.kinematics import (
    sequence_to_motor_speeds,
    twist_to_motor_speeds,
)
import pytest


LIMITS = {
    'max_linear_speed': 0.5,
    'max_angular_speed': 1.5,
    'max_motor_speed': 40.0,
}


def test_forward_uses_all_wheels_equally():
    speeds = twist_to_motor_speeds(0.5, 0.0, 0.0, **LIMITS)
    assert list(speeds.values()) == [40, 40, 40, 40]


def test_left_strafe_uses_x_pattern():
    speeds = twist_to_motor_speeds(0.0, 0.5, 0.0, **LIMITS)
    assert list(speeds.values()) == [-40, 40, 40, -40]


def test_counter_clockwise_rotation_uses_opposite_sides():
    speeds = twist_to_motor_speeds(0.0, 0.0, 1.5, **LIMITS)
    assert list(speeds.values()) == [-40, 40, -40, 40]


def test_combined_command_is_normalized_to_speed_limit():
    speeds = twist_to_motor_speeds(0.5, 0.5, 1.5, **LIMITS)
    assert max(abs(value) for value in speeds.values()) == 40
    assert list(speeds.values()) == [-13, 40, 13, 13]


def test_invalid_or_non_finite_limits_are_rejected():
    with pytest.raises(ValueError):
        twist_to_motor_speeds(0.0, 0.0, 0.0, **{
            **LIMITS, 'max_linear_speed': 0.0})
    with pytest.raises(ValueError):
        twist_to_motor_speeds(float('nan'), 0.0, 0.0, **LIMITS)


def test_direct_speed_order_and_clamping():
    assert sequence_to_motor_speeds([120, -120, 25.6, -25.6]) == {
        'dianteiro_esquerdo': 100,
        'dianteiro_direito': -100,
        'traseiro_esquerdo': 26,
        'traseiro_direito': -26,
    }
    with pytest.raises(ValueError):
        sequence_to_motor_speeds([1, 2, 3])


def test_direct_speed_obeys_configured_safety_limit():
    assert list(sequence_to_motor_speeds(
        [50, -50, 5, -5],
        max_motor_speed=15.0,
    ).values()) == [15, -15, 5, -5]
