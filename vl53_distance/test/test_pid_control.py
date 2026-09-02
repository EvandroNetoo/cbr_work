import pytest

from vl53_distance.control import (
    FollowWallController,
    limit_mecanum_command,
)
from vl53_distance.pid import PIDConfig, PIDController


def _pid(kp, *, ki=0.0, kd=0.0, limit=1.0):
    return PIDController(PIDConfig(
        kp=kp,
        ki=ki,
        kd=kd,
        integral_limit=10.0,
        derivative_filter_alpha=0.2,
        output_limit=limit,
    ))


def test_pid_saturates_and_does_not_wind_up_further():
    pid = _pid(0.0, ki=1.0, limit=0.1)
    assert pid.update(1.0, 1.0) == pytest.approx(0.1)
    assert pid.update(0.0, 1.0) == pytest.approx(0.0)


def _wall_controller():
    return FollowWallController(
        _pid(0.8, limit=0.1),
        _pid(0.8, limit=0.1),
        _pid(4.0, limit=0.5),
        wheel_linear_speed=0.238,
        kinematic_lever=0.2225,
    )


def test_follow_wall_converts_public_right_positive_to_ros_y_negative():
    controller = _wall_controller()
    right = controller.calculate(300, 300, 300, 10, 0.0, 500, 10, 0.1)
    assert right.linear_y_velocity_mps == pytest.approx(-0.1)
    assert right.travel_error_mm == pytest.approx(500.0)

    controller.reset()
    left = controller.calculate(300, 300, 300, 10, 0.0, -500, 10, 0.1)
    assert left.linear_y_velocity_mps == pytest.approx(0.1)
    assert left.travel_error_mm == pytest.approx(-500.0)


def test_follow_wall_controls_all_axes_and_slows_near_target():
    controller = _wall_controller()
    command = controller.calculate(350, 400, 300, 10, 480.0, 500, 5, 0.1)
    assert command.linear_x_velocity_mps > 0.0
    assert command.linear_y_velocity_mps == pytest.approx(-0.016)
    assert command.angular_velocity_rad_s > 0.0
    assert not command.inside_tolerance


def test_follow_wall_stops_only_when_wall_and_travel_are_inside_tolerance():
    controller = _wall_controller()
    inside = controller.calculate(290, 310, 300, 10, 495.0, 500, 10, 0.1)
    assert inside.inside_tolerance
    assert inside.linear_x_velocity_mps == 0.0
    assert inside.linear_y_velocity_mps == 0.0
    assert inside.angular_velocity_rad_s == 0.0

    wall_outside = controller.calculate(
        289, 311, 300, 10, 500.0, 500, 10, 0.1)
    assert not wall_outside.inside_tolerance


def test_mecanum_limit_scales_three_axes_together():
    command = limit_mecanum_command(0.2, -0.2, 1.0, 0.238, 0.2225)
    assert command[0] > 0.0
    assert command[1] < 0.0
    assert command[2] > 0.0
    assert (
        abs(command[0]) + abs(command[1]) + 0.2225 * abs(command[2])
    ) == pytest.approx(0.238)
