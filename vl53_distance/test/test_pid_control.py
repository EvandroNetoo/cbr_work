import pytest

from vl53_distance.control import DistanceController
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


def test_controller_moves_forward_when_far_and_backward_when_too_close():
    controller = DistanceController(_pid(0.8, limit=0.1), _pid(4.0, limit=0.5))

    far = controller.calculate(500, 500, 300, 10, 0.1)
    assert far.linear_velocity_mps == pytest.approx(0.1)
    assert far.angular_velocity_rad_s == pytest.approx(0.0)

    controller.reset()
    close = controller.calculate(200, 200, 300, 10, 0.1)
    assert close.linear_velocity_mps == pytest.approx(-0.08)
    assert close.angular_velocity_rad_s == pytest.approx(0.0)


def test_controller_alignment_uses_right_minus_left():
    controller = DistanceController(_pid(0.8, limit=0.1), _pid(4.0, limit=0.5))
    command = controller.calculate(350, 400, 300, 10, 0.1)
    assert command.alignment_error_mm == pytest.approx(50.0)
    assert command.angular_velocity_rad_s == pytest.approx(0.2)


def test_both_sensors_must_be_inside_tolerance():
    controller = DistanceController(_pid(0.8), _pid(4.0))
    inside = controller.calculate(290, 310, 300, 10, 0.1)
    assert inside.inside_tolerance
    assert inside.linear_velocity_mps == 0.0
    assert inside.angular_velocity_rad_s == 0.0

    one_outside = controller.calculate(289, 311, 300, 10, 0.1)
    assert not one_outside.inside_tolerance
