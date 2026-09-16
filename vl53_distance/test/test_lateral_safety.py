import math

import pytest

from vl53_distance.lateral_safety import (
    lateral_clearances_from_scan,
    PlanarTransform,
)


def _scan(transform):
    ranges = [math.nan] * 360
    ranges[89:92] = [0.255, 0.255, 0.255]
    ranges[269:272] = [0.355, 0.355, 0.355]
    return lateral_clearances_from_scan(
        ranges,
        angle_min_rad=0.0,
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=transform,
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
    )


def test_scan_clearance_is_measured_from_footprint_edges():
    identity = PlanarTransform.from_quaternion(
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    result = _scan(identity)

    assert result.left_mm == pytest.approx(100.0, abs=0.1)
    assert result.right_mm == pytest.approx(200.0, abs=0.1)


def test_transform_accounts_for_inverted_lidar():
    roll_pi = PlanarTransform.from_quaternion(
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    result = _scan(roll_pi)

    assert result.left_mm == pytest.approx(200.0, abs=0.1)
    assert result.right_mm == pytest.approx(100.0, abs=0.1)


def test_isolated_ray_is_rejected():
    ranges = [math.nan] * 360
    ranges[90] = 0.20
    identity = PlanarTransform.from_quaternion(
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    result = lateral_clearances_from_scan(
        ranges,
        angle_min_rad=0.0,
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=identity,
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
    )

    assert result.left_mm is None
    assert result.right_mm is None


def test_points_inside_footprint_are_reported_as_zero_clearance():
    ranges = [math.nan] * 360
    ranges[89:92] = [0.12, 0.12, 0.12]
    identity = PlanarTransform.from_quaternion(
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    result = lateral_clearances_from_scan(
        ranges,
        angle_min_rad=0.0,
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=identity,
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
    )

    assert result.left_mm == 0.0
    assert result.right_mm is None


def test_close_front_wall_is_not_classified_as_lateral_obstacle():
    ranges = [math.nan] * 89
    angle_min = math.radians(-44.0)
    for index in range(len(ranges)):
        angle = angle_min + math.radians(index)
        # Parede plana 16 mm adiante da borda frontal de 119 mm.
        ranges[index] = 0.135 / math.cos(angle)
    identity = PlanarTransform.from_quaternion(
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    result = lateral_clearances_from_scan(
        ranges,
        angle_min_rad=angle_min,
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=identity,
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
    )

    assert result.left_mm is None
    assert result.right_mm is None
