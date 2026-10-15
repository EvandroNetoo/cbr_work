import math

import pytest

from vl53_distance.lateral_safety import (
    lateral_clearances_from_scan,
    longitudinal_corridor_bounds,
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


@pytest.mark.parametrize('inverted', [False, True])
def test_rear_band_rejects_front_points_and_keeps_both_sides(inverted):
    ranges = [math.nan] * 360
    ranges[60:63] = [0.20] * 3
    ranges[298:301] = [0.20] * 3
    ranges[110:113] = [0.30] * 3
    ranges[248:251] = [0.30] * 3
    transform = PlanarTransform.from_quaternion(
        0.0, 0.0, float(inverted), 0.0, 0.0, float(not inverted))
    geometry = dict(
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

    full = lateral_clearances_from_scan(ranges, **geometry)
    rear = lateral_clearances_from_scan(
        ranges, **geometry,
        longitudinal_min_x_m=-0.119,
        longitudinal_max_x_m=-0.020,
    )

    assert full.left_mm < 30.0
    assert full.right_mm < 30.0
    expected = (0.30 * math.sin(math.radians(112)) - 0.155) * 1000.0
    assert rear.left_mm == pytest.approx(expected)
    assert rear.right_mm == pytest.approx(expected)


@pytest.mark.parametrize('base_x', [-0.119, -0.020, 0.050])
def test_longitudinal_band_uses_base_frame_and_inclusive_limits(base_x):
    # The first ray lies exactly on the boundary; the next lies inside it.
    # A 90 degree yaw makes scan +X point toward the left of the base.
    transform = PlanarTransform(base_x, 0.0, 0.0, -1.0, 1.0, 0.0)
    result = lateral_clearances_from_scan(
        [0.255, 0.255],
        angle_min_rad=0.0,
        angle_increment_rad=math.radians(-1 if base_x == -0.119 else 1),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=transform,
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
        longitudinal_min_x_m=-0.119,
        longitudinal_max_x_m=-0.020,
    )

    if base_x > -0.020:
        assert result.left_mm is None
    else:
        assert result.left_mm == pytest.approx(100.0, abs=0.1)
    assert result.right_mm is None


@pytest.mark.parametrize('minimum, maximum', [
    (0.1, -0.1),
    (-0.02, -0.02),
    (math.nan, 0.0),
    (-0.119, math.inf),
    (-0.3, -0.2),
    (0.2, 0.3),
])
def test_invalid_or_empty_longitudinal_band_is_rejected(minimum, maximum):
    with pytest.raises(ValueError):
        longitudinal_corridor_bounds(0.119, 0.0, minimum, maximum)


def test_longitudinal_band_is_clipped_to_footprint_and_margin():
    assert longitudinal_corridor_bounds(
        0.119, 0.01, -1.0, 1.0) == pytest.approx((-0.129, 0.129))


def test_rear_band_excludes_tilted_front_wall_segment():
    # Front wall x + 0.4*y = 0.135 intersects the front half of the corridor.
    ranges = [
        0.135 / (math.cos(math.radians(angle))
                 + 0.4 * math.sin(math.radians(angle)))
        for angle in range(70, 81)
    ]
    geometry = dict(
        angle_min_rad=math.radians(70),
        angle_increment_rad=math.radians(1),
        range_min_m=0.10,
        range_max_m=3.0,
        transform=PlanarTransform.from_quaternion(
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        footprint_half_length_m=0.119,
        footprint_half_width_m=0.155,
        longitudinal_margin_m=0.0,
        minimum_consecutive_points=2,
    )

    full = lateral_clearances_from_scan(ranges, **geometry)
    rear = lateral_clearances_from_scan(
        ranges, **geometry,
        longitudinal_min_x_m=-0.119,
        longitudinal_max_x_m=-0.020,
    )

    assert full.left_mm is not None
    assert rear.left_mm is None
    assert rear.right_mm is None


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
