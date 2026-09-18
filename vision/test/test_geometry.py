from pathlib import Path

from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import pytest

from vision.geometry import (
    hsv_color_masks,
    load_geometry_profile,
    opening_evidence_score,
    project_open_container,
    rectangular_model_error,
)


ROOT = Path(__file__).parents[1]
GEOMETRY = load_geometry_profile(
    str(ROOT / 'config' / 'container_geometry_profiles.yaml'),
    'current_team_model',
)
K = np.array([[232.5257, 0.0, 160.5], [0.0, 235.77464, 120.5], [0, 0, 1]])


def downward_camera(height=0.50):
    transform = TransformStamped()
    transform.transform.translation.z = height
    # 180 degrees about X: optical +Z points toward the table.
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    return transform


def test_opencv_hsv_masks_keep_red_ranges_and_blue_separate():
    hsv = np.zeros((24, 72, 3), dtype=np.uint8)
    hsv[:, 0:12] = (6, 200, 150)
    hsv[:, 18:30] = (165, 200, 150)
    hsv[:, 36:48] = (100, 200, 150)
    hsv[:, 54:66] = (30, 200, 150)
    red, blue = hsv_color_masks(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
    assert red[:, 4:8].all() and red[:, 22:26].all()
    assert not red[:, 40:44].any() and not red[:, 58:62].any()
    assert blue[:, 40:44].all() and not blue[:, 4:8].any()


def test_cube_hypothesis_fits_42_mm_blob_better_than_container():
    measured = (0.043, 0.041)
    cube = rectangular_model_error(measured, (0.042, 0.042))
    container = rectangular_model_error(measured, (GEOMETRY.depth, GEOMETRY.width))
    assert cube < container
    assert rectangular_model_error(
        (0.171, 0.104), (GEOMETRY.depth, GEOMETRY.width)) < cube


@pytest.mark.parametrize('table_height', [0.05, 0.10, 0.15])
def test_projected_area_is_computed_independently_for_each_table(table_height):
    projection = project_open_container(
        (0.0, 0.0), 0.3, table_height, GEOMETRY, K,
        downward_camera(), (240, 320), 0.85, 0.55)
    assert projection is not None
    assert projection.expected_segmentable_area > 1.0
    assert 0.0 < projection.visible_fraction <= 1.0


@pytest.mark.parametrize('center', [(-0.24, 0), (0.24, 0), (0, -0.18), (0, 0.18)])
def test_projection_clips_each_image_border(center):
    projection = project_open_container(
        center, 0.0, 0.10, GEOMETRY, K,
        downward_camera(), (240, 320), 0.85, 0.55)
    assert projection is not None
    assert 0.0 < projection.visible_fraction < 1.0


def test_empty_one_and_two_cube_openings_remain_explainable():
    area = 10000.0
    cube = area * 0.042**2 / (GEOMETRY.inner_depth * GEOMETRY.inner_width)
    assert opening_evidence_score(
        area, 0, 0.042, GEOMETRY.inner_depth, GEOMETRY.inner_width, 2) == 1.0
    assert opening_evidence_score(
        area, cube, 0.042, GEOMETRY.inner_depth, GEOMETRY.inner_width, 2) == 1.0
    assert opening_evidence_score(
        area, 2*cube, 0.042, GEOMETRY.inner_depth, GEOMETRY.inner_width, 2) == 1.0
    assert opening_evidence_score(
        area, area, 0.042, GEOMETRY.inner_depth, GEOMETRY.inner_width, 0) == 0.0


def test_profiles_are_distinct_physical_objects():
    official = load_geometry_profile(
        str(ROOT / 'config' / 'container_geometry_profiles.yaml'),
        'robocup_2026_type_28')
    assert (GEOMETRY.depth, GEOMETRY.width, GEOMETRY.height) == (0.173, 0.102, 0.073)
    assert (official.depth, official.width, official.height) == (0.160, 0.135, 0.082)
    assert GEOMETRY.physical_object != official.physical_object
