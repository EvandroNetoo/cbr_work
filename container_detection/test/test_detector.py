from pathlib import Path
from types import SimpleNamespace

from container_detection.container_detector import (
    BLUE,
    Candidate,
    ContainerDetector,
    RED,
)
import cv2
from geometry_msgs.msg import Pose
from interfaces.msg import ContainerStampedDetection
import numpy as np
import pytest
from sensor_msgs.msg import Image
import yaml


PACKAGE = Path(__file__).parents[1]


def _detector_for_vision() -> ContainerDetector:
    detector = object.__new__(ContainerDetector)
    detector.min_saturation = 80
    detector.min_value = 45
    detector.red_ranges = ((0, 12), (168, 179))
    detector.blue_range = (92, 138)
    detector.morphology_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3))
    detector.min_contour_area = 100.0
    detector.max_contour_fraction = 0.85
    detector.min_rectangularity = 0.55
    detector.polygon_epsilon_fraction = 0.035
    detector.max_pose_error = 12.0
    detector.external_width = 0.102
    detector.external_depth = 0.173
    return detector


def test_profile_records_measured_external_and_internal_dimensions():
    parameters = yaml.safe_load(
        (PACKAGE / 'config' / 'container_detection.yaml').read_text()
    )['container_detector']['ros__parameters']
    assert parameters['external_height_m'] == 0.073
    assert parameters['external_width_m'] == 0.102
    assert parameters['external_depth_m'] == 0.173
    assert parameters['internal_height_m'] == 0.057
    assert parameters['internal_width_m'] == 0.090
    assert parameters['internal_depth_m'] == 0.140


def test_color_masks_separate_red_and_blue_regions():
    detector = _detector_for_vision()
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 20), (80, 80), (0, 0, 255), -1)
    cv2.rectangle(image, (120, 20), (190, 80), (255, 0, 0), -1)

    masks = detector.color_masks(image)

    assert masks[RED][50, 40] == 255
    assert masks[RED][50, 150] == 0
    assert masks[BLUE][50, 150] == 255
    assert masks[BLUE][50, 40] == 0


def test_known_projected_rectangle_recovers_center_and_small_error():
    detector = _detector_for_vision()
    camera = np.array([
        [300.0, 0.0, 160.0],
        [0.0, 300.0, 120.0],
        [0.0, 0.0, 1.0],
    ])
    object_points = np.array([
        [-0.173 / 2.0, -0.102 / 2.0, 0.0],
        [0.173 / 2.0, -0.102 / 2.0, 0.0],
        [0.173 / 2.0, 0.102 / 2.0, 0.0],
        [-0.173 / 2.0, 0.102 / 2.0, 0.0],
    ])
    rotation = np.array([[0.12], [-0.08], [0.2]])
    translation = np.array([[0.025], [-0.015], [0.55]])
    corners, _ = cv2.projectPoints(
        object_points, rotation, translation, camera, np.zeros((4, 1)))

    pose, error = detector.estimate_pose(corners.reshape(4, 2), camera)

    assert pose is not None
    assert error < 1e-5
    assert pose.position.x == pytest.approx(0.025, abs=1e-5)
    assert pose.position.y == pytest.approx(-0.015, abs=1e-5)
    assert pose.position.z == pytest.approx(0.55, abs=1e-5)


def test_debug_image_contains_masks_geometry_and_mvp_notice():
    detector = _detector_for_vision()
    published = []
    detector.debug_image_publisher = SimpleNamespace(publish=published.append)
    source = Image()
    source.header.frame_id = 'camera_optical_frame'
    image = np.zeros((100, 160, 3), dtype=np.uint8)
    masks = {RED: np.zeros((100, 160), np.uint8), BLUE: np.zeros((100, 160), np.uint8)}
    masks[RED][30:70, 40:120] = 255
    contour = np.array([[[40, 30]], [[120, 30]], [[120, 70]], [[40, 70]]])
    candidate = Candidate(
        color=RED,
        contour=contour,
        corners=contour.reshape(4, 2).astype(float),
        area=3200.0,
        rectangularity=1.0,
        accepted=True,
        reason='ok',
        pose=Pose(),
        pose_error=0.5,
    )

    detector.publish_debug(source, image, masks, [candidate])

    assert len(published) == 1
    output = published[0]
    assert output.encoding == 'bgr8'
    assert (output.height, output.width, output.step) == (100, 160, 480)
    pixels = np.frombuffer(output.data, dtype=np.uint8).reshape(100, 160, 3)
    assert np.any(pixels[:, :, 1] == 220)


def test_best_sample_prefers_pose_error_then_shape_then_area():
    def item(error, rectangularity, area):
        output = ContainerStampedDetection()
        output.color = RED
        output.pose_error = error
        output.rectangularity = rectangularity
        output.contour_area_px = area
        return output

    best = {}
    ContainerDetector._update_best(best, item(2.0, 0.9, 1000.0))
    ContainerDetector._update_best(best, item(1.0, 0.6, 500.0))
    assert best[RED].pose_error == 1.0
    ContainerDetector._update_best(best, item(1.0, 0.8, 400.0))
    assert best[RED].rectangularity == 0.8
    ContainerDetector._update_best(best, item(1.0, 0.8, 900.0))
    assert best[RED].contour_area_px == 900.0


def test_interface_and_topics_are_explicit():
    action = (
        PACKAGE.parent / 'interfaces' / 'action' / 'AnalyzeContainers.action'
    ).read_text()
    source = (
        PACKAGE / 'container_detection' / 'container_detector.py'
    ).read_text()
    assert 'best_detections_camera' in action
    assert 'best_detections_base' in action
    assert "'containers/analyze'" in source
    assert "'containers/debug_image'" in source
    assert "'containers/detections_camera'" in source
    assert "'containers/detections'" in source
    assert 'exterior-only MVP' in source
