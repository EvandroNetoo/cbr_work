from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import math

import cv2
from geometry_msgs.msg import Pose, TransformStamped
from interfaces.action import AnalyzeScene
from interfaces.msg import AprilTagStampedDetection, ContainerStampedDetection
import numpy as np
import pytest
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from tf2_ros import TransformException

try:
    import pupil_apriltags  # noqa: F401
except ModuleNotFoundError:
    pupil_apriltags = ModuleType('pupil_apriltags')
    pupil_apriltags.Detector = object
    sys.modules['pupil_apriltags'] = pupil_apriltags

from vision.scene_analyzer import (  # noqa: E402
    _capture_request_succeeded,
    BLUE,
    ContainerCandidate,
    RED,
    SceneAnalyzer,
    Session,
)
from vision.partial_container import fit_partial_container
import yaml


PACKAGE = Path(__file__).parents[1]


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _CompletedFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response


class _VisionLedClient:
    def __init__(self):
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request.data)
        return _CompletedFuture(SetBool.Response(success=True, message='ok'))


def _analyzer_for_container() -> SceneAnalyzer:
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.min_saturation = 80
    analyzer.min_value = 45
    analyzer.red_ranges = ((0, 12), (168, 179))
    analyzer.blue_range = (92, 138)
    analyzer.morphology_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3))
    analyzer.min_contour_area = 100.0
    analyzer.min_partial_contour_area = 50.0
    analyzer.container_border_margin_px = 6
    analyzer.max_contour_fraction = 0.85
    analyzer.min_rectangularity = 0.55
    analyzer.polygon_epsilon_fraction = 0.035
    analyzer.max_container_pose_error = 12.0
    analyzer.external_width = 0.102
    analyzer.external_depth = 0.173
    analyzer.external_height = 0.073
    return analyzer


def _tag_item(tag_id, error, margin, hamming=0, stamp=0):
    item = AprilTagStampedDetection()
    item.id = tag_id
    item.pose_error = error
    item.decision_margin = margin
    item.hamming = hamming
    item.header.stamp.nanosec = stamp
    return item


def _container_item(color, x, y=0.0, z=0.40, yaw=0.0, frame='camera'):
    item = ContainerStampedDetection()
    item.header.frame_id = frame
    item.color = color
    item.pose.position.x = x
    item.pose.position.y = y
    item.pose.position.z = z
    item.pose.orientation.z = np.sin(yaw / 2.0)
    item.pose.orientation.w = np.cos(yaw / 2.0)
    item.pose_error = 1.0
    item.rectangularity = 0.9
    item.contour_area_px = 7000.0
    item.external_width_m = 0.102
    item.external_depth_m = 0.173
    item.external_height_m = 0.073
    return item


def _tracking_analyzer():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.container_association_distance = 0.05
    analyzer.container_final_merge_distance = 0.07
    analyzer.min_container_observations = 3
    analyzer.max_container_position_deviation = 0.04
    analyzer.max_container_yaw_deviation = np.deg2rad(20.0)
    return analyzer


def test_temporal_container_result_merges_converging_tracks():
    analyzer = _tracking_analyzer()
    tracks = []
    # One bad depth estimate starts a second blue track. The final track
    # representatives are close, like the reported physical observation.
    for frame, x, z in (
        (0, 0.130, 0.355), (1, 0.130, 0.356),
        (2, 0.130, 0.357), (3, 0.137, 0.440),
        (4, 0.137, 0.392), (5, 0.137, 0.391),
        (6, 0.137, 0.393),
    ):
        yaw = 0.0 if frame < 3 else np.pi
        camera = _container_item(BLUE, x, z=z)
        base = _container_item(BLUE, x, z=z, yaw=yaw, frame='arm_base_link')
        analyzer._update_container_tracks(tracks, [camera], [base], frame)

    camera, base = analyzer._confirmed_container_results(tracks)

    assert len(camera) == len(base) == 1
    assert base[0].observation_count == 6
    assert base[0].position_spread_m < 0.04
    assert base[0].yaw_spread_deg == pytest.approx(0.0, abs=1e-5)
    assert base[0].pose.position.z == pytest.approx(0.374, abs=0.002)


def test_one_frame_contour_is_not_confirmed_and_two_bins_stay_separate():
    analyzer = _tracking_analyzer()
    tracks = []
    for frame in range(4):
        camera = [_container_item(BLUE, 0.0), _container_item(BLUE, 0.14)]
        base = [_container_item(BLUE, 0.0, frame='arm_base_link'),
                _container_item(BLUE, 0.14, frame='arm_base_link')]
        if frame == 0:
            camera.append(_container_item(RED, 0.35))
            base.append(_container_item(RED, 0.35, frame='arm_base_link'))
        analyzer._update_container_tracks(tracks, camera, base, frame)

    _camera, base = analyzer._confirmed_container_results(tracks)

    assert len(base) == 2
    assert all(item.color == BLUE and item.observation_count == 4
               for item in base)
    assert abs(base[0].pose.position.x - base[1].pose.position.x) > 0.1


def test_same_frame_blue_contours_are_not_counted_as_repeated_observations():
    analyzer = _tracking_analyzer()
    tracks = []
    analyzer._update_container_tracks(
        tracks, [_container_item(BLUE, 0.0), _container_item(BLUE, 0.03)],
        [_container_item(BLUE, 0.0, frame='arm_base_link'),
         _container_item(BLUE, 0.03, frame='arm_base_link')], 0)

    _camera, base = analyzer._confirmed_container_results(tracks)

    assert len(tracks) == 2
    assert base == []


def test_unstable_container_positions_are_rejected():
    analyzer = _tracking_analyzer()
    analyzer.container_association_distance = 0.1
    tracks = []
    for frame, z in enumerate((0.35, 0.399, 0.448)):
        item = _container_item(RED, 0.0, z=z)
        base = _container_item(RED, 0.0, z=z, frame='arm_base_link')
        analyzer._update_container_tracks(tracks, [item], [base], frame)

    _camera, confirmed = analyzer._confirmed_container_results(tracks)

    assert confirmed == []


def test_unstable_base_yaw_does_not_become_a_placement_pose():
    analyzer = _tracking_analyzer()
    tracks = []
    for frame, yaw in enumerate((0.0, np.pi / 3.0, 2.0 * np.pi / 3.0)):
        camera = _container_item(RED, 0.0)
        base = _container_item(RED, 0.0, yaw=yaw, frame='arm_base_link')
        analyzer._update_container_tracks(tracks, [camera], [base], frame)

    camera, base = analyzer._confirmed_container_results(tracks)

    assert len(camera) == 1
    assert base == []


def test_one_public_action_owns_one_camera_session():
    source = (PACKAGE / 'vision' / 'scene_analyzer.py').read_text()
    assert "'vision/analyze_scene'" in source
    assert 'AnalyzeScene' in source
    assert 'AnalyzeAprilTags' not in source
    assert 'AnalyzeContainers' not in source
    assert 'self._create_inputs_locked()' in source
    assert 'self._destroy_inputs_locked()' in source
    assert 'self._set_vision_led(True)' in source
    assert 'self._wait_for_camera_capture()' in source
    assert source.count('self.apriltag_detector.detect(') == 1
    assert 'container_color_masks(bgr)' in source


def test_goal_requires_a_known_nonempty_detector_mask():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.sessions_lock = threading.RLock()
    analyzer.state = 'idle'
    analyzer.get_logger = lambda: _Logger()
    analyzer._create_inputs_locked = lambda: None

    def request(mask):
        return SimpleNamespace(
            requested_detectors=mask,
            duration=SimpleNamespace(sec=1, nanosec=0),
            work_surface_height_m=0.125)

    assert analyzer.goal_callback(request(0)).name == 'REJECT'
    assert analyzer.goal_callback(request(4)).name == 'REJECT'
    assert analyzer.goal_callback(request(AnalyzeScene.Goal.APRILTAGS)).name == 'ACCEPT'
    assert analyzer.state == 'activating'
    assert analyzer.goal_callback(request(AnalyzeScene.Goal.CONTAINERS)).name == 'REJECT'


def test_profile_combines_apriltag_and_measured_bin3_parameters():
    parameters = yaml.safe_load(
        (PACKAGE / 'config' / 'vision.yaml').read_text()
    )['scene_analyzer']['ros__parameters']
    assert parameters['tag_size_m'] == 0.032
    assert parameters['external_height_m'] == 0.073
    assert parameters['external_width_m'] == 0.102
    assert parameters['external_depth_m'] == 0.173
    assert parameters['internal_height_m'] == 0.057
    assert parameters['internal_width_m'] == 0.090
    assert parameters['internal_depth_m'] == 0.140
    assert parameters['manage_camera_capture'] is True
    assert parameters['manage_vision_led'] is True
    assert parameters['min_contour_area_px'] == 4000.0
    assert parameters['container_border_margin_px'] == 6
    assert parameters['container_warmup_sec'] == 0.5
    assert parameters['min_container_observations'] == 3


def test_color_masks_separate_red_and_blue_regions():
    analyzer = _analyzer_for_container()
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 20), (80, 80), (0, 0, 255), -1)
    cv2.rectangle(image, (120, 20), (190, 80), (255, 0, 0), -1)

    masks = analyzer.container_color_masks(image)

    assert masks[RED][50, 40] == 255
    assert masks[RED][50, 150] == 0
    assert masks[BLUE][50, 150] == 255
    assert masks[BLUE][50, 40] == 0


def test_border_container_reaches_partial_pose_fitting_stage():
    analyzer = _analyzer_for_container()
    analyzer.min_contour_area = 20000.0
    analyzer.min_partial_contour_area = 800.0
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (225, 65), (319, 175), (0, 0, 255), -1)
    masks = analyzer.container_color_masks(image)
    camera = np.array([[400.0, 0.0, 160.0],
                       [0.0, 400.0, 120.0], [0.0, 0.0, 1.0]])

    candidates = analyzer.detect_container_candidates(
        masks, image.shape[:2], camera)

    assert any(item.color == RED and item.reason == 'border'
               for item in candidates)


def test_border_margin_prevents_frame_to_frame_edge_classification_jitter():
    analyzer = _analyzer_for_container()
    shape = (240, 320)
    near_edge = np.array([
        [4.0, 50.0], [80.0, 50.0], [80.0, 130.0], [4.0, 130.0],
    ])
    clear = near_edge.copy()
    clear[:, 0] += 4.0

    assert analyzer._container_touches_border(near_edge, shape)
    assert not analyzer._container_touches_border(clear, shape)


def test_known_projected_container_recovers_center_and_dimensions():
    analyzer = _analyzer_for_container()
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

    pose, error = analyzer.estimate_container_pose(
        corners.reshape(4, 2), camera)

    assert pose is not None
    assert error < 1e-5
    assert pose.position.x == pytest.approx(0.025, abs=1e-5)
    assert pose.position.y == pytest.approx(-0.015, abs=1e-5)
    assert pose.position.z == pytest.approx(0.55, abs=1e-5)
    candidate = ContainerCandidate(
        RED, np.empty((0, 1, 2)), corners.reshape(4, 2), 1000.0, 1.0,
        True, 'ok', Pose(), error)
    item = analyzer.container_to_stamped(candidate, Image().header)
    assert item.external_width_m == pytest.approx(0.102)
    assert item.external_depth_m == pytest.approx(0.173)
    assert item.external_height_m == pytest.approx(0.073)
    item.partial = True
    item.position_uncertainty_m = 0.025
    item.partial_fit_overlap = 0.8
    copied = analyzer.copy_container_stamped(item)
    assert copied.external_height_m == pytest.approx(0.073)
    assert copied.partial
    published = analyzer.container_detection_array('arm_base_link', Image(), [copied])
    assert published.detections[0].external_height_m == pytest.approx(0.073)
    assert published.detections[0].position_uncertainty_m == pytest.approx(0.025)
    assert published.detections[0].partial_fit_overlap == pytest.approx(0.8)


def test_clipped_container_fit_recovers_center_outside_visible_silhouette():
    camera = np.array([[400.0, 0.0, 160.0],
                       [0.0, 400.0, 120.0], [0.0, 0.0, 1.0]])
    # Camera is 30 cm above the container top and points vertically down.
    transform = SimpleNamespace(transform=SimpleNamespace(
        rotation=SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0),
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.5),
    ))
    x, y, yaw = 0.15, 0.01, 0.35
    depth, width = 0.173, 0.102
    direction = np.array([math.cos(yaw), math.sin(yaw)])
    side = np.array([-direction[1], direction[0]])
    center = np.array([x, y])
    corners = np.array([
        center + along * direction + across * side
        for along, across in ((-depth/2, -width/2),
                              (depth/2, -width/2),
                              (depth/2, width/2),
                              (-depth/2, width/2))
    ])
    projected = np.column_stack((
        160.0 + 400.0 * corners[:, 0] / 0.3,
        120.0 - 400.0 * corners[:, 1] / 0.3,
    )).astype(np.float32)
    viewport = np.array([[0., 0.], [319., 0.],
                         [319., 239.], [0., 239.]], dtype=np.float32)
    _area, clipped = cv2.intersectConvexConvex(projected, viewport)

    fit = fit_partial_container(
        clipped.reshape(-1, 1, 2), (240, 320), camera,
        transform, 0.2, depth, width,
    )

    assert fit is not None
    assert fit.overlap > 0.9
    assert fit.x == pytest.approx(x, abs=0.01)
    assert fit.y == pytest.approx(y, abs=0.01)
    assert abs((fit.yaw - yaw + math.pi/2) % math.pi - math.pi/2) < 0.1
    assert fit.position_uncertainty_m > 0.02


def test_partial_container_track_is_returned_when_no_full_view_exists():
    analyzer = _tracking_analyzer()
    tracks = []
    for frame in range(3):
        camera = _container_item(RED, 0.1 + frame * 0.001)
        base = _container_item(RED, 0.1 + frame * 0.001,
                               frame='arm_base_link')
        for item in (camera, base):
            item.partial = True
            item.position_uncertainty_m = 0.025
        analyzer._update_container_tracks(tracks, [camera], [base], frame)

    _camera, confirmed = analyzer._confirmed_container_results(tracks)

    assert len(confirmed) == 1
    assert confirmed[0].partial
    assert confirmed[0].position_uncertainty_m == pytest.approx(0.025)


def test_image_callback_publishes_and_confirms_border_container():
    analyzer = _analyzer_for_container()
    analyzer.base_frame = 'arm_base_link'
    analyzer.sessions_lock = threading.RLock()
    analyzer.session = Session(
        goal_handle=SimpleNamespace(is_cancel_requested=False),
        duration=2.0, requested_detectors=AnalyzeScene.Goal.CONTAINERS,
        work_surface_height_m=0.127,
    )
    analyzer.session.containers_ready_at = 0.0
    analyzer.last_detection_time = float('-inf')
    analyzer.detection_period = 0.0
    analyzer.publish_debug_image = False
    analyzer.container_association_distance = 0.07
    analyzer.container_final_merge_distance = 0.07
    analyzer.min_container_observations = 3
    analyzer.max_container_position_deviation = 0.04
    analyzer.max_container_yaw_deviation = math.radians(20.0)
    info = SimpleNamespace(
        p=[400.0, 0.0, 160.0, 0.0, 0.0, 400.0, 120.0, 0.0,
           0.0, 0.0, 1.0, 0.0],
        header=SimpleNamespace(frame_id='camera'),
    )
    analyzer.camera_info = info
    transform = TransformStamped()
    transform.header.frame_id = 'arm_base_link'
    transform.child_frame_id = 'camera'
    transform.transform.translation.z = 0.5
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    transform_calls = []

    def intermittent_transform(*_args, **_kwargs):
        transform_calls.append(1)
        if len(transform_calls) == 1:
            return transform
        raise TransformException('synthetic timestamp gap')

    analyzer.tf_buffer = SimpleNamespace(
        lookup_transform=intermittent_transform)
    base_outputs = []
    analyzer.container_detection_publisher = SimpleNamespace(
        publish=base_outputs.append)
    analyzer.container_camera_detection_publisher = SimpleNamespace(
        publish=lambda *_args: None)
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    x, y, yaw = 0.15, 0.01, 0.35
    direction = np.array([math.cos(yaw), math.sin(yaw)])
    side = np.array([-direction[1], direction[0]])
    corners = np.array([
        np.array([x, y]) + along * direction + across * side
        for along, across in ((-0.173/2, -0.102/2),
                              (0.173/2, -0.102/2),
                              (0.173/2, 0.102/2),
                              (-0.173/2, 0.102/2))
    ])
    pixels = np.column_stack((
        160.0 + 400.0 * corners[:, 0] / 0.3,
        120.0 - 400.0 * corners[:, 1] / 0.3,
    )).astype(np.int32)
    cv2.fillConvexPoly(image, pixels, (0, 0, 255))
    message = Image()
    message.header.frame_id = 'camera'
    message.height, message.width = image.shape[:2]
    message.encoding = 'bgr8'
    message.step = message.width * 3
    message.data = image.tobytes()

    for frame in range(3):
        message.header.stamp.nanosec = frame + 1
        analyzer.image_callback(message)
    _camera, base = analyzer._confirmed_container_results(
        analyzer.session.container_tracks)

    assert len(base_outputs) == 3
    assert len(transform_calls) == 5
    assert base_outputs[-1].detections[0].partial
    assert base_outputs[-1].detections[0].partial_fit_overlap > 0.35
    assert len(base) == 1
    assert base[0].partial
    assert base[0].pose.position.x == pytest.approx(x, abs=0.02)


def test_debug_images_are_algorithm_specific():
    source = (PACKAGE / 'vision' / 'scene_analyzer.py').read_text()
    assert "'apriltags/debug_image'" in source
    assert "'containers/debug_image'" in source
    assert 'exterior-only MVP' in source


def test_best_apriltag_order_is_preserved():
    best = {}
    SceneAnalyzer._update_best(best, _tag_item(4, 2.0, 80.0, 0, 10))
    SceneAnalyzer._update_best(best, _tag_item(4, 1.0, 20.0, 1, 20))
    assert best[4].pose_error == 1.0
    SceneAnalyzer._update_best(best, _tag_item(4, 1.0, 30.0, 0, 30))
    assert best[4].decision_margin == 30.0


def test_usb_camera_and_vision_led_lifecycle_helpers_are_preserved():
    started = SimpleNamespace(success=False, message='Start Capturing')
    stopped = SimpleNamespace(success=False, message='Stop Capturing')
    assert _capture_request_succeeded(started, True)
    assert _capture_request_succeeded(stopped, False)

    analyzer = object.__new__(SceneAnalyzer)
    analyzer.manage_vision_led = True
    analyzer.vision_led_timeout = 0.1
    analyzer.vision_led_service = '/base_hardware/set_vision_led'
    analyzer.vision_led_client = _VisionLedClient()
    analyzer.get_logger = lambda: _Logger()
    assert analyzer._set_vision_led(True)
    assert analyzer._set_vision_led(False)
    assert analyzer.vision_led_client.requests == [True, False]


def test_analyze_scene_interface_contains_both_modalities():
    action = (
        PACKAGE.parent / 'interfaces' / 'action' / 'AnalyzeScene.action'
    ).read_text()
    assert 'uint8 APRILTAGS=1' in action
    assert 'uint8 CONTAINERS=2' in action
    assert 'best_apriltags_base' in action
    assert 'best_containers_base' in action
    assert 'bool continuous' in action
