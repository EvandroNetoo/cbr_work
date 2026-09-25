import math
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import cv2
from geometry_msgs.msg import Pose, TransformStamped
from interfaces.action import AnalyzeScene
from interfaces.msg import (
    AprilTagStampedDetection, ContainerStampedDetection, TableSurfaceGrid,
)
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

from vision.constants import BLUE, RED
from vision.scene_analyzer import (  # noqa: E402
    _capture_request_succeeded,
    ContainerDebugFrame,
    SceneAnalyzer,
    Session,
)
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
    analyzer.container_border_margin_px = 6
    analyzer.max_contour_fraction = 0.85
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
    item.mask_area_px = 7000.0
    return item














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


def test_scene_is_closed_before_vision_led_is_turned_off():
    source = (PACKAGE / 'vision' / 'scene_analyzer.py').read_text()
    teardown = source.split('        finally:', 1)[1].split(
        '    def _feedback', 1)[0]

    assert teardown.index('self.session = None') < teardown.index(
        'self._set_vision_led(False)')


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
            work_surface_height_m=0.125,
            table_search_x_min_m=-0.1,
            table_search_x_max_m=0.1,
            table_search_y_min_m=-0.3,
            table_search_y_max_m=-0.1,
            table_grid_resolution_m=0.01,
        )

    assert analyzer.goal_callback(request(0)).name == 'REJECT'
    assert analyzer.goal_callback(request(16)).name == 'REJECT'
    assert analyzer.goal_callback(request(AnalyzeScene.Goal.APRILTAGS)).name == 'ACCEPT'
    assert analyzer.state == 'activating'
    assert analyzer.goal_callback(request(AnalyzeScene.Goal.CONTAINERS_HSV)).name == 'REJECT'
    analyzer.state = 'idle'
    assert analyzer.goal_callback(request(8)).name == 'ACCEPT'


def test_goal_accepts_geometry_independent_white_table_grid():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.sessions_lock = threading.RLock()
    analyzer.state = 'idle'
    analyzer.get_logger = lambda: _Logger()
    analyzer._create_inputs_locked = lambda: None
    request = SimpleNamespace(
        requested_detectors=AnalyzeScene.Goal.TABLE_SURFACE,
        duration=SimpleNamespace(sec=1, nanosec=0),
        work_surface_height_m=0.125,
        table_search_x_min_m=-0.27,
        table_search_x_max_m=0.27,
        table_search_y_min_m=-0.35,
        table_search_y_max_m=-0.03,
        table_grid_resolution_m=0.01,
    )

    assert analyzer.goal_callback(request).name == 'ACCEPT'


def test_profile_combines_apriltag_and_measured_bin3_parameters():
    parameters = yaml.safe_load(
        (PACKAGE / 'config' / 'vision.yaml').read_text()
    )['scene_analyzer']['ros__parameters']
    assert parameters['tag_size_m'] == 0.032
    assert parameters['external_height_m'] == 0.073
    assert parameters['manage_camera_capture'] is True
    assert parameters['manage_vision_led'] is True
    assert parameters['container_border_margin_px'] == 6
    assert parameters['container_warmup_sec'] == 2.0
    assert parameters['nthreads'] == 3
    assert parameters['opencv_threads'] == 2
    assert parameters['apriltag_detection_rate_hz'] == 15.0
    assert parameters['hsv_container_detection_rate_hz'] == 12.0
    assert parameters['table_surface_detection_rate_hz'] == 8.0


def test_detector_rates_are_independent_and_tags_do_not_wait_for_warmup():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.detector_periods = {
        AnalyzeScene.Goal.APRILTAGS: 0.05,
        AnalyzeScene.Goal.CONTAINERS_HSV: 0.10,
        AnalyzeScene.Goal.TABLE_SURFACE: 0.20,
    }
    session = Session(
        goal_handle=SimpleNamespace(is_cancel_requested=False),
        duration=1.0,
        requested_detectors=(
            AnalyzeScene.Goal.APRILTAGS
            | AnalyzeScene.Goal.CONTAINERS_HSV
            | AnalyzeScene.Goal.TABLE_SURFACE),
    )
    session.containers_ready_at = 10.0

    assert analyzer._due_detectors(session, 9.0) == AnalyzeScene.Goal.APRILTAGS

    session.last_detector_times[AnalyzeScene.Goal.APRILTAGS] = 9.98
    due = analyzer._due_detectors(session, 10.0)
    assert not due & AnalyzeScene.Goal.APRILTAGS
    assert due & AnalyzeScene.Goal.CONTAINERS_HSV
    assert due & AnalyzeScene.Goal.TABLE_SURFACE


def test_apriltag_worker_does_not_wait_for_slow_container_worker():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.image_condition = threading.Condition()
    analyzer.pending_images = {
        AnalyzeScene.Goal.APRILTAGS: None,
        AnalyzeScene.Goal.CONTAINERS_HSV: None,
    }
    analyzer.image_worker_stopping = False
    container_started = threading.Event()
    release_container = threading.Event()
    apriltag_finished = threading.Event()

    def process(_message, detector):
        if detector == AnalyzeScene.Goal.CONTAINERS_HSV:
            container_started.set()
            release_container.wait(timeout=1.0)
        else:
            apriltag_finished.set()

    analyzer._process_image = process
    workers = [
        threading.Thread(target=analyzer._image_worker_loop, args=(detector,))
        for detector in analyzer.pending_images
    ]
    for worker in workers:
        worker.start()
    try:
        with analyzer.image_condition:
            for detector in analyzer.pending_images:
                analyzer.pending_images[detector] = Image()
            analyzer.image_condition.notify_all()

        assert container_started.wait(timeout=0.5)
        assert apriltag_finished.wait(timeout=0.5)
        assert not release_container.is_set()
    finally:
        release_container.set()
        with analyzer.image_condition:
            analyzer.image_worker_stopping = True
            analyzer.image_condition.notify_all()
        for worker in workers:
            worker.join(timeout=1.0)


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


















def test_container_debug_overlay_marks_only_cleaned_mask_pixels():
    from vision.debug_images import overlay_container_masks

    image = np.full((80, 80, 3), 30, dtype=np.uint8)
    blue_mask = np.zeros((80, 80), dtype=np.uint8)
    blue_mask[25:55, 25:55] = 255
    result = overlay_container_masks(image, {BLUE: blue_mask})

    assert np.array_equal(result[10, 10], image[10, 10])
    assert result[40, 40, 0] > image[40, 40, 0]
    assert np.array_equal(result[25, 40], (255, 255, 255))
    assert np.array_equal(image[40, 40], (30, 30, 30))


def test_debug_images_are_algorithm_specific():
    source = (PACKAGE / 'vision' / 'scene_analyzer.py').read_text()
    assert "'apriltags/debug_image'" in source
    assert "'containers/debug_image'" in source
    assert 'publish_final_debug_images' in source
    assert 'TRANSIENT_LOCAL' in source


def test_observation_fps_uses_recent_completed_frames():
    session = Session(
        goal_handle=SimpleNamespace(), duration=2.0,
        requested_detectors=AnalyzeScene.Goal.CONTAINERS_HSV,
    )
    session.recent_frame_times = [10.0, 10.25, 10.5]
    session.recent_detector_times[AnalyzeScene.Goal.CONTAINERS_HSV] = [
        10.0, 10.2, 10.4]

    assert SceneAnalyzer._observation_fps(session) == pytest.approx(4.0)
    assert SceneAnalyzer._detector_observation_fps(
        session, AnalyzeScene.Goal.CONTAINERS_HSV) == pytest.approx(5.0)


def test_final_summaries_use_each_detector_own_fps_frames_and_tf():
    session = Session(
        goal_handle=SimpleNamespace(), duration=2.0,
        requested_detectors=(
            AnalyzeScene.Goal.APRILTAGS | AnalyzeScene.Goal.CONTAINERS_HSV),
    )
    session.detector_frame_counts = {
        AnalyzeScene.Goal.APRILTAGS: 12,
        AnalyzeScene.Goal.CONTAINERS_HSV: 3,
    }
    session.detector_frames_with_base_transform = {
        AnalyzeScene.Goal.APRILTAGS: 10,
        AnalyzeScene.Goal.CONTAINERS_HSV: 2,
    }
    session.detector_first_frame_times = {
        AnalyzeScene.Goal.APRILTAGS: 10.0,
        AnalyzeScene.Goal.CONTAINERS_HSV: 10.0,
    }
    session.detector_last_frame_times = {
        AnalyzeScene.Goal.APRILTAGS: 11.0,
        AnalyzeScene.Goal.CONTAINERS_HSV: 11.0,
    }

    assert SceneAnalyzer._detector_summary(
        session, AnalyzeScene.Goal.APRILTAGS, 'FINAL'
    ) == 'FINAL 11.0fps F12 TF10/12'
    assert SceneAnalyzer._detector_summary(
        session, AnalyzeScene.Goal.CONTAINERS_HSV, 'FINAL'
    ) == 'FINAL 2.0fps F3 TF2/3'


def test_final_debug_uses_only_action_result_detections():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.tag_size_m = 0.032
    analyzer.latest_container_debug_frame = None
    tag_outputs = []
    container_outputs = []
    analyzer.debug_image_publisher = SimpleNamespace(publish=tag_outputs.append)
    analyzer.container_debug_image_publisher = SimpleNamespace(
        publish=container_outputs.append)

    image = np.zeros((240, 320, 3), dtype=np.uint8)
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    frame = ContainerDebugFrame(
        header=Image().header,
        image=image,
        camera_matrix=np.array([
            [400.0, 0.0, 160.0],
            [0.0, 400.0, 120.0],
            [0.0, 0.0, 1.0],
        ]),
        camera_to_base=transform,
    )
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    mask[80:111, 80:111] = 255
    frame.container_masks = {BLUE: mask}
    session = Session(
        goal_handle=SimpleNamespace(), duration=2.0,
        requested_detectors=(
            AnalyzeScene.Goal.APRILTAGS | AnalyzeScene.Goal.CONTAINERS_HSV),
        started=time.monotonic() - 2.0,
        frames_processed=20,
        frames_with_base_transform=18,
        latest_debug_frame=frame,
    )
    tag = _tag_item(7, 0.4, 80.0)
    tag.pose.position.z = 0.5
    tag.pose.orientation.w = 1.0
    container = _container_item(RED, 0.0, z=0.5, frame='arm_base_link')
    container.observation_count = 12
    result = SimpleNamespace(
        best_apriltags_base=[tag],
        best_containers_base=[container],
        frames_processed=20,
        frames_with_base_transform=18,
    )

    analyzer.publish_final_debug_images(session, result, 'FINAL')

    assert len(tag_outputs) == 1
    assert len(container_outputs) == 1
    assert tag_outputs[0].encoding == 'bgr8'
    assert container_outputs[0].encoding == 'bgr8'
    assert analyzer.latest_container_debug_frame is not None
    assert np.count_nonzero(np.frombuffer(tag_outputs[0].data, np.uint8)) > 0
    assert np.count_nonzero(
        np.frombuffer(container_outputs[0].data, np.uint8)) > 0
    final_pixels = np.frombuffer(container_outputs[0].data, np.uint8).reshape(
        image.shape)
    assert final_pixels[95, 95, 0] > 0  # Final image retains the blue mask.

    empty_result = SimpleNamespace(
        best_apriltags_base=[],
        best_containers_base=[],
        frames_processed=20,
        frames_with_base_transform=18,
    )
    analyzer.publish_final_debug_images(session, empty_result, 'FINAL')
    assert len(tag_outputs) == 2
    assert len(container_outputs) == 2


def test_session_is_frozen_before_final_debug_is_published():
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.sessions_lock = threading.RLock()
    analyzer.publish_debug_image = True
    analyzer.get_logger = lambda: _Logger()
    session = Session(
        goal_handle=SimpleNamespace(), duration=2.0,
        requested_detectors=AnalyzeScene.Goal.CONTAINERS_HSV,
    )
    analyzer.session = session
    expected = SimpleNamespace()
    analyzer._result = lambda current, message: expected
    observed = []
    analyzer.publish_final_debug_images = (
        lambda current, result, status: observed.append(
            (analyzer.session, current, result, status)))

    result = analyzer._finish_session(session, 'done', 'FINAL')

    assert result is expected
    assert observed == [(None, session, expected, 'FINAL')]


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
    assert 'uint8 CONTAINERS_HSV=8' in action
    assert 'uint8 TABLE_SURFACE=4' in action
    assert 'best_apriltags_base' in action
    assert 'best_containers_base' in action
    assert 'interfaces/TableSurfaceGrid table_surface_grid' in action
    assert 'table_candidate_poses_base' not in action
    assert 'table_footprint_half_extent' not in action
    grid = (
        PACKAGE.parent / 'interfaces' / 'msg' / 'TableSurfaceGrid.msg'
    ).read_text()
    assert 'uint8 FREE=1' in grid
    assert 'float64 resolution_m' in grid
    assert 'uint8[] cells' in grid
    assert 'bool continuous' in action


def _white_surface_analyzer() -> SceneAnalyzer:
    analyzer = object.__new__(SceneAnalyzer)
    analyzer.white_surface_max_saturation = 45
    analyzer.white_surface_min_value = 40
    analyzer.white_surface_max_value = 250
    analyzer.white_surface_min_fraction = 0.95
    analyzer.white_surface_max_unknown_fraction = 0.05
    return analyzer


def _white_surface_session() -> Session:
    return Session(
        goal_handle=SimpleNamespace(is_cancel_requested=False),
        duration=1.0,
        requested_detectors=AnalyzeScene.Goal.TABLE_SURFACE,
        work_surface_height_m=0.0,
        table_search_x_min_m=0.0,
        table_search_y_min_m=0.0,
        table_grid_resolution_m=0.10,
        table_grid_width=1,
        table_grid_height=1,
    )


def _downward_camera_transform() -> TransformStamped:
    transform = TransformStamped()
    transform.transform.translation.z = 1.0
    # Camera optical +Z points toward base -Z.
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    return transform


def test_white_surface_grid_cell_requires_its_local_patch_to_be_white():
    analyzer = _white_surface_analyzer()
    session = _white_surface_session()
    matrix = np.array([[100.0, 0.0, 50.0],
                       [0.0, 100.0, 50.0],
                       [0.0, 0.0, 1.0]])
    image = np.full((100, 100, 3), 220, dtype=np.uint8)

    observed, confirmed, _debug = analyzer.evaluate_white_table_grid(
        session, image, matrix, _downward_camera_transform())
    assert observed == [True]
    assert confirmed == [True]

    image[43:58, 43:58] = (0, 0, 180)
    _observed, confirmed, _debug = analyzer.evaluate_white_table_grid(
        session, image, matrix, _downward_camera_transform())
    assert confirmed == [False]


def test_white_surface_grid_uses_all_pixels_inside_projected_cell():
    analyzer = _white_surface_analyzer()
    session = _white_surface_session()
    matrix = np.array([[100.0, 0.0, 50.0],
                       [0.0, 100.0, 50.0],
                       [0.0, 0.0, 1.0]])
    image = np.full((100, 100, 3), 220, dtype=np.uint8)

    # This stripe lies between the former 3x3 sample columns (45, 50, 55).
    # Rasterizing the complete projected cell must still detect it.
    image[45:56, 47:49] = (0, 0, 180)

    observed, confirmed, _debug = analyzer.evaluate_white_table_grid(
        session, image, matrix, _downward_camera_transform())

    assert observed == [True]
    assert confirmed == [False]


def test_white_surface_does_not_treat_clipped_highlights_as_free_space():
    analyzer = _white_surface_analyzer()
    session = _white_surface_session()
    matrix = np.array([[100.0, 0.0, 50.0],
                       [0.0, 100.0, 50.0],
                       [0.0, 0.0, 1.0]])
    image = np.full((100, 100, 3), 255, dtype=np.uint8)

    observed, confirmed, _debug = analyzer.evaluate_white_table_grid(
        session, image, matrix, _downward_camera_transform())
    assert observed == [False]
    assert confirmed == [False]


def test_white_surface_result_requires_repeated_confirmation():
    analyzer = _white_surface_analyzer()
    analyzer.sessions_lock = threading.RLock()
    analyzer.base_frame = 'arm_base_link'
    analyzer.white_surface_min_confirmed_frames = 2
    analyzer.white_surface_min_confirmed_ratio = 0.60
    session = _white_surface_session()
    session.table_cell_observations = [3]
    session.table_cell_confirmations = [2]

    result = analyzer._result(session, 'ok')

    assert list(result.table_surface_grid.cells) == [TableSurfaceGrid.FREE]

    session.table_cell_confirmations = [1]
    result = analyzer._result(session, 'ok')
    assert list(result.table_surface_grid.cells) == [TableSurfaceGrid.BLOCKED]


def test_table_surface_gets_its_own_final_debug_summary():
    analyzer = _white_surface_analyzer()
    outputs = []
    analyzer.table_surface_debug_image_publisher = SimpleNamespace(
        publish=outputs.append)
    session = _white_surface_session()
    session.detector_frame_counts[AnalyzeScene.Goal.TABLE_SURFACE] = 5
    session.detector_frames_with_base_transform[
        AnalyzeScene.Goal.TABLE_SURFACE] = 4
    session.detector_first_frame_times[
        AnalyzeScene.Goal.TABLE_SURFACE] = 10.0
    session.detector_last_frame_times[
        AnalyzeScene.Goal.TABLE_SURFACE] = 11.0
    image = np.full((100, 100, 3), 220, dtype=np.uint8)
    frame = ContainerDebugFrame(
        header=Image().header,
        image=image,
        camera_matrix=np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ]),
        camera_to_base=_downward_camera_transform(),
    )
    session.latest_debug_frames[AnalyzeScene.Goal.TABLE_SURFACE] = frame
    grid = TableSurfaceGrid()
    grid.cells = [TableSurfaceGrid.FREE]
    result = SimpleNamespace(table_surface_grid=grid)

    analyzer.publish_final_debug_images(session, result, 'FINAL')

    assert len(outputs) == 1
    assert outputs[0].encoding == 'bgr8'
    assert np.count_nonzero(np.frombuffer(outputs[0].data, np.uint8)) > 0


def test_hsv_result_requires_three_stable_frames_and_keeps_fixed_top_height():
    from vision.hsv_container import Blob, PixelObservation, PixelTrack

    analyzer = object.__new__(SceneAnalyzer)
    analyzer.hsv_min_frames = 3
    analyzer.hsv_center_tolerance = 6.0
    observations = []
    for frame, x in enumerate((40.0, 42.0, 41.0), 1):
        camera = ContainerStampedDetection()
        camera.color = RED
        camera.pose.position.x = x / 100.0
        camera.pose.position.z = 0.30
        camera.pose.orientation.w = 1.0
        base = ContainerStampedDetection()
        base.color = RED
        base.pose.position.x = x / 100.0
        base.pose.position.z = 0.173
        base.pose.orientation.w = 1.0
        observations.append(PixelObservation(
            frame, Blob(RED, (x, 30.0), 8000, False), camera, base))
    track = PixelTrack(RED, observations)
    assert analyzer._confirmed_hsv_container_results([
        PixelTrack(RED, observations[:2])]) == ([], [])
    camera, base = analyzer._confirmed_hsv_container_results([track])
    assert len(camera) == len(base) == 1
    assert base[0].observation_count == 3
    assert base[0].pose.position.x == pytest.approx(0.41)
    assert base[0].pose.position.z == pytest.approx(0.173)
    assert base[0].color == RED


def test_hsv_image_callback_returns_confirmed_base_pose_from_pixel_only():
    analyzer = _analyzer_for_container()
    analyzer.base_frame = 'base_link'
    analyzer.floor_frame = 'base_footprint'
    analyzer.sessions_lock = threading.RLock()
    analyzer.session = Session(
        goal_handle=SimpleNamespace(is_cancel_requested=False),
        duration=2.0, requested_detectors=AnalyzeScene.Goal.CONTAINERS_HSV,
        work_surface_height_m=0.10)
    analyzer.session.containers_ready_at = 0.0
    analyzer.last_detection_time = float('-inf')
    analyzer.detection_period = 0.0
    analyzer.publish_debug_image = False
    analyzer.hsv_min_areas = (100, 100, 100)
    analyzer.hsv_min_partial_areas = (100, 100, 100)
    analyzer.hsv_min_frames = 3
    analyzer.hsv_center_tolerance = 5.0
    analyzer.camera_info = SimpleNamespace(
        p=[400.0, 0.0, 160.0, 0.0, 0.0, 400.0, 120.0, 0.0,
           0.0, 0.0, 1.0, 0.0],
        header=SimpleNamespace(frame_id='camera'))
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera'
    transform.transform.translation.z = 0.5
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    floor_transform = TransformStamped()
    floor_transform.transform.translation.z = -0.097
    analyzer.tf_buffer = SimpleNamespace(
        lookup_transform=lambda _target, source, *_args, **_kwargs:
        floor_transform if source == 'base_footprint' else transform)
    outputs = []
    analyzer.container_detection_publisher = SimpleNamespace(
        publish=outputs.append)
    analyzer.container_camera_detection_publisher = SimpleNamespace(
        publish=lambda *_args: None)
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.rectangle(image, (180, 130), (219, 169), (255, 0, 0), -1)
    message = Image()
    message.header.frame_id = 'camera'
    message.height, message.width = image.shape[:2]
    message.encoding = 'bgr8'
    message.step = message.width * 3
    message.data = image.tobytes()
    for frame in range(3):
        message.header.stamp.nanosec = frame + 1
        analyzer.image_callback(message)
    assert len(outputs) == 3
    assert len(outputs[-1].detections) == 1
    _camera, confirmed = analyzer._confirmed_hsv_container_results(
        analyzer.session.hsv_container_tracks)
    assert len(confirmed) == 1
    assert confirmed[0].color == BLUE
    assert confirmed[0].observation_count == 3
    assert confirmed[0].pose.position.z == pytest.approx(0.076)
    assert confirmed[0].pose.position.x == pytest.approx(
        (199.5 - 160) / 400 * (0.5 - 0.076))


def test_hsv_area_parameter_names_match_height_bands():
    parameters = yaml.safe_load((PACKAGE / 'config' / 'vision.yaml').read_text())[
        'scene_analyzer']['ros__parameters']
    assert parameters['hsv_container_min_area_le_7_5cm_px'] == 4000
    assert parameters['hsv_container_min_area_le_12_5cm_px'] == 6500
    assert parameters['hsv_container_min_area_gt_12_5cm_px'] == 9000
    assert parameters['hsv_container_min_partial_area_le_5cm_px'] == 1800
    assert parameters['hsv_container_min_partial_area_le_10cm_px'] == 2500
    assert parameters['hsv_container_min_partial_area_gt_10cm_px'] == 3500
