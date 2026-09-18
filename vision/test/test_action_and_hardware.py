import inspect
import math
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped
from interfaces.msg import ContainerStampedDetection
import cv2
import numpy as np
import pytest
from rclpy.action import GoalResponse
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException


if 'pupil_apriltags' not in sys.modules:
    pupil = ModuleType('pupil_apriltags')
    pupil.Detector = object
    sys.modules['pupil_apriltags'] = pupil

from vision.geometry import load_geometry_profile, project_open_container  # noqa: E402
from vision.scene_analyzer import AnalysisSession, RED, SceneAnalyzer  # noqa: E402
from pathlib import Path  # noqa: E402


def request(tags=True, containers=False, height=0.10, seconds=1):
    return SimpleNamespace(
        analyze_apriltags=tags,
        analyze_containers=containers,
        table_height_m=height,
        duration=Duration(sec=seconds),
    )


def bare_node():
    node = SceneAnalyzer.__new__(SceneAnalyzer)
    node.lock = threading.RLock()
    node.reserved = False
    node.session = None
    node.hardware_release_at = None
    return node


def test_invalid_and_concurrent_goals_are_rejected():
    node = bare_node()
    assert node._goal_callback(request(False, False)) == GoalResponse.REJECT
    assert node._goal_callback(request(height=-0.01)) == GoalResponse.REJECT
    assert node._goal_callback(request(height=math.nan)) == GoalResponse.REJECT
    assert node._goal_callback(request(seconds=-1)) == GoalResponse.REJECT
    assert node._goal_callback(request()) == GoalResponse.ACCEPT
    assert node._goal_callback(request()) == GoalResponse.REJECT


def test_action_result_uses_rclpy_goal_execution_lifecycle():
    executed = []
    SceneAnalyzer._handle_accepted(None, SimpleNamespace(
        execute=lambda: executed.append(True)))
    assert executed == [True]


def test_hardware_idle_release_and_active_reuse_are_serialized():
    node = bare_node()
    node.hardware_lock = threading.Lock()
    node.hardware_active = True
    node.manage_camera = True
    node.manage_led = True
    node.camera_client = object()
    node.led_client = object()
    node.camera_timeout = node.led_timeout = 0.1
    calls = []
    node._call_bool_service = lambda client, state, timeout, camera=False: (
        calls.append((client, state, camera)) or True)
    assert node._acquire_hardware()
    assert calls == []
    node.hardware_release_at = time.monotonic() - 1.0
    node._hardware_idle_tick()
    assert calls == [
        (node.camera_client, False, True),
        (node.led_client, False, False),
    ]
    assert node.hardware_active is False


def test_tf_lookup_contract_uses_image_timestamp_and_never_latest():
    source = inspect.getsource(SceneAnalyzer._image_callback)
    assert 'message.header.stamp' in source
    assert 'lookup_transform' in source
    assert 'Time()' not in source
    assert 'transform_failures += 1' in source


def test_container_requires_stable_repeated_observations():
    node = bare_node()
    node.temporal_association_distance_m = 0.08
    node.min_container_observations = 2
    node.max_temporal_position_spread = 0.025
    node.max_temporal_yaw_spread = math.radians(12)
    node.min_confidence = 0.20
    item = ContainerStampedDetection()
    item.color = item.RED
    item.pose.orientation.w = 1.0
    item.confidence = 0.9
    stable, rejected = node._stable_containers([item])
    assert stable == []
    assert rejected[0].rejection_code == item.REJECTION_TEMPORAL_UNSTABLE
    repeated = ContainerStampedDetection()
    repeated.color = item.RED
    repeated.pose.position.x = 0.002
    repeated.pose.orientation.w = 1.0
    repeated.confidence = 0.8
    stable, rejected = node._stable_containers([item, repeated])
    assert len(stable) == 1
    assert rejected == []


def test_cancel_cleans_session_and_schedules_hardware_release(monkeypatch):
    node = bare_node()
    node.reserved = True
    node.last_detection = 0.0
    node.hardware_grace = 0.0
    node.feedback_period = 1.0
    node._acquire_hardware = lambda: True
    node.min_container_observations = 2
    canceled = []
    goal = SimpleNamespace(
        request=request(tags=True, seconds=10),
        is_cancel_requested=True,
        is_active=True,
        canceled=lambda result: canceled.append(result),
    )
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    result = node._execute(goal)
    assert result.message == 'Analysis cancelled.'
    assert canceled == [result]
    assert node.session is None
    assert node.reserved is False
    assert node.hardware_release_at is not None


GEOMETRY = load_geometry_profile(
    str(Path(__file__).parents[1] / 'config' / 'container_geometry_profiles.yaml'),
    'current_team_model')
MATRIX = np.array([
    [232.5257, 0.0, 160.5], [0.0, 235.77464, 120.5], [0.0, 0.0, 1.0]])


def _downward_camera():
    transform = TransformStamped()
    transform.transform.translation.z = 0.50
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    return transform


def _synthetic_node():
    node = bare_node()
    node.geometry = GEOMETRY
    node.output_frame = 'arm_base_link'
    for name, value in {
        'min_component_area_px': 80, 'max_component_area_fraction': 0.85,
        'cube_size_m': 0.042, 'rim_segmentable_fraction': 0.85,
        'side_segmentable_fraction': 0.55, 'max_opening_cubes': 2,
        'max_dimension_relative_error': 0.55, 'cube_model_margin': 0.1,
        'min_area_ratio': 0.25, 'max_area_ratio': 2.2,
        'min_visible_fraction': 0.3, 'min_rim_support': 0.22,
        'min_opening_score': 0.25, 'partial_ambiguity_visible_fraction': 0.48,
        'min_confidence': 0.2,
        'minimum_observable_opening_fraction': 0.18,
    }.items():
        setattr(node, name, value)
    return node


def test_detector_accepts_projected_open_container_not_colored_cube():
    node = _synthetic_node()
    projection = project_open_container(
        (0.0, 0.0), 0.0, 0.10, GEOMETRY,
        MATRIX, _downward_camera(), (240, 320), 0.85, 0.55)
    image = Image()
    image.height, image.width = 240, 320
    image.header.frame_id = 'camera_optical_frame'
    empty = np.zeros((240, 320), dtype=np.uint8)
    accepted = node._analyze_containers(
        {RED: projection.rim_mask}, image, MATRIX, _downward_camera(), 0.10)
    assert len(accepted) == 1
    assert accepted[0].detection.rejection_code == 0
    assert abs(accepted[0].detection.pose.position.x) < 0.005
    assert accepted[0].detection.pose.position.z == 0.173
    cube = empty.copy()
    cv2.rectangle(cube, (147, 107), (173, 133), 255, -1)
    rejected = node._analyze_containers(
        {RED: cube}, image, MATRIX, _downward_camera(), 0.10)
    assert rejected
    assert all(item.detection.rejection_code != 0 for item in rejected)


def test_missing_timestamped_tf_records_diagnostic_and_rejected_candidate():
    node = _synthetic_node()
    node.detection_period = 0.0
    node.last_detection = -math.inf
    node.tf_timeout = 0.1
    node.publish_debug = False
    node._publish_compatibility_topics = lambda *_args: None
    image = Image()
    image.height, image.width, image.step = 240, 320, 960
    image.encoding = 'bgr8'
    image.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()
    image.header.frame_id = 'camera_optical_frame'
    image.header.stamp.sec = 17
    info = CameraInfo()
    info.width, info.height = 320, 240
    info.header.frame_id = image.header.frame_id
    info.p = [232.5257, 0, 160.5, 0, 0, 235.77464, 120.5, 0, 0, 0, 1, 0]
    node.camera_info = info
    mask = np.zeros((240, 320), dtype=np.uint8)
    cv2.rectangle(mask, (90, 90), (150, 155), 255, -1)
    node.color_masks = lambda _: {RED: mask}
    looked_up = []
    def lookup(target, source, stamp, timeout):
        looked_up.append((target, source, stamp.sec, timeout.nanoseconds))
        raise TransformException('transform too late for image')
    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    goal = SimpleNamespace(is_cancel_requested=False)
    session = AnalysisSession(goal, 1.0, False, True, 0.10)
    node.session = session
    node._image_callback(image)
    assert looked_up == [('arm_base_link', 'camera_optical_frame', 17, 100_000_000)]
    assert session.transform_failures == 1
    assert session.rejected_containers[0].rejection_code == (
        ContainerStampedDetection.REJECTION_TF_UNAVAILABLE)


def test_floor_height_uses_timestamped_tf_even_below_arm_origin():
    node = _synthetic_node()
    node.height_frame = 'base_footprint'
    node.detection_period = 0.0
    node.last_detection = -math.inf
    node.tf_timeout = 0.1
    node.publish_debug = False
    node._publish_compatibility_topics = lambda *_args: None
    image = Image()
    image.height, image.width, image.step = 240, 320, 960
    image.encoding = 'bgr8'
    image.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()
    image.header.frame_id = 'camera_optical_frame'
    image.header.stamp.sec = 23
    info = CameraInfo()
    info.width, info.height = 320, 240
    info.header.frame_id = image.header.frame_id
    info.p = [232.5257, 0, 160.5, 0, 0, 235.77464, 120.5, 0, 0, 0, 1, 0]
    node.camera_info = info
    node.color_masks = lambda _: {RED: np.zeros((240, 320), dtype=np.uint8)}
    seen, calls = [], []
    node._analyze_containers = lambda masks, image, matrix, tf, z: (
        seen.append(z) or [])

    def lookup(target, source, stamp, timeout):
        calls.append((target, source, stamp.sec))
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        if source == 'base_footprint':
            transform.transform.translation.z = -0.112
        return transform

    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    node.session = AnalysisSession(SimpleNamespace(is_cancel_requested=False),
                                   1.0, False, True, 0.05)
    node._image_callback(image)
    assert seen == [pytest.approx(-0.062)]
    assert calls == [
        ('arm_base_link', 'camera_optical_frame', 23),
        ('arm_base_link', 'base_footprint', 23),
    ]

    def missing_floor(target, source, stamp, timeout):
        if source == 'base_footprint':
            raise TransformException('floor TF delayed')
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        return transform

    node.tf_buffer = SimpleNamespace(lookup_transform=missing_floor)
    node.session = AnalysisSession(SimpleNamespace(is_cancel_requested=False),
                                   1.0, False, True, 0.05)
    node._image_callback(image)
    assert node.session.frames_processed == 1
    assert node.session.frames_with_transform == 0
    assert node.session.transform_failures == 1
