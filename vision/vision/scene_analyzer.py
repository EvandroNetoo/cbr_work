"""Unified, on-demand AprilTag and open-container scene analysis."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
import threading
import time
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from interfaces.action import SceneAnalyzer as SceneAnalyzerAction
from interfaces.msg import (
    AprilTagDetection, AprilTagDetectionArray, AprilTagStampedDetection,
    ContainerDetection, ContainerDetectionArray, ContainerStampedDetection,
)
import numpy as np
from pupil_apriltags import Detector
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geometry import (
    ContainerGeometry, axial_angle_distance, hsv_color_masks, load_geometry_profile,
    opening_evidence_score,
    pixels_to_plane, project_open_container, rectangular_model_error,
    transform_parts, yaw_quaternion,
)


RED = ContainerStampedDetection.RED
BLUE = ContainerStampedDetection.BLUE
COLOR_NAMES = {RED: 'red', BLUE: 'blue'}
DEBUG_COLORS = {RED: (20, 20, 255), BLUE: (255, 90, 20)}


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _duration_message(seconds: float):
    from builtin_interfaces.msg import Duration as DurationMessage
    value = max(0.0, float(seconds))
    output = DurationMessage()
    output.sec = int(value)
    output.nanosec = int((value - output.sec) * 1e9)
    return output


def _capture_response_ok(response, enabled: bool) -> bool:
    if response is None:
        return False
    if response.success:
        return True
    expected = 'start capturing' if enabled else 'stop capturing'
    return response.message.strip().casefold() == expected


def _quaternion_from_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        return (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        scale = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        return (0.25*scale, (matrix[0, 1]+matrix[1, 0])/scale,
                (matrix[0, 2]+matrix[2, 0])/scale,
                (matrix[2, 1]-matrix[1, 2])/scale)
    if index == 1:
        scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        return ((matrix[0, 1]+matrix[1, 0])/scale, 0.25*scale,
                (matrix[1, 2]+matrix[2, 1])/scale,
                (matrix[0, 2]-matrix[2, 0])/scale)
    scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    return ((matrix[0, 2]+matrix[2, 0])/scale,
            (matrix[1, 2]+matrix[2, 1])/scale, 0.25*scale,
            (matrix[1, 0]-matrix[0, 1])/scale)


@dataclass
class ContainerCandidate:
    detection: ContainerStampedDetection
    contour: np.ndarray
    outer_projection: np.ndarray | None = None
    inner_projection: np.ndarray | None = None


@dataclass
class AnalysisSession:
    goal_handle: object
    duration: float
    analyze_apriltags: bool
    analyze_containers: bool
    table_height: float
    started: float = field(default_factory=time.monotonic)
    frames_processed: int = 0
    frames_with_transform: int = 0
    transform_failures: int = 0
    latest_tags: list[AprilTagStampedDetection] = field(default_factory=list)
    latest_containers: list[ContainerStampedDetection] = field(default_factory=list)
    latest_rejected_tags: list[AprilTagStampedDetection] = field(default_factory=list)
    latest_rejected_containers: list[ContainerStampedDetection] = field(default_factory=list)
    tag_best: dict[int, AprilTagStampedDetection] = field(default_factory=dict)
    container_observations: list[ContainerStampedDetection] = field(default_factory=list)
    rejected_tags: list[AprilTagStampedDetection] = field(default_factory=list)
    rejected_containers: list[ContainerStampedDetection] = field(default_factory=list)
    last_feedback: float = 0.0


class SceneAnalyzer(Node):
    """Own the camera/LED lease and analyze both modalities per image."""

    def __init__(self) -> None:
        super().__init__('vision')
        defaults = {
            'image_topic': '/camera/image_rect',
            'camera_info_topic': '/camera/camera_info',
            'output_frame': 'arm_base_link',
            'work_surface_height_frame': 'base_footprint',
            'geometry_profiles_file': '',
            'geometry_profile': 'current_team_model',
            'family': 'tag36h11', 'tag_size_m': 0.032,
            'tag_frame_prefix': 'apriltag', 'nthreads': 1,
            'quad_decimate': 1.0, 'min_decision_margin': 30.0,
            'max_hamming': 0, 'min_saturation': 80, 'min_value': 45,
            'red_hue_low_1': 0, 'red_hue_high_1': 12,
            'red_hue_low_2': 150, 'red_hue_high_2': 179,
            'blue_hue_low': 70, 'blue_hue_high': 138,
            'morphology_open_px': 3, 'morphology_close_px': 5,
            'min_component_area_px': 80.0,
            'max_component_area_fraction': 0.85,
            'cube_size_m': 0.042, 'min_visible_fraction': 0.30,
            'min_rim_support': 0.22, 'min_opening_score': 0.25,
            'min_area_ratio': 0.25, 'max_area_ratio': 2.20,
            'max_dimension_relative_error': 0.55,
            'cube_model_margin': 0.10, 'min_confidence': 0.20,
            'partial_ambiguity_visible_fraction': 0.48,
            'rim_segmentable_fraction': 0.85,
            'side_segmentable_fraction': 0.55,
            'max_opening_cubes': 2,
            'minimum_observable_opening_fraction': 0.18,
            'temporal_association_distance_m': 0.08,
            'min_container_observations': 2,
            'max_temporal_position_spread_m': 0.025,
            'max_temporal_yaw_spread_deg': 12.0,
            'max_detection_rate_hz': 10.0, 'feedback_rate_hz': 5.0,
            'tf_timeout_sec': 0.12, 'publish_debug_images': False,
            'manage_camera_capture': True,
            'camera_capture_service': '/camera/set_capture',
            'camera_capture_timeout_sec': 5.0,
            'manage_vision_led': True,
            'vision_led_service': '/base_hardware/set_vision_led',
            'vision_led_timeout_sec': 5.0,
            'hardware_idle_grace_sec': 0.75,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.output_frame = str(self.get_parameter('output_frame').value)
        self.height_frame = str(self.get_parameter('work_surface_height_frame').value)
        self.tag_size = self._positive('tag_size_m')
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.min_margin = self._positive('min_decision_margin')
        self.max_hamming = int(self.get_parameter('max_hamming').value)
        profile_path = str(self.get_parameter('geometry_profiles_file').value)
        if not profile_path:
            profile_path = str(
                __import__('pathlib').Path(get_package_share_directory('vision'))
                / 'config' / 'container_geometry_profiles.yaml')
        self.geometry: ContainerGeometry = load_geometry_profile(
            profile_path, str(self.get_parameter('geometry_profile').value))
        self._configure_container_parameters()
        self.detection_period = 1.0 / self._positive('max_detection_rate_hz')
        self.feedback_period = 1.0 / self._positive('feedback_rate_hz')
        self.tf_timeout = self._positive('tf_timeout_sec')
        self.publish_debug = bool(self.get_parameter('publish_debug_images').value)

        self.detector = Detector(
            families=str(self.get_parameter('family').value),
            nthreads=int(self.get_parameter('nthreads').value),
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            refine_edges=1,
        )
        self.lock = threading.RLock()
        self.hardware_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        self.session: AnalysisSession | None = None
        self.reserved = False
        self.last_detection = float('-inf')
        self.camera_info: CameraInfo | None = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback, qos_profile_sensor_data,
            callback_group=self.callback_group)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._image_callback, qos_profile_sensor_data,
            callback_group=self.callback_group)

        self.tag_camera_publisher = self.create_publisher(
            AprilTagDetectionArray, 'apriltags/detections_camera', 1)
        self.tag_publisher = self.create_publisher(
            AprilTagDetectionArray, 'apriltags/detections', 1)
        self.tag_pose_publisher = self.create_publisher(
            PoseArray, 'apriltags/poses', 1)
        self.container_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections', 1)
        self.tag_debug_publisher = self.create_publisher(
            Image, 'vision/debug/apriltags', qos_profile_sensor_data)
        self.container_debug_publisher = self.create_publisher(
            Image, 'vision/debug/containers', qos_profile_sensor_data)

        self.manage_camera = bool(self.get_parameter('manage_camera_capture').value)
        self.manage_led = bool(self.get_parameter('manage_vision_led').value)
        self.camera_timeout = self._positive('camera_capture_timeout_sec')
        self.led_timeout = self._positive('vision_led_timeout_sec')
        self.camera_client = self.create_client(
            SetBool, str(self.get_parameter('camera_capture_service').value),
            callback_group=self.callback_group) \
            if self.manage_camera else None
        self.led_client = self.create_client(
            SetBool, str(self.get_parameter('vision_led_service').value),
            callback_group=self.callback_group) \
            if self.manage_led else None
        self.hardware_active = False
        self.hardware_release_at: float | None = None
        self.hardware_grace = max(
            0.0, float(self.get_parameter('hardware_idle_grace_sec').value))
        self.hardware_timer = self.create_timer(
            0.1, self._hardware_idle_tick, callback_group=self.callback_group)

        self.action_server = ActionServer(
            self, SceneAnalyzerAction, '/vision/analyze',
            goal_callback=self._goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            handle_accepted_callback=self._handle_accepted,
            execute_callback=self._execute,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            f"Vision ready with geometry '{self.geometry.name}': "
            f'{self.geometry.physical_object}.')

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value

    def _configure_container_parameters(self) -> None:
        self.min_saturation = int(self.get_parameter('min_saturation').value)
        self.min_value = int(self.get_parameter('min_value').value)
        self.red_ranges = (
            (int(self.get_parameter('red_hue_low_1').value),
             int(self.get_parameter('red_hue_high_1').value)),
            (int(self.get_parameter('red_hue_low_2').value),
             int(self.get_parameter('red_hue_high_2').value)),
        )
        self.blue_range = (
            int(self.get_parameter('blue_hue_low').value),
            int(self.get_parameter('blue_hue_high').value))
        for value in (*self.red_ranges[0], *self.red_ranges[1], *self.blue_range):
            if not 0 <= value <= 179:
                raise ValueError('Hue limits must use the OpenCV [0, 179] scale')
        self.open_kernel = self._kernel('morphology_open_px')
        self.close_kernel = self._kernel('morphology_close_px')
        names = (
            'min_component_area_px', 'max_component_area_fraction',
            'cube_size_m', 'min_visible_fraction', 'min_rim_support',
            'min_opening_score', 'min_area_ratio', 'max_area_ratio',
            'max_dimension_relative_error', 'cube_model_margin',
            'min_confidence', 'partial_ambiguity_visible_fraction',
            'rim_segmentable_fraction', 'side_segmentable_fraction',
            'temporal_association_distance_m',
            'minimum_observable_opening_fraction',
        )
        for name in names:
            setattr(self, name, float(self.get_parameter(name).value))
        self.max_opening_cubes = int(self.get_parameter('max_opening_cubes').value)
        self.min_container_observations = int(
            self.get_parameter('min_container_observations').value)
        self.max_temporal_position_spread = self._positive(
            'max_temporal_position_spread_m')
        self.max_temporal_yaw_spread = math.radians(self._positive(
            'max_temporal_yaw_spread_deg'))
        if self.min_container_observations <= 0 or self.max_opening_cubes < 0:
            raise ValueError('Temporal and opening cube counts must be nonnegative')

    def _kernel(self, name: str) -> np.ndarray:
        size = int(self.get_parameter(name).value)
        if size <= 0:
            raise ValueError(f'{name} must be positive')
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _goal_callback(self, request) -> GoalResponse:
        duration = _duration_seconds(request.duration)
        height = float(request.table_height_m)
        if (not request.analyze_apriltags and not request.analyze_containers):
            return GoalResponse.REJECT
        if duration < 0.0 or not math.isfinite(height) or height < 0.0:
            return GoalResponse.REJECT
        with self.lock:
            if self.reserved or self.session is not None:
                return GoalResponse.REJECT
            self.reserved = True
            self.hardware_release_at = None
        return GoalResponse.ACCEPT

    def _handle_accepted(self, goal_handle) -> None:
        goal_handle.execute()

    def _execute(self, goal_handle):
        request = goal_handle.request
        session = AnalysisSession(
            goal_handle, _duration_seconds(request.duration),
            bool(request.analyze_apriltags), bool(request.analyze_containers),
            float(request.table_height_m))
        with self.lock:
            self.session = session
            self.last_detection = float('-inf')
        try:
            if not self._acquire_hardware():
                result = self._result(session, 'Camera or vision LED could not be acquired.')
                goal_handle.abort(result)
                return result
            while rclpy.ok() and goal_handle.is_active:
                time.sleep(0.02)
                elapsed = time.monotonic() - session.started
                if goal_handle.is_cancel_requested:
                    result = self._result(session, 'Analysis cancelled.')
                    goal_handle.canceled(result)
                    return result
                if session.duration > 0.0 and elapsed >= session.duration:
                    reliable = (
                        session.frames_processed > 0
                        and session.frames_with_transform > 0)
                    result = self._result(
                        session,
                        'Analysis completed.' if reliable
                        else 'No image with timestamped TF was processed.')
                    if reliable:
                        goal_handle.succeed(result)
                    else:
                        goal_handle.abort(result)
                    return result
                if time.monotonic() - session.last_feedback >= self.feedback_period:
                    session.last_feedback = time.monotonic()
                    goal_handle.publish_feedback(self._feedback(session))
            result = self._result(session, 'ROS shutdown interrupted analysis.')
            if goal_handle.is_active:
                goal_handle.abort(result)
            return result
        except Exception as error:
            self.get_logger().error(f'Vision analysis failed: {error!r}')
            result = self._result(session, f'Analysis failed: {error}')
            if goal_handle.is_active:
                goal_handle.abort(result)
            return result
        finally:
            with self.lock:
                if self.session is session:
                    self.session = None
                self.reserved = False
                self.hardware_release_at = time.monotonic() + self.hardware_grace

    def _feedback(self, session: AnalysisSession):
        feedback = SceneAnalyzerAction.Feedback()
        with self.lock:
            feedback.apriltags = copy.deepcopy(session.latest_tags)
            feedback.containers = copy.deepcopy(session.latest_containers)
            feedback.rejected_apriltags = copy.deepcopy(session.latest_rejected_tags)
            feedback.rejected_containers = copy.deepcopy(session.latest_rejected_containers)
            feedback.frames_processed = session.frames_processed
            feedback.frames_with_transform = session.frames_with_transform
            feedback.transform_failures = session.transform_failures
        feedback.phase = feedback.ANALYZING
        feedback.hardware_active = self.hardware_active
        elapsed = time.monotonic() - session.started
        feedback.elapsed = _duration_message(elapsed)
        feedback.continuous = session.duration == 0.0
        feedback.remaining = _duration_message(
            0.0 if feedback.continuous else session.duration - elapsed)
        return feedback

    def _result(self, session: AnalysisSession, message: str):
        result = SceneAnalyzerAction.Result()
        with self.lock:
            result.apriltags = [copy.deepcopy(session.tag_best[key])
                                for key in sorted(session.tag_best)]
            stable, unstable = self._stable_containers(session.container_observations)
            result.containers = stable
            result.rejected_apriltags = copy.deepcopy(session.rejected_tags[-64:])
            result.rejected_containers = copy.deepcopy(
                (session.rejected_containers + unstable)[-64:])
            result.frames_processed = session.frames_processed
            result.frames_with_transform = session.frames_with_transform
            result.transform_failures = session.transform_failures
        result.elapsed = _duration_message(time.monotonic() - session.started)
        result.message = message
        return result

    def _stable_containers(self, observations):
        clusters: list[list[ContainerStampedDetection]] = []
        for item in sorted(observations, key=lambda value: (
                value.color, value.header.stamp.sec, value.header.stamp.nanosec)):
            position = np.array([item.pose.position.x, item.pose.position.y])
            selected = None
            for cluster in clusters:
                if cluster[0].color != item.color:
                    continue
                center = np.median(np.array([
                    [member.pose.position.x, member.pose.position.y]
                    for member in cluster]), axis=0)
                if np.linalg.norm(position - center) <= self.temporal_association_distance_m:
                    selected = cluster
                    break
            if selected is None:
                clusters.append([item])
            else:
                selected.append(item)
        output, rejected = [], []
        for cluster in clusters:
            if len(cluster) < self.min_container_observations:
                item = copy.deepcopy(max(cluster, key=lambda value: value.confidence))
                item.rejection_code = item.REJECTION_TEMPORAL_UNSTABLE
                item.rejection_detail = (
                    f'{len(cluster)} observation(s), '
                    f'minimum {self.min_container_observations}')
                rejected.append(item)
                continue
            representative = max(cluster, key=lambda item: item.confidence)
            result = copy.deepcopy(representative)
            positions = np.array([
                [item.pose.position.x, item.pose.position.y, item.pose.position.z]
                for item in cluster])
            median_xy = np.median(positions[:, :2], axis=0)
            position_spread = max(float(np.linalg.norm(
                position[:2] - median_xy)) for position in positions)
            yaws = [self._pose_yaw(item.pose) for item in cluster]
            yaw_reference = yaws[0]
            yaw_spread = max(
                axial_angle_distance(yaw, yaw_reference) for yaw in yaws)
            if (position_spread > self.max_temporal_position_spread
                    or yaw_spread > self.max_temporal_yaw_spread):
                result.rejection_code = result.REJECTION_TEMPORAL_UNSTABLE
                result.rejection_detail = (
                    f'position spread={position_spread:.3f} m, '
                    f'axial yaw spread={math.degrees(yaw_spread):.1f} deg')
                rejected.append(result)
                continue
            stability_score = max(0.0, 1.0 - max(
                position_spread / self.max_temporal_position_spread,
                yaw_spread / self.max_temporal_yaw_spread))
            result.pose.position.x, result.pose.position.y, result.pose.position.z = (
                map(float, np.median(positions, axis=0)))
            result.observation_count = len(cluster)
            result.confidence = min(result.confidence, stability_score)
            if result.confidence < self.min_confidence:
                result.rejection_code = result.REJECTION_TEMPORAL_UNSTABLE
                result.rejection_detail = (
                    f'temporal confidence={stability_score:.3f}')
                rejected.append(result)
                continue
            output.append(result)
        return output, rejected

    @staticmethod
    def _pose_yaw(pose: Pose) -> float:
        q = pose.orientation
        return math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if (message.width > 0 and message.height > 0
                and message.p[0] > 0.0 and message.p[5] > 0.0
                and all(math.isfinite(value) for value in message.p)):
            self.camera_info = message

    def _image_callback(self, message: Image) -> None:
        with self.lock:
            session = self.session
            info = self.camera_info
            now = time.monotonic()
            if session is None or info is None or now - self.last_detection < self.detection_period:
                return
            self.last_detection = now
        if (message.width != info.width or message.height != info.height
                or (message.header.frame_id and info.header.frame_id
                    and message.header.frame_id != info.header.frame_id)):
            self.get_logger().warning(
                'CameraInfo dimension/frame mismatch: image cannot be analyzed.',
                throttle_duration_sec=2.0)
            return
        camera_frame = message.header.frame_id or info.header.frame_id
        if not camera_frame:
            return
        try:
            bgr = self.image_to_bgr(message)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'Image rejected: {error}', throttle_duration_sec=2.0)
            return
        camera_matrix = np.array([
            [info.p[0], 0.0, info.p[2]],
            [0.0, info.p[5], info.p[6]],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame, camera_frame, message.header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as error:
            session.transform_failures += 1
            transform_error = str(error)
        else:
            transform_error = ''

        tags, rejected_tags = ([], [])
        if session.analyze_apriltags:
            tags, rejected_tags = self._analyze_tags(
                bgr, message, camera_frame, camera_matrix, transform, transform_error)
        containers, rejected_containers, candidates, masks = ([], [], [], {})
        if session.analyze_containers:
            masks = self.color_masks(bgr)
            height_error = ''
            floor_to_output = None
            if transform is not None:
                try:
                    floor_to_output = self.tf_buffer.lookup_transform(
                        self.output_frame, self.height_frame, message.header.stamp,
                        timeout=Duration(seconds=self.tf_timeout))
                    rotation, translation = transform_parts(floor_to_output)
                    axis = rotation[:, 2]
                    if (max(abs(float(axis[0])), abs(float(axis[1]))) > 0.001
                            or float(axis[2]) < 0.999):
                        raise ValueError('floor plane is not horizontal in output frame')
                    table_z = float(translation[2] + session.table_height*axis[2])
                except (TransformException, ValueError) as error:
                    session.transform_failures += 1
                    height_error = f'work surface TF: {error}'
            if transform is None or height_error:
                rejected_containers = self._tf_rejected_contours(
                    masks, message, camera_frame, height_error or transform_error)
            else:
                candidates = self._analyze_containers(
                    masks, message, camera_matrix, transform, table_z)
                containers = [item.detection for item in candidates
                              if item.detection.rejection_code == item.detection.REJECTION_NONE]
                rejected_containers = [item.detection for item in candidates
                                       if item.detection.rejection_code != item.detection.REJECTION_NONE]
        # Count only frames with every TF required by the requested modalities.
        # A camera TF alone cannot certify an empty work surface when its floor
        # plane TF is missing: table placement must fail closed in that case.
        if transform is not None and not (session.analyze_containers and height_error):
            session.frames_with_transform += 1
        if self.publish_debug:
            if session.analyze_apriltags:
                self._publish_tag_debug(message, bgr, tags, rejected_tags)
            if session.analyze_containers:
                self._publish_container_debug(message, bgr, masks, candidates)

        self._publish_compatibility_topics(message, camera_frame, tags, containers)
        with self.lock:
            if self.session is not session or session.goal_handle.is_cancel_requested:
                return
            session.frames_processed += 1
            session.latest_tags = copy.deepcopy(tags)
            session.latest_containers = copy.deepcopy(containers)
            session.latest_rejected_tags = copy.deepcopy(rejected_tags)
            session.latest_rejected_containers = copy.deepcopy(rejected_containers)
            session.rejected_tags.extend(copy.deepcopy(rejected_tags))
            session.rejected_containers.extend(copy.deepcopy(rejected_containers))
            session.container_observations.extend(copy.deepcopy(containers))
            for item in tags:
                old = session.tag_best.get(item.id)
                if old is None or item.confidence > old.confidence:
                    session.tag_best[item.id] = copy.deepcopy(item)

    def _analyze_tags(self, bgr, image, camera_frame, matrix, transform, tf_error):
        mono = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        parameters = (matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2])
        detections = self.detector.detect(
            mono, estimate_tag_pose=True, camera_params=parameters,
            tag_size=self.tag_size)
        accepted, rejected, transforms, camera_accepted = [], [], [], []
        for raw in detections:
            item = AprilTagStampedDetection()
            item.header = copy.deepcopy(image.header)
            item.header.frame_id = camera_frame
            item.family = (raw.tag_family.decode() if isinstance(raw.tag_family, bytes)
                           else str(raw.tag_family))
            item.id = int(raw.tag_id)
            item.decision_margin = float(raw.decision_margin)
            item.hamming = int(raw.hamming)
            item.pose_error = float(raw.pose_err)
            item.size_m = self.tag_size
            item.partial = False
            item.position_uncertainty_m = 0.0
            item.yaw_uncertainty_deg = 0.0
            margin_score = min(1.0, max(0.0, item.decision_margin / (2*self.min_margin)))
            error_score = 1.0 / (1.0 + max(0.0, item.pose_error))
            item.confidence = min(margin_score, error_score)
            if item.hamming > self.max_hamming:
                item.rejection_code = item.REJECTION_HAMMING
                item.rejection_detail = f'hamming={item.hamming} > {self.max_hamming}'
            elif item.decision_margin < self.min_margin:
                item.rejection_code = item.REJECTION_LOW_MARGIN
                item.rejection_detail = (
                    f'margin={item.decision_margin:.2f} < {self.min_margin:.2f}')
            try:
                translation = np.asarray(raw.pose_t, dtype=np.float64).reshape(3)
                rotation = np.asarray(raw.pose_R, dtype=np.float64).reshape(3, 3)
                if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rotation)):
                    raise ValueError('non-finite pose')
                item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, translation)
                (item.pose.orientation.x, item.pose.orientation.y,
                 item.pose.orientation.z, item.pose.orientation.w) = _quaternion_from_matrix(rotation)
                item.pose_valid = True
            except (ValueError, TypeError):
                item.pose_valid = False
                item.rejection_code = item.REJECTION_INVALID_POSE
                item.rejection_detail = 'invalid native pose'
            camera_pose = PoseStamped(header=item.header, pose=item.pose)
            if item.rejection_code == item.REJECTION_NONE:
                camera_accepted.append(copy.deepcopy(item))
            if item.rejection_code == item.REJECTION_NONE and transform is None:
                item.rejection_code = item.REJECTION_TF_UNAVAILABLE
                item.rejection_detail = tf_error or 'transform unavailable at image timestamp'
            if item.pose_valid and transform is not None:
                base_pose = do_transform_pose_stamped(camera_pose, transform)
                item.header = base_pose.header
                item.header.stamp = copy.deepcopy(image.header.stamp)
                item.header.frame_id = self.output_frame
                item.pose = base_pose.pose
            if item.rejection_code != item.REJECTION_NONE:
                rejected.append(item)
                continue
            accepted.append(item)
            tag_tf = TransformStamped()
            tag_tf.header = copy.deepcopy(image.header)
            tag_tf.header.frame_id = camera_frame
            tag_tf.child_frame_id = f'{self.tag_frame_prefix}_{item.family}_{item.id}'
            tag_tf.transform.translation.x = camera_pose.pose.position.x
            tag_tf.transform.translation.y = camera_pose.pose.position.y
            tag_tf.transform.translation.z = camera_pose.pose.position.z
            tag_tf.transform.rotation = camera_pose.pose.orientation
            transforms.append(tag_tf)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
        camera_array = AprilTagDetectionArray()
        camera_array.header = copy.deepcopy(image.header)
        camera_array.header.frame_id = camera_frame
        for item in camera_accepted:
            camera_array.detections.append(AprilTagDetection(
                family=item.family, id=item.id,
                decision_margin=item.decision_margin, hamming=item.hamming,
                pose_error=item.pose_error, pose=item.pose))
        self.tag_camera_publisher.publish(camera_array)
        return accepted, rejected

    def color_masks(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        red, blue = hsv_color_masks(
            bgr, min_saturation=self.min_saturation, min_value=self.min_value,
            red_ranges=self.red_ranges, blue_range=self.blue_range,
            open_size=self.open_kernel.shape[0], close_size=self.close_kernel.shape[0])
        return {RED: red, BLUE: blue}

    def _tf_rejected_contours(self, masks, image, frame, detail):
        output = []
        for color, mask in masks.items():
            contours, hierarchy = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for index, contour in enumerate(contours):
                if hierarchy is not None and hierarchy[0, index, 3] >= 0:
                    continue
                if cv2.contourArea(contour) < self.min_component_area_px:
                    continue
                item = self._empty_container(image, color)
                item.header.frame_id = frame
                item.contour_area_px = float(cv2.contourArea(contour))
                item.rejection_code = item.REJECTION_TF_UNAVAILABLE
                item.rejection_detail = detail or 'transform unavailable at image timestamp'
                output.append(item)
        return output

    def _empty_container(self, image, color):
        item = ContainerStampedDetection()
        item.header = copy.deepcopy(image.header)
        item.color = color
        item.external_dimensions_m.x = self.geometry.depth
        item.external_dimensions_m.y = self.geometry.width
        item.external_dimensions_m.z = self.geometry.height
        item.internal_dimensions_m.x = self.geometry.inner_depth
        item.internal_dimensions_m.y = self.geometry.inner_width
        item.internal_dimensions_m.z = self.geometry.inner_height
        return item

    def _analyze_containers(self, masks, image, matrix, transform, table_height):
        output = []
        image_area = float(image.height * image.width)
        top_height = table_height + self.geometry.height
        for color, mask in masks.items():
            contours, hierarchy = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for index, contour in enumerate(contours):
                if hierarchy is not None and hierarchy[0, index, 3] >= 0:
                    continue
                area = float(cv2.contourArea(contour))
                item = self._empty_container(image, color)
                item.header.frame_id = self.output_frame
                item.contour_area_px = area
                if area < self.min_component_area_px:
                    item.rejection_code = item.REJECTION_TOO_SMALL
                    item.rejection_detail = 'segmented component below minimum area'
                    output.append(ContainerCandidate(item, contour))
                    continue
                if area / image_area > self.max_component_area_fraction:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = 'component occupies implausible image fraction'
                    output.append(ContainerCandidate(item, contour))
                    continue
                hull = cv2.convexHull(contour).reshape(-1, 2)
                world = pixels_to_plane(hull, matrix, transform, top_height)
                if world is None or len(world) < 3:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = 'color contour cannot be intersected with top plane'
                    output.append(ContainerCandidate(item, contour))
                    continue
                rect = cv2.minAreaRect(world[:, :2].astype(np.float32))
                measured = sorted(map(float, rect[1]), reverse=True)
                if measured[1] <= 1e-5:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = 'degenerate planar component'
                    output.append(ContainerCandidate(item, contour))
                    continue
                dimension_error = rectangular_model_error(
                    tuple(measured), (self.geometry.depth, self.geometry.width))
                cube_error = rectangular_model_error(
                    tuple(measured), (self.cube_size_m, self.cube_size_m))
                hole = None
                if hierarchy is not None:
                    child = int(hierarchy[0, index, 2])
                    while child >= 0:
                        if hole is None or cv2.contourArea(contours[child]) > cv2.contourArea(hole):
                            hole = contours[child]
                        child = int(hierarchy[0, child, 0])
                if hole is None:
                    if (cube_error + self.cube_model_margin < dimension_error
                            and not self._touches_border(contour, mask.shape)):
                        item.pose.position.x = float(rect[0][0])
                        item.pose.position.y = float(rect[0][1])
                        item.pose.position.z = top_height
                        item.pose_valid = True
                        item.rejection_code = item.REJECTION_CUBE_LIKE
                        item.rejection_detail = '42 mm cube, not an open container'
                    else:
                        item.rejection_code = (
                            item.REJECTION_PARTIAL_INSUFFICIENT
                            if self._touches_border(contour, mask.shape)
                            else item.REJECTION_NO_OPENING)
                        item.rejection_detail = 'no observable opening boundary'
                    output.append(ContainerCandidate(item, contour))
                    continue
                opening_world = pixels_to_plane(
                    cv2.convexHull(hole).reshape(-1, 2), matrix, transform, top_height)
                if opening_world is None or len(opening_world) < 3:
                    item.rejection_code = item.REJECTION_NO_OPENING
                    item.rejection_detail = 'opening boundary cannot meet top plane'
                    output.append(ContainerCandidate(item, contour))
                    continue
                opening_rect = cv2.minAreaRect(opening_world[:, :2].astype(np.float32))
                center = tuple(map(float, opening_rect[0]))
                opening_error = rectangular_model_error(
                    opening_rect[1],
                    (self.geometry.inner_depth, self.geometry.inner_width))
                box = cv2.boxPoints(opening_rect)
                edge = box[1] - box[0]
                yaw = math.atan2(float(edge[1]), float(edge[0]))
                edge_length = float(np.linalg.norm(edge))
                if (abs(edge_length - self.geometry.inner_depth)
                        > abs(edge_length - self.geometry.inner_width)):
                    yaw += math.pi / 2.0
                yaw %= math.pi
                opening_fraction = min(
                    1.0, cv2.contourArea(hole) / max(
                        cv2.contourArea(contour)
                        * (self.geometry.inner_depth*self.geometry.inner_width)
                        / (self.geometry.depth*self.geometry.width), 1.0))
                projection = project_open_container(
                    center, yaw, table_height, self.geometry, matrix, transform,
                    mask.shape, self.rim_segmentable_fraction,
                    self.side_segmentable_fraction)
                if projection is None:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = 'container model projects behind camera'
                    output.append(ContainerCandidate(item, contour))
                    continue
                observed = float(np.count_nonzero(cv2.bitwise_and(
                    mask, projection.silhouette_mask)))
                area_ratio = observed / projection.expected_segmentable_area
                rim_pixels = max(1, int(np.count_nonzero(projection.rim_mask)))
                rim_support = float(np.count_nonzero(cv2.bitwise_and(
                    mask, projection.rim_mask))) / rim_pixels
                inner_pixels = max(1, int(np.count_nonzero(projection.inner_mask)))
                inner_colored = float(np.count_nonzero(cv2.bitwise_and(
                    mask, projection.inner_mask)))
                opening_score = opening_evidence_score(
                    inner_pixels, inner_colored, self.cube_size_m,
                    self.geometry.inner_depth, self.geometry.inner_width,
                    self.max_opening_cubes)
                opening_score = min(
                    opening_score,
                    max(0.0, 1.0 - opening_error / self.max_dimension_relative_error),
                    opening_fraction / self.minimum_observable_opening_fraction)
                partial = projection.visible_fraction < 0.999 or self._touches_border(
                    contour, mask.shape)
                item.rectangularity = area / max(float(cv2.contourArea(
                    cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32))), 1.0)
                item.visible_fraction = projection.visible_fraction
                item.expected_segmentable_area_px = projection.expected_segmentable_area
                item.observed_segmentable_area_px = observed
                item.area_ratio = area_ratio
                item.opening_score = opening_score
                item.partial = partial
                item.position_uncertainty_m = (
                    0.005 + 0.04 * (1.0 - projection.visible_fraction)
                    + 0.02 * max(dimension_error, opening_error))
                item.yaw_uncertainty_deg = math.degrees(
                    min(math.pi/2, 0.05 + max(dimension_error, opening_error) * 0.5
                        + (1.0 - projection.visible_fraction) * 0.4))
                geometry_score = max(0.0, 1.0 - dimension_error /
                                     self.max_dimension_relative_error)
                area_score = min(
                    1.0,
                    area_ratio / self.min_area_ratio,
                    self.max_area_ratio / max(area_ratio, 1e-9),
                )
                visible_score = min(1.0, projection.visible_fraction /
                                    self.min_visible_fraction)
                rim_score = min(1.0, rim_support / self.min_rim_support)
                opening_normalized = min(1.0, opening_score / self.min_opening_score)
                item.confidence = max(0.0, min(
                    geometry_score, area_score, visible_score,
                    rim_score, opening_normalized))
                item.pose.position.x, item.pose.position.y = center
                item.pose.position.z = top_height
                (item.pose.orientation.x, item.pose.orientation.y,
                 item.pose.orientation.z, item.pose.orientation.w) = yaw_quaternion(yaw)
                item.pose_valid = True
                item.pose_error = dimension_error
                item.observation_count = 1
                if cube_error + self.cube_model_margin < dimension_error:
                    item.rejection_code = item.REJECTION_CUBE_LIKE
                    item.rejection_detail = '42 mm cube model fits better than container model'
                elif dimension_error > self.max_dimension_relative_error:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = f'dimension relative error {dimension_error:.3f}'
                elif not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                    item.rejection_code = item.REJECTION_AREA_MISMATCH
                    item.rejection_detail = f'segmentable area ratio {area_ratio:.3f}'
                elif rim_support < self.min_rim_support or opening_score < self.min_opening_score:
                    item.rejection_code = item.REJECTION_NO_OPENING
                    item.rejection_detail = (
                        f'rim={rim_support:.3f}, opening={opening_score:.3f}')
                elif partial and projection.visible_fraction < self.min_visible_fraction:
                    item.rejection_code = item.REJECTION_PARTIAL_INSUFFICIENT
                    item.rejection_detail = f'visible fraction {projection.visible_fraction:.3f}'
                elif partial and projection.visible_fraction < self.partial_ambiguity_visible_fraction:
                    item.rejection_code = item.REJECTION_POSE_AMBIGUOUS
                    item.rejection_detail = 'multiple centers remain plausible from clipped evidence'
                elif item.confidence < self.min_confidence:
                    item.rejection_code = item.REJECTION_GEOMETRY
                    item.rejection_detail = f'confidence {item.confidence:.3f}'
                output.append(ContainerCandidate(
                    item, contour, projection.outer, projection.inner))
        return output

    @staticmethod
    def _touches_border(contour, shape, margin=2):
        points = contour.reshape(-1, 2)
        height, width = shape
        return bool(np.any(
            (points[:, 0] <= margin) | (points[:, 1] <= margin)
            | (points[:, 0] >= width - 1 - margin)
            | (points[:, 1] >= height - 1 - margin)))

    def _publish_compatibility_topics(self, image, camera_frame, tags, containers):
        tag_array = AprilTagDetectionArray()
        tag_array.header = copy.deepcopy(image.header)
        tag_array.header.frame_id = self.output_frame
        poses = PoseArray(header=tag_array.header)
        for item in tags:
            tag_array.detections.append(AprilTagDetection(
                family=item.family, id=item.id,
                decision_margin=item.decision_margin, hamming=item.hamming,
                pose_error=item.pose_error, pose=item.pose))
            poses.poses.append(item.pose)
        self.tag_publisher.publish(tag_array)
        self.tag_pose_publisher.publish(poses)
        container_array = ContainerDetectionArray()
        container_array.header = copy.deepcopy(image.header)
        container_array.header.frame_id = self.output_frame
        for item in containers:
            container_array.detections.append(ContainerDetection(
                color=item.color, contour_area_px=item.contour_area_px,
                rectangularity=item.rectangularity,
                pose_error=item.pose_error, pose=item.pose))
        self.container_publisher.publish(container_array)

    @staticmethod
    def image_to_bgr(message: Image) -> np.ndarray:
        height, width, step = int(message.height), int(message.width), int(message.step)
        if min(height, width, step) <= 0:
            raise ValueError('invalid image shape')
        data = np.frombuffer(message.data, dtype=np.uint8)
        if data.size < height * step:
            raise ValueError('image payload is shorter than declared step')
        rows = data[:height*step].reshape(height, step)
        encoding = message.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            return cv2.cvtColor(rows[:, :width], cv2.COLOR_GRAY2BGR)
        conversions = {
            'bgr8': (3, None), 'rgb8': (3, cv2.COLOR_RGB2BGR),
            'bgra8': (4, cv2.COLOR_BGRA2BGR),
            'rgba8': (4, cv2.COLOR_RGBA2BGR),
        }
        if encoding not in conversions:
            raise ValueError(f'unsupported encoding {message.encoding}')
        channels, code = conversions[encoding]
        value = rows[:, :width*channels].reshape(height, width, channels)
        return value.copy() if code is None else cv2.cvtColor(value, code)

    def _publish_tag_debug(self, source, bgr, accepted, rejected):
        debug = bgr.copy()
        cv2.putText(debug, f'accepted={len(accepted)} rejected={len(rejected)}',
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        self.tag_debug_publisher.publish(self._image_message(source, debug))

    def _publish_container_debug(self, source, bgr, masks, candidates):
        debug = bgr.copy()
        tint = np.zeros_like(debug)
        for color, mask in masks.items():
            tint[mask > 0] = DEBUG_COLORS[color]
        debug = cv2.addWeighted(debug, 0.78, tint, 0.22, 0.0)
        for candidate in candidates:
            item = candidate.detection
            accepted = item.rejection_code == item.REJECTION_NONE
            color = (0, 220, 0) if accepted else (0, 165, 255)
            cv2.drawContours(debug, [candidate.contour], -1, color, 1)
            if candidate.outer_projection is not None:
                cv2.polylines(debug, [np.rint(candidate.outer_projection).astype(np.int32)],
                              True, color, 2)
            if candidate.inner_projection is not None:
                cv2.polylines(debug, [np.rint(candidate.inner_projection).astype(np.int32)],
                              True, (255, 255, 255), 1)
            x, y, w, h = cv2.boundingRect(candidate.contour)
            label = (f'{COLOR_NAMES[item.color]} c={item.confidence:.2f} '
                     f'A={item.observed_segmentable_area_px:.0f}/'
                     f'{item.expected_segmentable_area_px:.0f} '
                     f'{item.rejection_detail or "ok"}')
            cv2.putText(debug, label, (x, max(14, y-4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, color, 1)
        self.container_debug_publisher.publish(self._image_message(source, debug))

    @staticmethod
    def _image_message(source, image):
        output = Image()
        output.header = copy.deepcopy(source.header)
        output.height, output.width = image.shape[:2]
        output.encoding = 'bgr8'
        output.step = output.width * 3
        output.data = image.tobytes()
        return output

    def _call_bool_service(self, client, enabled: bool, timeout: float,
                           camera: bool = False) -> bool:
        if client is None:
            return True
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        future = client.call_async(SetBool.Request(data=enabled))
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout):
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return _capture_response_ok(response, enabled) if camera else bool(
            response is not None and response.success)

    def _acquire_hardware(self) -> bool:
        with self.hardware_lock:
            with self.lock:
                self.hardware_release_at = None
                if self.hardware_active:
                    return True
            led_ok = (not self.manage_led or self._call_bool_service(
                self.led_client, True, self.led_timeout))
            if not led_ok:
                return False
            camera_ok = (not self.manage_camera or self._call_bool_service(
                self.camera_client, True, self.camera_timeout, camera=True))
            if not camera_ok:
                if self.manage_led:
                    self._call_bool_service(self.led_client, False, self.led_timeout)
                return False
            with self.lock:
                self.hardware_active = True
            return True

    def _hardware_idle_tick(self) -> None:
        with self.lock:
            deadline = self.hardware_release_at
            idle = self.session is None and not self.reserved
        if deadline is None or not idle or time.monotonic() < deadline:
            return
        with self.hardware_lock:
            with self.lock:
                if (self.session is not None or self.reserved
                        or self.hardware_release_at != deadline):
                    return
            if self.manage_camera:
                self._call_bool_service(
                    self.camera_client, False, self.camera_timeout, camera=True)
            if self.manage_led:
                self._call_bool_service(self.led_client, False, self.led_timeout)
            with self.lock:
                self.hardware_active = False
                self.hardware_release_at = None

    def destroy_node(self):
        with self.lock:
            active = self.hardware_active
            self.session = None
            self.reserved = False
        if active:
            if self.manage_camera:
                self._call_bool_service(
                    self.camera_client, False, self.camera_timeout, camera=True)
            if self.manage_led:
                self._call_bool_service(self.led_client, False, self.led_timeout)
        return super().destroy_node()


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneAnalyzer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
