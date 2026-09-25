"""On-demand, single-owner AprilTag and container scene analysis."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
import os
import threading
import time
from typing import Iterable

import cv2
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from interfaces.action import AnalyzeScene
from interfaces.msg import (
    AprilTagDetection,
    AprilTagDetectionArray,
    AprilTagStampedDetection,
    ContainerDetectionArray,
    ContainerStampedDetection,
    TableSurfaceGrid,
)
import numpy as np
from pupil_apriltags import Detector
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .constants import APRILTAGS, CONTAINERS_HSV, TABLE_SURFACE
from .container_pipeline import ContainerPipelineMixin
from .debug_images import ContainerDebugFrame, DebugImagesMixin
from .geometry import rotation_from_quaternion
from .image_encoding import ImageEncodingMixin
from .table_surface import TableSurfaceMixin
from .hsv_container import (
    area_thresholds_for_height, detect_blobs, pixel_on_base_plane,
    PixelObservation, PixelTrack, update_tracks,
)


def quaternion_from_rotation(matrix: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        return ((matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale)
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        scale = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        return (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale)
    if index == 1:
        scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        return ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale)
    scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    return ((matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale,
            0.25 * scale, (matrix[1, 0] - matrix[0, 1]) / scale)


class NativeWarningFilter:
    """Filter exactly one known apriltag C warning while preserving stderr."""

    _needle = b'Error, more than one new minima found.'

    def __init__(self) -> None:
        self._original = os.dup(2)
        self._read, write_fd = os.pipe()
        os.dup2(write_fd, 2)
        os.close(write_fd)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='stderr-filter', daemon=True)
        self._thread.start()

    def _run(self) -> None:
        pending = b''
        while not self._stop.is_set():
            try:
                chunk = os.read(self._read, 4096)
            except OSError:
                break
            if not chunk:
                break
            pending += chunk
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                if self._needle not in line:
                    try:
                        os.write(self._original, line + b'\n')
                    except OSError:
                        return
        if pending and self._needle not in pending:
            try:
                os.write(self._original, pending)
            except OSError:
                pass

    def close(self) -> None:
        self._stop.set()
        try:
            os.dup2(self._original, 2)
            os.close(self._read)
        except OSError:
            pass
        self._thread.join(timeout=1.0)
        os.close(self._original)


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _ros_duration(seconds: float):
    from builtin_interfaces.msg import Duration as DurationMsg
    seconds = max(0.0, float(seconds))
    msg = DurationMsg()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - msg.sec) * 1e9)
    return msg


def _capture_request_succeeded(response, target: bool) -> bool:
    """Normalize the non-standard SetBool response used by usb_cam 0.8.x.

    That driver leaves ``success`` false and reports the completed operation
    only through ``Start Capturing`` or ``Stop Capturing``.  Keep the exception
    deliberately narrow so genuine failures are still reported and retried.
    """
    if response is None:
        return False
    if response.success:
        return True
    expected = 'start capturing' if target else 'stop capturing'
    return response.message.strip().casefold() == expected


@dataclass
class Session:
    goal_handle: object
    duration: float
    requested_detectors: int
    work_surface_height_m: float = 0.0
    started: float = field(default_factory=time.monotonic)
    frames_processed: int = 0
    frames_with_base_transform: int = 0
    best_camera: dict[int, AprilTagStampedDetection] = field(default_factory=dict)
    best_base: dict[int, AprilTagStampedDetection] = field(default_factory=dict)
    latest_camera: list[AprilTagStampedDetection] = field(default_factory=list)
    latest_base: list[AprilTagStampedDetection] = field(default_factory=list)
    hsv_container_tracks: list[PixelTrack] = field(default_factory=list)
    latest_containers_camera: list[ContainerStampedDetection] = field(default_factory=list)
    latest_containers_base: list[ContainerStampedDetection] = field(default_factory=list)
    containers_ready_at: float = math.inf
    last_feedback: float = 0.0
    last_base_transform: TransformStamped | None = None
    recent_frame_times: list[float] = field(default_factory=list)
    recent_detector_times: dict[int, list[float]] = field(default_factory=dict)
    last_detector_times: dict[int, float] = field(default_factory=dict)
    detector_frame_counts: dict[int, int] = field(default_factory=dict)
    detector_frames_with_base_transform: dict[int, int] = field(
        default_factory=dict)
    detector_first_frame_times: dict[int, float] = field(default_factory=dict)
    detector_last_frame_times: dict[int, float] = field(default_factory=dict)
    last_debug_publish_times: dict[int, float] = field(default_factory=dict)
    latest_debug_frame: 'ContainerDebugFrame | None' = None
    latest_debug_frames: dict[int, 'ContainerDebugFrame'] = field(
        default_factory=dict)
    table_search_x_min_m: float = 0.0
    table_search_y_min_m: float = 0.0
    table_grid_resolution_m: float = 0.0
    table_grid_width: int = 0
    table_grid_height: int = 0
    table_cell_observations: list[int] = field(default_factory=list)
    table_cell_confirmations: list[int] = field(default_factory=list)


class SceneAnalyzer(
        ContainerPipelineMixin, TableSurfaceMixin, DebugImagesMixin,
        ImageEncodingMixin, Node):
    def __init__(self) -> None:
        super().__init__('scene_analyzer')
        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('floor_frame', 'base_link')
        self.declare_parameter(
            'container_target_topic', '/manipulation/container_release_target')
        self.declare_parameter('tag_frame_prefix', 'apriltag')
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('tag_size_m', 0.032)
        self.declare_parameter('nthreads', 1)
        self.declare_parameter('quad_decimate', 1.0)
        self.declare_parameter('max_detection_rate_hz', 10.0)
        self.declare_parameter('apriltag_detection_rate_hz', 0.0)
        self.declare_parameter('hsv_container_detection_rate_hz', 0.0)
        self.declare_parameter('table_surface_detection_rate_hz', 0.0)
        self.declare_parameter('debug_image_rate_hz', 5.0)
        self.declare_parameter('opencv_threads', 0)
        self.declare_parameter('min_decision_margin', 30.0)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('feedback_rate_hz', 5.0)
        self.declare_parameter('suppress_native_pose_warning', True)
        self.declare_parameter('manage_camera_capture', True)
        self.declare_parameter(
            'camera_capture_service', '/camera/set_capture')
        self.declare_parameter('camera_capture_timeout_sec', 5.0)
        self.declare_parameter('camera_idle_timeout_sec', 0.0)
        self.declare_parameter('camera_capture_retry_sec', 1.0)
        self.declare_parameter('manage_vision_led', True)
        self.declare_parameter(
            'vision_led_service', '/base_hardware/set_vision_led')
        self.declare_parameter('vision_led_timeout_sec', 5.0)
        self.declare_parameter('external_height_m', 0.073)
        self.declare_parameter('min_saturation', 80)
        self.declare_parameter('min_value', 45)
        self.declare_parameter('red_hue_low_1', 0)
        self.declare_parameter('red_hue_high_1', 12)
        self.declare_parameter('red_hue_low_2', 168)
        self.declare_parameter('red_hue_high_2', 179)
        self.declare_parameter('blue_hue_low', 92)
        self.declare_parameter('blue_hue_high', 138)
        self.declare_parameter('morphology_kernel_px', 5)
        self.declare_parameter('container_border_margin_px', 6)
        self.declare_parameter('max_contour_area_fraction', 0.85)
        self.declare_parameter('container_warmup_sec', 0.5)
        self.declare_parameter('hsv_container_min_area_le_7_5cm_px', 4000)
        self.declare_parameter('hsv_container_min_area_le_12_5cm_px', 6500)
        self.declare_parameter('hsv_container_min_area_gt_12_5cm_px', 9000)
        self.declare_parameter('hsv_container_min_partial_area_le_5cm_px', 1800)
        self.declare_parameter('hsv_container_min_partial_area_le_10cm_px', 2500)
        self.declare_parameter('hsv_container_min_partial_area_gt_10cm_px', 3500)
        self.declare_parameter('hsv_container_min_confirmed_frames', 3)
        self.declare_parameter('hsv_container_center_tolerance_px', 12.0)
        self.declare_parameter('white_surface_max_saturation', 45)
        self.declare_parameter('white_surface_min_value', 40)
        self.declare_parameter('white_surface_max_value', 250)
        self.declare_parameter('white_surface_min_fraction', 0.88)
        self.declare_parameter('white_surface_max_unknown_fraction', 0.12)
        self.declare_parameter('white_surface_min_confirmed_frames', 2)
        self.declare_parameter('white_surface_min_confirmed_ratio', 0.60)

        self.warning_filter = (NativeWarningFilter()
                               if bool(self.get_parameter('suppress_native_pose_warning').value)
                               else None)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.floor_frame = str(self.get_parameter('floor_frame').value)
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_hamming = int(self.get_parameter('max_hamming').value)
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)
        fallback_detection_rate = float(
            self.get_parameter('max_detection_rate_hz').value)
        if (not math.isfinite(fallback_detection_rate)
                or fallback_detection_rate <= 0.0):
            raise ValueError(
                'max_detection_rate_hz must be positive and finite.')
        self.detector_periods = {}
        for detector, parameter in (
            (APRILTAGS, 'apriltag_detection_rate_hz'),
            (CONTAINERS_HSV, 'hsv_container_detection_rate_hz'),
            (TABLE_SURFACE, 'table_surface_detection_rate_hz'),
        ):
            rate = float(self.get_parameter(parameter).value)
            if rate == 0.0:
                rate = fallback_detection_rate
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError(
                    f'{parameter} must be nonnegative and finite; '
                    'zero selects max_detection_rate_hz.')
            self.detector_periods[detector] = 1.0 / rate
        debug_rate = float(self.get_parameter('debug_image_rate_hz').value)
        if not math.isfinite(debug_rate) or debug_rate < 0.0:
            raise ValueError(
                'debug_image_rate_hz must be nonnegative and finite.')
        self.debug_image_period = (
            1.0 / debug_rate if debug_rate > 0.0 else math.inf)
        opencv_threads = int(self.get_parameter('opencv_threads').value)
        if opencv_threads < 0:
            raise ValueError('opencv_threads must be nonnegative.')
        if opencv_threads:
            cv2.setNumThreads(opencv_threads)
        self.feedback_period = 1.0 / max(0.1, float(self.get_parameter('feedback_rate_hz').value))
        self.manage_camera_capture = bool(
            self.get_parameter('manage_camera_capture').value)
        self.camera_capture_timeout = max(
            0.1, float(self.get_parameter('camera_capture_timeout_sec').value))
        self.camera_capture_retry = max(
            0.1, float(self.get_parameter('camera_capture_retry_sec').value))
        self.camera_info: CameraInfo | None = None
        self.camera_info_subscription = None
        self.latest_image_subscription = None
        self.tf_buffer = None
        self.tf_listener = None
        self.sessions_lock = threading.RLock()
        self.session: Session | None = None
        # Keep capture responsive even when one inference takes hundreds of
        # milliseconds. Each detector retains only its newest pending frame,
        # bounding latency/memory without coupling detector throughput.
        self.image_condition = threading.Condition()
        self.pending_images: dict[int, Image | None] = {
            APRILTAGS: None,
            CONTAINERS_HSV: None,
            TABLE_SURFACE: None,
        }
        self.image_worker_stopping = False
        self.image_workers = [
            threading.Thread(
                target=self._image_worker_loop,
                args=(detector,),
                name=f'scene-analysis-{name}',
                daemon=True,
            )
            for detector, name in (
                (APRILTAGS, 'apriltags'),
                (CONTAINERS_HSV, 'containers-hsv'),
                (TABLE_SURFACE, 'table-surface'),
            )
        ]
        self.state = 'idle'
        # Entity creation/destruction must run in an executor callback.  Action
        # workers only request teardown through this guard condition, avoiding
        # rclpy wait-set races with destroyed subscriptions.
        self.input_lifecycle_guard = self.create_guard_condition(
            self._deactivate_inputs_from_executor)
        self.apriltag_detector = Detector(
            families=self.get_parameter('family').value,
            nthreads=int(self.get_parameter('nthreads').value),
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            refine_edges=1)
        self._configure_hsv_container_detector()
        self._configure_white_surface_detector()
        self.tf_broadcaster = TransformBroadcaster(self)
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.camera_pose_publisher = self.create_publisher(
            PoseArray, 'apriltags/poses_camera', output_qos)
        self.pose_publisher = self.create_publisher(
            PoseArray, 'apriltags/poses', output_qos)
        self.camera_detection_publisher = self.create_publisher(
            AprilTagDetectionArray, 'apriltags/detections_camera', output_qos)
        self.detection_publisher = self.create_publisher(
            AprilTagDetectionArray, 'apriltags/detections', output_qos)
        self.debug_image_publisher = self.create_publisher(
            Image, 'apriltags/debug_image', debug_qos)
        self.container_camera_detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections_camera', output_qos)
        self.container_detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections', output_qos)
        self.container_debug_image_publisher = self.create_publisher(
            Image, 'containers/debug_image', debug_qos)
        self.table_surface_debug_image_publisher = self.create_publisher(
            Image, 'table_surface/debug_image', debug_qos)
        self.latest_container_debug_frame: ContainerDebugFrame | None = None
        self.container_target_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('container_target_topic').value),
            self.container_target_callback,
            output_qos,
        )
        self.capture_condition = threading.Condition(threading.RLock())
        self.capture_state: bool | None = None
        self.capture_future = None
        self.capture_target: bool | None = None
        self.next_capture_attempt = 0.0
        self.capture_client = None
        self.camera_idle_timer = None
        self.manage_vision_led = bool(
            self.get_parameter('manage_vision_led').value)
        self.vision_led_timeout = max(
            0.1, float(self.get_parameter('vision_led_timeout_sec').value))
        self.vision_led_client = None
        if self.manage_vision_led:
            self.vision_led_service = str(
                self.get_parameter('vision_led_service').value)
            self.vision_led_client = self.create_client(
                SetBool, self.vision_led_service)
        if self.manage_camera_capture:
            self.camera_capture_service = str(
                self.get_parameter('camera_capture_service').value)
            self.capture_client = self.create_client(
                SetBool,
                self.camera_capture_service,
            )
            self._schedule_camera_stop()
        self.action_server = ActionServer(self, AnalyzeScene, 'vision/analyze_scene',
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          handle_accepted_callback=self.handle_accepted_callback)
        for worker in self.image_workers:
            worker.start()
        self.get_logger().info(
            'Scene analyzer idle; waiting for /vision/analyze_scene goals.')


    def _configure_white_surface_detector(self) -> None:
        """Validate the conservative white-table classification thresholds."""
        self.white_surface_max_saturation = int(
            self.get_parameter('white_surface_max_saturation').value)
        self.white_surface_min_value = int(
            self.get_parameter('white_surface_min_value').value)
        self.white_surface_max_value = int(
            self.get_parameter('white_surface_max_value').value)
        self.white_surface_min_fraction = float(
            self.get_parameter('white_surface_min_fraction').value)
        self.white_surface_max_unknown_fraction = float(
            self.get_parameter('white_surface_max_unknown_fraction').value)
        self.white_surface_min_confirmed_frames = int(
            self.get_parameter('white_surface_min_confirmed_frames').value)
        self.white_surface_min_confirmed_ratio = float(
            self.get_parameter('white_surface_min_confirmed_ratio').value)
        if not 0 <= self.white_surface_max_saturation <= 255:
            raise ValueError('white_surface_max_saturation must be in [0, 255]')
        if not (
            0 <= self.white_surface_min_value
            < self.white_surface_max_value <= 255
        ):
            raise ValueError(
                'white surface value limits must satisfy 0 <= min < max <= 255')
        for name, value in (
            ('white_surface_min_fraction', self.white_surface_min_fraction),
            ('white_surface_max_unknown_fraction',
             self.white_surface_max_unknown_fraction),
            ('white_surface_min_confirmed_ratio',
             self.white_surface_min_confirmed_ratio),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        if self.white_surface_min_confirmed_frames <= 0:
            raise ValueError(
                'white_surface_min_confirmed_frames must be positive')

    def destroy_node(self):
        with self.sessions_lock:
            self.session = None
            self._destroy_inputs_locked()
        with self.image_condition:
            self.image_worker_stopping = True
            for detector in self.pending_images:
                self.pending_images[detector] = None
            self.image_condition.notify_all()
        for worker in self.image_workers:
            worker.join(timeout=2.0)
        if self.warning_filter is not None:
            self.warning_filter.close()
            self.warning_filter = None
        return super().destroy_node()

    def goal_callback(self, goal_request) -> GoalResponse:
        seconds = _duration_seconds(goal_request.duration)
        if seconds < 0.0:
            self.get_logger().warning('Rejecting scene goal with negative duration.')
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.work_surface_height_m)):
            self.get_logger().warning('Rejecting non-finite work surface height.')
            return GoalResponse.REJECT
        requested = int(goal_request.requested_detectors)
        known = APRILTAGS | TABLE_SURFACE | CONTAINERS_HSV
        if requested == 0 or requested & ~known:
            self.get_logger().warning(
                f'Rejecting scene goal with invalid detector mask: {requested}.')
            return GoalResponse.REJECT
        if requested & TABLE_SURFACE:
            x_min = float(goal_request.table_search_x_min_m)
            x_max = float(goal_request.table_search_x_max_m)
            y_min = float(goal_request.table_search_y_min_m)
            y_max = float(goal_request.table_search_y_max_m)
            resolution = float(goal_request.table_grid_resolution_m)
            if (
                not all(math.isfinite(value) for value in (
                    x_min, x_max, y_min, y_max, resolution))
                or x_min > x_max
                or y_min > y_max
                or resolution <= 0.0
            ):
                self.get_logger().warning(
                    'Rejecting invalid white-table grid geometry.')
                return GoalResponse.REJECT
            grid_width = math.ceil((x_max - x_min) / resolution - 1e-9) + 1
            grid_height = math.ceil((y_max - y_min) / resolution - 1e-9) + 1
            if grid_width * grid_height > 250_000:
                self.get_logger().warning(
                    'Rejecting white-table grid with more than 250000 cells.')
                return GoalResponse.REJECT
        with self.sessions_lock:
            if self.state != 'idle':
                self.get_logger().warning(
                    'Rejecting scene goal: another analysis is active.')
                return GoalResponse.REJECT
            try:
                self._create_inputs_locked()
            except Exception as error:
                self.get_logger().error(
                    f'Could not activate AprilTag inputs: {error}')
                self._destroy_inputs_locked()
                return GoalResponse.REJECT
            self.state = 'activating'
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        """Run each accepted goal outside the ROS executor worker pool."""
        threading.Thread(target=self.execute_callback, args=(goal_handle,),
                         name='scene-analysis-goal', daemon=True).start()

    def _create_inputs_locked(self) -> None:
        """Create camera/TF inputs from an executor callback before a goal runs."""
        if self.latest_image_subscription is not None:
            return
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.latest_image_subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback, qos)

    def _destroy_inputs_locked(self) -> None:
        """Destroy input entities from the executor or final node teardown."""
        if self.latest_image_subscription is not None:
            self.destroy_subscription(self.latest_image_subscription)
            self.latest_image_subscription = None
        if self.camera_info_subscription is not None:
            self.destroy_subscription(self.camera_info_subscription)
            self.camera_info_subscription = None
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None
        self.tf_buffer = None
        self.camera_info = None

    def _deactivate_inputs_from_executor(self) -> None:
        """Finish an action's teardown on the executor thread."""
        with self.sessions_lock:
            if self.state != 'deactivating':
                return
            self._destroy_inputs_locked()
            self.state = 'idle'
        self._schedule_camera_stop()
        self.get_logger().info('Scene analyzer idle.')

    def _begin_capture_request(self, enabled: bool) -> bool:
        if not self.manage_camera_capture or self.capture_client is None:
            return True
        with self.capture_condition:
            if self.capture_future is not None:
                return False
            if self.capture_state is enabled:
                return True
            if time.monotonic() < self.next_capture_attempt:
                return False
            if not self.capture_client.service_is_ready():
                return False
            request = SetBool.Request()
            request.data = enabled
            self.capture_target = enabled
            self.capture_future = self.capture_client.call_async(request)
            self.capture_future.add_done_callback(self._capture_response)
            return False

    def _capture_response(self, future) -> None:
        with self.capture_condition:
            target = self.capture_target
            try:
                response = future.result()
                if target is not None and _capture_request_succeeded(response, target):
                    self.capture_state = target
                    self.next_capture_attempt = 0.0
                else:
                    self.next_capture_attempt = (
                        time.monotonic() + self.camera_capture_retry)
                    message = response.message if response is not None else 'sem resposta'
                    self.get_logger().warning(
                        f'Não foi possível alterar a captura da câmera: {message}',
                        throttle_duration_sec=5.0)
            except Exception as error:
                self.next_capture_attempt = (
                    time.monotonic() + self.camera_capture_retry)
                self.get_logger().warning(
                    f'Falha no serviço de captura da câmera: {error}',
                    throttle_duration_sec=5.0)
            finally:
                self.capture_future = None
                self.capture_target = None
                self.capture_condition.notify_all()

    def _wait_for_camera_capture(self) -> bool:
        if not self.manage_camera_capture:
            return True
        deadline = time.monotonic() + self.camera_capture_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._begin_capture_request(True):
                return True
            with self.capture_condition:
                self.capture_condition.wait(timeout=0.05)
        self.get_logger().error(
            f'A câmera não respondeu pelo serviço '
            f'{self.camera_capture_service} em '
            f'{self.camera_capture_timeout:.1f} s.')
        return False

    def _schedule_camera_stop(self) -> None:
        """Stop USB capture without leaving an idle timer behind."""
        if not self.manage_camera_capture:
            return
        if self.camera_idle_timer is None:
            self.camera_idle_timer = self.create_timer(
                0.25, self._stop_camera_when_idle)

    def _set_vision_led(self, enabled: bool) -> bool:
        """Liga ou desliga a iluminação através do dono da serial do brick."""
        if not self.manage_vision_led:
            return True
        if self.vision_led_client is None or not self.vision_led_client.wait_for_service(
            timeout_sec=self.vision_led_timeout
        ):
            self.get_logger().error(
                f'O serviço de iluminação {self.vision_led_service} não está disponível.')
            return False

        request = SetBool.Request()
        request.data = enabled
        future = self.vision_led_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=self.vision_led_timeout):
            self.get_logger().error(
                f'O serviço de iluminação {self.vision_led_service} não respondeu.')
            return False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Falha ao chamar o serviço de iluminação: {error}')
            return False
        if response is None or not response.success:
            message = response.message if response is not None else 'sem resposta'
            self.get_logger().error(
                f'Não foi possível alterar o LED de visão: {message}')
            return False
        return True

    def _stop_camera_when_idle(self) -> None:
        with self.sessions_lock:
            idle = self.state == 'idle'
        if not idle:
            return
        stopped = self._begin_capture_request(False)
        with self.capture_condition:
            stop_confirmed = self.capture_state is False
        if stopped or stop_confirmed:
            timer = self.camera_idle_timer
            self.camera_idle_timer = None
            if timer is not None:
                self.destroy_timer(timer)

    def execute_callback(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            goal_handle.executing()
        duration = _duration_seconds(goal_handle.request.duration)
        resolution = float(goal_handle.request.table_grid_resolution_m)
        x_min = float(goal_handle.request.table_search_x_min_m)
        x_max = float(goal_handle.request.table_search_x_max_m)
        y_min = float(goal_handle.request.table_search_y_min_m)
        y_max = float(goal_handle.request.table_search_y_max_m)
        table_requested = bool(
            int(goal_handle.request.requested_detectors) & TABLE_SURFACE)
        grid_width = (
            math.ceil((x_max - x_min) / resolution - 1e-9) + 1
            if table_requested else 0)
        grid_height = (
            math.ceil((y_max - y_min) / resolution - 1e-9) + 1
            if table_requested else 0)
        session = Session(
            goal_handle=goal_handle,
            duration=duration,
            requested_detectors=int(goal_handle.request.requested_detectors),
            work_surface_height_m=float(
                goal_handle.request.work_surface_height_m),
            table_search_x_min_m=x_min if table_requested else 0.0,
            table_search_y_min_m=y_min if table_requested else 0.0,
            table_grid_resolution_m=resolution if table_requested else 0.0,
            table_grid_width=grid_width,
            table_grid_height=grid_height,
        )
        cell_count = grid_width * grid_height
        session.table_cell_observations = [0] * cell_count
        session.table_cell_confirmations = [0] * cell_count
        with self.sessions_lock:
            self.session = session
            self.state = 'analyzing'
        with self.image_condition:
            for detector in self.pending_images:
                self.pending_images[detector] = None
        vision_led_enabled = False
        try:
            if not self._set_vision_led(True):
                result = self._result(
                    session, 'A iluminação da câmera não pôde ser ligada.')
                goal_handle.abort(result)
                return result
            vision_led_enabled = self.manage_vision_led
            if not self._wait_for_camera_capture():
                result = self._result(
                    session, 'A câmera não iniciou dentro do tempo limite.')
                goal_handle.abort(result)
                return result
            now = time.monotonic()
            session.containers_ready_at = (
                now + self.container_warmup
                if session.requested_detectors & (CONTAINERS_HSV | TABLE_SURFACE)
                else now
            )
            # The requested analysis duration starts after exposure/white-balance
            # stabilization, rather than consuming the useful observation window.
            session.started = session.containers_ready_at
            while rclpy.ok() and goal_handle.is_active:
                time.sleep(0.02)
                elapsed = max(0.0, time.monotonic() - session.started)
                if goal_handle.is_cancel_requested:
                    result = self._finish_session(
                        session,
                        'Canceled; returning accumulated detections.',
                        'CANCELED',
                    )
                    goal_handle.canceled(result)
                    return result
                if duration > 0.0 and elapsed >= duration:
                    if session.frames_processed == 0:
                        result = self._finish_session(
                            session,
                            'No calibrated image was processed during the '
                            'requested window.',
                            'ABORTED',
                        )
                        goal_handle.abort(result)
                        return result
                    result = self._finish_session(
                        session, 'Analysis completed.', 'FINAL')
                    goal_handle.succeed(result)
                    return result
                now = time.monotonic()
                if now - session.last_feedback >= self.feedback_period:
                    session.last_feedback = now
                    goal_handle.publish_feedback(self._feedback(session))
        finally:
            with self.sessions_lock:
                if self.session is session:
                    self.session = None
                self.state = 'deactivating'
            if vision_led_enabled:
                self._set_vision_led(False)
            self.input_lifecycle_guard.trigger()

    def _finish_session(
        self, session: Session, message: str, status: str,
    ):
        """Freeze one session, then publish exactly the result being returned."""
        with self.sessions_lock:
            if self.session is session:
                self.session = None
            result = self._result(session, message)
        if self.publish_debug_image:
            try:
                self.publish_final_debug_images(session, result, status)
            except Exception as error:
                # Debug rendering must never turn a valid perception result
                # into a failed action.
                self.get_logger().error(
                    f'Could not publish final debug image: {error}')
        detector_names = (
            (APRILTAGS, 'apriltags'),
            (CONTAINERS_HSV, 'containers_hsv'),
            (TABLE_SURFACE, 'table_surface'),
        )
        rates = [
            f'{name}={self._average_detector_fps(session, detector):.1f}fps'
            for detector, name in detector_names
            if session.requested_detectors & detector
        ]
        self.get_logger().info('Analysis throughput: ' + ', '.join(rates))
        return result

    def _feedback(self, session: Session):
        feedback = AnalyzeScene.Feedback()
        feedback.apriltags_camera = list(session.latest_camera)
        feedback.apriltags_base = list(session.latest_base)
        feedback.containers_camera = list(session.latest_containers_camera)
        feedback.containers_base = list(session.latest_containers_base)
        feedback.frames_processed = session.frames_processed
        feedback.frames_with_base_transform = session.frames_with_base_transform
        elapsed = max(0.0, time.monotonic() - session.started)
        feedback.elapsed = _ros_duration(elapsed)
        feedback.continuous = session.duration == 0.0
        feedback.remaining = _ros_duration(
            0.0 if feedback.continuous else session.duration - elapsed)
        return feedback

    @staticmethod
    def _observation_fps(session: Session) -> float:
        """Return completed-frame throughput over the latest one-second window."""
        stamps = session.recent_frame_times
        if len(stamps) < 2:
            return 0.0
        elapsed = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / elapsed if elapsed > 0.0 else 0.0

    @staticmethod
    def _detector_observation_fps(session: Session, detector: int) -> float:
        stamps = session.recent_detector_times.get(detector, [])
        if len(stamps) < 2:
            return 0.0
        elapsed = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / elapsed if elapsed > 0.0 else 0.0

    @staticmethod
    def _average_detector_fps(session: Session, detector: int) -> float:
        count = session.detector_frame_counts.get(detector, 0)
        first = session.detector_first_frame_times.get(detector)
        last = session.detector_last_frame_times.get(detector)
        if count < 2 or first is None or last is None or last <= first:
            return 0.0
        return (count - 1) / (last - first)

    @classmethod
    def _detector_summary(
        cls, session: Session, detector: int, status: str,
    ) -> str:
        frames = session.detector_frame_counts.get(detector, 0)
        transforms = session.detector_frames_with_base_transform.get(
            detector, 0)
        fps = cls._average_detector_fps(session, detector)
        return f'{status} {fps:.1f}fps F{frames} TF{transforms}/{frames}'

    @staticmethod
    def _average_observation_fps(session: Session) -> float:
        elapsed = max(0.0, time.monotonic() - session.started)
        return session.frames_processed / elapsed if elapsed > 0.0 else 0.0

    def _result(self, session: Session, message: str):
        # Image callbacks may still be adding observations on the executor
        # while the action worker builds its result.
        with self.sessions_lock:
            apriltags_camera = list(session.best_camera.values())
            apriltags_base = list(session.best_base.values())
            hsv_tracks = [PixelTrack(
                color=track.color, observations=list(track.observations))
                for track in session.hsv_container_tracks]
            frames_processed = session.frames_processed
            frames_with_base_transform = session.frames_with_base_transform
            table_observations = list(session.table_cell_observations)
            table_confirmations = list(session.table_cell_confirmations)
        result = AnalyzeScene.Result()
        result.best_apriltags_camera = apriltags_camera
        result.best_apriltags_base = apriltags_base
        camera, base = self._confirmed_hsv_container_results(hsv_tracks)
        result.best_containers_camera = camera
        result.best_containers_base = base
        grid = TableSurfaceGrid()
        grid.header.frame_id = self.base_frame
        grid.resolution_m = session.table_grid_resolution_m
        grid.x_min_m = session.table_search_x_min_m
        grid.y_min_m = session.table_search_y_min_m
        grid.width = session.table_grid_width
        grid.height = session.table_grid_height
        grid.cells = [
            (
                TableSurfaceGrid.FREE
                if observations >= self.white_surface_min_confirmed_frames
                and confirmations / observations
                >= self.white_surface_min_confirmed_ratio
                else TableSurfaceGrid.BLOCKED
            ) if observations >= self.white_surface_min_confirmed_frames
            else TableSurfaceGrid.UNKNOWN
            for observations, confirmations in zip(
                table_observations, table_confirmations)
        ]
        result.table_surface_grid = grid
        result.frames_processed = frames_processed
        result.frames_with_base_transform = frames_with_base_transform
        result.elapsed = _ros_duration(time.monotonic() - session.started)
        result.message = message
        return result

    def camera_info_callback(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            with self.sessions_lock:
                if self.state == 'analyzing':
                    self.camera_info = message

    def image_callback(self, message: Image) -> None:
        """Hand the latest frame to CV without blocking the ROS executor."""
        # The fallback keeps direct unit use of an uninitialized instance
        # synchronous; production nodes always own the worker.
        if not hasattr(self, 'image_condition'):
            self._process_image(message)
            return
        with self.sessions_lock:
            session = self.session
            calibrated = self.camera_info is not None
        if session is None or not calibrated:
            return
        with self.image_condition:
            for detector in self.pending_images:
                if session.requested_detectors & detector:
                    self.pending_images[detector] = message
            self.image_condition.notify_all()

    def _image_worker_loop(self, detector: int) -> None:
        while True:
            with self.image_condition:
                self.image_condition.wait_for(
                    lambda: (self.pending_images[detector] is not None
                             or self.image_worker_stopping))
                if self.image_worker_stopping:
                    return
                message = self.pending_images[detector]
                self.pending_images[detector] = None
            try:
                self._process_image(message, detector)
            except Exception as error:
                # A malformed camera frame must not permanently kill vision.
                self.get_logger().error(
                    f'Unhandled image processing error: {error}',
                    throttle_duration_sec=2.0)

    def _due_detectors(self, session: Session, now: float) -> int:
        due = 0
        for detector, period in self.detector_periods.items():
            if not session.requested_detectors & detector:
                continue
            if (detector & (CONTAINERS_HSV | TABLE_SURFACE)
                    and now < session.containers_ready_at):
                continue
            last = session.last_detector_times.get(detector, float('-inf'))
            if now - last >= period:
                due |= detector
        return due

    def _process_image(
        self, message: Image, assigned_detector: int | None = None,
    ) -> None:
        with self.sessions_lock:
            session = self.session
            info = self.camera_info
            tf_buffer = self.tf_buffer
        if session is None or info is None:
            return
        now = time.monotonic()
        with self.sessions_lock:
            if self.session is not session:
                return
            if hasattr(self, 'detector_periods'):
                active_detectors = self._due_detectors(session, now)
                if assigned_detector is not None:
                    active_detectors &= assigned_detector
            else:
                # Compatibility for focused unit fixtures.
                if now - self.last_detection_time < self.detection_period:
                    return
                self.last_detection_time = now
                active_detectors = session.requested_detectors
            if not active_detectors:
                return
            for detector in getattr(self, 'detector_periods', {}):
                if active_detectors & detector:
                    session.last_detector_times[detector] = now
            debug_period = getattr(self, 'debug_image_period', 0.0)
            debug_detectors = 0
            if self.publish_debug_image:
                for detector in getattr(
                        self, 'detector_periods', {active_detectors: 0.0}):
                    last = session.last_debug_publish_times.get(
                        detector, float('-inf'))
                    if (active_detectors & detector
                            and now - last >= debug_period):
                        debug_detectors |= detector
                        session.last_debug_publish_times[detector] = now
        camera_frame = message.header.frame_id or info.header.frame_id
        if not camera_frame or info.p[0] <= 0.0 or info.p[5] <= 0.0:
            return
        try:
            bgr = self.image_to_bgr8(message)
        except ValueError as error:
            self.get_logger().warning(
                f'Could not convert image: {error}', throttle_duration_sec=2.0)
            return
        image = (cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                 if active_detectors & APRILTAGS else None)
        parameters = (
            float(info.p[0]), float(info.p[5]),
            float(info.p[2]), float(info.p[6]))
        camera_matrix = np.array([
            [info.p[0], 0.0, info.p[2]],
            [0.0, info.p[5], info.p[6]],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        detections = []
        camera_items: list[AprilTagStampedDetection] = []
        camera_poses: list[PoseStamped] = []
        transforms: list[TransformStamped] = []
        if active_detectors & APRILTAGS:
            detections = self.apriltag_detector.detect(
                image, estimate_tag_pose=True,
                camera_params=parameters, tag_size=self.tag_size_m)
            valid = [
                detection for detection in detections
                if detection.hamming <= self.max_hamming
                and detection.decision_margin >= self.min_decision_margin
            ]
            for detection in valid:
                family = (
                    detection.tag_family.decode()
                    if isinstance(detection.tag_family, bytes)
                    else str(detection.tag_family))
                try:
                    pose = self.to_pose(
                        np.asarray(detection.pose_t),
                        np.asarray(detection.pose_R))
                except (TypeError, ValueError, np.linalg.LinAlgError) as error:
                    self.get_logger().warning(
                        f'Ignoring invalid pose for tag {detection.tag_id}: {error}',
                        throttle_duration_sec=2.0)
                    continue
                item = self.to_stamped_detection(
                    detection, family, pose,
                    float(detection.pose_err), message.header)
                camera_items.append(item)
                stamped = PoseStamped(header=item.header, pose=pose)
                camera_poses.append(stamped)
                transform = TransformStamped()
                transform.header.stamp = message.header.stamp
                transform.header.frame_id = camera_frame
                transform.child_frame_id = (
                    f'{self.tag_frame_prefix}_{family}_{detection.tag_id}')
                transform.transform.translation.x = pose.position.x
                transform.transform.translation.y = pose.position.y
                transform.transform.translation.z = pose.position.z
                transform.transform.rotation = pose.orientation
                transforms.append(transform)

        masks: dict[int, np.ndarray] = {}
        container_camera_items: list[ContainerStampedDetection] = []
        hsv_blobs = []
        if active_detectors & CONTAINERS_HSV:
            minimum_full, minimum_partial = area_thresholds_for_height(
                session.work_surface_height_m, self.hsv_min_areas,
                self.hsv_min_partial_areas)
            masks = self.container_color_masks(bgr)
            hsv_blobs = detect_blobs(
                masks, bgr.shape[:2], minimum_full, minimum_partial,
                self.container_border_margin_px, self.max_contour_fraction)

        base_items: list[AprilTagStampedDetection] = []
        base_poses: list[PoseStamped] = []
        container_base_items: list[ContainerStampedDetection] = []
        base_transform = None
        needs_base = bool(camera_poses or active_detectors &
                          (TABLE_SURFACE | CONTAINERS_HSV))
        if needs_base and tf_buffer is not None:
            try:
                base_transform = tf_buffer.lookup_transform(
                    self.base_frame, camera_frame, message.header.stamp,
                    timeout=Duration())
            except TransformException:
                if active_detectors & (TABLE_SURFACE | CONTAINERS_HSV):
                    try:
                        # The arm remains stationary during one analysis goal.
                        base_transform = tf_buffer.lookup_transform(
                            self.base_frame, camera_frame, Time(),
                            timeout=Duration())
                    except TransformException:
                        base_transform = session.last_base_transform
            if base_transform is not None:
                session.last_base_transform = copy.deepcopy(base_transform)
                for item, pose_camera in zip(camera_items, camera_poses):
                    pose_base = do_transform_pose_stamped(
                        pose_camera, base_transform)
                    base_items.append(self.to_stamped_detection_from_item(
                        item, pose_base.pose, self.base_frame))
                    base_poses.append(pose_base)
        hsv_observations = []
        if active_detectors & CONTAINERS_HSV and base_transform is not None:
            floor_z = 0.0
            floor_frame = getattr(self, 'floor_frame', self.base_frame)
            if floor_frame != self.base_frame:
                try:
                    floor_transform = tf_buffer.lookup_transform(
                        self.base_frame, floor_frame, Time(),
                        timeout=Duration())
                    floor_z = float(floor_transform.transform.translation.z)
                except TransformException:
                    floor_z = math.nan
                    self.get_logger().warning(
                        f'No TF from {floor_frame} to {self.base_frame} for '
                        'HSV container height', throttle_duration_sec=2.0)
            top_z = floor_z + session.work_surface_height_m + self.external_height
            rotation = rotation_from_quaternion(base_transform.transform.rotation)
            origin = np.array([
                base_transform.transform.translation.x,
                base_transform.transform.translation.y,
                base_transform.transform.translation.z])
            for blob in hsv_blobs:
                base_point = pixel_on_base_plane(
                    blob.center, camera_matrix, base_transform, top_z)
                if base_point is None:
                    continue
                camera_point = rotation.T @ (base_point - origin)
                camera_pose = Pose()
                (camera_pose.position.x, camera_pose.position.y,
                 camera_pose.position.z) = map(float, camera_point)
                camera_pose.orientation.w = 1.0
                base_pose = Pose()
                (base_pose.position.x, base_pose.position.y,
                 base_pose.position.z) = map(float, base_point)
                base_pose.orientation.w = 1.0
                camera_item = self.hsv_blob_to_stamped(
                    blob, message.header, camera_pose, camera_frame)
                base_item = self.copy_container_stamped(
                    camera_item, base_pose, self.base_frame)
                container_camera_items.append(camera_item)
                container_base_items.append(base_item)
                hsv_observations.append(PixelObservation(
                    session.frames_processed + 1, blob, camera_item, base_item))
        table_observed = [False] * (
            session.table_grid_width * session.table_grid_height)
        table_confirmed = [False] * len(table_observed)
        if (
            active_detectors & TABLE_SURFACE
            and base_transform is not None
        ):
            table_observed, table_confirmed, table_debug = (
                self.evaluate_white_table_grid(
                    session, bgr, camera_matrix, base_transform,
                    render_debug=bool(debug_detectors & TABLE_SURFACE)))
            if debug_detectors & TABLE_SURFACE:
                self.publish_table_surface_debug_image(message, table_debug)
        if active_detectors & APRILTAGS:
            self.camera_pose_publisher.publish(
                self.pose_array(camera_frame, message, camera_poses))
            self.camera_detection_publisher.publish(
                self.detection_array(camera_frame, message, camera_items))
            if transforms:
                self.tf_broadcaster.sendTransform(transforms)
            self.pose_publisher.publish(
                self.pose_array(self.base_frame, message, base_poses))
            self.detection_publisher.publish(
                self.detection_array(self.base_frame, message, base_items))
        if active_detectors & CONTAINERS_HSV:
            self.container_camera_detection_publisher.publish(
                self.container_detection_array(
                    camera_frame, message, container_camera_items))
            self.container_detection_publisher.publish(
                self.container_detection_array(
                    self.base_frame, message, container_base_items))
        with self.sessions_lock:
            if self.session is not session or session.goal_handle.is_cancel_requested:
                return
            session.frames_processed += 1
            for detector in getattr(self, 'detector_periods', {}):
                if active_detectors & detector:
                    session.detector_frame_counts[detector] = (
                        session.detector_frame_counts.get(detector, 0) + 1)
                    if base_transform is not None:
                        session.detector_frames_with_base_transform[detector] = (
                            session.detector_frames_with_base_transform.get(
                                detector, 0) + 1)
            if base_transform is not None:
                session.frames_with_base_transform += 1
            if active_detectors & APRILTAGS:
                session.latest_camera = [
                    self.copy_stamped(x) for x in camera_items]
                session.latest_base = [
                    self.copy_stamped(x) for x in base_items]
            if active_detectors & CONTAINERS_HSV:
                session.latest_containers_camera = [
                    self.copy_container_stamped(item)
                    for item in container_camera_items]
                session.latest_containers_base = [
                    self.copy_container_stamped(item)
                    for item in container_base_items]
            for index, observed in enumerate(table_observed):
                if observed:
                    session.table_cell_observations[index] += 1
                if table_confirmed[index]:
                    session.table_cell_confirmations[index] += 1
            for item in camera_items:
                self._update_best(session.best_camera, item)
            for item in base_items:
                self._update_best(session.best_base, item)
            if active_detectors & CONTAINERS_HSV:
                update_tracks(session.hsv_container_tracks, hsv_observations,
                              self.hsv_center_tolerance)
            completed_at = time.monotonic()
            session.recent_frame_times.append(completed_at)
            cutoff = completed_at - 1.0
            session.recent_frame_times = [
                stamp for stamp in session.recent_frame_times
                if stamp >= cutoff
            ]
            for detector in getattr(self, 'detector_periods', {}):
                if not active_detectors & detector:
                    continue
                stamps = session.recent_detector_times.setdefault(detector, [])
                stamps.append(completed_at)
                session.recent_detector_times[detector] = [
                    stamp for stamp in stamps if stamp >= cutoff]
                session.detector_first_frame_times.setdefault(
                    detector, completed_at)
                session.detector_last_frame_times[detector] = completed_at
            debug_frame = ContainerDebugFrame(
                header=copy.deepcopy(message.header),
                image=bgr.copy(),
                camera_matrix=camera_matrix.copy(),
                camera_to_base=copy.deepcopy(
                    base_transform or session.last_base_transform),
                container_masks=(
                    {color: mask.copy() for color, mask in masks.items()}
                    if active_detectors & CONTAINERS_HSV else None),
            )
            session.latest_debug_frame = debug_frame
            for detector in getattr(
                    self, 'detector_periods', {active_detectors: 0.0}):
                if active_detectors & detector:
                    session.latest_debug_frames[detector] = debug_frame
            if debug_detectors:
                if debug_detectors & APRILTAGS:
                    self.publish_detection_debug_image(
                        message, image, detections, session,
                        self._detector_observation_fps(session, APRILTAGS))
                if debug_detectors & CONTAINERS_HSV:
                    self.publish_hsv_container_debug_image(
                        message, bgr, masks, hsv_blobs, session)

    @staticmethod
    def to_pose(translation: np.ndarray, rotation: np.ndarray) -> Pose:
        """Convert pupil_apriltags pose arrays into a ROS Pose message."""
        translation = np.asarray(translation, dtype=np.float64).reshape(-1)
        rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        if translation.size != 3 or not np.all(np.isfinite(translation)):
            raise ValueError('pose translation must contain three finite values')
        if not np.all(np.isfinite(rotation)):
            raise ValueError('pose rotation contains non-finite values')
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, translation)
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion_from_rotation(rotation)
        return pose

    @staticmethod
    def _update_best(best, item) -> None:
        old = best.get(item.id)
        item_time = (
            int(item.header.stamp.sec), int(item.header.stamp.nanosec))
        old_time = (
            (int(old.header.stamp.sec), int(old.header.stamp.nanosec))
            if old is not None else (0, 0))
        score = (
            item.pose_error, -item.decision_margin, item.hamming,
            (-item_time[0], -item_time[1]))
        old_score = (
            (
                old.pose_error, -old.decision_margin, old.hamming,
                (-old_time[0], -old_time[1])
            ) if old is not None else None)
        if old is None or score < old_score:
            best[item.id] = SceneAnalyzer.copy_stamped(item)

    @staticmethod
    def copy_stamped(item):
        result = AprilTagStampedDetection()
        result.header = item.header
        result.family = item.family
        result.id = item.id
        result.decision_margin = item.decision_margin
        result.hamming = item.hamming
        result.pose_error = item.pose_error
        result.pose = item.pose
        return result

    @staticmethod
    def to_stamped_detection(detection, family, pose, pose_error, header):
        item = AprilTagStampedDetection()
        item.header = header
        item.family = family
        item.id = int(detection.tag_id)
        item.decision_margin = float(detection.decision_margin)
        item.hamming = int(detection.hamming)
        item.pose_error = pose_error
        item.pose = pose
        return item

    @staticmethod
    def to_stamped_detection_from_item(item, pose, frame):
        result = SceneAnalyzer.copy_stamped(item)
        result.header.frame_id = frame
        result.pose = pose
        return result

    @staticmethod
    def pose_array(frame, image, poses):
        output = PoseArray()
        output.header.frame_id = frame
        output.header.stamp = image.header.stamp
        output.poses = [pose.pose for pose in poses]
        return output

    @staticmethod
    def detection_array(frame, image, items):
        output = AprilTagDetectionArray()
        output.header.frame_id = frame
        output.header.stamp = image.header.stamp
        for item in items:
            detection = AprilTagDetection(family=item.family, id=item.id,
                                          decision_margin=item.decision_margin,
                                          hamming=item.hamming, pose_error=item.pose_error,
                                          pose=item.pose)
            output.detections.append(detection)
        return output


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneAnalyzer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
