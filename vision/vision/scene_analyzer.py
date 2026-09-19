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
    ContainerDetection,
    ContainerDetectionArray,
    ContainerStampedDetection,
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

from .partial_container import fit_partial_container, rotation_from_quaternion


APRILTAGS = AnalyzeScene.Goal.APRILTAGS
CONTAINERS = AnalyzeScene.Goal.CONTAINERS
RED = ContainerStampedDetection.RED
BLUE = ContainerStampedDetection.BLUE
COLOR_NAMES = {RED: 'red', BLUE: 'blue'}
DEBUG_COLORS = {RED: (30, 30, 255), BLUE: (255, 80, 20)}


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
    container_tracks: list['ContainerTrack'] = field(default_factory=list)
    latest_containers_camera: list[ContainerStampedDetection] = field(default_factory=list)
    latest_containers_base: list[ContainerStampedDetection] = field(default_factory=list)
    containers_ready_at: float = math.inf
    last_feedback: float = 0.0
    last_base_transform: TransformStamped | None = None


@dataclass
class ContainerCandidate:
    color: int
    contour: np.ndarray
    corners: np.ndarray
    area: float
    rectangularity: float
    accepted: bool = False
    reason: str = ''
    pose: Pose | None = None
    pose_error: float = math.inf
    partial: bool = False
    position_uncertainty_m: float = 0.0
    yaw_uncertainty_deg: float = 0.0
    partial_fit_overlap: float = 1.0


@dataclass
class ContainerObservation:
    """One same-frame camera detection and its optional base transform."""

    frame_index: int
    camera: ContainerStampedDetection
    base: ContainerStampedDetection | None = None


@dataclass
class ContainerTrack:
    """Temporal observations believed to belong to one physical container."""

    color: int
    observations: list[ContainerObservation] = field(default_factory=list)


@dataclass
class ContainerDebugFrame:
    """Annotated observation plus the calibration valid for that frame."""

    header: object
    image: np.ndarray
    camera_matrix: np.ndarray
    camera_to_base: TransformStamped | None


class SceneAnalyzer(Node):
    def __init__(self) -> None:
        super().__init__('scene_analyzer')
        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'container_target_topic', '/manipulation/container_release_target')
        self.declare_parameter('tag_frame_prefix', 'apriltag')
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('tag_size_m', 0.032)
        self.declare_parameter('nthreads', 1)
        self.declare_parameter('quad_decimate', 1.0)
        self.declare_parameter('max_detection_rate_hz', 10.0)
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
        self.declare_parameter('external_width_m', 0.102)
        self.declare_parameter('external_depth_m', 0.173)
        self.declare_parameter('internal_height_m', 0.057)
        self.declare_parameter('internal_width_m', 0.090)
        self.declare_parameter('internal_depth_m', 0.140)
        self.declare_parameter('min_saturation', 80)
        self.declare_parameter('min_value', 45)
        self.declare_parameter('red_hue_low_1', 0)
        self.declare_parameter('red_hue_high_1', 12)
        self.declare_parameter('red_hue_low_2', 168)
        self.declare_parameter('red_hue_high_2', 179)
        self.declare_parameter('blue_hue_low', 92)
        self.declare_parameter('blue_hue_high', 138)
        self.declare_parameter('morphology_kernel_px', 5)
        self.declare_parameter('min_contour_area_px', 350.0)
        self.declare_parameter('min_partial_contour_area_px', 800.0)
        self.declare_parameter('container_border_margin_px', 6)
        self.declare_parameter('max_contour_area_fraction', 0.85)
        self.declare_parameter('min_rectangularity', 0.55)
        self.declare_parameter('polygon_epsilon_fraction', 0.035)
        self.declare_parameter('max_container_pose_error_px', 12.0)
        self.declare_parameter('container_warmup_sec', 0.5)
        self.declare_parameter('container_association_distance_m', 0.07)
        self.declare_parameter('container_final_merge_distance_m', 0.07)
        self.declare_parameter('min_container_observations', 3)
        self.declare_parameter('max_container_position_deviation_m', 0.04)
        self.declare_parameter('max_container_yaw_deviation_deg', 20.0)

        self.warning_filter = (NativeWarningFilter()
                               if bool(self.get_parameter('suppress_native_pose_warning').value)
                               else None)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_hamming = int(self.get_parameter('max_hamming').value)
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)
        detection_rate = float(
            self.get_parameter('max_detection_rate_hz').value)
        if not math.isfinite(detection_rate) or detection_rate <= 0.0:
            raise ValueError(
                'max_detection_rate_hz must be positive and finite.')
        self.detection_period = 1.0 / detection_rate
        self.last_detection_time = float('-inf')
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
        self._configure_container_detector()
        self.tf_broadcaster = TransformBroadcaster(self)
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
            Image, 'apriltags/debug_image', qos_profile_sensor_data)
        self.container_camera_detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections_camera', output_qos)
        self.container_detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections', output_qos)
        self.container_debug_image_publisher = self.create_publisher(
            Image, 'containers/debug_image', qos_profile_sensor_data)
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
        self.get_logger().info(
            'Scene analyzer idle; waiting for /vision/analyze_scene goals.')

    def _configure_container_detector(self) -> None:
        """Validate and cache the known Bin 3 geometry and HSV profile."""
        def positive(name: str) -> float:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
            return value

        def bounded(name: str, maximum: int) -> int:
            value = int(self.get_parameter(name).value)
            if not 0 <= value <= maximum:
                raise ValueError(f'{name} must be in [0, {maximum}]')
            return value

        self.external_width = positive('external_width_m')
        self.external_depth = positive('external_depth_m')
        self.external_height = positive('external_height_m')
        positive('internal_height_m')
        positive('internal_width_m')
        positive('internal_depth_m')
        self.min_saturation = bounded('min_saturation', 255)
        self.min_value = bounded('min_value', 255)
        self.red_ranges = (
            (bounded('red_hue_low_1', 179), bounded('red_hue_high_1', 179)),
            (bounded('red_hue_low_2', 179), bounded('red_hue_high_2', 179)),
        )
        self.blue_range = (
            bounded('blue_hue_low', 179), bounded('blue_hue_high', 179))
        kernel_size = int(self.get_parameter('morphology_kernel_px').value)
        if kernel_size <= 0:
            raise ValueError('morphology_kernel_px must be positive')
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.min_contour_area = positive('min_contour_area_px')
        self.min_partial_contour_area = positive(
            'min_partial_contour_area_px')
        self.container_border_margin_px = int(
            self.get_parameter('container_border_margin_px').value)
        self.max_contour_fraction = float(
            self.get_parameter('max_contour_area_fraction').value)
        self.min_rectangularity = float(
            self.get_parameter('min_rectangularity').value)
        self.polygon_epsilon_fraction = positive('polygon_epsilon_fraction')
        self.max_container_pose_error = positive(
            'max_container_pose_error_px')
        self.container_warmup = float(
            self.get_parameter('container_warmup_sec').value)
        self.container_association_distance = positive(
            'container_association_distance_m')
        self.container_final_merge_distance = positive(
            'container_final_merge_distance_m')
        self.min_container_observations = int(
            self.get_parameter('min_container_observations').value)
        self.max_container_position_deviation = positive(
            'max_container_position_deviation_m')
        self.max_container_yaw_deviation = math.radians(positive(
            'max_container_yaw_deviation_deg'))
        if not 0.0 < self.max_contour_fraction <= 1.0:
            raise ValueError('max_contour_area_fraction must be in (0, 1]')
        if not 0.0 < self.min_rectangularity <= 1.0:
            raise ValueError('min_rectangularity must be in (0, 1]')
        if self.container_border_margin_px < 0:
            raise ValueError('container_border_margin_px must be nonnegative')
        if not math.isfinite(self.container_warmup) or self.container_warmup < 0.0:
            raise ValueError('container_warmup_sec must be finite and nonnegative')
        if self.min_container_observations <= 0:
            raise ValueError('min_container_observations must be positive')
        if self.max_container_yaw_deviation > math.pi / 2.0:
            raise ValueError(
                'max_container_yaw_deviation_deg must not exceed 90 degrees')

    def destroy_node(self):
        with self.sessions_lock:
            self.session = None
            self._destroy_inputs_locked()
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
        known = APRILTAGS | CONTAINERS
        if requested == 0 or requested & ~known:
            self.get_logger().warning(
                f'Rejecting scene goal with invalid detector mask: {requested}.')
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
        session = Session(
            goal_handle=goal_handle,
            duration=duration,
            requested_detectors=int(goal_handle.request.requested_detectors),
            work_surface_height_m=float(
                goal_handle.request.work_surface_height_m),
        )
        with self.sessions_lock:
            self.session = session
            self.state = 'analyzing'
            self.last_detection_time = float('-inf')
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
                if session.requested_detectors & CONTAINERS
                else now
            )
            # The requested analysis duration starts after exposure/white-balance
            # stabilization, rather than consuming the useful observation window.
            session.started = session.containers_ready_at
            while rclpy.ok() and goal_handle.is_active:
                time.sleep(0.02)
                elapsed = max(0.0, time.monotonic() - session.started)
                if goal_handle.is_cancel_requested:
                    result = self._result(
                        session, 'Canceled; returning accumulated detections.')
                    goal_handle.canceled(result)
                    return result
                if duration > 0.0 and elapsed >= duration:
                    if session.frames_processed == 0:
                        result = self._result(
                            session,
                            'No calibrated image was processed during the '
                            'requested window.')
                        goal_handle.abort(result)
                        return result
                    result = self._result(session, 'Analysis completed.')
                    goal_handle.succeed(result)
                    return result
                now = time.monotonic()
                if now - session.last_feedback >= self.feedback_period:
                    session.last_feedback = now
                    goal_handle.publish_feedback(self._feedback(session))
        finally:
            if vision_led_enabled:
                self._set_vision_led(False)
            with self.sessions_lock:
                self.session = None
                self.state = 'deactivating'
            self.input_lifecycle_guard.trigger()

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

    def _result(self, session: Session, message: str):
        # Image callbacks may still be adding observations on the executor
        # while the action worker builds its result.
        with self.sessions_lock:
            apriltags_camera = list(session.best_camera.values())
            apriltags_base = list(session.best_base.values())
            tracks = [ContainerTrack(
                color=track.color, observations=list(track.observations))
                for track in session.container_tracks]
            frames_processed = session.frames_processed
            frames_with_base_transform = session.frames_with_base_transform
        result = AnalyzeScene.Result()
        result.best_apriltags_camera = apriltags_camera
        result.best_apriltags_base = apriltags_base
        camera, base = self._confirmed_container_results(tracks)
        result.best_containers_camera = camera
        result.best_containers_base = base
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
        with self.sessions_lock:
            session = self.session
            info = self.camera_info
            tf_buffer = self.tf_buffer
        if session is None or info is None:
            return
        now = time.monotonic()
        if now < session.containers_ready_at:
            return
        with self.sessions_lock:
            if now - self.last_detection_time < self.detection_period:
                return
            self.last_detection_time = now
        camera_frame = message.header.frame_id or info.header.frame_id
        if not camera_frame or info.p[0] <= 0.0 or info.p[5] <= 0.0:
            return
        try:
            bgr = self.image_to_bgr8(message)
        except ValueError as error:
            self.get_logger().warning(
                f'Could not convert image: {error}', throttle_duration_sec=2.0)
            return
        image = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        parameters = (
            float(info.p[0]), float(info.p[5]),
            float(info.p[2]), float(info.p[6]))
        camera_items: list[AprilTagStampedDetection] = []
        camera_poses: list[PoseStamped] = []
        transforms: list[TransformStamped] = []
        if session.requested_detectors & APRILTAGS:
            detections = self.apriltag_detector.detect(
                image, estimate_tag_pose=True,
                camera_params=parameters, tag_size=self.tag_size_m)
            valid = [
                detection for detection in detections
                if detection.hamming <= self.max_hamming
                and detection.decision_margin >= self.min_decision_margin
            ]
            if self.publish_debug_image:
                self.publish_detection_debug_image(message, image, detections)
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

        container_camera_items: list[ContainerStampedDetection] = []
        container_camera_poses: list[PoseStamped] = []
        if session.requested_detectors & CONTAINERS:
            camera_matrix = np.array([
                [info.p[0], 0.0, info.p[2]],
                [0.0, info.p[5], info.p[6]],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            masks = self.container_color_masks(bgr)
            candidates = self.detect_container_candidates(
                masks, bgr.shape[:2], camera_matrix)
            for candidate in candidates:
                if not candidate.accepted:
                    continue
                item = self.container_to_stamped(candidate, message.header)
                container_camera_items.append(item)
                container_camera_poses.append(PoseStamped(
                    header=item.header, pose=item.pose))

        base_items: list[AprilTagStampedDetection] = []
        base_poses: list[PoseStamped] = []
        container_base_items: list[ContainerStampedDetection] = []
        base_transform = None
        has_partial = (
            session.requested_detectors & CONTAINERS
            and any(candidate.reason == 'border' for candidate in candidates)
        )
        if (camera_poses or container_camera_poses or has_partial) and tf_buffer is not None:
            try:
                base_transform = tf_buffer.lookup_transform(
                    self.base_frame, camera_frame, message.header.stamp,
                    timeout=Duration())
            except TransformException:
                if has_partial:
                    try:
                        # The arm is stationary during scene analysis. A latest
                        # transform is preferable to dropping a border contour
                        # because its exact image timestamp arrived first.
                        base_transform = tf_buffer.lookup_transform(
                            self.base_frame, camera_frame, Time(),
                            timeout=Duration())
                    except TransformException:
                        base_transform = session.last_base_transform
            if base_transform is not None:
                session.last_base_transform = copy.deepcopy(base_transform)
        if has_partial and base_transform is None:
            for candidate in candidates:
                if candidate.reason == 'border':
                    candidate.reason = 'partial_waiting_tf'
        if base_transform is not None:
            for item, pose_camera in zip(camera_items, camera_poses):
                pose_base = do_transform_pose_stamped(pose_camera, base_transform)
                base_items.append(self.to_stamped_detection_from_item(
                    item, pose_base.pose, self.base_frame))
                base_poses.append(pose_base)
            for item, pose_camera in zip(
                    container_camera_items, container_camera_poses):
                pose_base = do_transform_pose_stamped(
                    pose_camera, base_transform)
                container_base_items.append(self.copy_container_stamped(
                    item, pose_base.pose, self.base_frame))
            if has_partial:
                for candidate in candidates:
                    if candidate.reason != 'border':
                        continue
                    fit = fit_partial_container(
                        candidate.contour, bgr.shape[:2], camera_matrix,
                        base_transform, session.work_surface_height_m +
                        self.external_height,
                        self.external_depth, self.external_width,
                    )
                    if fit is None:
                        candidate.reason = 'partial_unresolved'
                        continue
                    pose_base = Pose()
                    pose_base.position.x = fit.x
                    pose_base.position.y = fit.y
                    pose_base.position.z = (
                        session.work_surface_height_m + self.external_height)
                    pose_base.orientation.z = math.sin(fit.yaw / 2.0)
                    pose_base.orientation.w = math.cos(fit.yaw / 2.0)
                    transform = base_transform.transform
                    rotation = rotation_from_quaternion(transform.rotation)
                    origin = np.array([
                        transform.translation.x, transform.translation.y,
                        transform.translation.z,
                    ])
                    camera_position = rotation.T @ (
                        np.array([fit.x, fit.y, pose_base.position.z]) - origin)
                    camera_rotation = rotation.T @ np.array([
                        [math.cos(fit.yaw), -math.sin(fit.yaw), 0.0],
                        [math.sin(fit.yaw), math.cos(fit.yaw), 0.0],
                        [0.0, 0.0, 1.0],
                    ])
                    pose_camera = Pose()
                    (pose_camera.position.x, pose_camera.position.y,
                     pose_camera.position.z) = map(float, camera_position)
                    (pose_camera.orientation.x, pose_camera.orientation.y,
                     pose_camera.orientation.z, pose_camera.orientation.w) = (
                        quaternion_from_rotation(camera_rotation))
                    candidate.pose = pose_camera
                    candidate.pose_error = fit.edge_error_px
                    candidate.partial = True
                    candidate.position_uncertainty_m = fit.position_uncertainty_m
                    candidate.yaw_uncertainty_deg = fit.yaw_uncertainty_deg
                    candidate.partial_fit_overlap = fit.overlap
                    candidate.accepted = True
                    candidate.reason = f'partial iou={fit.overlap:.2f}'
                    camera_item = self.container_to_stamped(
                        candidate, message.header)
                    container_camera_items.append(camera_item)
                    container_base_items.append(self.copy_container_stamped(
                        camera_item, pose_base, self.base_frame))
        if session.requested_detectors & CONTAINERS and self.publish_debug_image:
            self.publish_container_debug_image(
                message, bgr, masks, candidates, camera_matrix, base_transform)
        if session.requested_detectors & APRILTAGS:
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
        if session.requested_detectors & CONTAINERS:
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
            if base_transform is not None:
                session.frames_with_base_transform += 1
            session.latest_camera = [self.copy_stamped(x) for x in camera_items]
            session.latest_base = [self.copy_stamped(x) for x in base_items]
            session.latest_containers_camera = [
                self.copy_container_stamped(item)
                for item in container_camera_items]
            session.latest_containers_base = [
                self.copy_container_stamped(item)
                for item in container_base_items]
            for item in camera_items:
                self._update_best(session.best_camera, item)
            for item in base_items:
                self._update_best(session.best_base, item)
            self._update_container_tracks(
                session.container_tracks,
                container_camera_items,
                container_base_items,
                session.frames_processed,
            )

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

    def container_color_masks(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in self.red_ranges:
            red = cv2.bitwise_or(red, cv2.inRange(
                hsv, (low, self.min_saturation, self.min_value),
                (high, 255, 255)))
        blue = cv2.inRange(
            hsv,
            (self.blue_range[0], self.min_saturation, self.min_value),
            (self.blue_range[1], 255, 255))
        output = {}
        for color, mask in ((RED, red), (BLUE, blue)):
            cleaned = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.morphology_kernel)
            cleaned = cv2.morphologyEx(
                cleaned, cv2.MORPH_CLOSE, self.morphology_kernel)
            output[color] = cleaned
        return output

    def detect_container_candidates(
        self,
        masks: dict[int, np.ndarray],
        image_shape: tuple[int, int],
        camera_matrix: np.ndarray,
    ) -> list[ContainerCandidate]:
        image_area = float(image_shape[0] * image_shape[1])
        candidates = []
        for color, mask in masks.items():
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                rectangle = cv2.minAreaRect(contour)
                rectangle_area = float(rectangle[1][0] * rectangle[1][1])
                rectangularity = (
                    area / rectangle_area if rectangle_area > 0.0 else 0.0)
                perimeter = float(cv2.arcLength(contour, True))
                approximation = cv2.approxPolyDP(
                    contour, self.polygon_epsilon_fraction * perimeter, True)
                if (
                    len(approximation) == 4
                    and cv2.isContourConvex(approximation)
                ):
                    corners = approximation.reshape(4, 2).astype(np.float64)
                else:
                    corners = cv2.boxPoints(rectangle).astype(np.float64)
                candidate = ContainerCandidate(
                    color=color, contour=contour, corners=corners,
                    area=area, rectangularity=rectangularity)
                at_border = self._container_touches_border(
                    corners, image_shape)
                minimum_area = (self.min_partial_contour_area if at_border
                                else self.min_contour_area)
                if area < minimum_area:
                    candidate.reason = 'small'
                elif area > image_area * self.max_contour_fraction:
                    candidate.reason = 'large'
                elif at_border:
                    candidate.reason = 'border'
                elif rectangularity < self.min_rectangularity:
                    candidate.reason = 'shape'
                else:
                    pose, error = self.estimate_container_pose(
                        corners, camera_matrix)
                    candidate.pose = pose
                    candidate.pose_error = error
                    if pose is None or error > self.max_container_pose_error:
                        candidate.reason = 'pose'
                    else:
                        candidate.accepted = True
                        candidate.reason = 'ok'
                candidates.append(candidate)
        return candidates

    def _container_touches_border(
        self, corners: np.ndarray, shape: tuple[int, int],
    ) -> bool:
        height, width = shape
        margin = float(self.container_border_margin_px)
        return bool(
            np.any(corners[:, 0] <= margin)
            or np.any(corners[:, 1] <= margin)
            or np.any(corners[:, 0] >= width - 1.0 - margin)
            or np.any(corners[:, 1] >= height - 1.0 - margin))

    def estimate_container_pose(
        self, corners: np.ndarray, camera_matrix: np.ndarray,
    ) -> tuple[Pose | None, float]:
        center = corners.mean(axis=0)
        angles = np.arctan2(
            corners[:, 1] - center[1], corners[:, 0] - center[0])
        cyclic = corners[np.argsort(angles)].astype(np.float64)
        depth = self.external_depth
        width = self.external_width
        object_points = np.array([
            [-depth / 2.0, -width / 2.0, 0.0],
            [depth / 2.0, -width / 2.0, 0.0],
            [depth / 2.0, width / 2.0, 0.0],
            [-depth / 2.0, width / 2.0, 0.0],
        ], dtype=np.float64)
        best = None
        distortion = np.zeros((4, 1), dtype=np.float64)
        for winding in (cyclic, cyclic[::-1]):
            for offset in range(4):
                image_points = np.roll(winding, offset, axis=0)
                success, rotation_vector, translation = cv2.solvePnP(
                    object_points, image_points, camera_matrix, distortion,
                    flags=cv2.SOLVEPNP_IPPE)
                if not success or float(translation[2, 0]) <= 0.0:
                    continue
                projected, _ = cv2.projectPoints(
                    object_points, rotation_vector, translation,
                    camera_matrix, distortion)
                residual = projected.reshape(4, 2) - image_points
                error = float(np.sqrt(np.mean(np.sum(
                    residual * residual, axis=1))))
                if math.isfinite(error) and (
                    best is None or error < best[0]
                ):
                    best = (error, rotation_vector, translation)
        if best is None:
            return None, math.inf
        rotation, _ = cv2.Rodrigues(best[1])
        translation = best[2].reshape(3)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(
            float, translation)
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion_from_rotation(rotation)
        return pose, best[0]

    def container_to_stamped(
        self, candidate: ContainerCandidate, header,
    ) -> ContainerStampedDetection:
        item = ContainerStampedDetection()
        item.header = header
        item.color = candidate.color
        item.contour_area_px = candidate.area
        item.rectangularity = candidate.rectangularity
        item.pose_error = candidate.pose_error
        item.external_width_m = self.external_width
        item.external_depth_m = self.external_depth
        item.external_height_m = self.external_height
        item.observation_count = 1
        item.position_spread_m = 0.0
        item.yaw_spread_deg = 0.0
        item.partial = candidate.partial
        item.position_uncertainty_m = candidate.position_uncertainty_m
        item.yaw_uncertainty_deg = candidate.yaw_uncertainty_deg
        item.partial_fit_overlap = candidate.partial_fit_overlap
        item.pose = candidate.pose
        return item

    @staticmethod
    def copy_container_stamped(
        item: ContainerStampedDetection,
        pose: Pose | None = None,
        frame: str | None = None,
    ) -> ContainerStampedDetection:
        result = ContainerStampedDetection()
        result.header = copy.deepcopy(item.header)
        if frame is not None:
            result.header.frame_id = frame
        result.color = item.color
        result.contour_area_px = item.contour_area_px
        result.rectangularity = item.rectangularity
        result.pose_error = item.pose_error
        result.external_width_m = item.external_width_m
        result.external_depth_m = item.external_depth_m
        result.external_height_m = item.external_height_m
        result.observation_count = item.observation_count
        result.position_spread_m = item.position_spread_m
        result.yaw_spread_deg = item.yaw_spread_deg
        result.partial = item.partial
        result.position_uncertainty_m = item.position_uncertainty_m
        result.yaw_uncertainty_deg = item.yaw_uncertainty_deg
        result.partial_fit_overlap = item.partial_fit_overlap
        result.pose = item.pose if pose is None else pose
        return result

    @staticmethod
    def container_detection_array(frame, image, items):
        output = ContainerDetectionArray()
        output.header.frame_id = frame
        output.header.stamp = image.header.stamp
        for item in items:
            detection = ContainerDetection()
            detection.color = item.color
            detection.contour_area_px = item.contour_area_px
            detection.rectangularity = item.rectangularity
            detection.pose_error = item.pose_error
            detection.external_width_m = item.external_width_m
            detection.external_depth_m = item.external_depth_m
            detection.external_height_m = item.external_height_m
            detection.observation_count = item.observation_count
            detection.position_spread_m = item.position_spread_m
            detection.yaw_spread_deg = item.yaw_spread_deg
            detection.partial = item.partial
            detection.position_uncertainty_m = item.position_uncertainty_m
            detection.yaw_uncertainty_deg = item.yaw_uncertainty_deg
            detection.partial_fit_overlap = item.partial_fit_overlap
            detection.pose = item.pose
            output.detections.append(detection)
        return output

    @staticmethod
    def _container_position(item: ContainerStampedDetection) -> np.ndarray:
        return np.array([
            item.pose.position.x,
            item.pose.position.y,
            item.pose.position.z,
        ], dtype=np.float64)

    @classmethod
    def _track_center(cls, track: ContainerTrack) -> np.ndarray:
        positions = np.array([
            cls._container_position(observation.camera)
            for observation in track.observations
        ])
        return np.median(positions, axis=0)

    def _update_container_tracks(
        self,
        tracks: list[ContainerTrack],
        camera_items: list[ContainerStampedDetection],
        base_items: list[ContainerStampedDetection],
        frame_index: int,
    ) -> None:
        """Associate a frame one-to-one, preventing duplicate hits per track."""
        base_for_item = base_items if len(base_items) == len(camera_items) else [
            None for _ in camera_items]
        pairs = []
        for track_index, track in enumerate(tracks):
            center = self._track_center(track)
            for item_index, item in enumerate(camera_items):
                if track.color != item.color:
                    continue
                distance = float(np.linalg.norm(
                    center - self._container_position(item)))
                if distance <= self.container_association_distance:
                    pairs.append((distance, track_index, item_index))

        assigned_tracks = set()
        assigned_items = set()
        for _distance, track_index, item_index in sorted(pairs):
            if track_index in assigned_tracks or item_index in assigned_items:
                continue
            tracks[track_index].observations.append(ContainerObservation(
                frame_index,
                self.copy_container_stamped(camera_items[item_index]),
                self.copy_container_stamped(base_for_item[item_index])
                if base_for_item[item_index] is not None else None,
            ))
            assigned_tracks.add(track_index)
            assigned_items.add(item_index)

        for item_index, item in enumerate(camera_items):
            if item_index in assigned_items:
                continue
            tracks.append(ContainerTrack(
                color=int(item.color),
                observations=[ContainerObservation(
                    frame_index,
                    self.copy_container_stamped(item),
                    self.copy_container_stamped(base_for_item[item_index])
                    if base_for_item[item_index] is not None else None,
                )],
            ))

    def _merged_container_tracks(
        self, tracks: list[ContainerTrack],
    ) -> list[ContainerTrack]:
        """Globally merge same-color tracks that converged after early noise."""
        merged = [ContainerTrack(
            color=track.color, observations=list(track.observations))
            for track in tracks]
        changed = True
        while changed:
            changed = False
            for left_index in range(len(merged)):
                left = merged[left_index]
                for right_index in range(left_index + 1, len(merged)):
                    right = merged[right_index]
                    if left.color != right.color:
                        continue
                    # Two detections in the same frame cannot be the same
                    # physical contour. Keep simultaneously visible bins apart.
                    if ({item.frame_index for item in left.observations}
                            & {item.frame_index for item in right.observations}):
                        continue
                    # Compare robust centers, not the closest pair of samples:
                    # single-link clustering could bridge two separate bins.
                    distance = float(np.linalg.norm(
                        self._track_center(left) - self._track_center(right)))
                    if distance > self.container_final_merge_distance:
                        continue
                    left.observations.extend(right.observations)
                    del merged[right_index]
                    changed = True
                    break
                if changed:
                    break
        return merged

    @staticmethod
    def _container_yaw(item: ContainerStampedDetection) -> float:
        orientation = item.pose.orientation
        return math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )

    @staticmethod
    def _axial_yaw_distance(left: float, right: float) -> float:
        """Rectangle yaw distance, where directions 180 degrees apart agree."""
        return abs((left - right + math.pi / 2.0) % math.pi - math.pi / 2.0)

    def _stable_container_items(
        self,
        items: list[ContainerStampedDetection],
        *,
        stabilize_yaw: bool,
    ) -> list[ContainerStampedDetection]:
        if len(items) < self.min_container_observations:
            return []
        positions = np.array([self._container_position(item) for item in items])
        center = np.median(positions, axis=0)
        items = [
            item for item, distance in zip(
                items, np.linalg.norm(positions - center, axis=1))
            if distance <= self.max_container_position_deviation
        ]
        if len(items) < self.min_container_observations or not stabilize_yaw:
            return items if len(items) >= self.min_container_observations else []

        yaws = [self._container_yaw(item) for item in items]
        groups = [
            [
                item for item, candidate_yaw in zip(items, yaws)
                if self._axial_yaw_distance(anchor, candidate_yaw)
                <= self.max_container_yaw_deviation
            ]
            for anchor in yaws
        ]
        best = max(
            groups,
            key=lambda group: (
                len(group),
                -float(np.median([item.pose_error for item in group])),
            ),
        )
        return best if len(best) >= self.min_container_observations else []

    def _summarize_container_items(
        self,
        items: list[ContainerStampedDetection],
        *,
        normalize_yaw: bool,
    ) -> ContainerStampedDetection | None:
        # Prefer complete observations whenever they form a stable track.
        complete = [item for item in items if not item.partial]
        selected = self._stable_container_items(
            complete, stabilize_yaw=normalize_yaw)
        items = selected or self._stable_container_items(
            items, stabilize_yaw=normalize_yaw)
        if not items:
            return None
        positions = np.array([self._container_position(item) for item in items])
        center = np.median(positions, axis=0)
        representative = min(items, key=lambda item: (
            item.pose_error, -item.rectangularity, -item.contour_area_px))
        result = self.copy_container_stamped(representative)
        result.pose = copy.deepcopy(representative.pose)
        result.pose.position.x, result.pose.position.y, result.pose.position.z = (
            map(float, center))
        result.contour_area_px = float(np.median([
            item.contour_area_px for item in items]))
        result.rectangularity = float(np.median([
            item.rectangularity for item in items]))
        result.pose_error = float(np.median([
            item.pose_error for item in items]))
        result.observation_count = len(items)
        result.position_spread_m = float(max(
            np.linalg.norm(positions - center, axis=1), default=0.0))
        result.partial = any(item.partial for item in items)
        result.position_uncertainty_m = float(max(
            item.position_uncertainty_m for item in items))
        result.yaw_uncertainty_deg = float(max(
            item.yaw_uncertainty_deg for item in items))
        result.partial_fit_overlap = float(min(
            item.partial_fit_overlap for item in items))
        result.yaw_spread_deg = 0.0
        if normalize_yaw:
            yaws = [self._container_yaw(item) for item in items]
            yaw = 0.5 * math.atan2(
                sum(math.sin(2.0 * value) for value in yaws),
                sum(math.cos(2.0 * value) for value in yaws),
            )
            result.pose.orientation.x = 0.0
            result.pose.orientation.y = 0.0
            result.pose.orientation.z = math.sin(yaw / 2.0)
            result.pose.orientation.w = math.cos(yaw / 2.0)
            result.yaw_spread_deg = math.degrees(max(
                self._axial_yaw_distance(yaw, value) for value in yaws))
        return result

    def _confirmed_container_results(
        self, tracks: list[ContainerTrack],
    ) -> tuple[list[ContainerStampedDetection], list[ContainerStampedDetection]]:
        camera_results = []
        base_results = []
        for track in self._merged_container_tracks(tracks):
            camera = self._summarize_container_items(
                [item.camera for item in track.observations],
                normalize_yaw=False,
            )
            base = self._summarize_container_items(
                [item.base for item in track.observations if item.base is not None],
                normalize_yaw=True,
            )
            if camera is not None:
                camera_results.append(camera)
            if base is not None:
                base_results.append(base)
        return camera_results, base_results

    def publish_container_debug_image(
        self,
        source: Image,
        bgr: np.ndarray,
        masks: dict[int, np.ndarray],
        candidates: list[ContainerCandidate],
        camera_matrix: np.ndarray,
        camera_to_base: TransformStamped | None,
    ) -> None:
        debug = bgr.copy()
        tint = np.zeros_like(debug)
        for color, mask in masks.items():
            tint[mask > 0] = DEBUG_COLORS[color]
        debug = cv2.addWeighted(debug, 0.78, tint, 0.22, 0.0)
        accepted = 0
        for candidate in candidates:
            accepted += int(candidate.accepted)
            line_color = (
                (0, 220, 0) if candidate.accepted else (0, 165, 255))
            cv2.drawContours(
                debug, [candidate.contour.astype(np.int32)], -1,
                DEBUG_COLORS[candidate.color], 1)
            corners = np.rint(candidate.corners).astype(np.int32)
            cv2.polylines(
                debug, [corners.reshape(-1, 1, 2)], True,
                line_color, 2, cv2.LINE_AA)
            for point in corners:
                cv2.circle(
                    debug, tuple(point), 3, line_color, -1, cv2.LINE_AA)
            center = tuple(np.rint(
                candidate.corners.mean(axis=0)).astype(int))
            cv2.drawMarker(
                debug, center, line_color, cv2.MARKER_CROSS,
                12, 2, cv2.LINE_AA)
            error = (
                f'{candidate.pose_error:.1f}px'
                if math.isfinite(candidate.pose_error) else '-')
            label = (
                f'{COLOR_NAMES[candidate.color]} {candidate.reason} '
                f'A={candidate.area:.0f} R={candidate.rectangularity:.2f} '
                f'E={error}')
            origin = (
                max(0, int(corners[:, 0].min())),
                max(38, int(corners[:, 1].min()) - 5))
            cv2.putText(
                debug, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                0.38, line_color, 1, cv2.LINE_AA)
        summary = (
            f'raw={len(candidates)} accepted={accepted} exterior-only MVP')
        cv2.rectangle(
            debug, (0, 0), (min(debug.shape[1] - 1, 319), 25),
            (0, 0, 0), -1)
        cv2.putText(
            debug, summary, (5, 17), cv2.FONT_HERSHEY_SIMPLEX,
            0.42, (255, 255, 255), 1, cv2.LINE_AA)
        self.latest_container_debug_frame = ContainerDebugFrame(
            header=copy.deepcopy(source.header),
            image=debug.copy(),
            camera_matrix=camera_matrix.copy(),
            camera_to_base=copy.deepcopy(camera_to_base),
        )
        self.container_debug_image_publisher.publish(
            self._bgr_image_message(source.header, debug))

    @staticmethod
    def _bgr_image_message(header, bgr: np.ndarray) -> Image:
        output = Image()
        output.header = header
        output.height, output.width = bgr.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = bgr.tobytes()
        return output

    def container_target_callback(self, target: PoseStamped) -> None:
        """Project the exact MoveIt TCP target over the cached camera frame."""
        cached = self.latest_container_debug_frame
        if cached is None or cached.camera_to_base is None:
            self.get_logger().warning(
                'Alvo do contêiner recebido sem frame/TF de visão armazenado.',
                throttle_duration_sec=2.0)
            return
        if target.header.frame_id != self.base_frame:
            self.get_logger().warning(
                'Alvo do contêiner fora do referencial base: '
                f'{target.header.frame_id!r}.',
                throttle_duration_sec=2.0)
            return

        transform = cached.camera_to_base.transform
        rotation = rotation_from_quaternion(transform.rotation)
        origin = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        point_base = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ], dtype=np.float64)
        if not np.all(np.isfinite(point_base)):
            self.get_logger().warning('Alvo do contêiner contém posição inválida.')
            return
        point_camera = rotation.T @ (point_base - origin)
        if not np.all(np.isfinite(point_camera)) or point_camera[2] <= 1e-6:
            self.get_logger().warning(
                'Alvo do contêiner está atrás da câmera no frame armazenado.')
            return

        matrix = cached.camera_matrix
        pixel = np.array([
            matrix[0, 0] * point_camera[0] / point_camera[2] + matrix[0, 2],
            matrix[1, 1] * point_camera[1] / point_camera[2] + matrix[1, 2],
        ])
        if not np.all(np.isfinite(pixel)):
            return
        x, y = map(int, np.rint(pixel))
        height, width = cached.image.shape[:2]
        if not (0 <= x < width and 0 <= y < height):
            self.get_logger().warning(
                f'Alvo MoveIt projetado fora da imagem: ({x}, {y}).')
            return

        debug = cached.image.copy()
        color = (255, 0, 255)
        cv2.circle(debug, (x, y), 4, color, -1, cv2.LINE_AA)
        cv2.drawMarker(
            debug, (x, y), color, cv2.MARKER_DIAMOND,
            20, 3, cv2.LINE_AA)
        label_origin = (min(x + 8, max(0, width - 112)), max(16, y - 8))
        cv2.putText(
            debug, 'MoveIt TCP target', label_origin,
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
        self.container_debug_image_publisher.publish(
            self._bgr_image_message(cached.header, debug))

    @staticmethod
    def image_to_bgr8(message: Image) -> np.ndarray:
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError('image dimensions and step must be positive')
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        if buffer.size < height * step:
            raise ValueError('image data is shorter than its declared step')
        rows = buffer[:height * step].reshape(height, step)
        encoding = message.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            mono = rows[:, :width].reshape(height, width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        conversions = {
            'bgr8': (3, None),
            'rgb8': (3, cv2.COLOR_RGB2BGR),
            'bgra8': (4, cv2.COLOR_BGRA2BGR),
            'rgba8': (4, cv2.COLOR_RGBA2BGR),
        }
        if encoding not in conversions:
            raise ValueError(f'unsupported encoding: {message.encoding}')
        channels, conversion = conversions[encoding]
        image = rows[:, :width * channels].reshape(height, width, channels)
        return (
            image.copy() if conversion is None
            else cv2.cvtColor(image, conversion))

    @staticmethod
    def image_to_mono8(message: Image) -> np.ndarray:
        height, width, step = int(message.height), int(message.width), int(message.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError('image dimensions and step must be positive')
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        if buffer.size < height * step:
            raise ValueError('image data is shorter than its declared step')
        rows = buffer[:height * step].reshape(height, step)
        encoding = message.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            return rows[:, :width].copy()
        channels_and_code = {'rgb8': (3, 7), 'bgr8': (3, 6), 'rgba8': (4, 11), 'bgra8': (4, 10)}
        if encoding not in channels_and_code:
            raise ValueError(f'unsupported encoding: {message.encoding}')
        channels, code = channels_and_code[encoding]
        return cv2.cvtColor(rows[:, :width * channels].reshape(height, width, channels), code)

    def publish_detection_debug_image(
            self, source: Image, mono: np.ndarray, detections) -> None:
        """Publish the detector input annotated with raw AprilTag candidates."""
        debug = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        accepted = 0
        for detection in detections:
            is_accepted = (
                detection.hamming <= self.max_hamming
                and detection.decision_margin >= self.min_decision_margin
            )
            accepted += int(is_accepted)
            color = (0, 200, 0) if is_accepted else (0, 0, 255)
            corners = np.rint(np.asarray(detection.corners)).astype(np.int32)
            cv2.polylines(debug, [corners.reshape(-1, 1, 2)], True,
                          color, 2, cv2.LINE_AA)
            center = tuple(map(
                int, np.rint(np.asarray(detection.center)).reshape(2)))
            cv2.circle(debug, center, 3, color, -1, cv2.LINE_AA)
            label = (
                f'id={int(detection.tag_id)} '
                f'm={float(detection.decision_margin):.1f} '
                f'h={int(detection.hamming)}'
            )
            text_origin = (max(0, int(corners[:, 0].min())),
                           max(16, int(corners[:, 1].min()) - 5))
            cv2.putText(debug, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)

        summary = f'raw={len(detections)} accepted={accepted}'
        cv2.rectangle(debug, (0, 0), (min(debug.shape[1] - 1, 245), 24),
                      (0, 0, 0), -1)
        cv2.putText(debug, summary, (6, 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

        output = Image()
        output.header = source.header
        output.height, output.width = debug.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = debug.tobytes()
        self.debug_image_publisher.publish(output)


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
