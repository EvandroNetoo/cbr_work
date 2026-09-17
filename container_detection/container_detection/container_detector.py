"""On-demand color segmentation and external-pose estimation for Bin 3."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
import threading
import time
from typing import Iterable

import cv2
from geometry_msgs.msg import Pose, PoseStamped
from interfaces.action import AnalyzeContainers
from interfaces.msg import (
    ContainerDetection,
    ContainerDetectionArray,
    ContainerStampedDetection,
)
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


RED = ContainerStampedDetection.RED
BLUE = ContainerStampedDetection.BLUE
COLOR_NAMES = {RED: 'red', BLUE: 'blue'}
DEBUG_COLORS = {RED: (30, 30, 255), BLUE: (255, 80, 20)}


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _ros_duration(seconds: float):
    from builtin_interfaces.msg import Duration as DurationMsg
    seconds = max(0.0, float(seconds))
    message = DurationMsg()
    message.sec = int(seconds)
    message.nanosec = int((seconds - message.sec) * 1e9)
    return message


def _capture_request_succeeded(response, target: bool) -> bool:
    if response is None:
        return False
    if response.success:
        return True
    expected = 'start capturing' if target else 'stop capturing'
    return response.message.strip().casefold() == expected


def _quaternion_from_rotation(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a finite 3x3 rotation matrix to an XYZW quaternion."""
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
        scale = 2.0 * math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        return (
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        )
    if index == 1:
        scale = 2.0 * math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        return (
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        )
    scale = 2.0 * math.sqrt(
        1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    return (
        (matrix[0, 2] + matrix[2, 0]) / scale,
        (matrix[1, 2] + matrix[2, 1]) / scale,
        0.25 * scale,
        (matrix[1, 0] - matrix[0, 1]) / scale,
    )


@dataclass
class Candidate:
    color: int
    contour: np.ndarray
    corners: np.ndarray
    area: float
    rectangularity: float
    accepted: bool = False
    reason: str = ''
    pose: Pose | None = None
    pose_error: float = math.inf


@dataclass
class Session:
    goal_handle: object
    duration: float
    started: float = field(default_factory=time.monotonic)
    frames_processed: int = 0
    frames_with_base_transform: int = 0
    best_camera: dict[int, ContainerStampedDetection] = field(default_factory=dict)
    best_base: dict[int, ContainerStampedDetection] = field(default_factory=dict)
    latest_camera: list[ContainerStampedDetection] = field(default_factory=list)
    latest_base: list[ContainerStampedDetection] = field(default_factory=list)
    last_feedback: float = 0.0


class ContainerDetector(Node):
    """Segment red/blue silhouettes and fit the known external rectangle."""

    def __init__(self) -> None:
        super().__init__('container_detector')
        defaults = {
            'image_topic': '/camera/image_rect',
            'camera_info_topic': '/camera/camera_info',
            'base_frame': 'base_link',
            'external_height_m': 0.073,
            'external_width_m': 0.102,
            'external_depth_m': 0.173,
            'internal_height_m': 0.057,
            'internal_width_m': 0.090,
            'internal_depth_m': 0.140,
            'min_saturation': 80,
            'min_value': 45,
            'red_hue_low_1': 0,
            'red_hue_high_1': 12,
            'red_hue_low_2': 168,
            'red_hue_high_2': 179,
            'blue_hue_low': 92,
            'blue_hue_high': 138,
            'morphology_kernel_px': 5,
            'min_contour_area_px': 350.0,
            'max_contour_area_fraction': 0.85,
            'min_rectangularity': 0.55,
            'polygon_epsilon_fraction': 0.035,
            'max_pose_error_px': 12.0,
            'max_detection_rate_hz': 10.0,
            'feedback_rate_hz': 5.0,
            'publish_debug_image': True,
            'manage_camera_capture': True,
            'camera_capture_service': '/camera/set_capture',
            'camera_capture_timeout_sec': 5.0,
            'camera_capture_retry_sec': 1.0,
            'manage_vision_led': True,
            'vision_led_service': '/base_hardware/set_vision_led',
            'vision_led_timeout_sec': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.external_width = self._positive_parameter('external_width_m')
        self.external_depth = self._positive_parameter('external_depth_m')
        self._positive_parameter('external_height_m')
        self._positive_parameter('internal_height_m')
        self._positive_parameter('internal_width_m')
        self._positive_parameter('internal_depth_m')
        self.min_saturation = self._byte_parameter('min_saturation')
        self.min_value = self._byte_parameter('min_value')
        self.red_ranges = (
            (self._hue_parameter('red_hue_low_1'),
             self._hue_parameter('red_hue_high_1')),
            (self._hue_parameter('red_hue_low_2'),
             self._hue_parameter('red_hue_high_2')),
        )
        self.blue_range = (
            self._hue_parameter('blue_hue_low'),
            self._hue_parameter('blue_hue_high'),
        )
        kernel_size = int(self.get_parameter('morphology_kernel_px').value)
        if kernel_size <= 0:
            raise ValueError('morphology_kernel_px must be positive')
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.min_contour_area = self._positive_parameter('min_contour_area_px')
        self.max_contour_fraction = float(
            self.get_parameter('max_contour_area_fraction').value)
        self.min_rectangularity = float(
            self.get_parameter('min_rectangularity').value)
        self.polygon_epsilon_fraction = self._positive_parameter(
            'polygon_epsilon_fraction')
        self.max_pose_error = self._positive_parameter('max_pose_error_px')
        if not 0.0 < self.max_contour_fraction <= 1.0:
            raise ValueError('max_contour_area_fraction must be in (0, 1]')
        if not 0.0 < self.min_rectangularity <= 1.0:
            raise ValueError('min_rectangularity must be in (0, 1]')
        detection_rate = self._positive_parameter('max_detection_rate_hz')
        self.detection_period = 1.0 / detection_rate
        self.feedback_period = 1.0 / self._positive_parameter('feedback_rate_hz')
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)

        self.lock = threading.RLock()
        self.session: Session | None = None
        self.state = 'idle'
        self.last_detection_time = float('-inf')
        self.camera_info: CameraInfo | None = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.image_callback, qos_profile_sensor_data)

        self.camera_detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections_camera', 1)
        self.detection_publisher = self.create_publisher(
            ContainerDetectionArray, 'containers/detections', 1)
        self.debug_image_publisher = self.create_publisher(
            Image, 'containers/debug_image', qos_profile_sensor_data)

        self.manage_camera_capture = bool(
            self.get_parameter('manage_camera_capture').value)
        self.camera_capture_service = str(
            self.get_parameter('camera_capture_service').value)
        self.camera_capture_timeout = self._positive_parameter(
            'camera_capture_timeout_sec')
        self.camera_capture_retry = self._positive_parameter(
            'camera_capture_retry_sec')
        self.capture_client = (
            self.create_client(SetBool, self.camera_capture_service)
            if self.manage_camera_capture else None)
        self.capture_condition = threading.Condition(threading.RLock())
        self.capture_state: bool | None = None
        self.capture_future = None
        self.capture_target: bool | None = None
        self.next_capture_attempt = 0.0
        self.camera_stop_timer = None

        self.manage_vision_led = bool(
            self.get_parameter('manage_vision_led').value)
        self.vision_led_service = str(
            self.get_parameter('vision_led_service').value)
        self.vision_led_timeout = self._positive_parameter(
            'vision_led_timeout_sec')
        self.vision_led_client = (
            self.create_client(SetBool, self.vision_led_service)
            if self.manage_vision_led else None)

        self.action_server = ActionServer(
            self, AnalyzeContainers, 'containers/analyze',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )
        self._schedule_camera_stop()
        self.get_logger().info(
            'Container detector idle; waiting for /containers/analyze goals.')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value

    def _byte_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if not 0 <= value <= 255:
            raise ValueError(f'{name} must be in [0, 255]')
        return value

    def _hue_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if not 0 <= value <= 179:
            raise ValueError(f'{name} must be in [0, 179]')
        return value

    def goal_callback(self, request) -> GoalResponse:
        if _duration_seconds(request.duration) < 0.0:
            return GoalResponse.REJECT
        with self.lock:
            if self.state != 'idle':
                self.get_logger().warning(
                    'Rejecting container goal: another analysis is active.')
                return GoalResponse.REJECT
            self.state = 'activating'
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        threading.Thread(
            target=self.execute_callback, args=(goal_handle,),
            name='container-action-goal', daemon=True).start()

    def execute_callback(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            goal_handle.executing()
        session = Session(
            goal_handle=goal_handle,
            duration=_duration_seconds(goal_handle.request.duration),
        )
        with self.lock:
            self.session = session
            self.state = 'analyzing'
            self.last_detection_time = float('-inf')
        led_enabled = False
        try:
            if not self._set_vision_led(True):
                result = self._result(session, 'Vision light could not be enabled.')
                goal_handle.abort(result)
                return result
            led_enabled = self.manage_vision_led
            if not self._wait_for_camera_capture():
                result = self._result(session, 'Camera did not start in time.')
                goal_handle.abort(result)
                return result
            while rclpy.ok() and goal_handle.is_active:
                time.sleep(0.02)
                elapsed = time.monotonic() - session.started
                if goal_handle.is_cancel_requested:
                    result = self._result(
                        session, 'Canceled; returning accumulated detections.')
                    goal_handle.canceled(result)
                    return result
                if session.duration > 0.0 and elapsed >= session.duration:
                    if session.frames_processed == 0:
                        result = self._result(
                            session, 'No calibrated image was processed.')
                        goal_handle.abort(result)
                        return result
                    result = self._result(session, 'Analysis completed.')
                    goal_handle.succeed(result)
                    return result
                if time.monotonic() - session.last_feedback >= self.feedback_period:
                    session.last_feedback = time.monotonic()
                    goal_handle.publish_feedback(self._feedback(session))
        finally:
            if led_enabled:
                self._set_vision_led(False)
            with self.lock:
                self.session = None
                self.state = 'idle'
            self._schedule_camera_stop()

    def _feedback(self, session: Session):
        feedback = AnalyzeContainers.Feedback()
        feedback.detections_camera = list(session.latest_camera)
        feedback.detections_base = list(session.latest_base)
        feedback.frames_processed = session.frames_processed
        feedback.frames_with_base_transform = session.frames_with_base_transform
        elapsed = time.monotonic() - session.started
        feedback.elapsed = _ros_duration(elapsed)
        feedback.continuous = session.duration == 0.0
        feedback.remaining = _ros_duration(
            0.0 if feedback.continuous else session.duration - elapsed)
        return feedback

    def _result(self, session: Session, message: str):
        result = AnalyzeContainers.Result()
        result.best_detections_camera = list(session.best_camera.values())
        result.best_detections_base = list(session.best_base.values())
        result.frames_processed = session.frames_processed
        result.frames_with_base_transform = session.frames_with_base_transform
        result.elapsed = _ros_duration(time.monotonic() - session.started)
        result.message = message
        return result

    def camera_info_callback(self, message: CameraInfo) -> None:
        if message.p[0] > 0.0 and message.p[5] > 0.0:
            self.camera_info = message

    def image_callback(self, message: Image) -> None:
        with self.lock:
            session = self.session
            info = self.camera_info
            if session is None or info is None:
                return
            now = time.monotonic()
            if now - self.last_detection_time < self.detection_period:
                return
            self.last_detection_time = now
        camera_frame = message.header.frame_id or info.header.frame_id
        if not camera_frame:
            return
        try:
            bgr = self.image_to_bgr8(message)
            masks = self.color_masks(bgr)
            matrix = np.array([
                [info.p[0], 0.0, info.p[2]],
                [0.0, info.p[5], info.p[6]],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            candidates = self.detect_candidates(masks, bgr.shape[:2], matrix)
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(
                f'Container frame rejected: {error}', throttle_duration_sec=2.0)
            return

        if self.publish_debug_image:
            self.publish_debug(message, bgr, masks, candidates)
        accepted = [candidate for candidate in candidates if candidate.accepted]
        camera_items = [
            self.to_stamped(candidate, message.header)
            for candidate in accepted
        ]
        base_items: list[ContainerStampedDetection] = []
        base_transform = None
        if camera_items:
            try:
                base_transform = self.tf_buffer.lookup_transform(
                    self.base_frame, camera_frame, message.header.stamp,
                    timeout=Duration())
            except TransformException:
                pass
        if base_transform is not None:
            for item in camera_items:
                stamped = PoseStamped(header=item.header, pose=item.pose)
                transformed = do_transform_pose_stamped(stamped, base_transform)
                base_items.append(self.copy_stamped(
                    item, transformed.pose, self.base_frame))

        self.camera_detection_publisher.publish(
            self.detection_array(camera_frame, message, camera_items))
        self.detection_publisher.publish(
            self.detection_array(self.base_frame, message, base_items))
        with self.lock:
            if self.session is not session or session.goal_handle.is_cancel_requested:
                return
            session.frames_processed += 1
            if base_transform is not None:
                session.frames_with_base_transform += 1
            session.latest_camera = camera_items
            session.latest_base = base_items
            for item in camera_items:
                self._update_best(session.best_camera, item)
            for item in base_items:
                self._update_best(session.best_base, item)

    def color_masks(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        saturation = self.min_saturation
        value = self.min_value
        red = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in self.red_ranges:
            red = cv2.bitwise_or(
                red, cv2.inRange(hsv, (low, saturation, value), (high, 255, 255)))
        blue = cv2.inRange(
            hsv,
            (self.blue_range[0], saturation, value),
            (self.blue_range[1], 255, 255),
        )
        output = {}
        for color, mask in ((RED, red), (BLUE, blue)):
            cleaned = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.morphology_kernel)
            cleaned = cv2.morphologyEx(
                cleaned, cv2.MORPH_CLOSE, self.morphology_kernel)
            output[color] = cleaned
        return output

    def detect_candidates(
        self,
        masks: dict[int, np.ndarray],
        image_shape: tuple[int, int],
        camera_matrix: np.ndarray,
    ) -> list[Candidate]:
        image_area = float(image_shape[0] * image_shape[1])
        candidates = []
        for color, mask in masks.items():
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                rectangle = cv2.minAreaRect(contour)
                rectangle_area = float(rectangle[1][0] * rectangle[1][1])
                rectangularity = area / rectangle_area if rectangle_area > 0.0 else 0.0
                perimeter = float(cv2.arcLength(contour, True))
                approximation = cv2.approxPolyDP(
                    contour, self.polygon_epsilon_fraction * perimeter, True)
                if len(approximation) == 4 and cv2.isContourConvex(approximation):
                    corners = approximation.reshape(4, 2).astype(np.float64)
                else:
                    corners = cv2.boxPoints(rectangle).astype(np.float64)
                candidate = Candidate(
                    color=color, contour=contour, corners=corners,
                    area=area, rectangularity=rectangularity)
                if area < self.min_contour_area:
                    candidate.reason = 'small'
                elif area > image_area * self.max_contour_fraction:
                    candidate.reason = 'large'
                elif rectangularity < self.min_rectangularity:
                    candidate.reason = 'shape'
                elif self._touches_border(corners, image_shape):
                    candidate.reason = 'border'
                else:
                    pose, error = self.estimate_pose(corners, camera_matrix)
                    candidate.pose = pose
                    candidate.pose_error = error
                    if pose is None or error > self.max_pose_error:
                        candidate.reason = 'pose'
                    else:
                        candidate.accepted = True
                        candidate.reason = 'ok'
                candidates.append(candidate)
        return candidates

    @staticmethod
    def _touches_border(corners: np.ndarray, shape: tuple[int, int]) -> bool:
        height, width = shape
        return bool(
            np.any(corners[:, 0] <= 1.0)
            or np.any(corners[:, 1] <= 1.0)
            or np.any(corners[:, 0] >= width - 2.0)
            or np.any(corners[:, 1] >= height - 2.0)
        )

    def estimate_pose(
        self, corners: np.ndarray, camera_matrix: np.ndarray,
    ) -> tuple[Pose | None, float]:
        center = corners.mean(axis=0)
        angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
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
                error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
                if not math.isfinite(error):
                    continue
                if best is None or error < best[0]:
                    best = (error, rotation_vector, translation)
        if best is None:
            return None, math.inf
        rotation, _ = cv2.Rodrigues(best[1])
        translation = best[2].reshape(3)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, translation)
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = _quaternion_from_rotation(rotation)
        return pose, best[0]

    def publish_debug(
        self,
        source: Image,
        bgr: np.ndarray,
        masks: dict[int, np.ndarray],
        candidates: list[Candidate],
    ) -> None:
        debug = bgr.copy()
        tint = np.zeros_like(debug)
        for color, mask in masks.items():
            tint[mask > 0] = DEBUG_COLORS[color]
        debug = cv2.addWeighted(debug, 0.78, tint, 0.22, 0.0)
        accepted = 0
        for candidate in candidates:
            accepted += int(candidate.accepted)
            line_color = (0, 220, 0) if candidate.accepted else (0, 165, 255)
            contour = candidate.contour.astype(np.int32)
            cv2.drawContours(debug, [contour], -1, DEBUG_COLORS[candidate.color], 1)
            corners = np.rint(candidate.corners).astype(np.int32)
            cv2.polylines(
                debug, [corners.reshape(-1, 1, 2)], True,
                line_color, 2, cv2.LINE_AA)
            for point in corners:
                cv2.circle(debug, tuple(point), 3, line_color, -1, cv2.LINE_AA)
            center = tuple(np.rint(candidate.corners.mean(axis=0)).astype(int))
            cv2.drawMarker(
                debug, center, line_color, cv2.MARKER_CROSS, 12, 2,
                cv2.LINE_AA)
            error = (
                f'{candidate.pose_error:.1f}px'
                if math.isfinite(candidate.pose_error) else '-')
            label = (
                f'{COLOR_NAMES[candidate.color]} {candidate.reason} '
                f'A={candidate.area:.0f} R={candidate.rectangularity:.2f} E={error}')
            origin = (
                max(0, int(corners[:, 0].min())),
                max(38, int(corners[:, 1].min()) - 5),
            )
            cv2.putText(
                debug, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                0.38, line_color, 1, cv2.LINE_AA)
        summary = f'raw={len(candidates)} accepted={accepted} exterior-only MVP'
        cv2.rectangle(
            debug, (0, 0), (min(debug.shape[1] - 1, 319), 25),
            (0, 0, 0), -1)
        cv2.putText(
            debug, summary, (5, 17), cv2.FONT_HERSHEY_SIMPLEX,
            0.42, (255, 255, 255), 1, cv2.LINE_AA)
        output = Image()
        output.header = source.header
        output.height, output.width = debug.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = debug.tobytes()
        self.debug_image_publisher.publish(output)

    @staticmethod
    def image_to_bgr8(message: Image) -> np.ndarray:
        height, width, step = int(message.height), int(message.width), int(message.step)
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
        return image.copy() if conversion is None else cv2.cvtColor(image, conversion)

    @staticmethod
    def to_stamped(candidate: Candidate, header) -> ContainerStampedDetection:
        item = ContainerStampedDetection()
        item.header = header
        item.color = candidate.color
        item.contour_area_px = candidate.area
        item.rectangularity = candidate.rectangularity
        item.pose_error = candidate.pose_error
        item.pose = candidate.pose
        return item

    @staticmethod
    def copy_stamped(
        item: ContainerStampedDetection, pose: Pose | None = None,
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
        result.pose = item.pose if pose is None else pose
        return result

    @staticmethod
    def detection_array(
        frame: str, image: Image,
        items: list[ContainerStampedDetection],
    ) -> ContainerDetectionArray:
        output = ContainerDetectionArray()
        output.header.frame_id = frame
        output.header.stamp = image.header.stamp
        for item in items:
            detection = ContainerDetection()
            detection.color = item.color
            detection.contour_area_px = item.contour_area_px
            detection.rectangularity = item.rectangularity
            detection.pose_error = item.pose_error
            detection.pose = item.pose
            output.detections.append(detection)
        return output

    @staticmethod
    def _update_best(best: dict, item: ContainerStampedDetection) -> None:
        previous = best.get(item.color)
        score = (item.pose_error, -item.rectangularity, -item.contour_area_px)
        if previous is None:
            best[item.color] = ContainerDetector.copy_stamped(item)
            return
        previous_score = (
            previous.pose_error,
            -previous.rectangularity,
            -previous.contour_area_px,
        )
        if score < previous_score:
            best[item.color] = ContainerDetector.copy_stamped(item)

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
                    self.next_capture_attempt = time.monotonic() + self.camera_capture_retry
            except Exception as error:
                self.next_capture_attempt = time.monotonic() + self.camera_capture_retry
                self.get_logger().warning(
                    f'Camera capture service failed: {error}',
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
        return False

    def _schedule_camera_stop(self) -> None:
        if self.manage_camera_capture and self.camera_stop_timer is None:
            self.camera_stop_timer = self.create_timer(0.25, self._stop_camera_when_idle)

    def _stop_camera_when_idle(self) -> None:
        with self.lock:
            if self.state != 'idle':
                return
        stopped = self._begin_capture_request(False)
        with self.capture_condition:
            confirmed = self.capture_state is False
        if stopped or confirmed:
            timer = self.camera_stop_timer
            self.camera_stop_timer = None
            if timer is not None:
                self.destroy_timer(timer)

    def _set_vision_led(self, enabled: bool) -> bool:
        if not self.manage_vision_led:
            return True
        if self.vision_led_client is None or not self.vision_led_client.wait_for_service(
            timeout_sec=self.vision_led_timeout
        ):
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self.vision_led_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=self.vision_led_timeout):
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return response is not None and response.success


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ContainerDetector()
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
