"""On-demand AprilTag detection and pose publication."""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import cv2
from cbr_interfaces.action import AnalyzeAprilTags
from cbr_interfaces.msg import AprilTagDetection, AprilTagDetectionArray, AprilTagStampedDetection
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from pupil_apriltags import Detector
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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

    _needle = b"Error, more than one new minima found."

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
    started: float = field(default_factory=time.monotonic)
    frames_processed: int = 0
    frames_with_base_transform: int = 0
    best_camera: dict[int, AprilTagStampedDetection] = field(default_factory=dict)
    best_base: dict[int, AprilTagStampedDetection] = field(default_factory=dict)
    latest_camera: list[AprilTagStampedDetection] = field(default_factory=list)
    latest_base: list[AprilTagStampedDetection] = field(default_factory=list)
    last_feedback: float = 0.0


class AprilTagDetector(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_detector')
        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tag_frame_prefix', 'apriltag')
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('tag_size_m', 0.032)
        self.declare_parameter('nthreads', 1)
        self.declare_parameter('quad_decimate', 1.0)
        self.declare_parameter('max_detection_rate_hz', 10.0)
        self.declare_parameter('min_decision_margin', 30.0)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('feedback_rate_hz', 5.0)
        self.declare_parameter('suppress_native_pose_warning', True)
        self.declare_parameter('manage_camera_capture', True)
        self.declare_parameter(
            'camera_capture_service', '/camera/set_capture')
        self.declare_parameter('camera_capture_timeout_sec', 5.0)
        self.declare_parameter('camera_idle_timeout_sec', 0.0)
        self.declare_parameter('camera_capture_retry_sec', 1.0)

        self.warning_filter = (NativeWarningFilter()
                               if bool(self.get_parameter('suppress_native_pose_warning').value)
                               else None)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_hamming = int(self.get_parameter('max_hamming').value)
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
        self.detector = Detector(families=self.get_parameter('family').value,
                                 nthreads=int(self.get_parameter('nthreads').value),
                                 quad_decimate=float(self.get_parameter('quad_decimate').value),
                                 refine_edges=1)
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
        self.capture_condition = threading.Condition(threading.RLock())
        self.capture_state: bool | None = None
        self.capture_future = None
        self.capture_target: bool | None = None
        self.next_capture_attempt = 0.0
        self.capture_client = None
        self.camera_idle_timer = None
        if self.manage_camera_capture:
            self.camera_capture_service = str(
                self.get_parameter('camera_capture_service').value)
            self.capture_client = self.create_client(
                SetBool,
                self.camera_capture_service,
            )
            self._schedule_camera_stop()
        self.action_server = ActionServer(self, AnalyzeAprilTags, 'apriltags/analyze',
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          handle_accepted_callback=self.handle_accepted_callback)
        self.get_logger().info('AprilTag detector idle; waiting for /apriltags/analyze goals.')

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
            self.get_logger().warning('Rejecting AprilTag goal with negative duration.')
            return GoalResponse.REJECT
        with self.sessions_lock:
            if self.state != 'idle':
                self.get_logger().warning(
                    'Rejecting AprilTag goal: another analysis is active.')
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
                         name='apriltag-action-goal', daemon=True).start()

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
        self.get_logger().info('AprilTag detector idle.')

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
        session = Session(goal_handle=goal_handle, duration=duration)
        with self.sessions_lock:
            self.session = session
            self.state = 'analyzing'
            self.last_detection_time = float('-inf')
        try:
            if not self._wait_for_camera_capture():
                result = self._result(
                    session, 'A câmera não iniciou dentro do tempo limite.')
                goal_handle.abort(result)
                return result
            while rclpy.ok() and goal_handle.is_active:
                time.sleep(0.02)
                elapsed = time.monotonic() - session.started
                if goal_handle.is_cancel_requested:
                    result = self._result(session, 'Canceled; returning accumulated detections.')
                    goal_handle.canceled(result)
                    return result
                if duration > 0.0 and elapsed >= duration:
                    if session.frames_processed == 0:
                        result = self._result(session, 'No calibrated image was processed during the requested window.')
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
            with self.sessions_lock:
                self.session = None
                self.state = 'deactivating'
            self.input_lifecycle_guard.trigger()

    def _feedback(self, session: Session):
        feedback = AnalyzeAprilTags.Feedback()
        feedback.detections_camera = list(session.latest_camera)
        feedback.detections_base = list(session.latest_base)
        feedback.frames_processed = session.frames_processed
        feedback.frames_with_base_transform = session.frames_with_base_transform
        elapsed = time.monotonic() - session.started
        feedback.elapsed = _ros_duration(elapsed)
        feedback.continuous = session.duration == 0.0
        feedback.remaining = _ros_duration(0.0 if feedback.continuous else session.duration - elapsed)
        return feedback

    def _result(self, session: Session, message: str):
        result = AnalyzeAprilTags.Result()
        result.best_detections_camera = list(session.best_camera.values())
        result.best_detections_base = list(session.best_base.values())
        result.frames_processed = session.frames_processed
        result.frames_with_base_transform = session.frames_with_base_transform
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
        with self.sessions_lock:
            if now - self.last_detection_time < self.detection_period:
                return
            self.last_detection_time = now
        camera_frame = message.header.frame_id or info.header.frame_id
        if not camera_frame or info.p[0] <= 0.0 or info.p[5] <= 0.0:
            return
        try:
            image = self.image_to_mono8(message)
        except ValueError as error:
            self.get_logger().warning(f'Could not convert image: {error}', throttle_duration_sec=2.0)
            return
        parameters = (float(info.p[0]), float(info.p[5]), float(info.p[2]), float(info.p[6]))
        detections = self.detector.detect(image, estimate_tag_pose=True,
                                          camera_params=parameters, tag_size=self.tag_size_m)
        valid = [d for d in detections if d.hamming <= self.max_hamming and d.decision_margin >= self.min_decision_margin]
        camera_items: list[AprilTagStampedDetection] = []
        camera_poses: list[PoseStamped] = []
        transforms: list[TransformStamped] = []
        for detection in valid:
            family = detection.tag_family.decode() if isinstance(detection.tag_family, bytes) else str(detection.tag_family)
            try:
                pose = self.to_pose(np.asarray(detection.pose_t), np.asarray(detection.pose_R))
            except (TypeError, ValueError, np.linalg.LinAlgError) as error:
                self.get_logger().warning(
                    f'Ignoring invalid pose for tag {detection.tag_id}: {error}',
                    throttle_duration_sec=2.0)
                continue
            item = self.to_stamped_detection(detection, family, pose, float(detection.pose_err), message.header)
            camera_items.append(item)
            stamped = PoseStamped(header=item.header, pose=pose)
            camera_poses.append(stamped)
            # Tag TF is a camera measurement. Its parent must always be the
            # optical camera frame; base_link is used only for derived poses.
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = camera_frame
            transform.child_frame_id = f'{self.tag_frame_prefix}_{family}_{detection.tag_id}'
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.append(transform)
        base_items: list[AprilTagStampedDetection] = []
        base_poses: list[PoseStamped] = []
        base_transform = None
        if camera_poses and tf_buffer is not None:
            try:
                base_transform = tf_buffer.lookup_transform(
                    self.base_frame, camera_frame, message.header.stamp,
                    timeout=Duration())
            except TransformException:
                pass
        if base_transform is not None:
            for item, pose_camera in zip(camera_items, camera_poses):
                pose_base = do_transform_pose_stamped(pose_camera, base_transform)
                base_items.append(self.to_stamped_detection_from_item(item, pose_base.pose, self.base_frame))
                base_poses.append(pose_base)
        self.camera_pose_publisher.publish(self.pose_array(camera_frame, message, camera_poses))
        self.camera_detection_publisher.publish(self.detection_array(camera_frame, message, camera_items))
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
        self.pose_publisher.publish(self.pose_array(self.base_frame, message, base_poses))
        self.detection_publisher.publish(self.detection_array(self.base_frame, message, base_items))
        with self.sessions_lock:
            if self.session is not session or session.goal_handle.is_cancel_requested:
                return
            session.frames_processed += 1
            if base_transform is not None:
                session.frames_with_base_transform += 1
            session.latest_camera = [self.copy_stamped(x) for x in camera_items]
            session.latest_base = [self.copy_stamped(x) for x in base_items]
            for item in camera_items:
                self._update_best(session.best_camera, item)
            for item in base_items:
                self._update_best(session.best_base, item)

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
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_rotation(rotation)
        return pose

    @staticmethod
    def _update_best(best, item) -> None:
        old = best.get(item.id)
        item_time = (int(item.header.stamp.sec), int(item.header.stamp.nanosec))
        old_time = (int(old.header.stamp.sec), int(old.header.stamp.nanosec)) if old is not None else (0, 0)
        if old is None or (item.pose_error, -item.decision_margin, item.hamming, (-item_time[0], -item_time[1])) < (old.pose_error, -old.decision_margin, old.hamming, (-old_time[0], -old_time[1])):
            best[item.id] = AprilTagDetector.copy_stamped(item)

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
        result = AprilTagDetector.copy_stamped(item)
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


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AprilTagDetector()
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
