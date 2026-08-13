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
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
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
        self.declare_parameter('nthreads', 2)
        self.declare_parameter('quad_decimate', 1.0)
        self.declare_parameter('min_decision_margin', 30.0)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('feedback_rate_hz', 5.0)
        self.declare_parameter('suppress_native_pose_warning', True)

        self.warning_filter = (NativeWarningFilter()
                               if bool(self.get_parameter('suppress_native_pose_warning').value)
                               else None)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_hamming = int(self.get_parameter('max_hamming').value)
        self.feedback_period = 1.0 / max(0.1, float(self.get_parameter('feedback_rate_hz').value))
        self.camera_info: CameraInfo | None = None
        self.latest_image_subscription = None
        self.sessions: dict[bytes, Session] = {}
        self.sessions_lock = threading.RLock()
        self.detector = Detector(families=self.get_parameter('family').value,
                                 nthreads=int(self.get_parameter('nthreads').value),
                                 quad_decimate=float(self.get_parameter('quad_decimate').value),
                                 refine_edges=1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_pose_publisher = self.create_publisher(PoseArray, 'apriltags/poses_camera', 10)
        self.pose_publisher = self.create_publisher(PoseArray, 'apriltags/poses', 10)
        self.camera_detection_publisher = self.create_publisher(AprilTagDetectionArray, 'apriltags/detections_camera', 10)
        self.detection_publisher = self.create_publisher(AprilTagDetectionArray, 'apriltags/detections', 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.camera_info_callback, qos_profile_sensor_data)
        self.action_server = ActionServer(self, AnalyzeAprilTags, 'apriltags/analyze',
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          handle_accepted_callback=self.handle_accepted_callback)
        self.get_logger().info('AprilTag detector idle; waiting for /apriltags/analyze goals.')

    def destroy_node(self):
        if self.warning_filter is not None:
            self.warning_filter.close()
            self.warning_filter = None
        return super().destroy_node()

    def goal_callback(self, goal_request) -> GoalResponse:
        seconds = _duration_seconds(goal_request.duration)
        if seconds < 0.0:
            self.get_logger().warning('Rejecting AprilTag goal with negative duration.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        """Run each accepted goal outside the ROS executor worker pool."""
        threading.Thread(target=self.execute_callback, args=(goal_handle,),
                         name='apriltag-action-goal', daemon=True).start()

    def _ensure_image_subscription(self) -> None:
        if self.latest_image_subscription is not None:
            return
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.latest_image_subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback, qos)

    def _remove_image_subscription_if_idle(self) -> None:
        if self.sessions or self.latest_image_subscription is None:
            return
        self.destroy_subscription(self.latest_image_subscription)
        self.latest_image_subscription = None

    def execute_callback(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            goal_handle.executing()
        duration = _duration_seconds(goal_handle.request.duration)
        session = Session(goal_handle=goal_handle, duration=duration)
        key = bytes(goal_handle.goal_id.uuid)
        with self.sessions_lock:
            self.sessions[key] = session
            self._ensure_image_subscription()
        try:
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
                self.sessions.pop(key, None)
                self._remove_image_subscription_if_idle()
                if not self.sessions:
                    self.get_logger().info('AprilTag detector idle.')

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
            self.camera_info = message

    def image_callback(self, message: Image) -> None:
        with self.sessions_lock:
            sessions = list(self.sessions.values())
        if not sessions or self.camera_info is None:
            return
        camera_frame = message.header.frame_id or self.camera_info.header.frame_id
        info = self.camera_info
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
        if camera_poses:
            try:
                base_transform = self.tf_buffer.lookup_transform(self.base_frame, camera_frame,
                                                                  message.header.stamp, timeout=Duration(seconds=0.05))
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
        now = time.monotonic()
        with self.sessions_lock:
            for session in sessions:
                if session.goal_handle.is_cancel_requested:
                    continue
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
    executor = MultiThreadedExecutor(num_threads=3)
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
