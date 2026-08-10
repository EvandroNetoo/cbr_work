"""Publish AprilTag poses in the robot base frame from a calibrated ROS camera."""

from __future__ import annotations

import math
from typing import Iterable

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from pupil_apriltags import Detector
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_from_rotation(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized ROS quaternion."""
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
        return (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale)
    if index == 1:
        scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        return ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale)
    scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    return ((matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale, (matrix[1, 0] - matrix[0, 1]) / scale)


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

        self.base_frame = self.get_parameter('base_frame').value
        self.tag_frame_prefix = self.get_parameter('tag_frame_prefix').value
        self.tag_size_m = float(self.get_parameter('tag_size_m').value)
        if self.tag_size_m <= 0.0:
            raise ValueError('tag_size_m must be positive')
        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_hamming = int(self.get_parameter('max_hamming').value)
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.detector = Detector(
            families=self.get_parameter('family').value,
            nthreads=int(self.get_parameter('nthreads').value),
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            refine_edges=1,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(PoseArray, 'apriltags/poses', 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info('Waiting for calibrated image and CameraInfo topics.')

    def camera_info_callback(self, message: CameraInfo) -> None:
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            self.get_logger().warn('Ignoring CameraInfo without valid focal lengths.', throttle_duration_sec=5.0)
            return
        self.camera_info = message

    def image_callback(self, message: Image) -> None:
        if self.camera_info is None:
            self.get_logger().warn('No valid CameraInfo received; pose estimation is disabled.', throttle_duration_sec=5.0)
            return
        camera_frame = message.header.frame_id or self.camera_info.header.frame_id
        if not camera_frame:
            self.get_logger().error('Image and CameraInfo must specify an optical frame_id.', throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert image: {error}')
            return
        info = self.camera_info
        parameters = (float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5]))
        detections = self.detector.detect(image, estimate_tag_pose=True, camera_params=parameters, tag_size=self.tag_size_m)
        valid = [d for d in detections if d.hamming <= self.max_hamming and d.decision_margin >= self.min_decision_margin]
        try:
            base_from_camera = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, message.header.stamp, timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warn(f'No TF {self.base_frame} <- {camera_frame}: {error}', throttle_duration_sec=2.0)
            return

        output = PoseArray()
        output.header.frame_id = self.base_frame
        output.header.stamp = message.header.stamp
        transforms: list[TransformStamped] = []
        for detection in valid:
            pose_camera = PoseStamped()
            pose_camera.header = message.header
            pose_camera.pose = self.to_pose(detection.pose_t, detection.pose_R)
            pose_base = do_transform_pose_stamped(pose_camera, base_from_camera)
            output.poses.append(pose_base.pose)
            transform = TransformStamped()
            transform.header = pose_base.header
            family = detection.tag_family.decode() if isinstance(detection.tag_family, bytes) else detection.tag_family
            transform.child_frame_id = f'{self.tag_frame_prefix}_{family}_{detection.tag_id}'
            transform.transform.translation.x = pose_base.pose.position.x
            transform.transform.translation.y = pose_base.pose.position.y
            transform.transform.translation.z = pose_base.pose.position.z
            transform.transform.rotation = pose_base.pose.orientation
            transforms.append(transform)
        self.pose_publisher.publish(output)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    @staticmethod
    def to_pose(translation: np.ndarray, rotation: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, translation.ravel())
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_rotation(rotation)
        return pose


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
