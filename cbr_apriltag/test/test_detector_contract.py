from pathlib import Path

import numpy as np
from cbr_interfaces.msg import AprilTagStampedDetection
from geometry_msgs.msg import Pose

from cbr_apriltag.apriltag_detector import AprilTagDetector


PACKAGE = Path(__file__).parents[1]


def _item(tag_id, error, margin, hamming=0, stamp=0):
    item = AprilTagStampedDetection()
    item.id = tag_id
    item.pose_error = error
    item.decision_margin = margin
    item.hamming = hamming
    item.header.stamp.nanosec = stamp
    return item


def test_action_and_idle_subscription_contract():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert 'ActionServer' in source
    assert "'apriltags/analyze'" in source
    assert 'handle_accepted_callback=self.handle_accepted_callback' in source
    assert "name='apriltag-action-goal'" in source
    assert 'self.latest_image_subscription = None' in source
    assert 'self._ensure_image_subscription()' in source
    assert 'self._remove_image_subscription_if_idle()' in source
    assert 'estimate_tag_pose=True' in source
    assert 'camera_params=parameters' in source
    assert 'tag_size=self.tag_size_m' in source


def test_one_detection_pass_is_shared_by_all_sessions():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert source.count('self.detector.detect(') == 1
    assert 'for session in sessions:' in source
    assert 'MultiThreadedExecutor(num_threads=3)' in source


def test_best_detection_order_is_error_margin_hamming_then_time():
    best = {}
    AprilTagDetector._update_best(best, _item(4, 2.0, 80.0, 0, 10))
    AprilTagDetector._update_best(best, _item(4, 1.0, 20.0, 1, 20))
    assert best[4].pose_error == 1.0
    AprilTagDetector._update_best(best, _item(4, 1.0, 30.0, 0, 30))
    assert best[4].decision_margin == 30.0
    AprilTagDetector._update_best(best, _item(4, 1.0, 30.0, 0, 40))
    assert best[4].header.stamp.nanosec == 40


def test_topics_keep_legacy_identity_and_pose_messages():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert "'apriltags/detections_camera'" in source
    assert "'apriltags/detections'" in source
    assert 'AprilTagDetectionArray' in source
    assert 'to_stamped_detection_from_item' in source
    assert 'transform.header.frame_id = camera_frame' in source
    assert 'transform.child_frame_id' in source


def test_action_interface_declares_camera_and_base_results():
    action = (PACKAGE.parent / 'cbr_interfaces' / 'action' / 'AnalyzeAprilTags.action').read_text()
    assert 'best_detections_camera' in action
    assert 'best_detections_base' in action
    assert 'frames_with_base_transform' in action
    assert 'bool continuous' in action


def test_native_pose_arrays_are_converted_to_ros_pose():
    pose = AprilTagDetector.to_pose(np.array([[1.0], [2.0], [3.0]]), np.eye(3))
    assert isinstance(pose, Pose)
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, 2.0, 3.0)
