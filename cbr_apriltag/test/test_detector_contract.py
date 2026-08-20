from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import yaml
from cbr_interfaces.msg import AprilTagStampedDetection
from geometry_msgs.msg import Pose

try:
    import pupil_apriltags  # noqa: F401
except ModuleNotFoundError:
    pupil_apriltags = ModuleType('pupil_apriltags')
    pupil_apriltags.Detector = object
    sys.modules['pupil_apriltags'] = pupil_apriltags

from cbr_apriltag.apriltag_detector import (
    AprilTagDetector,
    _capture_request_succeeded,
)


PACKAGE = Path(__file__).parents[1]


def _item(tag_id, error, margin, hamming=0, stamp=0):
    item = AprilTagStampedDetection()
    item.id = tag_id
    item.pose_error = error
    item.decision_margin = margin
    item.hamming = hamming
    item.header.stamp.nanosec = stamp
    return item


def test_action_and_lifetime_subscription_contract():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert 'ActionServer' in source
    assert "'apriltags/analyze'" in source
    assert 'handle_accepted_callback=self.handle_accepted_callback' in source
    assert "name='apriltag-action-goal'" in source
    assert 'self._create_inputs()' in source
    assert 'self._destroy_inputs_locked()' in source
    execute_callback = source.split('    def execute_callback', 1)[1].split(
        '    def _feedback', 1)[0]
    execute_finally = execute_callback.split('        finally:', 1)[1]
    assert 'destroy_subscription' not in execute_finally
    assert '.unregister()' not in execute_finally
    main_finally = source.rsplit('    finally:', 1)[1]
    assert main_finally.index('executor.shutdown()') < main_finally.index(
        'node.destroy_node()')
    assert 'estimate_tag_pose=True' in source
    assert 'camera_params=parameters' in source
    assert 'tag_size=self.tag_size_m' in source


def test_one_detection_pass_is_shared_by_all_sessions():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert source.count('self.detector.detect(') == 1
    assert 'for session in sessions:' in source
    assert 'SingleThreadedExecutor()' in source
    assert 'MultiThreadedExecutor' not in source


def test_real_profile_stops_camera_while_idle():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'apriltag.yaml').read_text())
    parameters = config['apriltag_detector']['ros__parameters']
    assert parameters['manage_camera_capture'] is True
    assert parameters['camera_capture_service'] == '/camera/set_capture'
    assert parameters['camera_capture_timeout_sec'] == 5.0
    assert parameters['camera_idle_timeout_sec'] == 0.5
    assert parameters['camera_capture_retry_sec'] == 1.0
    assert parameters['max_detection_rate_hz'] == 10.0

    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert 'SetBool' in source
    assert 'self._stop_camera_if_idle' in source
    assert 'self._wait_for_camera_capture()' in source
    assert 'now - self.last_detection_time < self.detection_period' in source


def test_continuous_apriltag_outputs_keep_only_latest_sample():
    source = (PACKAGE / 'cbr_apriltag' / 'apriltag_detector.py').read_text()
    assert 'output_qos = QoSProfile(' in source
    assert 'depth=1' in source


def test_usb_cam_start_and_stop_responses_are_treated_as_success():
    # usb_cam 0.8.x reports the operation in the message and leaves success false.
    started = SimpleNamespace(success=False, message='Start Capturing')
    stopped = SimpleNamespace(success=False, message='Stop Capturing')
    assert _capture_request_succeeded(started, True)
    assert not _capture_request_succeeded(started, False)
    assert _capture_request_succeeded(stopped, False)
    assert not _capture_request_succeeded(stopped, True)


def test_capture_service_failures_are_not_hidden():
    failed = SimpleNamespace(success=False, message='device unavailable')
    assert not _capture_request_succeeded(failed, False)
    assert not _capture_request_succeeded(None, False)


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
