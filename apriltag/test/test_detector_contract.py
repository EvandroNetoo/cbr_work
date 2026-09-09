from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import numpy as np
import yaml
from interfaces.msg import AprilTagStampedDetection
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

try:
    import pupil_apriltags  # noqa: F401
except ModuleNotFoundError:
    pupil_apriltags = ModuleType('pupil_apriltags')
    pupil_apriltags.Detector = object
    sys.modules['pupil_apriltags'] = pupil_apriltags

from apriltag.apriltag_detector import (
    AprilTagDetector,
    _capture_request_succeeded,
)


PACKAGE = Path(__file__).parents[1]


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _CompletedFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response


class _VisionLedClient:
    def __init__(self):
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request.data)
        return _CompletedFuture(SetBool.Response(success=True, message='ok'))


def _item(tag_id, error, margin, hamming=0, stamp=0):
    item = AprilTagStampedDetection()
    item.id = tag_id
    item.pose_error = error
    item.decision_margin = margin
    item.hamming = hamming
    item.header.stamp.nanosec = stamp
    return item


def test_action_activates_inputs_on_demand_and_tears_them_down_on_executor():
    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    assert 'ActionServer' in source
    assert "'apriltags/analyze'" in source
    assert 'handle_accepted_callback=self.handle_accepted_callback' in source
    assert "name='apriltag-action-goal'" in source
    assert 'self._create_inputs_locked()' in source
    assert 'self._destroy_inputs_locked()' in source
    assert 'self.input_lifecycle_guard.trigger()' in source
    assert 'def _deactivate_inputs_from_executor' in source
    execute_callback = source.split('    def execute_callback', 1)[1].split(
        '    def _feedback', 1)[0]
    assert "self.state = 'deactivating'" in execute_callback
    main_finally = source.rsplit('    finally:', 1)[1]
    assert main_finally.index('executor.shutdown()') < main_finally.index(
        'node.destroy_node()')
    assert 'estimate_tag_pose=True' in source
    assert 'camera_params=parameters' in source
    assert 'tag_size=self.tag_size_m' in source


def test_one_detection_pass_is_used_by_the_single_active_session():
    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    assert source.count('self.detector.detect(') == 1
    assert 'self.session is not session' in source
    assert "self.state != 'idle'" in source
    assert 'SingleThreadedExecutor()' in source
    assert 'MultiThreadedExecutor' not in source


def test_goal_lifecycle_rejects_concurrency_and_deactivates_inputs():
    detector = object.__new__(AprilTagDetector)
    detector.sessions_lock = threading.RLock()
    detector.state = 'idle'
    detector.get_logger = lambda: _Logger()
    created = []
    destroyed = []
    stopped = []
    detector._create_inputs_locked = lambda: created.append(True)
    detector._destroy_inputs_locked = lambda: destroyed.append(True)
    detector._schedule_camera_stop = lambda: stopped.append(True)
    request = SimpleNamespace(duration=SimpleNamespace(sec=1, nanosec=0))

    assert detector.goal_callback(request).name == 'ACCEPT'
    assert detector.state == 'activating'
    assert created == [True]
    assert detector.goal_callback(request).name == 'REJECT'

    detector.state = 'deactivating'
    detector._deactivate_inputs_from_executor()
    assert detector.state == 'idle'
    assert destroyed == [True]
    assert stopped == [True]


def test_real_profile_stops_camera_while_idle():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'apriltag.yaml').read_text())
    parameters = config['apriltag_detector']['ros__parameters']
    assert parameters['manage_camera_capture'] is True
    assert parameters['camera_capture_service'] == '/camera/set_capture'
    assert parameters['camera_capture_timeout_sec'] == 5.0
    assert parameters['camera_idle_timeout_sec'] == 0.0
    assert parameters['camera_capture_retry_sec'] == 1.0
    assert parameters['max_detection_rate_hz'] == 10.0
    assert parameters['manage_vision_led'] is True
    assert parameters['vision_led_service'] == '/base_hardware/set_vision_led'
    assert parameters['vision_led_timeout_sec'] == 5.0

    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    assert 'SetBool' in source
    assert 'self._schedule_camera_stop()' in source
    assert 'def _stop_camera_when_idle' in source
    assert 'self._wait_for_camera_capture()' in source
    assert 'now - self.last_detection_time < self.detection_period' in source


def test_vision_led_is_controlled_through_base_hardware_service():
    detector = object.__new__(AprilTagDetector)
    detector.manage_vision_led = True
    detector.vision_led_timeout = 0.1
    detector.vision_led_service = '/base_hardware/set_vision_led'
    detector.vision_led_client = _VisionLedClient()
    detector.get_logger = lambda: _Logger()

    assert detector._set_vision_led(True)
    assert detector._set_vision_led(False)
    assert detector.vision_led_client.requests == [True, False]

    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    execute_callback = source.split('    def execute_callback', 1)[1].split(
        '    def _feedback', 1)[0]
    assert execute_callback.index('self._set_vision_led(True)') < (
        execute_callback.index('self._wait_for_camera_capture()'))
    assert 'self._set_vision_led(False)' in execute_callback


def test_continuous_apriltag_outputs_keep_only_latest_sample():
    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
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
    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    assert "'apriltags/detections_camera'" in source
    assert "'apriltags/detections'" in source
    assert 'AprilTagDetectionArray' in source
    assert 'to_stamped_detection_from_item' in source
    assert 'transform.header.frame_id = camera_frame' in source
    assert 'transform.child_frame_id' in source


def test_debug_image_shows_raw_and_filtered_detections():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'apriltag.yaml').read_text())
    parameters = config['apriltag_detector']['ros__parameters']
    assert parameters['publish_debug_image'] is True

    source = (PACKAGE / 'apriltag' / 'apriltag_detector.py').read_text()
    assert "'apriltags/debug_image'" in source
    assert "output.encoding = 'bgr8'" in source
    assert 'publish_detection_debug_image(message, image, detections)' in source
    assert 'raw={len(detections)} accepted={accepted}' in source


def test_debug_image_is_publishable_and_marks_acceptance():
    detector = object.__new__(AprilTagDetector)
    detector.max_hamming = 0
    detector.min_decision_margin = 30.0
    published = []
    detector.debug_image_publisher = SimpleNamespace(publish=published.append)
    source = Image()
    source.header.frame_id = 'camera_optical_frame'
    mono = np.full((80, 100), 127, dtype=np.uint8)
    accepted = SimpleNamespace(
        tag_id=1, decision_margin=45.0, hamming=0,
        corners=np.array([[10, 30], [35, 30], [35, 55], [10, 55]]),
        center=np.array([22.5, 42.5]),
    )
    rejected = SimpleNamespace(
        tag_id=2, decision_margin=20.0, hamming=0,
        corners=np.array([[60, 30], [85, 30], [85, 55], [60, 55]]),
        center=np.array([72.5, 42.5]),
    )

    detector.publish_detection_debug_image(
        source, mono, [accepted, rejected])

    assert len(published) == 1
    output = published[0]
    assert output.header.frame_id == 'camera_optical_frame'
    assert (output.height, output.width, output.step) == (80, 100, 300)
    assert output.encoding == 'bgr8'
    pixels = np.frombuffer(output.data, dtype=np.uint8).reshape(80, 100, 3)
    assert np.any((pixels[:, :, 1] == 200) & (pixels[:, :, 2] == 0))
    assert np.any((pixels[:, :, 2] == 255) & (pixels[:, :, 1] == 0))


def test_action_interface_declares_camera_and_base_results():
    action = (PACKAGE.parent / 'interfaces' / 'action' / 'AnalyzeAprilTags.action').read_text()
    assert 'best_detections_camera' in action
    assert 'best_detections_base' in action
    assert 'frames_with_base_transform' in action
    assert 'bool continuous' in action


def test_native_pose_arrays_are_converted_to_ros_pose():
    pose = AprilTagDetector.to_pose(np.array([[1.0], [2.0], [3.0]]), np.eye(3))
    assert isinstance(pose, Pose)
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, 2.0, 3.0)
