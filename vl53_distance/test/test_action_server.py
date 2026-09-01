import threading
import time
from types import SimpleNamespace

from rclpy.action import GoalResponse

import vl53_distance.action_server as action_module
from vl53_distance.action_server import VL53DistanceAction
from vl53_distance.control import ControlCommand
from vl53_distance.sensor_pair import DistanceSample, SensorPairConfig


class FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class FakeGoal:
    def __init__(self, *, cancel=False):
        self.request = SimpleNamespace(
            distance_mm=300,
            tolerance_mm=10,
            timeout=SimpleNamespace(sec=10, nanosec=0),
        )
        self.is_cancel_requested = cancel
        self.is_active = True
        self.terminal = None
        self.feedback = []

    def executing(self):
        pass

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self, _result):
        self.terminal = 'succeeded'
        self.is_active = False

    def abort(self, _result):
        self.terminal = 'aborted'
        self.is_active = False

    def canceled(self, _result):
        self.terminal = 'canceled'
        self.is_active = False


class FakeController:
    def __init__(self, inside=False):
        self.inside = inside

    def reset(self):
        pass

    def calculate(self, left, right, target, tolerance, dt):
        del left, right, target, tolerance, dt
        return ControlCommand(0.02, 0.0, 300.0, 0.0, 0.0, self.inside)


class SequencePair:
    def __init__(self, values):
        self.values = list(values)
        self.read_count = 0

    def reset_filter(self):
        pass

    def read(self):
        self.read_count += 1
        value = self.values.pop(0)
        if isinstance(value, Exception):
            raise value
        return value


def _bare_server(pair, controller=None):
    server = object.__new__(VL53DistanceAction)
    server._sensor_config = SensorPairConfig()
    server._control_rate_hz = 100000.0
    server._freshness_timeout = 0.2
    server._settle_time = 0.0
    server._failure_limit = 3
    server._controller = controller or FakeController()
    server._sensor_pair = pair
    server._lock = threading.RLock()
    server._state = 'idle'
    server._desired_command = (0.0, 0.0)
    server._desired_updated = float('-inf')
    server._desired_valid = False
    server._shutdown_event = threading.Event()
    server._goal_wakeup = threading.Event()
    server._worker_thread = None
    server._publish_twist = lambda *_args: None
    server.get_logger = lambda: FakeLogger()
    return server


def _request(distance=300, tolerance=10, timeout=10):
    return SimpleNamespace(
        distance_mm=distance,
        tolerance_mm=tolerance,
        timeout=SimpleNamespace(sec=timeout, nanosec=0),
    )


def test_goal_validation_and_single_goal_reservation():
    server = _bare_server(SequencePair([]))
    assert server._goal_callback(_request()).name == GoalResponse.ACCEPT.name
    assert server._state == 'accepted'
    assert server._goal_callback(_request()).name == GoalResponse.REJECT.name

    server._state = 'idle'
    assert server._goal_callback(_request(distance=2000)).name == GoalResponse.REJECT.name
    assert server._goal_callback(_request(tolerance=0)).name == GoalResponse.REJECT.name
    assert server._goal_callback(_request(timeout=0)).name == GoalResponse.REJECT.name


def test_failure_counter_resets_after_valid_sample_and_aborts_on_third_failure(
    monkeypatch,
):
    sample = DistanceSample(406, 348, 300, 300)
    pair = SequencePair([
        OSError('transiente'), sample,
        TimeoutError('um'), TimeoutError('dois'), TimeoutError('três'),
    ])
    server = _bare_server(pair)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeGoal()

    server._execute_goal(goal)

    assert goal.terminal == 'aborted'
    assert pair.read_count == 5
    assert [item.consecutive_read_failures for item in goal.feedback] == [
        1, 0, 1, 2, 3]
    assert not server._desired_valid
    assert server._state == 'idle'


def test_both_readings_must_remain_in_tolerance_before_success(monkeypatch):
    sample = DistanceSample(406, 348, 300, 300)
    pair = SequencePair([sample, sample])
    server = _bare_server(pair, controller=FakeController(inside=True))
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeGoal()

    result = server._execute_goal(goal)

    assert goal.terminal == 'succeeded'
    assert pair.read_count == 2
    assert result.has_valid_reading
    assert result.final_left_distance_mm == 300
    assert result.final_right_distance_mm == 300
    assert not server._desired_valid


def test_cancel_before_first_read_stops_without_touching_sensor(monkeypatch):
    pair = SequencePair([])
    server = _bare_server(pair)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeGoal(cancel=True)

    server._execute_goal(goal)

    assert goal.terminal == 'canceled'
    assert pair.read_count == 0
    assert not server._desired_valid


def test_command_watchdog_replaces_stale_velocity_with_stop():
    server = _bare_server(SequencePair([]))
    published = []
    server._publish_twist = lambda *command: published.append(command)
    server._state = 'executing'
    server._desired_command = (0.08, 0.2)
    server._desired_valid = True
    server._desired_updated = time.monotonic()

    server._publish_command_cycle()
    assert published[-1] == (0.08, 0.2)

    server._desired_updated = time.monotonic() - 1.0
    server._publish_command_cycle()
    assert published[-1] == (0.0, 0.0)

    server._state = 'idle'
    server._publish_command_cycle()
    assert len(published) == 2
