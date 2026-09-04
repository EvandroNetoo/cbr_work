import math
import threading
import time
from types import SimpleNamespace

import pytest
from rclpy.action import GoalResponse

import vl53_distance.action_server as action_module
from vl53_distance.action_server import VL53DistanceAction
from vl53_distance.action_server import (
    OdometryPose,
    odometry_pose,
    rightward_displacement_mm,
)
from vl53_distance.control import FollowWallCommand
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


class FakeFollowWallGoal(FakeGoal):
    def __init__(self, *, cancel=False):
        super().__init__(cancel=cancel)
        self.request = SimpleNamespace(
            wall_distance_mm=300,
            travel_distance_mm=500,
            wall_tolerance_mm=10,
            travel_tolerance_mm=10,
            timeout=SimpleNamespace(sec=10, nanosec=0),
        )


class FakeFollowWallController:
    def __init__(self, inside=False):
        self.inside = inside

    def reset(self):
        pass

    def calculate(self, left, right, wall, wall_tolerance, traveled,
                  travel, travel_tolerance, dt):
        del left, right, wall, wall_tolerance, travel_tolerance, dt
        return FollowWallCommand(
            0.02, -0.04, 0.0, 300.0, 0.0, 0.0,
            traveled, travel - traveled, self.inside)


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


class FakeTimer:
    def __init__(self):
        self.reset_count = 0
        self.cancel_count = 0

    def reset(self):
        self.reset_count += 1

    def cancel(self):
        self.cancel_count += 1


def _bare_server(pair):
    server = object.__new__(VL53DistanceAction)
    server._sensor_config = SensorPairConfig()
    server._control_rate_hz = 100000.0
    server._freshness_timeout = 0.2
    server._odom_start_timeout = 0.01
    server._odom_freshness_timeout = 0.2
    server._settle_time = 0.0
    server._failure_limit = 3
    server._wheel_linear_speed = 0.238
    server._kinematic_lever = 0.2225
    server._follow_wall_controller = FakeFollowWallController()
    server._owns_sensor_pair = False
    server._sensor_pair = pair
    server._sensor_pair_factory = lambda: pair
    server._odom_topic = '/odom'
    server._odom_subscription = None
    server._lock = threading.RLock()
    server._resource_lock = threading.RLock()
    server._state = 'idle'
    server._desired_command = (0.0, 0.0, 0.0)
    server._desired_updated = float('-inf')
    server._desired_valid = False
    server._shutdown_event = threading.Event()
    server._goal_wakeup = threading.Event()
    server._worker_thread = None
    server._command_timer = FakeTimer()
    server.create_subscription = lambda *_args: object()
    server.destroy_subscription = lambda _subscription: True
    server._publish_twist = lambda *_args: None
    server.get_logger = lambda: FakeLogger()
    return server


def test_goal_resources_are_active_only_during_execution():
    server = _bare_server(None)
    pair = SimpleNamespace(
        close_count=0,
        close=lambda: None,
    )
    closed = []
    pair.close = lambda: closed.append(True)
    subscriptions = []
    destroyed = []
    server._owns_sensor_pair = True
    server._sensor_pair_factory = lambda: pair
    server.create_subscription = lambda *_args: subscriptions.append(
        object()) or subscriptions[-1]
    server.destroy_subscription = lambda subscription: destroyed.append(
        subscription) or True

    server._activate_goal_resources()

    assert server._sensor_pair is pair
    assert server._odom_subscription is subscriptions[0]
    assert server._command_timer.reset_count == 1

    server._deactivate_goal_resources()

    assert server._sensor_pair is None
    assert server._odom_subscription is None
    assert destroyed == subscriptions
    assert closed == [True]
    assert server._command_timer.cancel_count == 1


def _follow_request(
    wall=300,
    travel=500,
    wall_tolerance=10,
    travel_tolerance=10,
    timeout=10,
):
    return SimpleNamespace(
        wall_distance_mm=wall,
        travel_distance_mm=travel,
        wall_tolerance_mm=wall_tolerance,
        travel_tolerance_mm=travel_tolerance,
        timeout=SimpleNamespace(sec=timeout, nanosec=0),
    )


def test_follow_wall_goal_validation_and_single_goal_reservation():
    server = _bare_server(SequencePair([]))
    assert server._follow_wall_goal_callback(
        _follow_request()).name == GoalResponse.ACCEPT.name
    assert server._follow_wall_goal_callback(
        _follow_request()).name == GoalResponse.REJECT.name

    server._state = 'idle'
    assert server._follow_wall_goal_callback(
        _follow_request(travel=-500)).name == GoalResponse.ACCEPT.name
    server._state = 'idle'
    assert server._follow_wall_goal_callback(
        _follow_request(travel=0)).name == GoalResponse.ACCEPT.name
    server._state = 'idle'
    assert server._follow_wall_goal_callback(
        _follow_request(wall=2000)).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(
        _follow_request(wall_tolerance=0)).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(
        _follow_request(travel_tolerance=0)).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(
        _follow_request(timeout=0)).name == GoalResponse.REJECT.name


def test_command_watchdog_replaces_stale_velocity_with_stop():
    server = _bare_server(SequencePair([]))
    published = []
    server._publish_twist = lambda *command: published.append(command)
    server._state = 'executing'
    server._desired_command = (0.08, -0.04, 0.2)
    server._desired_valid = True
    server._desired_updated = time.monotonic()

    server._publish_command_cycle()
    assert published[-1] == (0.08, -0.04, 0.2)

    server._desired_updated = time.monotonic() - 1.0
    server._publish_command_cycle()
    assert published[-1] == (0.0, 0.0, 0.0)

    server._state = 'idle'
    server._publish_command_cycle()
    assert len(published) == 2


def test_rightward_displacement_uses_initial_robot_axis():
    initial = OdometryPose(10.0, 20.0, 0.0)
    assert rightward_displacement_mm(
        initial, OdometryPose(10.0, 19.5, 0.3)) == 500.0
    assert rightward_displacement_mm(
        initial, OdometryPose(10.0, 20.5, -0.2)) == -500.0

    facing_left = OdometryPose(2.0, 3.0, math.pi / 2.0)
    assert rightward_displacement_mm(
        facing_left, OdometryPose(2.4, 3.0, math.pi / 2.0)) == pytest.approx(400.0)


def test_odometry_pose_normalizes_quaternion_and_rejects_invalid_values():
    message = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sqrt(2.0),
                                    w=math.sqrt(2.0)),
    )))
    pose = odometry_pose(message)
    assert pose.x_m == 1.0
    assert pose.y_m == 2.0
    assert pose.yaw_rad == pytest.approx(math.pi / 2.0)

    message.pose.pose.orientation.z = 0.0
    message.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='quaternion nulo'):
        odometry_pose(message)


def test_follow_wall_aborts_without_initial_odometry(monkeypatch):
    pair = SequencePair([])
    server = _bare_server(pair)
    server._odom_start_timeout = 0.0
    server._odometry_snapshot = lambda _now=None: (None, False)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert not result.has_valid_odometry
    assert pair.read_count == 0
    assert not server._desired_valid


def test_follow_wall_cancel_before_odometry_does_not_read_sensor(monkeypatch):
    pair = SequencePair([])
    server = _bare_server(pair)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal(cancel=True)

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'canceled'
    assert not result.has_valid_odometry
    assert pair.read_count == 0


def test_follow_wall_timeout_stops_before_sensor_read(monkeypatch):
    pair = SequencePair([])
    server = _bare_server(pair)
    pose = OdometryPose(0.0, 0.0, 0.0)
    server._odometry_snapshot = lambda _now=None: (pose, True)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()
    goal.request.timeout = SimpleNamespace(sec=0, nanosec=0)

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert result.has_valid_odometry
    assert 'Timeout' in result.message
    assert pair.read_count == 0


def test_follow_wall_aborts_when_odometry_becomes_stale(monkeypatch):
    pair = SequencePair([])
    server = _bare_server(pair)
    poses = iter([
        (OdometryPose(0.0, 0.0, 0.0), True),
        (OdometryPose(0.0, 0.0, 0.0), False),
    ])
    server._odometry_snapshot = lambda _now=None: next(poses)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert result.has_valid_odometry
    assert 'Odometria' in result.message
    assert pair.read_count == 0


def test_follow_wall_succeeds_after_all_conditions_settle(monkeypatch):
    sample = DistanceSample(406, 348, 300, 300)
    pair = SequencePair([sample, sample])
    server = _bare_server(pair)
    server._follow_wall_controller = FakeFollowWallController(inside=True)
    poses = iter([
        (OdometryPose(0.0, 0.0, 0.0), True),
        (OdometryPose(0.0, -0.5, 0.0), True),
        (OdometryPose(0.0, -0.5, 0.0), True),
    ])
    server._odometry_snapshot = lambda _now=None: next(poses)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'succeeded'
    assert result.has_valid_reading
    assert result.has_valid_odometry
    assert result.traveled_distance_mm == pytest.approx(500.0)
    assert pair.read_count == 2
    assert len(goal.feedback) == 1
    assert goal.feedback[0].traveled_distance_mm == pytest.approx(500.0)
    assert not server._desired_valid


def test_follow_wall_sensor_failure_counter_aborts(monkeypatch):
    pair = SequencePair([
        TimeoutError('um'), TimeoutError('dois'), TimeoutError('três')])
    server = _bare_server(pair)
    pose = OdometryPose(0.0, 0.0, 0.0)
    server._odometry_snapshot = lambda _now=None: (pose, True)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert result.has_valid_odometry
    assert not result.has_valid_reading
    assert [item.consecutive_read_failures for item in goal.feedback] == [1, 2, 3]
