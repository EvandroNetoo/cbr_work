import math
import threading
import time
from types import SimpleNamespace

import pytest
from rclpy.action import GoalResponse

import vl53_distance.action_server as action_module
from vl53_distance.action_server import (
    odometry_pose,
    OdometryPose,
    rightward_displacement_mm,
    VL53DistanceAction,
)
from vl53_distance.control import FollowWallCommand
from vl53_distance.lateral_safety import LateralClearances
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
            max_alignment_error_mm=0,
            alignment_recovery_distance_mm=0,
            minimum_lateral_clearance_mm=0,
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
    server._wheel_linear_speed = 0.370
    server._kinematic_lever = 0.2225
    server._lateral_scan_timeout = 0.35
    server._lateral_scan_start_timeout = 1.0
    server._lateral_slowdown_margin_mm = 150.0
    server._footprint_half_length_m = 0.119
    server._footprint_half_width_m = 0.155
    server._lateral_longitudinal_margin_m = 0.0
    server._lateral_minimum_points = 2
    server._follow_wall_controller = FakeFollowWallController()
    server._owns_sensor_pair = False
    server._sensor_pair = pair
    server._sensor_pair_factory = lambda: pair
    server._odom_topic = '/odom'
    server._scan_topic = '/scan_front'
    server._odom_subscription = None
    server._scan_subscription = None
    server._lock = threading.RLock()
    server._resource_lock = threading.RLock()
    server._subscription_request_lock = threading.Lock()
    server._subscription_request_done = threading.Event()
    server._subscription_request = None
    server._subscription_request_error = None
    server._resource_guard = None
    server._latest_lateral_clearances = None
    server._lateral_scan_updated = float('-inf')
    server._lateral_scan_started = float('-inf')
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
    assert server._scan_subscription is None
    assert server._command_timer.reset_count == 1

    server._deactivate_goal_resources()

    assert server._sensor_pair is None
    assert server._odom_subscription is None
    assert destroyed == subscriptions
    assert closed == [True]
    assert server._command_timer.cancel_count == 1


def test_lidar_subscription_is_created_only_when_safety_is_enabled():
    server = _bare_server(SequencePair([]))
    subscriptions = []
    server.create_subscription = lambda *args: subscriptions.append(
        args[0]) or object()

    server._activate_goal_resources(lateral_safety_enabled=True)

    assert subscriptions == [action_module.Odometry, action_module.LaserScan]
    assert server._scan_subscription is not None

    server._deactivate_goal_resources()
    assert server._scan_subscription is None


def test_subscription_lifecycle_is_dispatched_to_executor_thread():
    server = _bare_server(SequencePair([]))
    subscriptions = []
    destroyed = []
    server.create_subscription = lambda *args: subscriptions.append(
        args[0]) or object()
    server.destroy_subscription = lambda subscription: destroyed.append(
        subscription) or True

    class DeferredGuard:
        def __init__(self):
            self.triggered = threading.Event()

        def trigger(self):
            self.triggered.set()

    guard = DeferredGuard()
    server._resource_guard = guard
    activation = threading.Thread(
        target=server._activate_goal_resources,
        kwargs={'lateral_safety_enabled': True},
    )
    activation.start()

    assert guard.triggered.wait(timeout=1.0)
    assert activation.is_alive()
    assert subscriptions == []

    server._process_subscription_request()
    activation.join(timeout=1.0)

    assert not activation.is_alive()
    assert subscriptions == [action_module.Odometry, action_module.LaserScan]

    guard.triggered.clear()
    deactivation = threading.Thread(target=server._deactivate_goal_resources)
    deactivation.start()

    assert guard.triggered.wait(timeout=1.0)
    assert deactivation.is_alive()
    assert destroyed == []

    server._process_subscription_request()
    deactivation.join(timeout=1.0)

    assert not deactivation.is_alive()
    assert len(destroyed) == 2


def _follow_request(
    wall=300,
    travel=500,
    wall_tolerance=10,
    travel_tolerance=10,
    max_alignment_error=0,
    recovery_distance=0,
    minimum_lateral_clearance=0,
    timeout=10,
):
    return SimpleNamespace(
        wall_distance_mm=wall,
        travel_distance_mm=travel,
        wall_tolerance_mm=wall_tolerance,
        travel_tolerance_mm=travel_tolerance,
        max_alignment_error_mm=max_alignment_error,
        alignment_recovery_distance_mm=recovery_distance,
        minimum_lateral_clearance_mm=minimum_lateral_clearance,
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
    assert server._follow_wall_goal_callback(_follow_request(
        minimum_lateral_clearance=-1,
    )).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(
        _follow_request(max_alignment_error=-1)).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(_follow_request(
        max_alignment_error=100,
        recovery_distance=-1,
    )).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(_follow_request(
        recovery_distance=100,
    )).name == GoalResponse.REJECT.name
    assert server._follow_wall_goal_callback(_follow_request(
        travel=0,
        max_alignment_error=100,
        recovery_distance=100,
    )).name == GoalResponse.REJECT.name
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


def test_lateral_safety_uses_only_the_side_of_linear_y():
    server = _bare_server(SequencePair([]))
    now = time.monotonic()
    server._latest_lateral_clearances = LateralClearances(
        left_mm=50.0, right_mm=200.0)
    server._lateral_scan_updated = now
    command = FollowWallCommand(
        0.02, -0.10, 0.3, 300.0, 0.0, 0.0, 0.0, 500.0, False)

    safe, error = server._apply_lateral_safety(command, 100, now)

    assert error is None
    assert safe.linear_x_velocity_mps == command.linear_x_velocity_mps
    assert safe.angular_velocity_rad_s == command.angular_velocity_rad_s
    assert safe.linear_y_velocity_mps == pytest.approx(-0.10 * 2.0 / 3.0)

    command_left = FollowWallCommand(
        0.02, 0.10, 0.3, 300.0, 0.0, 0.0, 0.0, 500.0, False)
    stopped, error = server._apply_lateral_safety(command_left, 100, now)
    assert stopped.linear_x_velocity_mps == command_left.linear_x_velocity_mps
    assert stopped.linear_y_velocity_mps == 0.0
    assert stopped.angular_velocity_rad_s == 0.0
    assert 'lado esquerdo' in error
    assert '50 mm' in error


def test_lateral_safety_ignores_rotation_without_lateral_motion():
    server = _bare_server(SequencePair([]))
    command = FollowWallCommand(
        0.0, 0.0, 0.5, 300.0, 0.0, 0.0, 0.0, 0.0, False)

    safe, error = server._apply_lateral_safety(
        command, 100, time.monotonic())

    assert safe == command
    assert error is None


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


def test_follow_wall_finishes_front_alignment_when_lateral_side_is_blocked(
    monkeypatch,
):
    far = DistanceSample(400, 400, 400, 400)
    aligned = DistanceSample(300, 300, 300, 300)
    pair = SequencePair([far, aligned, aligned])
    server = _bare_server(pair)

    class FrontThenStopController(FakeFollowWallController):
        def calculate(self, left, right, wall, wall_tolerance, traveled,
                      travel, travel_tolerance, dt):
            del travel_tolerance, dt
            wall_inside = (
                wall - wall_tolerance <= left <= wall + wall_tolerance
                and wall - wall_tolerance <= right <= wall + wall_tolerance
            )
            return FollowWallCommand(
                0.0 if wall_inside else 0.02,
                -0.04,
                0.3,
                (left + right) / 2.0,
                (left + right) / 2.0 - wall,
                float(right - left),
                traveled,
                travel - traveled,
                False,
            )

    server._follow_wall_controller = FrontThenStopController()
    server._activate_goal_resources = lambda **_kwargs: None
    server._deactivate_goal_resources = lambda: None
    server._latest_lateral_clearances = LateralClearances(
        left_mm=None, right_mm=0.0)
    server._lateral_scan_updated = time.monotonic()
    pose = OdometryPose(0.0, 0.0, 0.0)
    server._odometry_snapshot = lambda _now=None: (pose, True)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()
    goal.request.minimum_lateral_clearance_mm = 10

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert pair.read_count == 3
    assert result.final_average_distance_mm == pytest.approx(300.0)
    assert 'Aproximacao frontal concluida' in result.message
    assert goal.feedback[0].linear_x_velocity_mps == pytest.approx(0.02)
    assert goal.feedback[0].linear_y_velocity_mps == 0.0
    assert goal.feedback[0].angular_velocity_rad_s == 0.0


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


def test_follow_wall_aborts_when_alignment_exceeds_optional_limit(monkeypatch):
    sample = DistanceSample(400, 400, 250, 351)
    pair = SequencePair([sample])
    server = _bare_server(pair)
    pose = OdometryPose(0.0, 0.0, 0.0)
    server._odometry_snapshot = lambda _now=None: (pose, True)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()
    goal.request.max_alignment_error_mm = 100

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert result.has_valid_reading
    assert result.final_left_distance_mm == 250
    assert result.final_right_distance_mm == 351
    assert '101 mm' in result.message
    assert '100 mm' in result.message
    assert goal.feedback[-1].alignment_error_mm == pytest.approx(101.0)
    assert isinstance(goal.feedback[-1].alignment_error_mm, float)
    assert not server._desired_valid


def test_follow_wall_returns_laterally_and_aborts_after_recovery(monkeypatch):
    outside = DistanceSample(400, 400, 250, 351)
    aligned = DistanceSample(400, 400, 290, 300)
    pair = SequencePair([outside, outside, aligned])
    server = _bare_server(pair)
    calls = []

    class RecoveryController(FakeFollowWallController):
        def calculate(self, left, right, wall, wall_tolerance, traveled,
                      travel, travel_tolerance, dt):
            calls.append((left, right, traveled, travel))
            inside = (
                abs(travel - traveled) <= travel_tolerance
                and wall - wall_tolerance <= left <= wall + wall_tolerance
                and wall - wall_tolerance <= right <= wall + wall_tolerance
            )
            return FollowWallCommand(
                0.0, 0.0 if inside else 0.04, 0.0,
                (left + right) / 2.0, 0.0, float(right - left),
                traveled, travel - traveled, inside)

    server._follow_wall_controller = RecoveryController()
    poses = iter([
        (OdometryPose(0.0, 0.0, 0.0), True),
        (OdometryPose(0.0, -0.3, 0.0), True),
        (OdometryPose(0.0, -0.2, 0.0), True),
        (OdometryPose(0.0, -0.1, 0.0), True),
    ])
    server._odometry_snapshot = lambda _now=None: next(poses)
    monkeypatch.setattr(action_module.rclpy, 'ok', lambda: True)
    goal = FakeFollowWallGoal()
    goal.request.max_alignment_error_mm = 100
    goal.request.alignment_recovery_distance_mm = 200

    result = server._execute_follow_wall_goal(goal)

    assert goal.terminal == 'aborted'
    assert result.traveled_distance_mm == pytest.approx(100.0)
    assert 'Recuperação concluída' in result.message
    assert calls[0] == (300, 300, pytest.approx(300.0), 100)
    assert calls[1] == (300, 300, pytest.approx(200.0), 100)
    assert calls[2] == (290, 300, pytest.approx(100.0), 100)
    assert goal.feedback[0].alignment_error_mm == pytest.approx(101.0)
    assert not server._desired_valid
