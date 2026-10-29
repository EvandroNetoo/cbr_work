import copy
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from mission_manager.errors import ConfigurationError, MissionCanceled
from interfaces.action import ExecuteMission
from mission_manager.loaders import load_arena, load_plan
from mission_manager.node import MissionManager
from mission_manager.world_state import WorldState


def manager_fixture():
    manager = MissionManager.__new__(MissionManager)
    events = []
    manager._check_canceled = lambda: None
    manager.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=42)))
    manager.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    manager._initial_pose_publisher = SimpleNamespace(
        publish=lambda message: events.append(copy.deepcopy(message)))
    # Publishing an initial estimate must not subscribe to or wait for AMCL.
    manager.create_subscription = lambda *_args, **_kwargs: pytest.fail('não aguardar AMCL')
    return manager, events


@pytest.fixture
def arena():
    return load_arena(Path(__file__).parents[1] / 'config' / 'arena.yaml')


def test_empty_start_pose_preserves_localization(arena):
    # No ROS objects are needed for an omitted start_pose.
    MissionManager.__new__(MissionManager)._initialize_localization(arena, '')


@pytest.mark.parametrize('target', ['start', 'finish', 'WS_1', 'PP'])
def test_named_pose_uses_arena_frame_coordinates_and_rviz_covariance(arena, target):
    import math
    manager, events = manager_fixture()
    manager._initialize_localization(arena, target)
    message = events[0]
    pose = arena.pose_for(target)
    assert message.header.frame_id == arena.frame_id
    assert message.pose.pose.position.x == pose.x_m
    assert message.pose.pose.position.y == pose.y_m
    assert message.pose.pose.orientation.z == pytest.approx(math.sin(pose.yaw_rad / 2))
    assert message.pose.pose.orientation.w == pytest.approx(math.cos(pose.yaw_rad / 2))
    assert message.pose.covariance[0] == 0.25
    assert len(events) == 1


def test_unknown_location_fails_before_publishing(arena):
    manager = MissionManager.__new__(MissionManager)
    with pytest.raises(ConfigurationError, match='start_pose'):
        manager._initialize_localization(arena, 'missing')


def test_cancel_before_publishing_pose(arena):
    manager, events = manager_fixture()
    def cancel():
        raise MissionCanceled('cancel')
    manager._check_canceled = cancel
    with pytest.raises(MissionCanceled):
        manager._initialize_localization(arena, 'start')
    assert events == []


@pytest.mark.parametrize('target', ['', 'start', 'WS_1', 'missing'])
@pytest.mark.parametrize('wait_s', [0.0, 3.0, 5.5, -1.0, float('nan'), float('inf')])
def test_action_localizes_before_executing_and_rejects_unknown_target(arena, target, wait_s):
    manager = MissionManager.__new__(MissionManager)
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    manager._world_state = WorldState(['left', 'right'])
    plan = load_plan(Path(__file__).parents[1] / 'config' / 'plans' / '7_final_test.yaml')
    manager._load_goal_files = lambda _: (arena, plan)
    manager.get_logger = lambda: SimpleNamespace(error=lambda _: None, info=lambda _: None)
    manager.get_parameter = lambda name: SimpleNamespace(value=wait_s)
    events = []
    manager._initialize_localization = lambda _arena, name: events.append(('localize', name))
    manager._wait_for_localization = lambda seconds: events.append(('wait', seconds))
    def execute(_handle, _arena, actual_plan):
        expected = [('localize', target)]
        if target:
            expected.append(('wait', wait_s))
        assert events == expected
        assert actual_plan.initial_location == (target or plan.initial_location)
        events.append(('execute', actual_plan.initial_location))
        return ExecuteMission.Result()
    manager._execute_validated_mission = execute
    handle = SimpleNamespace(
        request=ExecuteMission.Goal(plan_id=plan.plan_id, start_pose=target), abort=lambda: None)
    result = manager._execute_callback(handle)
    if target == 'missing' or (target and not (0.0 <= wait_s < float('inf'))):
        assert result.code == ExecuteMission.Result.CONFIGURATION_ERROR
        assert events == []
    else:
        assert result.code == ExecuteMission.Result.SUCCESS
        assert len(events) == (3 if target else 2)
    assert not manager._busy


@pytest.mark.parametrize('wait_s', [0.0, 3.0, 5.5])
@pytest.mark.parametrize('cancel', [False, True])
def test_localization_wait_respects_duration_and_cancellation(monkeypatch, wait_s, cancel):
    manager = MissionManager.__new__(MissionManager)
    manager._cancel_event = threading.Event()
    manager._cancel_active_child = lambda: None
    elapsed = 0.0

    def wait(seconds):
        nonlocal elapsed
        elapsed += seconds
        if cancel:
            manager._cancel_event.set()

    monkeypatch.setattr('mission_manager.node.time.monotonic', lambda: elapsed)
    monkeypatch.setattr(manager._cancel_event, 'wait', wait)
    if cancel and wait_s:
        with pytest.raises(MissionCanceled):
            manager._wait_for_localization(wait_s)
        assert elapsed <= 0.05
    else:
        manager._wait_for_localization(wait_s)
        assert elapsed == pytest.approx(wait_s)


def test_cancel_during_start_pose_wait_prevents_mission_execution(arena):
    manager, events = manager_fixture()
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    manager._world_state = WorldState(['left', 'right'])
    manager.get_logger = lambda: SimpleNamespace(info=lambda _: None, error=lambda _: None)
    manager.get_parameter = lambda name: SimpleNamespace(value=3.0)
    plan = load_plan(Path(__file__).parents[1] / 'config' / 'plans' / '7_final_test.yaml')
    manager._load_goal_files = lambda _: (arena, plan)

    def cancel_wait(seconds):
        assert seconds == 3.0
        assert len(events) == 1
        raise MissionCanceled('cancel')

    manager._wait_for_localization = cancel_wait
    manager._execute_validated_mission = lambda *_: pytest.fail('missão cancelada')
    canceled = []
    handle = SimpleNamespace(
        request=ExecuteMission.Goal(plan_id=plan.plan_id, start_pose='start'),
        canceled=lambda: canceled.append(True))
    result = manager._execute_callback(handle)
    assert result.code == ExecuteMission.Result.CANCELED
    assert canceled == [True]
    assert not manager._busy
