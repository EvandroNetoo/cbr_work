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
def test_action_localizes_before_executing_and_rejects_unknown_target(arena, target):
    manager = MissionManager.__new__(MissionManager)
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    manager._world_state = WorldState(['left', 'right'])
    plan = load_plan(Path(__file__).parents[1] / 'config' / 'plans' / '7_final_test.yaml')
    manager._load_goal_files = lambda _: (arena, plan)
    manager.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    events = []
    manager._initialize_localization = lambda _arena, name: events.append(('localize', name))
    def execute(_handle, _arena, actual_plan):
        assert events == [('localize', target)]
        assert actual_plan.initial_location == (target or plan.initial_location)
        events.append(('execute', actual_plan.initial_location))
        return ExecuteMission.Result()
    manager._execute_validated_mission = execute
    handle = SimpleNamespace(
        request=ExecuteMission.Goal(plan_id=plan.plan_id, start_pose=target), abort=lambda: None)
    result = manager._execute_callback(handle)
    if target == 'missing':
        assert result.code == ExecuteMission.Result.CONFIGURATION_ERROR
        assert events == []
    else:
        assert result.code == ExecuteMission.Result.SUCCESS
        assert len(events) == 2
    assert not manager._busy
