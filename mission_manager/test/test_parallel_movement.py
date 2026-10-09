"""Exercise overlap and cancellation with pending ROS action futures."""
from concurrent.futures import Future, ThreadPoolExecutor
import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.node import MissionManager
from mission_manager.world_state import WorldState


class Child:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.canceled = threading.Event()

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.canceled.set()

    def finish(self, status=GoalStatus.STATUS_SUCCEEDED):
        self.result.set_result(SimpleNamespace(status=status, result=object()))


class Client:
    def __init__(self):
        self.child = Child()
        self.sent = threading.Event()
        self.goal = None

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goal = goal
        future = Future()
        future.set_result(self.child)
        self.sent.set()
        return future


def manager_and_clients():
    manager = MissionManager.__new__(MissionManager)
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    manager._active_children = {}
    manager._world_state = WorldState(['left', 'right'])
    manager._server_timeout = lambda: 0.5
    manager._manipulation_timeout = lambda: 1.0
    manager._manipulation_failure = lambda result: None
    prepare, wall = Client(), Client()
    manager._prepare_client = prepare
    return manager, prepare, wall


@pytest.mark.parametrize('loaded', [False, True])
@pytest.mark.parametrize('observation', [False, True])
def test_both_goals_start_before_either_finishes_and_join(loaded, observation):
    manager, prepare, wall = manager_and_clients()
    if loaded:
        manager._world_state.commit_pick(7)
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_with_manipulator_prepare,
                              (manager._prepare_for_pick_observation if observation
                               else manager._prepare_for_navigation),
                              lambda: manager._call_action(wall, object(), 'wall', 1.0))
        assert prepare.sent.wait(1.0)
        assert wall.sent.wait(1.0)
        assert prepare.goal.mode == (prepare.goal.OBSERVATION if observation
                                     else prepare.goal.NAVIGATION)
        assert prepare.goal.gripper_loaded is (loaded and not observation)
        wall.child.finish()
        assert not run.done()
        prepare.child.finish()
        assert run.result(timeout=2.0) is wall.child.result.result().result
    assert manager._active_children == {}


@pytest.mark.parametrize('ending', ['failure', 'timeout', 'cancel', 'rejected'])
def test_failure_waits_for_other_action_and_explicit_cancel_stops_both(ending):
    manager, prepare, wall = manager_and_clients()
    timeout = 0.15 if ending == 'timeout' else 1.0
    if ending == 'rejected':
        wall.child.accepted = False
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_with_manipulator_prepare,
                              manager._prepare_for_navigation,
                              lambda: manager._call_action(wall, object(), 'wall', timeout))
        assert wall.sent.wait(1.0)
        assert prepare.sent.wait(1.0)
        if ending == 'failure':
            wall.child.finish(GoalStatus.STATUS_ABORTED)
        elif ending == 'cancel':
            manager._cancel_callback(None)
        elif ending == 'timeout':
            assert wall.child.canceled.wait(1.0)
        if ending != 'cancel':
            with pytest.raises(TimeoutError):
                run.result(timeout=0.05)
            assert not prepare.child.canceled.is_set()
            prepare.child.finish()
        with pytest.raises(MissionCanceled if ending == 'cancel' else StepFailed):
            run.result(timeout=2.0)
    assert prepare.child.canceled.is_set() is (ending == 'cancel')
    if ending in ('timeout', 'cancel'):
        assert wall.child.canceled.is_set()
    assert manager._active_children == {}


def test_unknown_cargo_blocks_both_movements():
    manager, prepare, wall = manager_and_clients()
    manager._world_state.mark_unknown()
    with pytest.raises(StepFailed, match='incerto'):
        manager._run_with_manipulator_prepare(
            manager._prepare_for_navigation,
            lambda: manager._call_action(wall, object(), 'wall', 1.0))
    assert not prepare.sent.is_set()
    assert not wall.sent.is_set()


def test_goal_accepted_after_timeout_is_canceled():
    manager, prepare, wall = manager_and_clients()
    pending = Future()
    wall.send_goal_async = lambda goal: pending
    manager._server_timeout = lambda: 0.01
    with pytest.raises(StepFailed, match='Timeout'):
        manager._call_action(wall, object(), 'wall', 1.0)
    assert manager._active_children == {}
    pending.set_result(wall.child)
    assert wall.child.canceled.is_set()


def test_arm_failure_waits_for_wall_control_to_finish():
    manager, prepare, wall = manager_and_clients()
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_with_manipulator_prepare,
                              manager._prepare_for_navigation,
                              lambda: manager._call_action(wall, object(), 'wall', 1.0))
        assert prepare.sent.wait(1.0)
        assert wall.sent.wait(1.0)
        prepare.child.finish(GoalStatus.STATUS_ABORTED)
        with pytest.raises(TimeoutError):
            run.result(timeout=0.05)
        assert not wall.child.canceled.is_set()
        wall.child.finish()
        with pytest.raises(StepFailed):
            run.result(timeout=2.0)
    assert not wall.child.canceled.is_set()
    assert manager._active_children == {}


def test_explicit_cancel_after_one_failure_takes_precedence():
    manager, prepare, wall = manager_and_clients()
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_with_manipulator_prepare,
                              manager._prepare_for_navigation,
                              lambda: manager._call_action(wall, object(), 'wall', 1.0))
        assert prepare.sent.wait(1.0)
        assert wall.sent.wait(1.0)
        prepare.child.finish(GoalStatus.STATUS_ABORTED)
        with pytest.raises(TimeoutError):
            run.result(timeout=0.05)
        manager._cancel_callback(None)
        with pytest.raises(MissionCanceled):
            run.result(timeout=2.0)
    assert wall.child.canceled.is_set()
    assert manager._active_children == {}


@pytest.mark.parametrize('mode', ['disabled', 'always'])
@pytest.mark.parametrize('boundary_enabled', [True, False])
@pytest.mark.parametrize('boundary', [True, False])
def test_motion_pair_uses_independent_boundary_and_table_config(mode, boundary_enabled, boundary):
    from dataclasses import replace
    from mission_manager.models import AsyncMotionConfig
    from test_executor_helpers import _arena
    manager, prepare, wall = manager_and_clients()
    manager._arena = replace(_arena(), async_motion_defaults=AsyncMotionConfig(mode, boundary_enabled))
    manager._current_location = 'ws_1'
    concurrent = boundary_enabled if boundary else mode == 'always'
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_area_motion, manager._prepare_for_pick_observation,
                              lambda: manager._call_action(wall, object(), 'movement', 1.), boundary=boundary)
        assert prepare.sent.wait(1.)
        if concurrent:
            assert wall.sent.wait(1.)
        else:
            assert not wall.sent.is_set()
        prepare.child.finish()
        assert wall.sent.wait(1.)
        wall.child.finish()
        run.result(timeout=2.)
