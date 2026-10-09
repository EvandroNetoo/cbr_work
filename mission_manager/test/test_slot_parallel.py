"""Cargo transfer overlap starts at the safe feedback phase, in the opposite direction."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from interfaces.action import FollowWall, StoreObject, RetrieveObject
from interfaces.msg import ManipulationFeedback, ManipulationResult
import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.node import MissionManager
from mission_manager.models import Plan, SlotMovement, Step, TagObservation, Visit
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import EMPTY
from test_executor_helpers import _arena
from test_parallel_movement import Client, manager_and_clients


class FeedbackClient(Client):
    def send_goal_async(self, goal, feedback_callback=None):
        self.callback = feedback_callback
        return super().send_goal_async(goal)

    def phase(self, phase):
        if self.callback is not None:
            self.callback(SimpleNamespace(feedback=SimpleNamespace(
                status=SimpleNamespace(phase=phase))))


def cargo_manager(operation, slot):
    manager, _, wall = manager_and_clients()
    manager._manipulation_failure = MissionManager._manipulation_failure
    cargo = FeedbackClient()
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._retreat_from_lateral_wall_before_slot_access = lambda *_args: None
    manager._remember_scene_observations = lambda _result: None
    manager._publish_world_state = lambda: None
    manager._world_state.commit_pick(7)
    if operation == 'retrieve':
        manager._world_state.commit_store(7, slot)
    manager._store_client = manager._retrieve_client = cargo
    commands = []

    def control(distance, tolerance, timeout, description, **kwargs):
        commands.append((distance, kwargs))
        return manager._call_action(wall, object(), description, timeout)

    manager._control_wall = control
    return manager, cargo, wall, commands


def finish_cargo(cargo, operation, code=ManipulationResult.SUCCESS):
    result = (StoreObject if operation == 'store' else RetrieveObject).Result()
    result.outcome.code = code
    result.outcome.effect_known = True
    result.outcome.final_object_location = (
        ManipulationResult.LOCATION_CARGO if operation == 'store'
        else ManipulationResult.LOCATION_GRIPPER)
    if code != ManipulationResult.SUCCESS:
        result.outcome.final_object_location = ManipulationResult.LOCATION_SOURCE
    cargo.child.result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result))


def finish_wall(wall, travel, distance=200):
    result = FollowWall.Result()
    result.has_valid_reading = result.has_valid_odometry = True
    result.final_average_distance_mm = float(distance)
    result.traveled_distance_mm = float(travel)
    wall.child.result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result))


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('slot,travel', [('left', 100), ('right', -100)])
def test_opposite_movement_waits_for_safe_phase_and_confirms_cargo(operation, slot, travel):
    manager, cargo, wall, commands = cargo_manager(operation, slot)
    before = manager._world_state.snapshot()
    step = Step('transfer', operation, tag_id=7, slot_id=slot)
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step, step, SlotMovement(200, travel))
        assert cargo.sent.wait(1.0)
        assert not wall.sent.is_set()
        cargo.phase(ManipulationFeedback.PREPARING)
        assert not wall.sent.is_set()
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        assert manager._world_state.snapshot() == before
        assert commands[0][1]['travel_distance_mm'] == travel
        assert commands[0][1]['alignment_recovery_distance_mm'] == 0
        assert commands[0][1]['accept_safety_abort'] is True
        finish_cargo(cargo, operation)
        assert not run.done()
        finish_wall(wall, travel)
        run.result(timeout=2.0)
    known, gripper, slots = manager._world_state.snapshot()
    assert known
    assert gripper == (EMPTY if operation == 'store' else 7)
    assert slots[slot] == (7 if operation == 'store' else EMPTY)
    assert manager._current_lateral_position_mm == travel
    assert manager._active_children == {}


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('slot,travel', [('left', -100), ('right', 100),
                                         ('left', 0), ('right', 0),
                                         ('custom', 100)])
def test_same_direction_zero_or_unknown_slot_keeps_sequential_execution(operation, slot, travel):
    if slot == 'custom':
        manager, cargo, wall, commands = cargo_manager(operation, 'left')
        assert not manager._slot_movement_is_opposite(slot, SlotMovement(200, travel))
        return
    manager, cargo, wall, commands = cargo_manager(operation, slot)
    step = Step('transfer', operation, tag_id=7, slot_id=slot)
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step, step, SlotMovement(200, travel))
        assert cargo.sent.wait(1.0)
        assert cargo.callback is None
        finish_cargo(cargo, operation)
        run.result(timeout=2.0)
    assert not wall.sent.is_set()
    assert commands == []


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('ending', ['before_ready', 'cargo_failure', 'base_failure', 'cancel'])
def test_overlap_failure_waits_for_other_action_and_preserves_inventory(operation, ending):
    manager, cargo, wall, _ = cargo_manager(operation, 'left')
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', operation, tag_id=7, slot_id='left'),
                              SlotMovement(200, 100))
        assert cargo.sent.wait(1.0)
        if ending != 'before_ready':
            cargo.phase(ManipulationFeedback.APPROACHING)
            assert wall.sent.wait(1.0)
        if ending in {'before_ready', 'cargo_failure'}:
            finish_cargo(cargo, operation, ManipulationResult.MOTION_FAILED)
        elif ending == 'base_failure':
            wall.child.finish(GoalStatus.STATUS_ABORTED)
        else:
            manager._cancel_callback(None)
        if ending in {'cargo_failure', 'base_failure'}:
            with pytest.raises(TimeoutError):
                run.result(timeout=0.05)
            if ending == 'cargo_failure':
                assert not wall.child.canceled.is_set()
                finish_wall(wall, 100)
            else:
                assert not cargo.child.canceled.is_set()
                finish_cargo(cargo, operation)
        with pytest.raises(MissionCanceled if ending == 'cancel' else StepFailed):
            run.result(timeout=2.0)
    if ending == 'before_ready':
        assert not wall.sent.is_set()
    elif ending == 'cancel':
        assert cargo.child.canceled.is_set()
        assert wall.child.canceled.is_set()
        assert not manager._world_state.snapshot()[0]
    else:
        assert not cargo.child.canceled.is_set()
        assert not wall.child.canceled.is_set()
        assert manager._world_state.snapshot()[0]
    assert manager._active_children == {}


def test_no_feedback_does_not_release_base():
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(200, 100))
        assert cargo.sent.wait(1.0)
        finish_cargo(cargo, 'store')
        run.result(timeout=2.0)
    assert not wall.sent.is_set()


def test_direction_is_checked_after_existing_clearance_retreat():
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    manager._retreat_from_lateral_wall_before_slot_access = lambda *_args: setattr(
        manager, '_current_lateral_position_mm', 200.0)
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(200, 100))
        assert cargo.sent.wait(1.0)
        assert cargo.callback is None
        finish_cargo(cargo, 'store')
        run.result(timeout=2.0)
    assert not wall.sent.is_set()


def test_async_departure_is_not_repeated_and_nav_waits_for_cargo():
    manager, cargo, wall, commands = cargo_manager('store', 'left')
    manager._current_lateral_position_mm = -100.0
    events = []
    manager._deactivate_service_area_vision = lambda: events.append('off')
    manager._prepare_for_navigation = lambda: events.append('home')
    manager._navigate_client = object()
    manager._navigation_timeout = lambda: 1.0
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(250, 0, True))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        finish_wall(wall, 100, 250)
        assert not run.done()
        finish_cargo(cargo, 'store')
        run.result(timeout=2.0)
    assert manager._departure_completed_for == 'ws_1'
    assert events == ['off']
    manager._call_action = lambda *_args: events.append('nav')
    manager._navigate('start')
    assert events == ['off', 'home', 'nav']
    assert len(commands) == 1
    assert manager._departure_completed_for is None


def planning_manager():
    manager, _, _, _ = cargo_manager('store', 'left')
    manager._tag_observations = {}
    manager._container_observations = {}
    manager._placed_tag_viewpoints = {}
    manager._visited_search_positions = {'ws_1': {0}}
    manager._blocked_search_positions = {}
    manager._current_scene_observed = lambda: True
    return manager


def test_store_lookahead_uses_next_cube_memory_without_committing_inventory():
    manager = planning_manager()
    plan = Plan('test', (Visit('v1', 'ws_1', (
        Step('pick1', 'pick', tag_id=7), Step('pick2', 'pick', tag_id=8))),
        Visit('v2', 'ws_1', (Step('place1', 'place_on_table', tag_id=7),
                              Step('place2', 'place_on_table', tag_id=8)))))
    scheduler = Scheduler(plan, ('left', 'right'))
    picked = next(c for c in scheduler.choices(scheduler.initial_state) if c.step.tag_id == 7)
    store = next(c for c in scheduler.choices(picked.next_state) if c.step.action == 'store')
    manager._tag_observations[('ws_1', 8)] = TagObservation('ws_1', 200, 100, 150, 120, object())
    before = manager._world_state.snapshot()
    assert manager._next_slot_movement(scheduler, store, plan) == SlotMovement(150, 120)
    assert manager._world_state.snapshot() == before


def test_lookahead_can_advance_existing_search_but_never_skips_observation():
    manager = planning_manager()
    plan = Plan('test', (Visit('v1', 'ws_1', (
        Step('pick1', 'pick', tag_id=7), Step('pick2', 'pick', tag_id=8))),
        Visit('v2', 'ws_1', (Step('place1', 'place_on_table', tag_id=7),
                              Step('place2', 'place_on_table', tag_id=8)))))
    scheduler = Scheduler(plan, ('left', 'right'))
    picked = next(c for c in scheduler.choices(scheduler.initial_state) if c.step.tag_id == 7)
    store = next(c for c in scheduler.choices(picked.next_state) if c.step.action == 'store')
    assert manager._next_slot_movement(scheduler, store, plan) == SlotMovement(200, 250)
    manager._current_scene_observed = lambda: False
    assert manager._next_slot_movement(scheduler, store, plan) is None


def test_retrieve_lookahead_uses_delivery_support_memory():
    manager = planning_manager()
    manager._tag_observations[('ws_1', 9)] = TagObservation('ws_1', 200, -100, 150, -120, object())
    step = Step('stack', 'stack', tag_id=7, support_tag_id=9)
    state = object()
    scheduler = SimpleNamespace(select=lambda *_args: SimpleNamespace(step=step))
    assert manager._next_slot_movement(scheduler, SimpleNamespace(
        next_state=state, step=Step('retrieve', 'retrieve')), None) == SlotMovement(150, -120)


@pytest.mark.parametrize('finish,next_target,expected', [
    (False, None, None), (True, None, SlotMovement(250, 0, True)),
    (False, 'ws_1', None), (False, 'finish', SlotMovement(250, 0, True)),
])
def test_departure_lookahead_requires_an_actual_next_navigation(finish, next_target, expected):
    manager = planning_manager()
    visits = ((Visit('next', next_target, ()),) if next_target else ())
    plan = Plan('test', visits, finish=finish)
    following = SimpleNamespace(step=Step('depart', 'depart'),
                                next_state=SimpleNamespace(visit=0))
    scheduler = SimpleNamespace(select=lambda *_args: following)
    assert manager._next_slot_movement(scheduler, SimpleNamespace(next_state=object()), plan) == expected


@pytest.mark.parametrize('ready', [False, True])
def test_slot_timeout_allows_started_base_movement_to_finish(ready):
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    manager._manipulation_timeout = lambda: 0.15
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(200, 100))
        assert cargo.sent.wait(1.0)
        if ready:
            cargo.phase(ManipulationFeedback.APPROACHING)
            assert wall.sent.wait(1.0)
        assert cargo.child.canceled.wait(1.0)
        if ready:
            with pytest.raises(TimeoutError):
                run.result(timeout=0.05)
            assert not wall.child.canceled.is_set()
            finish_wall(wall, 100)
        with pytest.raises(StepFailed, match='Timeout'):
            run.result(timeout=2.0)
    assert cargo.child.canceled.is_set()
    if ready:
        assert not wall.child.canceled.is_set()
    else:
        assert not wall.sent.is_set()
    assert not manager._world_state.snapshot()[0]
    assert manager._active_children == {}


def test_visit_scheduler_supplies_next_pick_movement_while_store_is_pending():
    from test_visit_executor import simulated_manager

    plan = Plan('test', (Visit('v1', 'ws_1', (
        Step('pick1', 'pick', tag_id=7), Step('pick2', 'pick', tag_id=8))),
        Visit('v2', 'ws_2', (Step('place1', 'place_on_table', tag_id=7),
                              Step('place2', 'place_on_table', tag_id=8)))))
    manager, *_ = simulated_manager(plan)
    original_pick = manager._pick_client.responder

    def pick(goal):
        result = original_pick(goal)
        if goal.tag_id == 7:
            memory = manager._tag_observations[('ws_1', 8)]
            manager._tag_observations[('ws_1', 8)] = replace(
                memory, lateral_position_mm=100, pickup_lateral_position_mm=100)
        return result

    manager._pick_client.responder = pick
    cargo, wall = FeedbackClient(), Client()
    manager._store_client = cargo
    movements = []

    def control(distance, tolerance, timeout, description, **kwargs):
        movements.append(kwargs['travel_distance_mm'])
        return manager._call_action(wall, object(), description, timeout)

    manager._control_wall = control
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._run_plan, SimpleNamespace(publish_feedback=lambda _: None), plan)
        assert cargo.sent.wait(1.0)
        assert cargo.goal.slot_id == 'left'
        assert not wall.sent.is_set()
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        assert movements == [100]
        assert manager._world_state.snapshot()[1] == 7
        finish_wall(wall, 100)
        finish_cargo(cargo, 'store')
        run.result(timeout=2.0)
    known, gripper, slots = manager._world_state.snapshot()
    assert known and gripper == EMPTY and all(tag == EMPTY for tag in slots.values())
    assert len(manager._pick_client.goals) == 2
    assert len(manager._retrieve_client.goals) == 1


def test_uncertain_cargo_result_waits_for_remaining_base_motion():
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(200, 100))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        result = StoreObject.Result()
        result.outcome.code = ManipulationResult.SUCCESS
        result.outcome.effect_known = False
        cargo.child.result.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=result))
        with pytest.raises(TimeoutError):
            run.result(timeout=0.05)
        assert not wall.child.canceled.is_set()
        finish_wall(wall, 100)
        with pytest.raises(StepFailed, match='carga incerta'):
            run.result(timeout=2.0)
    assert not wall.child.canceled.is_set()
    assert not manager._world_state.snapshot()[0]


def test_base_timeout_waits_for_pending_transfer_to_finish():
    manager, cargo, wall, _ = cargo_manager('retrieve', 'right')
    manager._arena = replace(manager._arena, pickup_recovery=replace(
        manager._arena.pickup_recovery, timeout_s=0.15))
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'retrieve', tag_id=7, slot_id='right'),
                              SlotMovement(200, -100))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        assert wall.child.canceled.wait(1.0)
        with pytest.raises(TimeoutError):
            run.result(timeout=0.05)
        assert not cargo.child.canceled.is_set()
        finish_cargo(cargo, 'retrieve')
        with pytest.raises(StepFailed, match='Timeout'):
            run.result(timeout=2.0)
    assert not cargo.child.canceled.is_set()
    assert wall.child.canceled.is_set()
    assert manager._world_state.snapshot()[0]
    assert manager._world_state.snapshot()[1] == 7


def test_direction_uses_clamped_target_and_tolerance():
    manager, *_ = cargo_manager('store', 'left')
    manager._current_lateral_position_mm = 275.0
    assert not manager._slot_movement_is_opposite('left', SlotMovement(200, 1000))
    manager._current_lateral_position_mm = 0.0
    assert not manager._slot_movement_is_opposite('left', SlotMovement(200, 5))


def use_real_wall_action(manager, wall):
    """Use the actual status validator and protection acceptance with fake transport."""
    manager._wall_control_client = wall
    manager._wall_max_alignment_error_mm = 100
    manager._wall_alignment_error_ignore_sec = 0.0
    manager._wall_alignment_recovery_distance_mm = 100
    manager._wall_minimum_lateral_clearance_mm = 10
    manager._last_wall_control_protection_stop = False
    manager._control_wall = lambda *args, **kwargs: MissionManager._control_wall(
        manager, *args, **kwargs)
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None,
                                               warning=lambda *_args: None)


def finish_protected_wall(wall, travel, distance=200, *, valid=True):
    result = FollowWall.Result()
    result.has_valid_reading = valid
    result.has_valid_odometry = True
    result.final_average_distance_mm = float(distance)
    result.traveled_distance_mm = float(travel)
    result.message = ('Obstaculo no lado direito a 3 mm do footprint; '
                      'minimo solicitado: 10 mm. Aproximacao frontal concluida; '
                      'deslocamento lateral interrompido.')
    wall.child.result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED, result=result))


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('slot,target,measured', [('left', 250, 70), ('right', -250, -70)])
def test_obstacle_stop_lets_cargo_finish_and_preserves_partial_position(
    operation, slot, target, measured,
):
    import threading

    manager, cargo, wall, _ = cargo_manager(operation, slot)
    use_real_wall_action(manager, wall)
    updated = threading.Event()
    original_record = manager._remember_blocked_search_destination

    def record(lateral, travel):
        original_record(lateral, travel)
        updated.set()

    manager._remember_blocked_search_destination = record
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', operation, tag_id=7, slot_id=slot),
                              SlotMovement(200, target))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        assert wall.goal.minimum_lateral_clearance_mm == 10
        assert wall.goal.alignment_recovery_distance_mm == 0
        finish_protected_wall(wall, measured)
        assert updated.wait(1.0)
        assert manager._current_lateral_position_mm == measured
        assert manager._blocked_search_positions['ws_1'] == {target}
        assert not cargo.child.canceled.is_set()
        assert not run.done()
        finish_cargo(cargo, operation)
        run.result(timeout=2.0)
    known, gripper, slots = manager._world_state.snapshot()
    assert known
    assert gripper == (EMPTY if operation == 'store' else 7)
    assert slots[slot] == (7 if operation == 'store' else EMPTY)
    assert not cargo.child.canceled.is_set()


def test_invalid_sensor_failure_waits_for_cargo_to_finish():
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    use_real_wall_action(manager, wall)
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(200, 250))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        finish_protected_wall(wall, 70, valid=False)
        with pytest.raises(TimeoutError):
            run.result(timeout=0.05)
        assert not cargo.child.canceled.is_set()
        finish_cargo(cargo, 'store')
        with pytest.raises(StepFailed):
            run.result(timeout=2.0)
    assert not cargo.child.canceled.is_set()
    assert manager._world_state.snapshot()[0]
    assert manager._world_state.snapshot()[2]['left'] == 7


def test_protected_partial_departure_is_retried_after_cargo_finishes():
    manager, cargo, wall, _ = cargo_manager('store', 'left')
    use_real_wall_action(manager, wall)
    manager._current_lateral_position_mm = -100.0
    manager._deactivate_service_area_vision = lambda: None
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', 'store', tag_id=7, slot_id='left'),
                              SlotMovement(250, 0, True))
        assert cargo.sent.wait(1.0)
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.0)
        finish_protected_wall(wall, 20, 250)
        finish_cargo(cargo, 'store')
        run.result(timeout=2.0)
    assert manager._current_lateral_position_mm == -80
    assert manager._departure_completed_for is None
    assert not cargo.child.canceled.is_set()
    assert manager._world_state.snapshot()[0]
    commands = []
    manager._control_wall = lambda *_args, **kwargs: commands.append(kwargs)
    manager._prepare_for_navigation = lambda: None
    manager._navigate_client = object()
    manager._navigation_timeout = lambda: 1.0
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    manager._call_action = lambda *_args: None
    manager._navigate('start')
    assert len(commands) == 1
    assert commands[0]['travel_distance_mm'] == 80


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('mode,travel,wall_distance,overlap', [
    ('disabled', 100, 200, False), ('opposite_sides', 100, 200, True),
    ('opposite_sides', -100, 200, False), ('always', -100, 200, True),
    ('always', 0, 150, True), ('always', 0, 200, False),
])
def test_configured_transfer_overlap_policy(operation, mode, travel, wall_distance, overlap):
    from mission_manager.models import AsyncMotionConfig
    manager, cargo, wall, commands = cargo_manager(operation, 'left')
    manager._arena = replace(manager._arena, async_motion_defaults=AsyncMotionConfig(mode, False))
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._execute_step,
                              Step('transfer', operation, tag_id=7, slot_id='left'),
                              SlotMovement(wall_distance, travel))
        assert cargo.sent.wait(1.)
        assert not wall.sent.is_set()
        if overlap:
            assert cargo.callback is not None
            cargo.phase(ManipulationFeedback.APPROACHING)
            assert wall.sent.wait(1.)
            finish_wall(wall, travel, wall_distance)
        else:
            assert cargo.callback is None
        finish_cargo(cargo, operation)
        run.result(timeout=2.)
    assert bool(commands) is overlap


@pytest.mark.parametrize('table_mode', ['disabled', 'opposite_sides', 'always'])
@pytest.mark.parametrize('boundary_enabled', [True, False])
def test_departure_overlap_uses_its_own_flag(table_mode, boundary_enabled):
    from mission_manager.models import AsyncMotionConfig
    manager, _, _, _ = cargo_manager('store', 'left')
    manager._arena = replace(manager._arena, async_motion_defaults=AsyncMotionConfig(table_mode, boundary_enabled))
    assert manager._slot_movement_can_overlap('left', SlotMovement(250, -100, True)) is boundary_enabled


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
@pytest.mark.parametrize('known_reference', [True, False])
def test_precision_organizer_passes_reference_destination_to_safe_overlap(operation, known_reference):
    from mission_manager.models import AsyncMotionConfig
    manager, cargo, wall, commands = cargo_manager(operation, 'left')
    manager._arena.service_areas['ws_1'] = replace(manager._arena.service_areas['ws_1'], area_type='PP',
                                                async_motion=AsyncMotionConfig('always', False))
    manager._pp_reference_observations = {('ws_1', 8): object()} if known_reference else {}
    manager._pp_reference_views = {('ws_1', 200, 0, False): frozenset({1}),
                                   ('ws_1', 200, 250, False): frozenset({2})}
    expected_travel = -100 if known_reference else -250
    manager._precision_memory_destination = lambda _memory: (200, -100)
    manager._pp_expected_occupancy = {}
    manager._completed_steps = 0
    manager._delivery_outcomes = []
    manager._report_scheduled_operation = lambda *_args: None
    manager._last_pp_scene = None
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(manager._pp_run_operation, None, None, Visit('pp', 'ws_1', ()),
                              operation, tag_id=7, reference_tag_id=8, slot_id='left')
        assert cargo.sent.wait(1.)
        assert not wall.sent.is_set()
        cargo.phase(ManipulationFeedback.APPROACHING)
        assert wall.sent.wait(1.)
        finish_wall(wall, expected_travel)
        finish_cargo(cargo, operation)
        run.result(timeout=2.)
    assert manager._completed_steps == 1
    assert commands[0][1]['travel_distance_mm'] == expected_travel
    assert manager._world_state.snapshot()[0]
