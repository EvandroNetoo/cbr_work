"""Fault injection through real action messages and the visit scheduler."""
from types import SimpleNamespace

import pytest

from interfaces.action import PickObject, PlaceOnShelf, RetrieveObject
from interfaces.msg import ManipulationResult
from mission_manager.errors import MissionCanceled, RetrieveFailed, StepFailed
from mission_manager.models import Plan, Step, Visit
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import EMPTY
from test_visit_executor import outcome, simulated_manager


def transport():
    return Plan('recovery', (
        Visit('source', 'WS_1', (Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2))),
        Visit('destination', 'SH_1', (Step('d1', 'place_on_shelf', tag_id=1),
                                     Step('d2', 'place_on_shelf', tag_id=2))),
    ), finish=True)


def run(manager, plan):
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _: None), plan)


def failed(action_type, location, code=ManipulationResult.MOTION_FAILED):
    result = outcome(action_type, location)
    result.outcome.code = code
    result.outcome.message = 'injected failure'
    return result


@pytest.mark.parametrize('exception', [False, True])
@pytest.mark.parametrize('persistent', [False, True])
def test_pick_retry_and_independent_deliveries(exception, persistent):
    plan = transport()
    manager, navs, _events, _ = simulated_manager(plan)
    original = manager._pick_client.responder
    attempts = []
    def pick(goal):
        if goal.tag_id == 1:
            attempts.append(goal.tag_id)
            if persistent or len(attempts) == 1:
                if exception:
                    raise ValueError('unexpected computation error')
                return failed(PickObject, ManipulationResult.LOCATION_SOURCE)
        return original(goal)
    manager._pick_client.responder = pick
    run(manager, plan)
    assert len(attempts) == 2
    assert navs == ['WS_1', 'SH_1', 'finish']
    assert {delivery.tag_id for delivery in manager._delivery_outcomes} == ({2} if persistent else {1, 2})
    assert bool(manager._mission_failures) is persistent
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})


def test_failed_shelf_delivery_retains_cube_and_delivers_other_cube():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    original = manager._place_shelf_client.responder
    attempts = []
    def place(goal):
        if manager._world_state.snapshot()[1] == 1:
            attempts.append(1)
            return failed(PlaceOnShelf, ManipulationResult.LOCATION_GRIPPER)
        return original(goal)
    manager._place_shelf_client.responder = place
    run(manager, plan)
    assert len(attempts) == 2
    assert navs[-1] == 'finish'
    assert [d.tag_id for d in manager._delivery_outcomes] == [2]
    known, held, slots = manager._world_state.snapshot()
    assert known and [held, *slots.values()].count(1) == 1


def test_failed_retrieve_stops_route_and_preserves_cargo():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    original = manager._retrieve_client.responder
    attempts = []
    def retrieve(goal):
        if manager._world_state.snapshot()[2][goal.slot_id] == 1:
            attempts.append(goal.slot_id)
            return failed(RetrieveObject, ManipulationResult.LOCATION_CARGO)
        return original(goal)
    # Ensure cube 1 enters cargo before its destination.
    manager._scheduler_observations = lambda: {('tag', 1): 0, ('tag', 2): 10}
    manager._retrieve_client.responder = retrieve
    with pytest.raises(RetrieveFailed, match='Missão interrompida'):
        run(manager, plan)
    assert len(attempts) == 4
    assert navs == ['WS_1', 'SH_1']
    assert 1 in manager._world_state.snapshot()[2].values()
    assert manager._world_state.snapshot()[:2] == (True, EMPTY)
    assert not manager._mission_failures


@pytest.mark.parametrize('failures', [1, 3])
def test_retrieve_recovers_empty_arm_and_eventually_delivers(failures):
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    original = manager._retrieve_client.responder
    attempts = []
    recoveries = []
    prepare = manager._prepare_for_navigation

    def recover():
        assert manager._world_state.snapshot() == (True, EMPTY, {'left': 1, 'right': EMPTY})
        recoveries.append(True)
        prepare()

    def retrieve(goal):
        attempts.append(goal.slot_id)
        if len(attempts) <= failures:
            return failed(RetrieveObject, ManipulationResult.LOCATION_CARGO)
        return original(goal)

    manager._scheduler_observations = lambda: {('tag', 1): 0, ('tag', 2): 10}
    manager._prepare_for_navigation = recover
    manager._retrieve_client.responder = retrieve
    run(manager, plan)
    assert len(attempts) == failures + 1
    assert len(recoveries) == failures
    assert navs[-1] == 'finish'
    assert {d.tag_id for d in manager._delivery_outcomes} == {1, 2}
    assert not manager._mission_failures
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})


def test_shelf_retreat_failure_never_abandons_11_or_starts_shelf_picks():
    plan = Plan('shelf', (
        Visit('ws4', 'WS_4', (Step('collect', 'pick', tag_ids=(12, 11)),)),
        Visit('sh1', 'SH_1', (Step('deliver', 'place_on_shelf', tag_ids=(11, 15)),
                              Step('pick_shelf', 'pick', tag_ids=(15, 14)))),
        Visit('ws1', 'WS_1', (Step('place', 'place_in_container', tag_ids=(14, 12),
                                   container_color='blue'),)),
    ))
    manager, navs, events, _ = simulated_manager(plan)
    original = manager._pick_client.responder
    manager._pick_client.responder = lambda goal: (
        failed(PickObject, ManipulationResult.LOCATION_SOURCE) if goal.tag_id == 12 else original(goal))
    manager._scheduler_observations = lambda: {('tag', 11): 0, ('tag', 12): 10, ('tag', 15): 0}
    attempts = []

    def retreat(_area):
        attempts.append(True)
        raise StepFailed('Recuo para retirada do slot na SH não confirmado.')

    manager._retreat_before_shelf_retrieve = retreat
    with pytest.raises(RetrieveFailed, match='Recuo para retirada'):
        run(manager, plan)
    assert len(attempts) == 4
    assert navs == ['WS_4', 'SH_1']
    assert ('pick', 15) not in events and ('pick', 14) not in events
    assert not manager._retrieve_client.goals
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': 11, 'right': EMPTY})
    assert len(manager._mission_failures) == 1  # Only the unreachable 12 was abandoned.


def test_confirmed_retrieve_is_not_repeated_after_post_transfer_failure():
    plan = transport()
    manager, _, _, _ = simulated_manager(plan)
    execute = manager._execute_step
    attempts = []

    def fail_after_transfer(step, **kwargs):
        execute(step, **kwargs)
        if step.action == 'retrieve':
            attempts.append(step.tag_id)
            raise StepFailed('preparação de observação falhou após retirada')

    manager._execute_step = fail_after_transfer
    run(manager, plan)
    assert len(attempts) == 1
    assert {d.tag_id for d in manager._delivery_outcomes} == {1, 2}
    assert not manager._mission_failures


def test_uncertain_retrieve_stops_without_recovery_or_new_pick():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)

    def uncertain(_goal):
        result = failed(RetrieveObject, ManipulationResult.LOCATION_UNKNOWN)
        result.outcome.effect_known = False
        return result

    manager._retrieve_client.responder = uncertain
    manager._prepare_for_navigation = lambda: pytest.fail('Não recuperar com carga incerta')
    with pytest.raises(StepFailed, match='estado físico não confirmado'):
        run(manager, plan)
    assert len(manager._retrieve_client.goals) == 1
    assert navs == ['WS_1', 'SH_1']
    assert not manager._world_state.snapshot()[0]


def test_retrieve_cancellation_does_not_retry():
    manager, _, _, _ = simulated_manager(transport())
    manager._world_state.commit_pick(11, 'WS_4')
    manager._world_state.commit_store(11, 'left')
    attempts = []

    def cancel():
        attempts.append(True)
        raise MissionCanceled('cancel requested')

    with pytest.raises(MissionCanceled):
        manager._retry_retrieve(cancel, Step('retrieve', 'retrieve', tag_id=11, slot_id='left'),
                                expected=SimpleNamespace(gripper=11, slots=(EMPTY, EMPTY)))
    assert len(attempts) == 1


def test_retrieve_uses_its_own_attempt_limit():
    manager, _, _, _ = simulated_manager(transport())
    manager._operation_max_attempts = 1
    manager._retrieve_max_attempts = 3
    manager._world_state.commit_pick(11, 'WS_4')
    manager._world_state.commit_store(11, 'left')
    attempts = []

    def fail():
        attempts.append(True)
        raise StepFailed('motion failed')

    with pytest.raises(RetrieveFailed):
        manager._retry_retrieve(fail, Step('retrieve', 'retrieve', tag_id=11, slot_id='left'),
                                expected=SimpleNamespace(gripper=11, slots=(EMPTY, EMPTY)))
    assert len(attempts) == 3


def test_pp_retrieve_failure_propagates_without_abandoning_or_continuing_route():
    visit = Visit('pp', 'PP', (), None, ((1, 1),))
    plan = Plan('pp_retrieve', (visit,), finish=True)
    manager, navs, _, _ = simulated_manager(plan)
    manager._retrieve_client.responder = lambda _goal: failed(
        RetrieveObject, ManipulationResult.LOCATION_CARGO)

    def organize(goal, loaded_plan, loaded_visit):
        manager._world_state.commit_pick(11, 'PP')
        manager._world_state.commit_store(11, 'left')
        manager._pp_run_operation(goal, loaded_plan, loaded_visit, 'retrieve', tag_id=11, slot_id='left')

    manager._run_precision_organization = organize
    with pytest.raises(RetrieveFailed):
        run(manager, plan)
    assert navs == ['PP']
    assert len(manager._retrieve_client.goals) == 4
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': 11, 'right': EMPTY})
    assert not manager._mission_failures


@pytest.mark.parametrize('failed_action', ['navigate', 'observe'])
def test_failed_visit_keeps_remaining_route(failed_action):
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    original = manager._navigate if failed_action == 'navigate' else manager._observe_visit
    calls = []
    def fail(*args, **kwargs):
        if (args and args[0] == 'WS_1') or (
                failed_action == 'observe' and manager._current_location == 'WS_1'):
            calls.append(True)
            raise RuntimeError('visit unavailable')
        return original(*args, **kwargs)
    if failed_action == 'navigate':
        manager._navigate = fail
    else:
        manager._observe_visit = fail
    run(manager, plan)
    assert len(calls) == 2
    assert navs[-2:] == ['SH_1', 'finish']
    assert not manager._delivery_outcomes
    assert manager._mission_failures


def test_confirmed_pick_is_not_repeated_after_retreat_failure():
    plan = transport()
    manager, _, _, _ = simulated_manager(plan)
    original = manager._execute_step
    picks = []
    def execute(step, **kwargs):
        original(step, **kwargs)
        if step.action == 'pick':
            picks.append(step.tag_id)
            raise RuntimeError('retreat failed after grasp')
    manager._execute_step = execute
    run(manager, plan)
    assert sorted(picks) == [1, 2]
    assert len(manager._delivery_outcomes) == 2
    assert not manager._mission_failures


def test_cancel_never_retries_or_skips():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    def cancel(_target):
        raise MissionCanceled('cancel requested')
    manager._navigate = cancel
    with pytest.raises(MissionCanceled):
        run(manager, plan)
    assert not manager._mission_failures
    assert not navs


def test_uncertain_transfer_suspends_movement_without_fabricating_inventory():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    def uncertain(_goal):
        result = failed(PickObject, ManipulationResult.LOCATION_UNKNOWN)
        result.outcome.effect_known = False
        return result
    manager._pick_client.responder = uncertain
    with pytest.raises(StepFailed, match='servidor permanece ativo'):
        run(manager, plan)
    assert navs == ['WS_1']
    assert len(manager._pick_client.goals) == 1
    assert not manager._world_state.snapshot()[0]


def test_abandoned_stack_member_does_not_become_support():
    plan = Plan('stack', (
        Visit('source', 'WS_1', (Step('pick', 'pick', tag_ids=(1, 2)),)),
        Visit('stack', 'ST', (Step('stack', 'stack', tag_ids=(1, 2), support_tag_id=14),)),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    state, removed = scheduler.abandon(scheduler.initial_state, 1, 'WS_1')
    assert len(removed) == 2
    assert scheduler.feasible(state)
    for _ in range(20):
        choice = scheduler.select(state, {('tag', 2): 0, ('tag', 14): 0}, allow_unobserved=True)
        if choice is None:
            break
        if choice.step.action == 'stack':
            assert choice.step.tag_id == 2 and choice.step.support_tag_id == 14
        state = choice.next_state
    assert scheduler.complete(state)


def test_failed_stack_base_removes_dependents_transitively():
    plan = Plan('stack', (
        Visit('a', 'WS_1', (Step('pick', 'pick', tag_ids=(1, 2, 3)),)),
        Visit('b', 'ST', (Step('base', 'place_on_table', tag_id=1),
                          Step('middle', 'stack', tag_ids=(2,), support_tag_id=1),
                          Step('top', 'stack', tag_ids=(3,), support_tag_id=2))),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    state, removed = scheduler.abandon(scheduler.initial_state, 1, 'WS_1')
    assert {step.tag_id for step in removed} == {1, 2, 3}
    assert state.done == (1 << len(scheduler.tasks)) - 1
    assert scheduler.feasible(state)


@pytest.mark.parametrize('failed_action', ['pick', 'place_on_precision_table'])
def test_reactive_pp_abandons_failed_cube_and_organizes_independent_pair(failed_action):
    from test_precision_runtime import Robot
    board = {1: 2, 2: 1, 3: 4, 4: 3}
    robot = Robot(board)
    robot.get_logger = lambda: SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    original = robot._execute_step
    attempts = []
    def execute(step, **kwargs):
        if step.action == failed_action and step.tag_id == 2:
            attempts.append(step)
            raise RuntimeError('PP cube failed')
        return original(step, **kwargs)
    robot._execute_step = execute
    final = {1: 1, 2: 2, 3: 3, 4: 4}
    visit = Visit('pp', 'PP', (), None, tuple(final.items()))
    robot._run_precision_organization(None, Plan('pp', (visit,)), visit)
    assert len(attempts) == 2
    assert robot.board[3] == 3 and robot.board[4] == 4
    assert robot._mission_failures
    assert robot.held == EMPTY


def test_recovery_of_duplicate_tag_keeps_other_source_copy():
    plan = Plan('duplicates', (
        Visit('a', 'WS_1', (Step('pick_a', 'pick', tag_id=3),)),
        Visit('b', 'WS_2', (Step('pick_b', 'pick', tag_id=3),)),
        Visit('c', 'SH_1', (Step('deliver', 'place_on_shelf', possible_tag_ids=(3,), tag_color='red'),)),
    ))
    manager, navs, _, _ = simulated_manager(plan)
    original = manager._pick_client.responder
    def pick(goal):
        if manager._current_location == 'WS_1':
            return failed(PickObject, ManipulationResult.LOCATION_SOURCE)
        return original(goal)
    manager._pick_client.responder = pick
    manager._world_state.remember_tag_color(3, 'red', 'WS_1')
    manager._world_state.remember_tag_color(3, 'red', 'WS_2')
    run(manager, plan)
    assert navs == ['WS_1', 'WS_2', 'SH_1']
    assert [d.tag_id for d in manager._delivery_outcomes] == [3]


def test_partial_mission_returns_failure_summary_after_finishing_route():
    plan = transport()
    manager, navs, _, _ = simulated_manager(plan)
    arena = manager._arena
    original = manager._pick_client.responder
    manager._pick_client.responder = lambda goal: (
        failed(PickObject, ManipulationResult.LOCATION_SOURCE) if goal.tag_id == 1 else original(goal))
    handle = SimpleNamespace(publish_feedback=lambda _: None, abort=lambda: None, succeed=lambda: None)
    result = manager._execute_validated_mission(handle, arena, plan)
    from interfaces.action import ExecuteMission
    assert result.code == ExecuteMission.Result.STEP_FAILED
    assert result.failed_step_id == 'p1'
    assert navs[-1] == 'finish'
    assert 'tarefas independentes foram executadas' in result.message
    assert result.completed_steps == manager._completed_steps


def test_uncertain_dispatched_action_is_not_retried():
    from mission_manager.errors import ActionUncertain
    manager, _, _, _ = simulated_manager(transport())
    calls = []
    def timeout():
        calls.append(True)
        raise ActionUncertain('goal still active')
    with pytest.raises(ActionUncertain):
        manager._retry_operation(timeout, 'navigation')
    assert len(calls) == 1


def test_mission_callback_exception_keeps_executor_running(monkeypatch):
    from mission_manager import node
    states = iter((True, True, True, False))
    monkeypatch.setattr(node.rclpy, 'ok', lambda: next(states))
    calls = []
    errors = []
    def spin():
        calls.append(True)
        if len(calls) == 1:
            raise RuntimeError('callback failure')
    node._spin_executor(SimpleNamespace(spin_once=spin),
                        SimpleNamespace(get_logger=lambda: SimpleNamespace(error=errors.append)))
    assert len(calls) == 2
    assert 'callback failure' in errors[0]


@pytest.mark.parametrize('previous_ending', ['canceled', 'failed', 'unconfirmed', 'succeeded'])
def test_new_mission_clears_previous_execution_even_after_cancellation(previous_ending):
    from interfaces.action import ExecuteMission
    from mission_manager.errors import ActionUncertain
    plan = Plan('reset', (Visit('visit', 'WS_1', ()),), finish=True)
    manager, navs, _, _ = simulated_manager(plan)
    arena = manager._arena
    manager._load_goal_files = lambda _: (arena, plan)
    original_run = manager._run_plan
    caches = ('_tag_observations', '_pp_reference_observations', '_pp_reference_views',
              '_placed_tag_viewpoints', '_container_observations', '_container_search_positions',
              '_visited_search_positions', '_blocked_search_positions', '_scene_observations',
              '_safety_search_history', '_safety_search_attempts', '_pp_final_state',
              '_pp_expected_occupancy', '_task_attempts')
    sets = ('_pp_verified', '_pp_owned_slots', '_pp_selection_views',
            '_pp_failed_tags', '_pp_abandoned_references', '_required_color_tags')

    def interrupted(_handle, _plan):
        manager._world_state.remember_tag_color(99, 'red', 'WS_1')
        manager._world_state.commit_pick(99, 'WS_1')
        manager._world_state.commit_store(99, 'left')
        manager._world_state.commit_pick(98, 'WS_2')
        if previous_ending != 'succeeded':
            manager._world_state.mark_unknown()
        for attr in caches:
            setattr(manager, attr, {'previous': object()})
        for attr in sets:
            setattr(manager, attr, {99})
        manager._pp_inventory_pending = True
        manager._unconfirmed_action_pending = previous_ending == 'unconfirmed'
        manager._completed_steps = manager._current_step_index = 7
        manager._mission_failures = [] if previous_ending == 'succeeded' else [('old', 'previous failure')]
        manager._delivery_outcomes = [object()]
        manager._last_delivery_action = 'place_on_table_fallback'
        manager._failed_step_id = 'old'
        manager._pp_direct_pick_tag = manager._pp_skip_correct_pick_tag = 99
        manager._pp_organizing = True
        if previous_ending == 'canceled':
            raise MissionCanceled('previous cancellation')
        if previous_ending == 'failed':
            raise StepFailed('previous failure')
        if previous_ending == 'unconfirmed':
            raise ActionUncertain('previous missing result')

    handle = SimpleNamespace(request=SimpleNamespace(plan_id=plan.plan_id),
                             publish_feedback=lambda _: None, succeed=lambda: None,
                             abort=lambda: None, canceled=lambda: None)
    manager._run_plan = interrupted
    previous_result = manager._execute_callback(handle)
    assert previous_result.code == {
        'canceled': ExecuteMission.Result.CANCELED,
        'failed': ExecuteMission.Result.STEP_FAILED,
        'unconfirmed': ExecuteMission.Result.STEP_FAILED,
        'succeeded': ExecuteMission.Result.SUCCESS,
    }[previous_ending]
    assert manager._inventory_recovery_pending

    def fresh(goal, loaded_plan):
        assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})
        assert manager._world_state.object_colors() == {}
        assert manager._world_state.inventory_sources() == (None, {'left': None, 'right': None})
        assert all(not getattr(manager, attr) for attr in (*caches, *sets))
        assert not manager._pp_inventory_pending
        assert not manager._inventory_recovery_pending
        assert not manager._unconfirmed_action_pending
        assert not manager._pp_organizing
        assert manager._pp_direct_pick_tag is manager._pp_skip_correct_pick_tag is None
        assert manager._completed_steps == manager._current_step_index == 0
        assert manager._failed_step_id == ''
        assert manager._mission_failures == manager._delivery_outcomes == []
        assert manager._last_delivery_action is None
        return original_run(goal, loaded_plan)

    manager._run_plan = fresh
    result = manager._execute_callback(handle)
    assert result.code == ExecuteMission.Result.SUCCESS
    assert result.completed_steps == plan.total_steps
    assert navs == ['WS_1', 'finish']
