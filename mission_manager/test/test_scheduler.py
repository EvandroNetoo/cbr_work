from dataclasses import replace

import pytest

from mission_manager.models import Plan, Step, Visit
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import EMPTY


def mission():
    return Plan('flexible', (
        Visit('ws1', 'ws_1', tuple(Step(f'pick_{i}', 'pick', tag_id=i) for i in (1, 2, 3))),
        Visit('ws2', 'ws_2', (Step('put_1', 'place_on_table', tag_id=1),
                            Step('put_2', 'place_in_container', tag_id=2, container_color='red'),
                            Step('pick_4', 'pick', tag_id=4))),
        Visit('ws3', 'ws_3', (Step('put_3', 'place_on_shelf', tag_id=3),
                            Step('put_4', 'place_on_table', tag_id=4))),
    ), finish=True)


def advance(scheduler, state, action, tag=None):
    return next(c.next_state for c in scheduler.viable_choices(state)
                if c.step.action == action and (tag is None or c.step.tag_id == tag))


def test_different_identifications_choose_different_first_pick():
    scheduler = Scheduler(mission(), ('left', 'right'))
    state = scheduler.initial_state
    assert scheduler.feasible(state)
    assert scheduler.select(state, {('tag', 2): 0}).step.tag_id == 2
    assert scheduler.select(state, {('tag', 3): 0}).step.tag_id == 3
    assert scheduler.select(state, {('tag', 99): 0}) is None


def test_last_pick_cannot_leave_later_delivery_in_gripper():
    scheduler = Scheduler(mission(), ('left', 'right'))
    state = scheduler.initial_state
    for tag in (1, 2):
        state = next(c.next_state for c in scheduler.choices(state)
                     if c.step.action == 'pick' and c.step.tag_id == tag)
        state = next(c.next_state for c in scheduler.choices(state) if c.step.action == 'store')
    assert not scheduler.feasible(state)
    assert not any(c.step.action == 'pick' for c in scheduler.viable_choices(state))


def test_later_tag_stored_and_mixed_next_visit_remains_viable():
    scheduler = Scheduler(mission(), ('left', 'right'))
    state = scheduler.initial_state
    for tag in (3, 2):
        state = advance(scheduler, state, 'pick', tag)
        state = advance(scheduler, state, 'store')
    state = advance(scheduler, state, 'pick', 1)
    state = advance(scheduler, state, 'depart')
    assert state.gripper == 1 and state.slots == (3, 2)
    choice = scheduler.select(state, {('tag', 4): 0, ('container', 'red'): 0})
    assert choice.step.action == 'place_on_table'
    state = choice.next_state
    # Identification of tag 4 can be acted upon only if completion stays feasible.
    assert scheduler.feasible(state)


def test_preflight_rejects_missing_delivery_and_excess_capacity():
    no_delivery = Plan('bad', (Visit('v', 'ws_1', (Step('p', 'pick', tag_id=1),)),))
    assert not Scheduler(no_delivery, ('left', 'right')).feasible(
        Scheduler(no_delivery, ('left', 'right')).initial_state)
    plan = mission()
    scheduler = Scheduler(plan, ())
    assert not scheduler.feasible(scheduler.initial_state)


@pytest.mark.parametrize('order', [(4, 5), (5, 4)])
def test_stack_group_uses_confirmed_top_in_either_order(order):
    plan = Plan('stack', (
        Visit('collect', 'ws_1', tuple(Step(f'p{i}', 'pick', tag_id=i) for i in order)),
        Visit('deliver', 'ws_2', (Step('pile', 'stack', support_tag_id=14, tag_ids=(4, 5)),)),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    state = scheduler.initial_state
    state = advance(scheduler, state, 'pick', order[1])
    state = advance(scheduler, state, 'store')
    state = advance(scheduler, state, 'pick', order[0])
    state = advance(scheduler, state, 'depart')
    choice = scheduler.select(state, {('tag', 14): 0})
    assert choice.step.tag_id == order[0] and choice.step.support_tag_id == 14
    state = choice.next_state
    state = advance(scheduler, state, 'retrieve', order[1])
    choice = scheduler.select(state, {})
    assert choice.step.support_tag_id == order[0]
    state = choice.next_state
    assert scheduler.complete(advance(scheduler, state, 'depart'))
    assert plan.total_steps == 6


def test_support_cannot_be_picked_with_an_object_stacked_above_it():
    plan = Plan('dependency', (
        Visit('v', 'ws_1', (Step('p4', 'pick', tag_id=4),
                           Step('pile', 'stack', support_tag_id=14, tag_ids=(4,)),
                           Step('p14', 'pick', tag_id=14),
                           Step('put14', 'place_on_table', tag_id=14))),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    assert scheduler.feasible(scheduler.initial_state)
    state = advance(scheduler, scheduler.initial_state, 'pick', 4)
    state = next(c.next_state for c in scheduler.choices(state) if c.step.action == 'stack')
    assert not any(c.step.action == 'pick' and c.step.tag_id == 14
                   for c in scheduler.choices(state))


def test_ranking_current_view_then_memory_then_task_id():
    scheduler = Scheduler(mission(), ('left', 'right'))
    state = scheduler.initial_state
    assert scheduler.select(state, {('tag', 1): 250, ('tag', 2): 0}).step.tag_id == 2
    assert scheduler.select(state, {('tag', 1): 250, ('tag', 2): 100}).step.tag_id == 2
    assert scheduler.select(state, {('tag', 1): 0, ('tag', 2): 0}).step.tag_id == 1


def test_empty_visit_does_not_require_unnecessary_storage():
    plan = Plan('route', (
        Visit('a', 'ws_1', (Step('p', 'pick', tag_id=1),)),
        Visit('b', 'ws_2', ()),
        Visit('c', 'ws_3', (Step('d', 'place_on_table', tag_id=1),)),
    ))
    scheduler = Scheduler(plan, ('right', 'left'))
    state = advance(scheduler, scheduler.initial_state, 'pick', 1)
    choice = scheduler.select(state, {})
    assert choice.step.action == 'depart'
    state = choice.next_state
    assert state.gripper == 1 and state.slots == (EMPTY, EMPTY)
    assert advance(scheduler, state, 'depart').gripper == 1


@pytest.mark.parametrize('empty_visits', [1, 2])
@pytest.mark.parametrize('order', [(1, 2, 3), (3, 2, 1)])
def test_full_cargo_can_cross_empty_visits_before_delivery(empty_visits, order):
    plan = Plan('transit', (
        Visit('collect', 'ws_4', tuple(Step(f'p{i}', 'pick', tag_id=i) for i in order)),
        *(Visit(f'transit{i}', 'ws_5', ()) for i in range(empty_visits)),
        Visit('deliver', 'ws_6', tuple(Step(f'd{i}', 'place_on_table', tag_id=i) for i in order)),
    ), finish=True)
    scheduler = Scheduler(plan, ('left', 'right'))
    assert scheduler.feasible(scheduler.initial_state)
    state = scheduler.initial_state
    for tag in order[:-1]:
        state = advance(scheduler, state, 'pick', tag)
        state = advance(scheduler, state, 'store')
    state = advance(scheduler, state, 'pick', order[-1])
    for _ in range(empty_visits + 1):
        choice = scheduler.select(state, {})
        assert choice.step.action == 'depart'
        state = choice.next_state
        assert state.gripper == order[-1] and state.slots == order[:-1]
    while not scheduler.complete(state):
        choice = scheduler.select(state, {})
        assert choice is not None
        state = choice.next_state
    assert state.gripper == EMPTY and state.slots == (EMPTY, EMPTY)


def test_empty_visits_do_not_allow_skipping_next_manipulation_visit():
    plan = mission()
    plan = replace(plan, visits=(plan.visits[0], Visit('transit', 'ws_4', ()), *plan.visits[1:]))
    scheduler = Scheduler(plan, ('left', 'right'))
    state = scheduler.initial_state
    for tag in (1, 2):
        state = next(c.next_state for c in scheduler.choices(state)
                     if c.step.action == 'pick' and c.step.tag_id == tag)
        state = next(c.next_state for c in scheduler.choices(state) if c.step.action == 'store')
    assert not scheduler.feasible(state)


def test_trailing_empty_visits_do_not_allow_undelivered_cargo():
    plan = Plan('missing_delivery', (
        Visit('collect', 'ws_4', (Step('p1', 'pick', tag_id=1),)),
        Visit('transit', 'ws_5', ()),
    ), finish=True)
    scheduler = Scheduler(plan, ('left', 'right'))
    assert not scheduler.feasible(scheduler.initial_state)


def test_search_checks_cancellation():
    def cancel():
        raise RuntimeError('canceled')
    scheduler = Scheduler(mission(), ('left', 'right'), cancel)
    with pytest.raises(RuntimeError, match='canceled'):
        scheduler.feasible(scheduler.initial_state)


def test_precision_delivery_ranks_reference_separately_from_same_id_object():
    plan = Plan('pp_delivery', (
        Visit('collect', 'ws_67', (Step('pick_1', 'pick', tag_id=1),
                                   Step('pick_2', 'pick', tag_id=2))),
        Visit('deliver', 'pp_67', (
            Step('place_1', 'place_on_precision_table', tag_id=1, reference_tag_id=1),
            Step('place_2', 'place_on_precision_table', tag_id=2, reference_tag_id=2))),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    state = advance(scheduler, scheduler.initial_state, 'pick', 1)
    state = advance(scheduler, state, 'store')
    state = advance(scheduler, state, 'pick', 2)
    state = advance(scheduler, state, 'depart')
    state = advance(scheduler, state, 'store')
    assert scheduler.select(state, {}) is None
    assert scheduler.select(state, {('tag', 1): 325}) is None
    choice = scheduler.select(state, {('reference', 1): 325}, 325)
    assert (choice.step.action, choice.step.tag_id) == ('retrieve', 1)
    pick_state = scheduler.initial_state
    assert scheduler.select(pick_state, {('reference', 1): 0}) is None
