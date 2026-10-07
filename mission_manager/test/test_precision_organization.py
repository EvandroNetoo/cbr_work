from dataclasses import replace
from itertools import permutations
from pathlib import Path

import pytest
import yaml

from mission_manager.errors import ConfigurationError
from mission_manager.loaders import load_arena, load_plan, validate_plan
from mission_manager.models import Plan, Visit
from mission_manager.precision_organization import organize_precision_slots
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import EMPTY

PACKAGE = Path(__file__).parents[1]


def scheduled_organization(start, final, cargo_slots=('left', 'right')):
    tasks = tuple(replace(t, step_id=f'move_{i}') for i, t in enumerate(
        organize_precision_slots(start, final)))
    visit = Visit('pp', 'pp_1', tasks, tuple(start.items()), tuple(final.items()))
    scheduler = Scheduler(Plan('pp', (visit,)), cargo_slots)
    state = scheduler.initial_state
    assert scheduler.feasible(state)
    current = dict(start)
    operations = []
    # Observations rank operations but must not override occupancy ordering.
    observations = {('tag', tag): -tag * 100 for tag in start.values() if tag is not None}
    observations.update({('tag', slot): 100 for slot in start})
    for _ in range(200):
        if scheduler.complete(state):
            break
        choice = scheduler.select(state, observations, allow_unobserved=True)
        assert choice is not None
        step = choice.step
        if step.action == 'pick':
            assert current[step.reference_tag_id] == step.tag_id
            current[step.reference_tag_id] = None
        elif step.action == 'place_on_precision_table':
            assert current[step.reference_tag_id] is None
            assert state.gripper == step.tag_id
            current[step.reference_tag_id] = step.tag_id
        else:
            assert step.action in {'store', 'retrieve', 'depart'}
        operations.append(step)
        state = choice.next_state
        assert sum(tag != EMPTY for tag in state.slots) <= len(cargo_slots)
    assert scheduler.complete(state)
    assert current == final
    assert state.gripper == EMPTY and all(tag == EMPTY for tag in state.slots)
    return operations


@pytest.mark.parametrize('values', list(permutations((1, 2, 3, None))))
def test_all_four_slot_permutations_keep_every_placement_in_an_empty_slot(values):
    start = dict(zip(range(21, 25), values))
    operations = scheduled_organization(start, {21: 1, 22: 2, 23: 3, 24: None})
    assert not any(s.action in {'store', 'retrieve'} for s in operations)


@pytest.mark.parametrize('values', list(permutations((1, 2, 3, 4))))
def test_full_table_cycles_use_at_most_one_internal_buffer(values):
    scheduled_organization(dict(zip(range(21, 25), values)),
                           {21: 1, 22: 2, 23: 3, 24: 4}, cargo_slots=('left',))


def test_correct_cubes_are_not_moved_and_empty_destination_can_change():
    ops = scheduled_organization({21: 1, 22: 2, 23: None}, {21: 1, 22: None, 23: 2})
    assert [(s.action, s.tag_id) for s in ops if s.action != 'depart'] == [
        ('pick', 2), ('place_on_precision_table', 2)]


def test_already_organized_board_has_no_manipulations():
    assert organize_precision_slots({21: 1, 22: None}, {21: 1, 22: None}) == ()


def test_full_swap_is_infeasible_without_cargo_and_feasible_with_one_slot():
    start, final = {21: 1, 22: 2}, {21: 2, 22: 1}
    tasks = organize_precision_slots(start, final)
    plan = Plan('pp', (Visit('pp', 'pp_1', tasks, tuple(start.items()), tuple(final.items())),))
    no_cargo = Scheduler(plan, ())
    assert not no_cargo.feasible(no_cargo.initial_state)
    ops = scheduled_organization(start, final, cargo_slots=('left',))
    assert sum(s.action == 'store' for s in ops) == 1
    assert sum(s.action == 'retrieve' for s in ops) == 1


@pytest.mark.parametrize('start,final,message', [
    ({21: 1, 22: 1}, {21: 1, 22: 2}, 'duplicados'),
    ({21: 1, 22: 2}, {21: 1, 22: 1}, 'duplicados'),
    ({21: 1}, {21: 2}, 'mesmos cubos'),
    ({21: 1}, {22: 1}, 'mesmos alojamentos'),
    ({}, {}, 'não vazios'),
    ({21: True}, {21: 1}, 'inteiros'),
    ({'21': 1}, {'21': 1}, 'inteiros'),
    ({21: 21}, {21: 21}, 'distintas'),
    ({21: -1}, {21: -1}, 'inteiros'),
])
def test_invalid_board_is_rejected(start, final, message):
    with pytest.raises(ConfigurationError, match=message):
        organize_precision_slots(start, final)


def write_plan(tmp_path, tasks, target='pp_1'):
    path = tmp_path / 'plan.yaml'
    path.write_text(yaml.safe_dump({'schema_version': 2, 'plan_id': 'pp',
                                   'visits': [{'target': target, 'tasks': tasks}]}))
    return path


def test_load_organization_with_requested_yaml_shape_and_validate_area(tmp_path):
    tasks = [{'start_state': {21: 2, 22: 1, 23: None}},
             {'final_state': {21: 1, 22: 2, 23: None}}]
    plan = load_plan(write_plan(tmp_path, tasks))
    arena = load_arena(PACKAGE / 'config/arena.yaml')
    validate_plan(plan, arena)
    visit = plan.visits[0]
    assert dict(visit.pp_start_state) == tasks[0]['start_state']
    assert dict(visit.pp_final_state) == tasks[1]['final_state']
    assert len({t.step_id for t in visit.tasks}) == len(visit.tasks)
    assert plan.total_steps == 1 + len(visit.tasks)
    wrong_area = load_plan(write_plan(tmp_path, tasks, target='ws_1'))
    with pytest.raises(ConfigurationError, match='área PP'):
        validate_plan(wrong_area, arena)


@pytest.mark.parametrize('tasks', [
    [{'start_state': {21: 1}}],
    [{'final_state': {21: 1}}],
    [{'start_state': {21: 1}}, {'start_state': {21: 1}}, {'final_state': {21: 1}}],
    [{'start_state': {21: 1}}, {'final_state': {21: 1}}, {'action': 'pick', 'tag_id': 1}],
    [{'start_state': {21: 1}, 'final_state': {21: 1}}],
])
def test_malformed_or_mixed_organization_is_rejected(tmp_path, tasks):
    with pytest.raises(ConfigurationError):
        load_plan(write_plan(tmp_path, tasks))


def test_six_cube_example_reaches_final_state_without_using_cargo():
    plan = load_plan(PACKAGE / 'config/plans/cubos_1_2_3.yaml')
    visit = plan.visits[0]
    scheduled_organization(dict(visit.pp_start_state), dict(visit.pp_final_state))
