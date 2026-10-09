"""The YAML contract requires one list per action and destination/filter."""
from pathlib import Path

import pytest
import yaml

from mission_manager.errors import ConfigurationError
from mission_manager.loaders import load_arena, load_plan, validate_plan
from mission_manager.scheduler import Scheduler


def load_tasks(tmp_path, tasks):
    path = tmp_path / 'grouped.yaml'
    path.write_text(yaml.safe_dump({'schema_version': 2, 'plan_id': 'grouped',
                                  'visits': [{'target': 'ws_1', 'tasks': tasks}]}))
    return load_plan(path)


@pytest.mark.parametrize('parameters', [
    {'action': 'pick'},
    {'action': 'place_on_table'},
    {'action': 'place_on_shelf'},
    {'action': 'place_in_container', 'container_color': 'red'},
    {'action': 'stack', 'support_tag_id': 14},
    {'action': 'place_on_precision_table', 'reference_tag_id': 14},
])
@pytest.mark.parametrize('explicit_ids', [False, True])
def test_fragmented_lists_are_rejected(tmp_path, parameters, explicit_ids):
    tasks = [dict(parameters, tag_ids=[1]), dict(parameters, tag_ids=[2])]
    if explicit_ids:
        for index, task in enumerate(tasks):
            task['id'] = f'task_{index}'
    with pytest.raises(ConfigurationError, match='agrupadas em uma única task'):
        load_tasks(tmp_path, tasks)


def test_fragmented_conditional_delivery_is_rejected(tmp_path):
    with pytest.raises(ConfigurationError, match='agrupadas em uma única task'):
        load_tasks(tmp_path, [
            {'action': 'place_on_table', 'possible_tag_ids': [1], 'tag_color': 'red'},
            {'action': 'place_on_table', 'possible_tag_ids': [2], 'tag_color': 'red'},
        ])


def test_singleton_lists_and_distinct_destinations_are_valid(tmp_path):
    plan = load_tasks(tmp_path, [
        {'action': 'pick', 'tag_ids': [1, 2, 3, 4]},
        {'action': 'place_on_table', 'tag_ids': [1, 2]},
        {'action': 'place_in_container', 'container_color': 'red', 'tag_ids': [3]},
        {'action': 'place_in_container', 'container_color': 'blue', 'tag_ids': [4]},
    ])
    validate_plan(plan, load_arena(Path(__file__).parents[1] / 'config/arena.yaml'))
    assert plan.total_steps == 9
    scheduler = Scheduler(plan, ('left', 'right'))
    assert sorted(task.tag_id for _, task, _ in scheduler.tasks if task.action == 'pick') == [1, 2, 3, 4]


def test_different_color_filters_and_pp_references_remain_separate(tmp_path):
    plan = load_tasks(tmp_path, [
        {'action': 'place_on_table', 'possible_tag_ids': [1, 2], 'tag_color': 'red'},
        {'action': 'place_on_table', 'possible_tag_ids': [1, 2], 'tag_color': 'blue'},
        {'action': 'place_on_precision_table', 'tag_ids': [3], 'reference_tag_id': 10},
        {'action': 'place_on_precision_table', 'tag_ids': [4], 'reference_tag_id': 11},
    ])
    assert len(plan.visits[0].tasks) == 4


def test_split_pick_is_rejected_even_with_an_intervening_delivery(tmp_path):
    with pytest.raises(ConfigurationError, match='agrupadas em uma única task'):
        load_tasks(tmp_path, [
            {'action': 'pick', 'tag_ids': [1]},
            {'action': 'place_on_table', 'tag_ids': [1]},
            {'action': 'pick', 'tag_ids': [2]},
        ])


@pytest.mark.parametrize('path', sorted((Path(__file__).parents[1] / 'config/plans').glob('*.yaml')))
def test_all_shipped_missions_load_with_grouped_task_contract(path):
    load_plan(path)
