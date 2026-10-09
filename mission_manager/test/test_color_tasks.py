"""Color routing and confirmation before physical collection."""
from dataclasses import replace
from itertools import product
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml
from rclpy.action import GoalResponse

from mission_manager.errors import ConfigurationError, StepFailed, TaskNotFound
from mission_manager.loaders import load_arena, load_plan, validate_plan
from mission_manager.models import Plan, Step, Visit
from mission_manager.node import MissionManager
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import WorldState


def write_plan(tmp_path, task):
    raw = {'schema_version': 2, 'plan_id': 'colors', 'visits': [
        {'target': 'ws_1', 'tasks': [task]}]}
    path = tmp_path / 'plan.yaml'
    path.write_text(yaml.safe_dump(raw))
    return load_plan(path)


@pytest.mark.parametrize('task', [
    {'action': 'pick', 'tag_id': 1},
    {'action': 'pick', 'tag_ids': []},
    {'action': 'pick', 'tag_ids': [1, 1]},
    {'action': 'pick', 'tag_ids': [True]},
    {'action': 'pick', 'tag_ids': [-1]},
    {'action': 'pick', 'possible_tag_ids': [1], 'tag_color': 'red'},
    {'action': 'place_on_table', 'possible_tag_ids': [1]},
    {'action': 'place_on_table', 'tag_ids': [1], 'tag_color': 'red'},
    {'action': 'place_on_table', 'tag_ids': [1], 'possible_tag_ids': [1], 'tag_color': 'red'},
    {'action': 'place_on_table', 'possible_tag_ids': [], 'tag_color': 'red'},
    {'action': 'place_on_table', 'possible_tag_ids': [1, 1], 'tag_color': 'red'},
    {'action': 'place_on_table', 'possible_tag_ids': [1], 'tag_color': 'unknown'},
    {'action': 'stack', 'possible_tag_ids': [1], 'tag_color': 'red', 'support_tag_id': 14},
])
def test_invalid_new_task_syntax(tmp_path, task):
    with pytest.raises(ConfigurationError):
        write_plan(tmp_path, task)


def route(split=False):
    blue = Step('blue', 'place_in_container', container_color='blue',
                possible_tag_ids=(1, 2), tag_color='blue')
    red = Step('red', 'place_on_table' if split else 'place_in_container',
               container_color=None if split else 'red', possible_tag_ids=(1, 2), tag_color='red')
    visits = [Visit('source', 'ws_1', (Step('pick', 'pick', tag_ids=(1, 2)),)),
              Visit('blue_destination', 'ws_2', (blue,) if split else (blue, red))]
    if split:
        visits.append(Visit('red_destination', 'ws_3', (red,)))
    return Plan('colors', tuple(visits))


@pytest.mark.parametrize('split', [False, True])
@pytest.mark.parametrize('colors', list(product(('red', 'blue'), repeat=2)))
def test_color_routes_deliver_all_matching_candidates_once(split, colors):
    scheduler = Scheduler(route(split), ('left', 'right'))
    assert scheduler.feasible(scheduler.initial_state)
    scheduler.set_tag_colors(dict(zip((1, 2), colors)))
    state = scheduler.initial_state
    delivered = []
    for _ in range(50):
        if scheduler.complete(state):
            break
        choice = scheduler.select(state, {('tag', 1): 0, ('tag', 2): 0,
                                          ('container', 'red'): 0, ('container', 'blue'): 0})
        assert choice is not None
        if choice.step.action.startswith('place'):
            delivered.append((choice.step.tag_id, choice.step.tag_color))
        state = choice.next_state
    assert scheduler.complete(state)
    assert sorted(delivered) == list(zip((1, 2), colors))


def test_unknown_color_cannot_authorize_conditional_delivery():
    scheduler = Scheduler(route(), ('left', 'right'))
    state = replace(scheduler.initial_state, visit=1, gripper=1)
    assert not any(c.step.action.startswith('place') or c.step.action == 'skip'
                   for c in scheduler.choices(state))


def test_group_pick_expands_to_independent_collects(tmp_path):
    plan = write_plan(tmp_path, {'action': 'pick', 'tag_ids': [1, 2]})
    scheduler = Scheduler(plan, ('left', 'right'))
    assert [task.tag_id for _, task, _ in scheduler.tasks] == [1, 2]
    assert plan.total_steps == 3


def test_conditional_candidates_require_prior_collection(tmp_path):
    plan = write_plan(tmp_path, {'action': 'place_on_table', 'possible_tag_ids': [1], 'tag_color': 'red'})
    arena = load_arena(Path(__file__).parents[1] / 'config/arena.yaml')
    with pytest.raises(ConfigurationError, match='coleta'):
        validate_plan(plan, arena)


def color_manager():
    manager = MissionManager.__new__(MissionManager)
    manager._world_state = WorldState(['left', 'right'])
    manager._required_color_tags = {1}
    manager._check_canceled = lambda: None
    manager._pickup_config = lambda: SimpleNamespace(
        preferred_tag_x_m=0.0, preferred_tag_y_m=-0.22,
        travel_tolerance_mm=5, wall_tolerance_mm=5, max_reposition_attempts=2)
    manager.get_logger = lambda: SimpleNamespace(warning=lambda *_args: None)
    return manager


def detection(color=0, x=0.0):
    return SimpleNamespace(color=color, pose=SimpleNamespace(position=SimpleNamespace(x=x, y=-0.22)))


def test_unknown_aligns_and_observes_before_random_fallback(monkeypatch):
    manager = color_manager()
    events = []
    manager._recover_pick = lambda *_args, **_kwargs: events.append('align')
    manager._observe_visit = lambda: events.append('observe')
    manager._take_direct_pick_detection = lambda _tag: detection()
    monkeypatch.setattr('mission_manager.node.random.choice', lambda _colors: events.append('random') or 'blue')
    manager._confirm_pick_color(Step('pick', 'pick', tag_id=1), detection(x=0.1))
    assert events == ['align', 'observe', 'random']
    assert manager._world_state.tag_colors() == {1: 'blue'}


def test_aligned_detected_color_wins_without_random(monkeypatch):
    manager = color_manager()
    manager._recover_pick = lambda *_args, **_kwargs: None
    manager._observe_visit = lambda: None
    manager._take_direct_pick_detection = lambda _tag: detection(color=1)
    monkeypatch.setattr('mission_manager.node.random.choice', lambda _: pytest.fail('unnecessary random'))
    manager._confirm_pick_color(Step('pick', 'pick', tag_id=1), detection(x=0.1))
    assert manager._world_state.tag_colors() == {1: 'red'}


def test_unrelated_or_already_known_tag_needs_no_alignment():
    manager = color_manager()
    manager._confirm_pick_color(Step('pick', 'pick', tag_id=3), detection(x=0.2))
    assert manager._world_state.tag_colors() == {}
    manager._world_state.remember_tag_color(1, 'red')
    manager._confirm_pick_color(Step('pick', 'pick', tag_id=1), detection(x=0.2))
    assert manager._world_state.tag_colors() == {1: 'red'}


def test_unaligned_or_missing_tag_cannot_get_random_color():
    manager = color_manager()
    manager._recover_pick = lambda *_args, **_kwargs: None
    manager._observe_visit = lambda: None
    manager._take_direct_pick_detection = lambda _tag: detection(x=0.1)
    with pytest.raises(StepFailed, match='alinhar'):
        manager._confirm_pick_color(Step('pick', 'pick', tag_id=1), detection(x=0.1))
    manager._take_direct_pick_detection = lambda _tag: None
    with pytest.raises(TaskNotFound):
        manager._confirm_pick_color(Step('pick', 'pick', tag_id=1), None)
    assert manager._world_state.tag_colors() == {}


def test_color_memory_survives_transfers_and_resets_with_mission():
    world = WorldState(['left'])
    world.remember_tag_color(1, 'red')
    world.commit_pick(1)
    world.commit_store(1, 'left')
    world.remember_tag_color(1, 'unknown')
    world.remember_tag_color(1, 'blue')
    world.commit_retrieve(1, 'left')
    world.commit_place()
    assert world.tag_colors() == {1: 'red'}
    world.reset()
    assert world.tag_colors() == {}


def test_goal_is_validated_before_acceptance():
    manager = color_manager()
    manager._busy = False
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    def invalid(_plan_id):
        raise ConfigurationError('possible_tag_ids exige tag_color')
    manager._load_goal_files = invalid
    assert manager._goal_callback(SimpleNamespace(plan_id='invalid')) == GoalResponse.REJECT
    assert not manager._busy
    manager._load_goal_files = lambda _: ('arena', 'validated_plan')
    assert manager._goal_callback(SimpleNamespace(plan_id='valid')) == GoalResponse.ACCEPT
    assert manager._accepted_mission_files == ('arena', 'validated_plan')
    assert manager._goal_callback(SimpleNamespace(plan_id='valid')) == GoalResponse.REJECT


@pytest.mark.parametrize('split', [False, True])
@pytest.mark.parametrize('colors', list(product(('red', 'blue'), repeat=2)))
def test_executor_uses_remembered_colors_for_real_action_goals(split, colors):
    from test_visit_executor import simulated_manager
    plan = route(split)
    fixture_plan = replace(plan, visits=(replace(plan.visits[0], tasks=(
        Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2))),) + plan.visits[1:])
    manager, _navs, events, _departures = simulated_manager(fixture_plan)
    from interfaces.msg import ContainerStampedDetection
    from mission_manager.models import AsyncMotionConfig
    manager._arena = replace(manager._arena, async_motion_defaults=AsyncMotionConfig('disabled', False))
    for area_id, area in manager._arena.service_areas.items():
        manager._arena.service_areas[area_id] = replace(area, async_motion=AsyncMotionConfig('disabled', False))
    analyze = manager._vision_client.responder
    def visible_containers(goal):
        result = analyze(goal)
        for tag in result.best_apriltags_base:
            tag.color = 1 if colors[tag.id - 1] == 'red' else 2
        for color in (1, 2):
            item = ContainerStampedDetection()
            item.color = color
            item.pose.position.y = -0.22
            result.best_containers_base.append(item)
        return result
    manager._vision_client.responder = visible_containers
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _: None), plan)
    delivered = sorted((event[1], 'red' if event[0] == 'PlaceOnTable' else
                        colors[event[1] - 1]) for event in events
                       if event[0] in {'PlaceOnTable', 'PlaceInContainer'})
    assert delivered == list(zip((1, 2), colors))
    assert manager._world_state.tag_colors() == dict(zip((1, 2), colors))
    for outcome in manager._delivery_outcomes:
        if outcome.actual_action == 'place_in_container':
            assert outcome.container_color == colors[outcome.tag_id - 1]


def test_example_color_routes_are_feasible_with_configured_service_areas():
    package = Path(__file__).parents[1]
    arena = load_arena(package / 'config/arena.yaml')
    # The deployed arena names this area ws_67; use its geometry only in this test.
    arena.service_areas['ws_6'] = replace(arena.service_areas['ws_67'], area_id='ws_6')
    for name in ('advanced_transportation_test_i', 'advanced_transportation_two_containers'):
        plan = load_plan(package / f'config/plans/{name}.yaml')
        validate_plan(plan, arena)
        for colors in product(('red', 'blue'), repeat=2):
            scheduler = Scheduler(plan, ('left', 'right'))
            scheduler.set_tag_colors(dict(zip((1, 2), colors)))
            assert scheduler.feasible(scheduler.initial_state)
