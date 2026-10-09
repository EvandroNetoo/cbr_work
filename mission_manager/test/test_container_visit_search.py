"""Container delivery must never trigger a search for unrelated AprilTags."""
from dataclasses import replace
from types import SimpleNamespace

import pytest
from interfaces.action import AnalyzeScene, PlaceInContainer
from interfaces.msg import ContainerStampedDetection, SceneObservation

from mission_manager.loaders import load_plan
from mission_manager.models import AsyncMotionConfig, Plan, Step, Visit
from mission_manager.scheduler import Scheduler
from mission_manager.world_state import EMPTY
from test_visit_executor import detection, simulated_manager


def container(color):
    item = ContainerStampedDetection()
    item.color = color
    item.pose.position.y = -0.22
    item.pose.orientation.w = 1.0
    return item


def disable_parallel_motion(manager):
    manager._async_motion_config = lambda *_args: AsyncMotionConfig('disabled', False)


@pytest.mark.parametrize('area_type', ['PP', 'WS', 'SH'])
@pytest.mark.parametrize('visible_blue', [True, False])
@pytest.mark.parametrize('conditional', [True, False])
def test_second_container_delivery_uses_hsv_and_its_own_search(area_type, visible_blue, conditional):
    blue = Step('blue', 'place_in_container', tag_id=1, container_color='blue')
    red = Step('red', 'place_in_container', tag_id=2, container_color='red')
    if conditional:
        blue = replace(blue, tag_id=None, possible_tag_ids=(1, 2), tag_color='blue')
        red = replace(red, tag_id=None, possible_tag_ids=(1, 2), tag_color='red')
    plan = Plan('two_containers', (
        Visit('collect', 'ws_67', (Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2))),
        Visit('deliver', 'pp_67', (blue, red)),
    ))
    manager, navs, events, _ = simulated_manager(plan)
    disable_parallel_motion(manager)
    manager._arena.service_areas['pp_67'] = replace(
        manager._arena.service_areas['pp_67'], area_type=area_type)
    # This regression is about destination perception; colors are already known.
    manager._world_state.remember_tag_color(1, 'blue')
    manager._world_state.remember_tag_color(2, 'red')
    original_analyze = manager._vision_client.responder
    scans = []
    def analyze(goal):
        if manager._current_location != 'pp_67':
            return original_analyze(goal)
        scans.append(goal.requested_detectors)
        assert goal.requested_detectors & AnalyzeScene.Goal.CONTAINERS_HSV
        assert goal.classify_pp_tags == (area_type == 'PP')
        result = AnalyzeScene.Result(frames_processed=10, frames_with_base_transform=10)
        if visible_blue:
            result.best_containers_base = [container(PlaceInContainer.Goal.BLUE)]
        return result
    manager._vision_client.responder = analyze
    def unrelated_tag_search(_tag):
        pytest.fail('Container delivery triggered AprilTag search')
    manager._move_to_next_search_position = unrelated_tag_search
    destination_searches = []
    original_destination_search = manager._move_to_next_place_search_position
    def destination_search(step, visited, positions):
        destination_searches.append(step.action)
        assert step.action == 'place_in_container'
        return original_destination_search(step, visited, positions)
    manager._move_to_next_place_search_position = destination_search
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _: None), plan)
    assert navs == ['ws_67', 'pp_67']
    assert [event for event in events if event[0] == 'PlaceInContainer'] == [
        ('PlaceInContainer', 2), ('PlaceInContainer', 1)]
    assert [goal.container_color for goal in manager._place_container_client.goals] == [
        PlaceInContainer.Goal.RED, PlaceInContainer.Goal.BLUE]
    assert len(scans) == 1
    assert destination_searches == ([] if visible_blue else ['place_in_container'])
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})


def test_two_container_plan_is_valid_new_syntax_and_keeps_requested_destinations(tmp_path):
    # Keep the regression independent of the user's currently selected mission.
    path = tmp_path / 'containers.yaml'
    path.write_text('schema_version: 2\nplan_id: containers\nvisits:\n  - target: ws_67\n    tasks:\n      - {action: pick, tag_ids: [1, 2]}\n  - target: pp_67\n    tasks:\n      - {action: place_in_container, container_color: blue, tag_ids: [1]}\n      - {action: place_in_container, container_color: red, tag_ids: [2]}\n')
    plan = load_plan(path)
    scheduler = Scheduler(plan, ('left', 'right'))
    assert scheduler.feasible(scheduler.initial_state)
    assert [(task.tag_ids, task.container_color) for task in plan.visits[1].tasks] == [
        ((1,), 'blue'), ((2,), 'red')]


@pytest.mark.parametrize('detector', [AnalyzeScene.Goal.APRILTAGS, AnalyzeScene.Goal.CONTAINERS_HSV])
def test_partial_scene_does_not_invalidate_other_detector_memory(detector):
    manager, _, _, _ = simulated_manager(Plan('memory', (Visit('v', 'ws_1', ()),)))
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.
    manager._prepare_for_pick_observation = lambda: None
    scene = SceneObservation(completed=True, requested_detectors=(
        SceneObservation.APRILTAGS | SceneObservation.CONTAINERS_HSV))
    scene.apriltags = [detection(1)]
    scene.containers = [container(PlaceInContainer.Goal.BLUE)]
    manager._remember_scene_observations(SimpleNamespace(scene_observation=scene))
    manager._vision_client.responder = lambda _goal: AnalyzeScene.Result(
        frames_processed=10, frames_with_base_transform=10)
    manager._observe_visit(requested_detectors=detector)
    if detector == AnalyzeScene.Goal.APRILTAGS:
        assert ('ws_1', 1) not in manager._tag_observations
        assert ('ws_1', PlaceInContainer.Goal.BLUE) in manager._container_observations
    else:
        assert ('ws_1', 1) in manager._tag_observations
        assert ('ws_1', PlaceInContainer.Goal.BLUE) not in manager._container_observations


def test_apriltag_scene_does_not_satisfy_container_observation():
    manager, _, _, _ = simulated_manager(Plan('memory', (Visit('v', 'ws_1', ()),)))
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.
    scene = SceneObservation(completed=True, requested_detectors=SceneObservation.APRILTAGS)
    manager._remember_scene_observations(SimpleNamespace(scene_observation=scene))
    assert manager._current_scene_observed()
    assert not manager._current_scene_observed(SceneObservation.CONTAINERS_HSV)
    assert not manager._current_scene_observed(SceneObservation.APRILTAGS | SceneObservation.CONTAINERS_HSV)


def test_store_lookahead_does_not_move_to_apriltag_search_for_container():
    from test_slot_parallel import planning_manager
    plan = Plan('lookahead', (
        Visit('source', 'ws_1', (Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2))),
        Visit('container', 'ws_1', (Step('blue', 'place_in_container', tag_id=1, container_color='blue'),)),
        Visit('table', 'ws_2', (Step('table', 'place_on_table', tag_id=2),)),
    ))
    scheduler = Scheduler(plan, ('left', 'right'))
    state = replace(scheduler.initial_state, visit=1, done=scheduler.masks[0],
                    gripper=2, slots=(1, EMPTY), locations=(None, None))
    store = next(c for c in scheduler.viable_choices(state) if c.step.action == 'store')
    manager = planning_manager()
    assert manager._scheduler_search_tag(scheduler, store.next_state) is None
    assert manager._next_slot_movement(scheduler, store, plan) is None
