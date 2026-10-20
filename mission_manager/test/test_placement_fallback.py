"""Destination fallback follows a full search and a confirmed held object."""
from dataclasses import replace

import pytest
from interfaces.action import PlaceInContainer, PlaceOnPrecisionTable, PlaceOnTable, StackObject
from interfaces.msg import ManipulationResult, SceneObservation
from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.models import Step
from mission_manager.world_state import EMPTY
from test_executor_helpers import _safety_manager


def placement_manager(action, found_at=None, *, uncertain=False, canceled=False):
    manager = _safety_manager()
    manager._world_state.commit_pick(5)
    manager._placed_tag_viewpoints = {}
    manager._place_table_client = object()
    manager._stack_client = object()
    manager._place_precision_client = object()
    manager._place_container_client = object()
    calls = []
    types = {'stack': StackObject, 'place_on_precision_table': PlaceOnPrecisionTable,
             'place_on_table': PlaceOnTable, 'place_in_container': PlaceInContainer}
    def call(client, goal, *_args, **_kwargs):
        fallback = bool(getattr(goal, 'use_fallback_pose', False))
        calls.append((manager._current_wall_distance_mm, manager._current_lateral_position_mm,
                      bool(getattr(manager, '_search_led_off', False)), fallback))
        if canceled:
            raise MissionCanceled('cancel during search')
        action_type = PlaceOnTable if fallback else types[action]
        result = action_type.Result()
        result.outcome.effect_known = not uncertain
        success = fallback or found_at == len(calls)
        result.outcome.code = (ManipulationResult.SUCCESS if success else
                              ManipulationResult.NO_FREE_SPACE if action == 'place_on_table' else
                              ManipulationResult.OBJECT_NOT_FOUND)
        result.outcome.final_object_location = (
            ManipulationResult.LOCATION_DESTINATION if success else ManipulationResult.LOCATION_GRIPPER)
        result.scene_observation.completed = True
        result.scene_observation.requested_detectors = SceneObservation.APRILTAGS | SceneObservation.CONTAINERS_HSV
        return result
    manager._call_action = call
    step = Step('deposit', action, tag_id=5, support_tag_id=9, reference_tag_id=9,
                container_color='red')
    def execute():
        if action == 'stack':
            manager._execute_stack_with_search(step, StackObject.Goal(support_tag_id=9), 5, 120.)
        elif action == 'place_on_precision_table':
            manager._execute_precision_with_search(step, PlaceOnPrecisionTable.Goal(reference_tag_id=9), 5, 120.)
        else:
            goal = PlaceOnTable.Goal() if action == 'place_on_table' else PlaceInContainer.Goal(container_color=1)
            client = manager._place_table_client if action == 'place_on_table' else manager._place_container_client
            manager._execute_place_with_recovery(step, client, goal, 5, 12.5, 120.)
    return manager, calls, execute


@pytest.mark.parametrize('action', ['place_on_table', 'place_in_container', 'stack', 'place_on_precision_table'])
@pytest.mark.parametrize('found_at', [None, 3, 17])
def test_search_every_configured_view_before_fallback(action, found_at):
    manager, calls, execute = placement_manager(action, found_at)
    execute()
    search = [call for call in calls if not call[3]]
    if found_at == 3:
        assert len(search) == 3
        assert not any(call[3] for call in calls)
    else:
        assert len(search) == 17
        config = manager._arena.pickup_recovery
        assert {p for wall, p, dark, _ in search if wall == 200.} == set(config.search_positions_mm)
        for dark in (False, True):
            assert {p for wall, p, off, _ in search if wall == 60. and off == dark} == set(config.safety_search_positions_mm)
        assert sum(call[3] for call in calls) == (1 if found_at is None else 0)
        if found_at is None:
            assert calls[-1][3] and not calls[-1][2]
            assert manager._last_delivery_action == 'place_on_table_fallback'
    assert manager._world_state.snapshot()[0:2] == (True, EMPTY)


@pytest.mark.parametrize('action', ['place_on_table', 'place_in_container', 'stack', 'place_on_precision_table'])
@pytest.mark.parametrize('failure', ['uncertain', 'canceled'])
def test_unknown_effect_and_cancellation_never_trigger_fallback(action, failure):
    manager, calls, execute = placement_manager(action, uncertain=failure == 'uncertain',
                                               canceled=failure == 'canceled')
    with pytest.raises(MissionCanceled if failure == 'canceled' else StepFailed):
        execute()
    assert len(calls) == 1
    assert not calls[0][3]


def test_next_cube_gets_a_fresh_safety_search():
    manager, calls, execute = placement_manager('place_in_container')
    execute()
    manager._world_state.commit_pick(5)
    manager._last_table_observation = None
    calls.clear()
    execute()
    assert len([call for call in calls if call[0] == 60. and not call[3]]) == 14
    assert calls[-1][3]


@pytest.mark.parametrize('retreat_failed', [False, True])
def test_stack_fallback_becomes_new_support_with_confirmed_pose(retreat_failed):
    from types import SimpleNamespace
    from mission_manager.models import Plan, Visit
    from test_visit_executor import outcome, simulated_manager
    plan = Plan('stack_fallback', (
        Visit('collect', 'WS_1', (Step('pick', 'pick', tag_ids=(1, 2)),)),
        Visit('deliver', 'ST', (Step('pile', 'stack', tag_ids=(1, 2), support_tag_id=14),)),
    ), finish=True)
    manager, navs, _, _ = simulated_manager(plan)
    manager._arena = replace(manager._arena, pickup_recovery=replace(
        manager._arena.pickup_recovery, safety_search_positions_mm=()))
    # Collect 2 first, so 1 arrives held and exercises fallback before the
    # following member uses cube 1 as its new support.
    manager._scheduler_observations = lambda: (
        {('tag', 2): 0., ('tag', 1): 10.} if manager._current_location == 'WS_1' else {})
    supports, previous_poses = [], []
    def stack(goal):
        held = manager._world_state.snapshot()[1]
        supports.append((held, goal.support_tag_id))
        if held == 2:
            previous_poses.append((goal.has_previous_placed_pose, goal.require_alignment,
                                   goal.previous_placed_pose))
        result = outcome(StackObject, ManipulationResult.LOCATION_GRIPPER if held == 1 else
                         ManipulationResult.LOCATION_DESTINATION)
        if held == 1:
            result.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
        return result
    def fallback(_goal):
        result = outcome(PlaceOnTable, ManipulationResult.LOCATION_DESTINATION)
        result.placed_pose.header.frame_id = 'arm_base_link'
        result.placed_pose.pose.position.y = -.20
        result.placed_pose.pose.position.z = .10
        result.placed_pose.pose.orientation.w = 1.
        if retreat_failed:
            result.outcome.code = ManipulationResult.MOTION_FAILED
            result.outcome.message = 'retreat failed after release'
        return result
    manager._stack_client.responder = stack
    manager._place_table_client.responder = fallback
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _: None), plan)
    assert navs[-1] == 'ST'
    assert 'finish' not in navs
    assert supports[-1] == (2, 1)
    assert previous_poses[0][0], previous_poses
    assert not previous_poses[0][1]
    assert previous_poses[0][2].pose.position.y == pytest.approx(-.20)
    assert previous_poses[0][2].pose.position.z == pytest.approx(.10)
    assert [d.actual_action for d in manager._delivery_outcomes] == ['place_on_table_fallback', 'stack']
    assert manager._delivery_outcomes[0].support_tag_id is None
    assert manager._world_state.snapshot()[0:2] == (True, EMPTY)
    assert manager._mission_failures


def test_reactive_pp_fallback_does_not_mark_requested_cavity_occupied():
    from mission_manager.models import Plan, Visit
    from test_precision_runtime import Robot
    robot = Robot({1: 2, 2: 1, 3: 4, 4: 3})
    original = robot._execute_step
    def execute(step, **kwargs):
        if step.action == 'place_on_precision_table' and step.tag_id == 2:
            robot._execute_place_fallback(step, 2, 12.5, 1.)
            return
        return original(step, **kwargs)
    robot._execute_step = execute
    visit = Visit('pp', 'PP', (), None, tuple({i: i for i in range(1, 5)}.items()))
    robot._run_precision_organization(None, Plan('pp', (visit,)), visit)
    assert robot.board[2] is None
    assert robot.board[3] == 3 and robot.board[4] == 4
    assert robot.fallback == [2]
    fallback = next(d for d in robot._delivery_outcomes if d.tag_id == 2)
    assert fallback.actual_action == 'place_on_table_fallback'
    assert fallback.reference_tag_id is None


def test_reactive_pp_missing_reference_checks_lit_and_dark_views_before_fallback():
    from types import SimpleNamespace
    from mission_manager.errors import DestinationUnavailable, PrecisionTaskFailed
    from mission_manager.models import Plan, Visit
    manager = _safety_manager()
    manager._arena.service_areas['WS_1'] = replace(manager._arena.service_areas['WS_1'], area_type='PP')
    manager._world_state.commit_pick(5)
    manager._pp_reference_observations = {}
    manager._last_pp_scene = None
    manager._check_canceled = lambda: None
    manager._manipulation_timeout = lambda: 1.
    manager._report_scheduled_operation = lambda *_: None
    manager._completed_steps = 0
    manager._delivery_outcomes = []
    manager._place_table_client = object()
    views, fallback = [], []
    def observe():
        views.append((manager._current_wall_distance_mm, manager._current_lateral_position_mm,
                      bool(getattr(manager, '_search_led_off', False))))
        manager._remember_scene_observations(SimpleNamespace(scene_observation=SceneObservation(
            completed=True, requested_detectors=SceneObservation.APRILTAGS)))
    manager._observe_visit = observe
    def release(_client, goal, *_args, **_kwargs):
        fallback.append(goal.use_fallback_pose)
        assert len(views) >= 17
        result = PlaceOnTable.Result()
        result.outcome.code = ManipulationResult.SUCCESS
        result.outcome.effect_known = True
        result.outcome.final_object_location = ManipulationResult.LOCATION_DESTINATION
        return result
    manager._call_action = release
    visit = Visit('pp', 'WS_1', ())
    with manager._search_session():
        with pytest.raises(DestinationUnavailable):
            manager._pp_seek_for_object(9, 5, visit)
        with pytest.raises(PrecisionTaskFailed, match='fallback'):
            manager._pp_fallback_for_object(None, Plan('pp', (visit,)), visit, 5, 9)
    config = manager._arena.pickup_recovery
    assert {p for wall, p, _dark in views if wall == 200.} == set(config.search_positions_mm)
    for dark in (False, True):
        assert {p for wall, p, off in views if wall == 60. and off == dark} == set(config.safety_search_positions_mm)
    assert fallback == [True]
    assert manager._world_state.snapshot()[0:2] == (True, EMPTY)


def test_full_pp_cargo_searches_other_vacancies_when_requested_references_are_missing():
    from test_precision_runtime import Robot
    robot = Robot({1: None, 2: None})
    robot.cargo = {'left': 5, 'right': 6}
    robot._observe_visit()
    slot, cube, target = robot._pp_full_cargo_destination(
        [('left', 5), ('right', 6)], {5: 8, 6: 9})
    assert (slot, cube) == ('left', 5)
    assert target in {1, 2}
    assert not robot.events
