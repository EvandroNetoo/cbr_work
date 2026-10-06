"""Exercise visit orchestration through the existing ROS action message protocol."""
from dataclasses import replace
from pathlib import Path
import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from interfaces.action import (
    AnalyzeScene, ExecuteMission, PickObject, PlaceInContainer, PlaceOnShelf,
    PlaceOnTable, PrepareManipulator, RetrieveObject, StackObject, StoreObject,
)
from interfaces.msg import AprilTagStampedDetection, ManipulationResult
from rclpy.task import Future
import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.loaders import load_arena
from mission_manager.models import Plan, Step, Visit
from mission_manager.node import MissionManager
from mission_manager.world_state import EMPTY, WorldState


def completed_future(value):
    future = Future()
    future.set_result(value)
    return future


class SimulatedActionClient:
    """Synchronous child server with the same futures consumed by _call_action."""
    def __init__(self, responder):
        self.responder = responder
        self.goals = []
        self.canceled = False

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        result = self.responder(goal)
        def cancel():
            self.canceled = True
            return completed_future(None)
        handle = SimpleNamespace(accepted=True, cancel_goal_async=cancel,
                                 get_result_async=lambda: completed_future(SimpleNamespace(
                                     status=GoalStatus.STATUS_SUCCEEDED, result=result)))
        return completed_future(handle)


def detection(tag):
    result = AprilTagStampedDetection()
    result.id = tag
    result.header.frame_id = 'arm_base_link'
    result.pose.orientation.w = 1.0
    result.pose.position.y = -0.22
    return result


def outcome(action_type, location):
    result = action_type.Result()
    result.outcome.code = ManipulationResult.SUCCESS
    result.outcome.effect_known = True
    result.outcome.final_object_location = location
    return result


def simulated_manager(plan, visible=None):
    manager = MissionManager.__new__(MissionManager)
    manager._arena = load_arena(Path(__file__).parents[1] / 'config' / 'arena.yaml')
    manager._world_state = WorldState(['left', 'right'])
    manager._lock = threading.RLock()
    manager._cancel_event = threading.Event()
    manager._active_child = None
    manager._completed_steps = 0
    manager._failed_step_id = ''
    manager._delivery_outcomes = []
    manager._flexible_pick = False
    manager._current_location = 'start'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._stack_alignment = None
    manager._service_area_vision_active = False
    manager._search_led_off = False
    manager._search_phase = 0
    manager._tag_observations = {}
    manager._placed_tag_viewpoints = {}
    manager._container_observations = {}
    manager._container_search_positions = {}
    manager._visited_search_positions = {}
    manager._blocked_search_positions = {}
    manager._last_table_observation = None
    manager._publish_world_state = lambda: None
    manager._server_timeout = lambda: 1.0
    manager._manipulation_timeout = lambda: 1.0
    manager._duration = lambda seconds: Duration(sec=int(seconds))
    manager._retreat_from_lateral_wall_before_slot_access = lambda *_args: None
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None, warning=lambda *_args: None)
    scenes = {visit.target: {task.tag_id for task in visit.tasks if task.action == 'pick'}
              for visit in plan.visits}
    navs = []
    events = []
    held_at_departure = []

    def navigate(target):
        known, gripper, slots = manager._world_state.snapshot()
        held_at_departure.append((target, gripper, slots))
        navs.append(target)
        manager._current_location = target
        manager._current_wall_distance_mm = 200.0
        manager._current_lateral_position_mm = 0.0
        manager._last_table_observation = None
    manager._navigate = navigate
    def move(wall, lateral, _description):
        manager._current_wall_distance_mm = float(wall)
        manager._current_lateral_position_mm = float(lateral)
        manager._last_table_observation = None
        return True
    manager._move_to_table_position = move

    def analyze(goal):
        assert goal.duration.sec == 2
        assert goal.requested_detectors == AnalyzeScene.Goal.APRILTAGS | AnalyzeScene.Goal.CONTAINERS_HSV
        assert manager._world_state.snapshot()[1] == EMPTY
        events.append(('observe', manager._current_location))
        result = AnalyzeScene.Result()
        result.frames_processed = 10
        result.frames_with_base_transform = 10
        tags = scenes.get(manager._current_location, set())
        if visible is not None:
            tags = visible(manager, tags)
        result.best_apriltags_base = [detection(tag) for tag in sorted(tags)]
        return result
    manager._vision_client = SimulatedActionClient(analyze)
    manager._prepare_client = SimulatedActionClient(lambda _goal: outcome(
        PrepareManipulator, ManipulationResult.LOCATION_UNKNOWN))

    def pick(goal):
        events.append(('pick', goal.tag_id))
        scenes[manager._current_location].discard(goal.tag_id)
        result = outcome(PickObject, ManipulationResult.LOCATION_GRIPPER)
        result.used_observed_detection = goal.use_observed_detection
        return result
    manager._pick_client = SimulatedActionClient(pick)
    manager._store_client = SimulatedActionClient(lambda goal: outcome(StoreObject, ManipulationResult.LOCATION_CARGO))
    manager._retrieve_client = SimulatedActionClient(lambda goal: outcome(RetrieveObject, ManipulationResult.LOCATION_GRIPPER))

    def place(action_type, goal):
        events.append((action_type.__name__, manager._world_state.snapshot()[1]))
        return outcome(action_type, ManipulationResult.LOCATION_DESTINATION)
    for name, action_type in (('_place_table_client', PlaceOnTable),
                              ('_place_container_client', PlaceInContainer),
                              ('_place_shelf_client', PlaceOnShelf), ('_stack_client', StackObject)):
        setattr(manager, name, SimulatedActionClient(lambda goal, t=action_type: place(t, goal)))
    manager._align_for_shelf_placement = lambda _area: None
    manager._restore_shelf_observation_distance = lambda _area: None
    return manager, navs, events, held_at_departure


def test_full_cargo_transits_empty_visit_without_observation_or_extra_transfer():
    plan = Plan('transit', (
        Visit('collect', 'ws_4', tuple(Step(f'p{i}', 'pick', tag_id=i) for i in (1, 2, 3))),
        Visit('transit', 'ws_5', ()),
        Visit('deliver', 'ws_6', tuple(Step(f'd{i}', 'place_on_table', tag_id=i) for i in (1, 2, 3))),
    ), finish=True)
    manager, navs, events, departures = simulated_manager(plan)
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _feedback: None), plan)
    assert navs == ['ws_4', 'ws_5', 'ws_6', 'finish']
    assert ('observe', 'ws_5') not in events
    transit_load = [(gripper, slots) for target, gripper, slots in departures
                    if target in {'ws_5', 'ws_6'}]
    assert transit_load == [(3, {'left': 1, 'right': 2})] * 2
    assert len(manager._store_client.goals) == 2
    assert len(manager._retrieve_client.goals) == 2
    assert manager._completed_steps == plan.total_steps == 10
    known, gripper, slots = manager._world_state.snapshot()
    assert known and gripper == EMPTY and all(tag == EMPTY for tag in slots.values())


def transport_plan():
    return Plan('transport', (
        Visit('a', 'ws_1', tuple(Step(f'p{i}', 'pick', tag_id=i) for i in (1, 2, 3))),
        Visit('b', 'ws_2', (Step('d1', 'place_on_table', tag_id=1),
                           Step('d2', 'place_in_container', tag_id=2, container_color='red'),
                           Step('p4', 'pick', tag_id=4))),
        Visit('c', 'ws_3', (Step('d3', 'place_on_table', tag_id=3),
                           Step('d4', 'place_on_table', tag_id=4))),
    ), finish=True)


def test_transport_through_child_action_futures():
    plan = transport_plan()
    def visible(manager, tags):
        # First view shows only the later-destination object and a distractor.
        if manager._current_location == 'ws_1' and manager._completed_steps == 1:
            return {3, 99}
        return tags
    manager, navs, events, departures = simulated_manager(plan, visible)
    feedback = []
    manager._run_plan(SimpleNamespace(publish_feedback=feedback.append), plan)
    assert navs == ['ws_1', 'ws_2', 'ws_3', 'finish']
    assert next(e for e in events if e[0] == 'pick') == ('pick', 3)
    assert set(tag for action, tag in events if action == 'pick') == {1, 2, 3, 4}
    _, gripper, slots = next(d for d in departures if d[0] == 'ws_2')
    assert gripper in {1, 2} and 3 in slots.values()
    assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})
    assert manager._completed_steps == plan.total_steps
    assert {out.tag_id for out in manager._delivery_outcomes} == {1, 2, 3, 4}
    assert all(f.total_steps == plan.total_steps for f in feedback)
    assert all(0 <= f.current_step_index < f.total_steps for f in feedback)
    assert any(f.operation == 'store' and f.step_id != 'auto_store' for f in feedback)
    assert len(manager._store_client.goals) >= 2
    assert len(manager._retrieve_client.goals) >= 2


def test_missing_first_pick_yields_to_other_identified_task():
    plan = Plan('missing', (Visit('a', 'ws_1', (
        Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2),
        Step('d1', 'place_on_table', tag_id=1), Step('d2', 'place_on_table', tag_id=2))),))
    manager, _navs, events, _departures = simulated_manager(plan)
    original = manager._pick_client.responder
    attempts = []
    def pick(goal):
        attempts.append(goal.tag_id)
        if attempts == [1]:
            result = outcome(PickObject, ManipulationResult.LOCATION_SOURCE)
            result.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
            result.observed_detections = [detection(2)]
            return result
        return original(goal)
    manager._pick_client.responder = pick
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert attempts[:2] == [1, 2]
    assert manager._completed_steps == plan.total_steps


def test_uncertain_effect_does_not_complete_task():
    plan = Plan('uncertain', (Visit('a', 'ws_1', (
        Step('p1', 'pick', tag_id=1), Step('d1', 'place_on_table', tag_id=1))),))
    manager, *_ = simulated_manager(plan)
    def ambiguous(_goal):
        result = outcome(PickObject, ManipulationResult.LOCATION_GRIPPER)
        result.outcome.effect_known = False
        return result
    manager._pick_client.responder = ambiguous
    with pytest.raises(StepFailed, match='não confirmado'):
        manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert manager._completed_steps == 1
    assert not manager._world_state.snapshot()[0]
    assert manager._failed_step_id == 'p1'


def test_failed_transfer_identifies_pending_task():
    plan = transport_plan()
    manager, *_ = simulated_manager(plan)
    manager._store_client.wait_for_server = lambda **_kwargs: False
    with pytest.raises(StepFailed, match='Servidor indisponível'):
        manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert manager._failed_step_id in {'p1', 'p2', 'p3'}


def test_cancel_is_checked_during_route_planning():
    manager, *_ = simulated_manager(transport_plan())
    manager._cancel_event.set()
    with pytest.raises(MissionCanceled):
        manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), transport_plan())


@pytest.mark.parametrize('first_pick', [4, 5])
def test_stack_group_through_existing_stack_action(first_pick):
    plan = Plan('pile', (
        Visit('a', 'ws_1', (Step('p4', 'pick', tag_id=4), Step('p5', 'pick', tag_id=5))),
        Visit('b', 'ws_2', (Step('pile', 'stack', support_tag_id=14, tag_ids=(4, 5)),)),
    ))
    def visible(manager, tags):
        return {first_pick} if manager._completed_steps == 1 else tags
    manager, *_ = simulated_manager(plan, visible)
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    supports = [goal.support_tag_id for goal in manager._stack_client.goals]
    assert supports == [14, 9 - first_pick]
    assert manager._completed_steps == plan.total_steps
    assert manager._delivery_outcomes[-1].support_tag_id == 9 - first_pick


def test_shelf_delivery_retrieves_declared_object():
    plan = Plan('shelf', (
        Visit('a', 'ws_1', (Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2))),
        Visit('b', 'sh_1', (Step('d1', 'place_on_shelf', tag_id=1),
                           Step('d2', 'place_on_shelf', tag_id=2))),
    ))
    manager, _navs, events, _departures = simulated_manager(plan)
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert {tag for action, tag in events if action == 'PlaceOnShelf'} == {1, 2}
    assert len(manager._retrieve_client.goals) == 1


def test_container_fallback_records_actual_table_destination():
    plan = Plan('fallback', (
        Visit('a', 'ws_1', (Step('p', 'pick', tag_id=1),)),
        Visit('b', 'ws_2', (Step('d', 'place_in_container', tag_id=1, container_color='red'),)),
    ))
    manager, *_ = simulated_manager(plan)
    def missing(_goal):
        result = outcome(PlaceInContainer, ManipulationResult.LOCATION_SOURCE)
        result.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
        result.outcome.message = 'container missing'
        return result
    manager._place_container_client.responder = missing
    manager._move_to_next_place_search_position = lambda *_args: False
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    delivery = manager._delivery_outcomes[0]
    assert delivery.requested_action == 'place_in_container'
    assert delivery.actual_action == 'place_on_table_fallback'
    assert delivery.area_id == 'ws_2' and delivery.container_color is None
    assert manager._completed_steps == plan.total_steps
    assert len(manager._place_table_client.goals) == 1


def test_scan_reselects_at_each_viewpoint_and_stops_when_exhausted():
    plan = Plan('scan', (Visit('a', 'ws_1', (
        Step('p1', 'pick', tag_id=1), Step('p2', 'pick', tag_id=2),
        Step('d1', 'place_on_table', tag_id=1), Step('d2', 'place_on_table', tag_id=2))),))
    def visible(manager, tags):
        if manager._current_lateral_position_mm == 250:
            return {2} & tags
        if manager._current_lateral_position_mm == -250:
            return {1} & tags
        return {99}
    manager, _navs, events, _departures = simulated_manager(plan, visible)
    moves = []
    def move_next(_tag):
        if len(moves) == 2:
            return False
        destination = (250, -250)[len(moves)]
        moves.append(destination)
        manager._current_lateral_position_mm = destination
        return True
    manager._move_to_next_search_position = move_next
    # Memory positioning remains physical but simulated base movement is immediate.
    def move(wall, lateral, _description):
        manager._current_wall_distance_mm = wall
        manager._current_lateral_position_mm = lateral
        return True
    manager._move_to_table_position = move
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert [tag for action, tag in events if action == 'pick'] == [2, 1]
    assert moves == [250, -250]

    missing_manager, *_ = simulated_manager(plan, lambda *_args: {99})
    calls = []
    missing_manager._move_to_next_search_position = lambda _tag: calls.append(_tag) and False
    with pytest.raises(StepFailed, match='pendentes não encontrados'):
        missing_manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert missing_manager._completed_steps == 1
    assert not missing_manager._pick_client.goals


def test_only_immediate_pick_uses_scene_and_later_picks_align_and_analyze():
    plan = transport_plan()
    manager, *_ = simulated_manager(plan)
    moves = []
    original_analyze = manager._vision_client.responder
    def analyze(goal):
        result = original_analyze(goal)
        # An off-center object is inside the tabletop reach interval. Its
        # precomputed centering target must not be visited before trying pick.
        for tag in result.best_apriltags_base:
            tag.pose.position.x = 0.12
        return result
    manager._vision_client.responder = analyze
    original_move = manager._move_to_table_position
    def move(*args):
        moves.append(args)
        return original_move(*args)
    manager._move_to_table_position = move
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert len(manager._vision_client.goals) == 2  # once at ws1, once at ws2
    assert [goal.use_observed_detection for goal in manager._pick_client.goals] == [True, False, False, True]
    assert manager._pick_client.goals[0].observed_detection.pose.position.x == 0.12
    assert len(moves) == 2
    assert all(move[:2] == (200, -120.0) for move in moves)


def test_known_remote_tag_goes_to_configured_alignment_and_requests_fresh_analysis():
    plan = Plan('remote', (Visit('a', 'ws_1', (
        Step('p', 'pick', tag_id=1), Step('d', 'place_on_table', tag_id=1))),))
    manager, *_ = simulated_manager(plan)
    from mission_manager.models import TagObservation
    known = detection(1)
    known.pose.position.x = 0.12
    manager._tag_observations[('ws_1', 1)] = TagObservation(
        'ws_1', 200.0, 250.0, 180, 130.0, known)
    moves = []
    original_move = manager._move_to_table_position
    def move(*args):
        moves.append(args[:2])
        return original_move(*args)
    manager._move_to_table_position = move
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert moves == [(180, 130.0)]
    assert not manager._vision_client.goals  # pick performs the new analysis itself
    assert not manager._pick_client.goals[0].use_observed_detection


def test_recovery_after_cached_out_of_reach_disables_old_pose():
    plan = Plan('recover', (Visit('a', 'ws_1', (
        Step('p', 'pick', tag_id=1), Step('d', 'place_on_table', tag_id=1))),))
    manager, *_ = simulated_manager(plan)
    original_pick = manager._pick_client.responder
    calls = []
    def pick(goal):
        calls.append(goal.use_observed_detection)
        if len(calls) == 1:
            result = outcome(PickObject, ManipulationResult.LOCATION_SOURCE)
            result.used_observed_detection = True
            result.outcome.code = ManipulationResult.MOTION_FAILED
            result.recovery_reason = PickObject.Result.RECOVERY_OUT_OF_REACH
            result.has_detected_pose = True
            result.detected_pose.pose.position.y = -0.10
            return result
        return original_pick(goal)
    manager._pick_client.responder = pick
    def recover(*_args, **_kwargs):
        manager._current_lateral_position_mm = 100.0
        manager._last_table_observation = None
    manager._recover_pick = recover
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert calls == [True, False]


def test_same_viewpoint_after_another_task_does_not_authorize_direct_pick():
    plan = Plan('single_use', (Visit('a', 'ws_1', (
        Step('p1', 'pick', tag_id=1), Step('d1', 'place_on_table', tag_id=1),
        Step('p2', 'pick', tag_id=2), Step('d2', 'place_on_table', tag_id=2))),))
    manager, *_ = simulated_manager(plan)
    manager._run_plan(SimpleNamespace(publish_feedback=lambda _f: None), plan)
    assert [goal.use_observed_detection for goal in manager._pick_client.goals] == [True, False]
    assert len(manager._vision_client.goals) == 1
    assert manager._current_lateral_position_mm == 0


def test_return_to_last_analysis_point_does_not_renew_direct_permission():
    from interfaces.action import FollowWall
    plan = Plan('return', (Visit('a', 'ws_1', (
        Step('p1', 'pick', tag_id=1), Step('d1', 'place_on_table', tag_id=1))),))
    manager, *_ = simulated_manager(plan)
    manager._current_location = 'ws_1'
    manager._observe_visit()
    result = FollowWall.Result()
    result.final_average_distance_mm = 200.0
    result.traveled_distance_mm = 100.0
    manager._update_table_position(result)
    result.traveled_distance_mm = -100.0
    manager._update_table_position(result)
    assert manager._current_scene_observed()  # history still avoids redundant search scans
    assert manager._direct_pick_observation is None
    manager._execute_pick(Step('p1', 'pick', tag_id=1), 1.0)
    assert not manager._pick_client.goals[0].use_observed_detection


@pytest.mark.parametrize('action', ['store', 'retrieve', 'place_on_table', 'navigate', 'finish'])
def test_intervening_operation_expires_immediate_scene_permission(action):
    plan = transport_plan()
    manager, *_ = simulated_manager(plan)
    manager._current_location = 'ws_1'
    manager._observe_visit()
    assert manager._direct_pick_observation is not None
    # Dispatch interception isolates lifetime of the permission from physical
    # preconditions of an unrelated operation (for example a loaded gripper).
    manager._execute_manipulation = lambda _step: None
    manager._execute_step(Step('intervening', action, target='ws_2'))
    assert manager._direct_pick_observation is None
