from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from interfaces.action import FollowWall, PickObject, PrepareManipulator
from interfaces.msg import AprilTagStampedDetection, ManipulationResult
from mission_manager.errors import StepFailed
from mission_manager.models import (
    AlignmentConfig,
    Arena,
    DepartureConfig,
    MapPose,
    PickupRecoveryConfig,
    ServiceArea,
    Step,
    TableObservation,
    TagObservation,
)
from mission_manager.node import MissionManager
from nav2_msgs.action import NavigateToPose
import pytest


def _arena():
    alignment = AlignmentConfig(200, 10, 10.0)
    departure = DepartureConfig(250, 10, 10.0)
    return Arena(
        frame_id='map',
        start=MapPose(0.0, 0.0, 0.0),
        finish=MapPose(3.0, 0.0, 3.14),
        alignment_defaults=alignment,
        departure_defaults=departure,
        pickup_recovery=PickupRecoveryConfig(
            enabled=True,
            minimum_wall_distance_mm=30,
            maximum_wall_distance_mm=250,
            preferred_tag_x_m=0.0,
            preferred_tag_y_m=-0.22,
            wall_tolerance_mm=5,
            travel_tolerance_mm=10,
            timeout_s=15.0,
            max_reposition_attempts=1,
            search_positions_mm=(0, 250, -250),
        ),
        service_areas={
            'ws_1': ServiceArea(
                area_id='ws_1',
                pose=MapPose(1.0, 2.0, 1.57),
                height_cm=12.5,
                area_type='WS',
                alignment=alignment,
                departure=departure,
            ),
        },
    )


def test_navigation_result_validator_preserves_nav2_error_message():
    result = NavigateToPose.Result()
    result.error_code = 7
    result.error_msg = 'sem caminho'

    assert MissionManager._navigation_failure(result) == 'sem caminho'
    result.error_code = NavigateToPose.Result.NONE
    assert MissionManager._navigation_failure(result) is None


def test_wall_control_requires_valid_distance_and_odometry():
    result = FollowWall.Result()
    result.has_valid_reading = False
    result.has_valid_odometry = True
    result.message = 'sensor indisponível'

    assert MissionManager._wall_control_failure(result) == 'sensor indisponível'
    result.has_valid_reading = True
    assert MissionManager._wall_control_failure(result) is None


def test_wall_control_uses_zero_travel_for_alignment():
    manager = MissionManager.__new__(MissionManager)
    manager._wall_control_client = object()
    manager._duration = lambda seconds: seconds
    calls = []

    def call_action(client, goal, *args):
        calls.append((client, goal, args))
        return FollowWall.Result()

    manager._call_action = call_action
    manager._control_wall(50, 5, 10.0, 'alinhamento')

    goal = calls[0][1]
    assert goal.wall_distance_mm == 50
    assert goal.travel_distance_mm == 0
    assert goal.wall_tolerance_mm == 5
    assert goal.travel_tolerance_mm == 5


def test_manipulation_validator_uses_semantic_outcome():
    success = SimpleNamespace(outcome=SimpleNamespace(
        SUCCESS=0, code=0, message='ok'
    ))
    failure = SimpleNamespace(outcome=SimpleNamespace(
        SUCCESS=0, code=7, message='garra vazia'
    ))

    assert MissionManager._manipulation_failure(success) is None
    assert MissionManager._manipulation_failure(failure) == 'garra vazia'


def test_aborted_action_cannot_be_mistaken_for_semantic_success():
    empty_result = PickObject.Result()

    with pytest.raises(StepFailed, match='estado'):
        MissionManager._validate_action_status(
            GoalStatus.STATUS_ABORTED,
            empty_result,
            'pick',
            allow_unsuccessful_status=True,
        )

    empty_result.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
    MissionManager._validate_action_status(
        GoalStatus.STATUS_ABORTED,
        empty_result,
        'pick',
        allow_unsuccessful_status=True,
    )


def test_pickup_recovery_moves_away_for_near_tag_and_centers_laterally():
    config = _arena().pickup_recovery

    wall, travel = MissionManager._pickup_recovery_correction(
        120.0, 0.08, -0.10, config
    )

    assert wall == 240
    assert travel == -80


def test_pickup_recovery_moves_closer_for_far_tag_respecting_minimum():
    config = _arena().pickup_recovery

    wall, travel = MissionManager._pickup_recovery_correction(
        120.0, 0.0, -0.35, config
    )

    assert wall == 30
    assert travel == 0


def test_pick_retries_after_one_recoverable_result():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._pick_client = object()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._tag_observations = {}
    manager._visited_search_positions = {}
    calls = []
    recoveries = []

    failure = PickObject.Result()
    failure.outcome.code = failure.outcome.MOTION_FAILED
    failure.outcome.message = 'fora do alcance'
    failure.has_detected_pose = True
    failure.recovery_reason = failure.RECOVERY_OUT_OF_REACH
    success = PickObject.Result()
    success.outcome.code = success.outcome.SUCCESS

    results = iter((failure, success))

    def call_action(client, goal, *_args, **kwargs):
        calls.append((client, goal, kwargs))
        return next(results)

    manager._call_action = call_action
    manager._recover_pick = lambda result, step: recoveries.append((result, step))
    step = Step('pick_cube', 'pick', tag_id=1)

    manager._execute_pick(step, 120.0)

    assert len(calls) == 2
    assert all(call[2]['allow_unsuccessful_status'] for call in calls)
    assert len(recoveries) == 1
    assert recoveries[0][1] == step


def _detection(tag_id, x, y):
    detection = AprilTagStampedDetection()
    detection.header.frame_id = 'arm_base_link'
    detection.id = tag_id
    detection.pose.position.x = x
    detection.pose.position.y = y
    detection.pose.position.z = 0.10
    return detection


def test_pick_observations_are_updated_individually_and_survive_area_changes():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._tag_observations = {}
    manager._visited_search_positions = {}

    first = PickObject.Result()
    first.observed_detections = [
        _detection(1, 0.08, -0.20),
        _detection(3, -0.12, -0.18),
    ]
    manager._remember_pick_observations(first)

    manager._current_lateral_position_mm = 100.0
    second = PickObject.Result()
    second.observed_detections = [_detection(1, 0.0, -0.22)]
    manager._remember_pick_observations(second)

    assert manager._tag_observations[('ws_1', 1)].lateral_position_mm == 100.0
    assert manager._tag_observations[('ws_1', 3)].lateral_position_mm == 0.0
    manager._current_location = 'another_area'
    assert ('ws_1', 3) in manager._tag_observations
    manager._forget_picked_tag(1)
    assert ('ws_1', 1) not in manager._tag_observations
    assert ('ws_1', 3) in manager._tag_observations


def test_search_selects_nearest_unvisited_absolute_position():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_lateral_position_mm = 100.0
    manager._visited_search_positions = {'ws_1': {0}}
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    moves = []
    manager._move_to_table_position = lambda wall, lateral, description: moves.append(
        (wall, lateral, description)
    ) or True

    assert manager._move_to_next_search_position(3)
    assert moves[0][1] == 250


def test_unknown_pick_skips_detection_at_last_observed_adjusted_position():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 136.0
    manager._current_lateral_position_mm = 49.0
    manager._tag_observations = {}
    manager._visited_search_positions = {'ws_1': {0}}
    manager._last_table_observation = TableObservation(
        area_id='ws_1',
        wall_distance_mm=136.0,
        lateral_position_mm=49.0,
        detected_tag_ids=frozenset({1}),
    )
    manager._pick_client = object()
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    moves = []
    action_positions = []

    def move(wall, lateral, description):
        moves.append((wall, lateral, description))
        manager._current_wall_distance_mm = float(wall)
        manager._current_lateral_position_mm = float(lateral)
        return True

    def call_action(*_args, **_kwargs):
        action_positions.append(manager._current_lateral_position_mm)
        result = PickObject.Result()
        result.outcome.code = ManipulationResult.SUCCESS
        result.observed_detections = [_detection(2, 0.0, -0.22)]
        return result

    manager._move_to_table_position = move
    manager._call_action = call_action

    manager._execute_pick(Step('pick_2', 'pick', tag_id=2), 120.0)

    assert len(action_positions) == 1
    assert action_positions == [250.0]
    assert moves[0][:2] == (200, 250)


def test_adjusted_wall_distance_does_not_mark_fixed_search_position_visited():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 136.0
    manager._current_lateral_position_mm = 0.0
    manager._tag_observations = {}
    manager._visited_search_positions = {}

    result = PickObject.Result()
    result.outcome.code = ManipulationResult.SUCCESS
    result.observed_detections = [_detection(1, 0.0, -0.22)]

    manager._remember_pick_observations(result)

    assert manager._visited_search_positions['ws_1'] == set()
    assert manager._last_table_observation.detected_tag_ids == frozenset({1})


def test_missing_tag_scans_every_position_once_and_then_fails():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._tag_observations = {}
    manager._visited_search_positions = {}
    manager._pick_client = object()
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    manager._prepare_for_pick_observation = lambda: None
    action_calls = []
    travels = []

    def call_action(*_args, **_kwargs):
        action_calls.append(True)
        result = PickObject.Result()
        result.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
        result.outcome.message = 'não encontrada'
        return result

    def control_wall(distance, *_args, **kwargs):
        travel = kwargs['travel_distance_mm']
        travels.append(travel)
        result = FollowWall.Result()
        result.has_valid_reading = True
        result.has_valid_odometry = True
        result.final_average_distance_mm = float(distance)
        result.traveled_distance_mm = float(travel)
        return result

    manager._call_action = call_action
    manager._control_wall = control_wall

    with pytest.raises(StepFailed, match='não encontrada'):
        manager._execute_pick(Step('pick_9', 'pick', tag_id=9), 120.0)

    assert len(action_calls) == 3
    assert travels == [250, -500]
    assert manager._visited_search_positions['ws_1'] == {0, 250, -250}


def test_cached_pick_falls_back_to_original_observation_before_search():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 180.0
    manager._current_lateral_position_mm = 0.0
    manager._pick_client = object()
    manager._visited_search_positions = {'ws_1': {0}}
    observation = TagObservation(
        area_id='ws_1',
        wall_distance_mm=180.0,
        lateral_position_mm=0.0,
        pickup_wall_distance_mm=220,
        pickup_lateral_position_mm=100.0,
        detection=_detection(3, -0.10, -0.18),
    )
    manager._tag_observations = {('ws_1', 3): observation}
    manager.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    moves = []

    def move(wall, lateral, description):
        moves.append((wall, lateral, description))
        manager._current_wall_distance_mm = float(wall)
        manager._current_lateral_position_mm = float(lateral)
        return True

    missing = PickObject.Result()
    missing.outcome.code = ManipulationResult.OBJECT_NOT_FOUND
    missing.outcome.message = 'não encontrada'
    success = PickObject.Result()
    success.outcome.code = ManipulationResult.SUCCESS
    results = iter((missing, success))
    manager._move_to_table_position = move
    manager._call_action = lambda *_args, **_kwargs: next(results)

    manager._execute_pick(Step('pick_3', 'pick', tag_id=3), 120.0)

    assert len(moves) == 2
    assert moves[0][:2] == (220, 100.0)
    assert moves[1][:2] == (180, 0.0)
    assert 'ponto original' in moves[1][2]


def test_navigation_keeps_apriltag_memory_for_later_return():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 80.0
    marker = object()
    manager._tag_observations = {('ws_1', 3): marker}
    manager._visited_search_positions = {'ws_1': {0}}
    manager._navigate_client = object()
    manager._prepare_for_navigation = lambda: None
    manager._navigation_timeout = lambda: 120.0
    manager._control_wall = lambda *_args, **_kwargs: FollowWall.Result()
    manager._call_action = lambda *_args, **_kwargs: NavigateToPose.Result()
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    manager._navigate('start')

    assert manager._tag_observations == {('ws_1', 3): marker}
    assert manager._visited_search_positions == {'ws_1': {0}}
    assert manager._current_lateral_position_mm == 0.0


def test_pick_recovery_prepares_arm_in_apriltag_observation_pose():
    manager = MissionManager.__new__(MissionManager)
    manager._prepare_client = object()
    manager._manipulation_timeout = lambda: 120.0
    calls = []
    manager._call_action = lambda client, goal, *_args: calls.append(
        (client, goal)
    )

    manager._prepare_for_pick_observation()

    assert calls[0][0] is manager._prepare_client
    assert calls[0][1].mode == PrepareManipulator.Goal.OBSERVATION


def test_executor_maps_sequential_steps_to_semantic_action_goals():
    manager = MissionManager.__new__(MissionManager)
    manager._arena = _arena()
    manager._current_location = 'ws_1'
    manager._current_wall_distance_mm = 200.0
    manager._current_lateral_position_mm = 0.0
    manager._tag_observations = {}
    manager._visited_search_positions = {}
    manager._manipulation_timeout = lambda: 120.0
    manager._pick_client = object()
    manager._store_client = object()
    manager._retrieve_client = object()
    manager._place_table_client = object()
    manager._place_container_client = object()
    manager._stack_client = object()
    manager._place_shelf_client = object()
    calls = []

    def call_action(client, goal, *_args, **_kwargs):
        calls.append((client, goal))
        return SimpleNamespace(
            outcome=SimpleNamespace(SUCCESS=0, code=0, message='ok'),
            observed_detections=[],
        )

    manager._call_action = call_action

    steps = (
        Step('pick', 'pick', tag_id=7),
        Step('store', 'store', slot_id='left'),
        Step('retrieve', 'retrieve', slot_id='right'),
        Step(
            'table', 'place_on_table',
            analyze_apriltags=True,
            analyze_containers=False,
        ),
        Step('container', 'place_in_container', container_color='blue'),
        Step('stack', 'stack', support_tag_id=3),
        Step('shelf', 'place_on_shelf'),
    )
    for step in steps:
        manager._execute_manipulation(step)

    assert calls[0][1].tag_id == 7
    assert calls[0][1].profile == ''
    assert calls[1][1].slot_id == 'left'
    assert calls[2][1].slot_id == 'right'
    assert calls[3][1].ws_height_cm == 12.5
    assert calls[3][1].analyze_apriltags is True
    assert calls[3][1].analyze_containers is False
    assert calls[4][1].ws_height_cm == 12.5
    assert calls[4][1].container_color == calls[4][1].BLUE
    assert calls[5][1].support_tag_id == 3
    assert not hasattr(calls[5][1], 'ws_height_cm')
