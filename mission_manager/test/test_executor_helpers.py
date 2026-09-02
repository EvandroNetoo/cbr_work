from types import SimpleNamespace

from interfaces.action import MoveToDistance, PickObject, PrepareManipulator
from nav2_msgs.action import NavigateToPose

from mission_manager.node import MissionManager
from mission_manager.models import (
    AlignmentConfig,
    Arena,
    DepartureConfig,
    MapPose,
    PickupRecoveryConfig,
    ServiceArea,
    Step,
)


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


def test_alignment_requires_a_valid_sensor_reading():
    result = MoveToDistance.Result()
    result.has_valid_reading = False
    result.message = 'sensor indisponível'

    assert MissionManager._alignment_failure(result) == 'sensor indisponível'
    result.has_valid_reading = True
    assert MissionManager._alignment_failure(result) is None


def test_manipulation_validator_uses_semantic_outcome():
    success = SimpleNamespace(outcome=SimpleNamespace(
        SUCCESS=0, code=0, message='ok'
    ))
    failure = SimpleNamespace(outcome=SimpleNamespace(
        SUCCESS=0, code=7, message='garra vazia'
    ))

    assert MissionManager._manipulation_failure(success) is None
    assert MissionManager._manipulation_failure(failure) == 'garra vazia'


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
        return SimpleNamespace(outcome=SimpleNamespace(
            SUCCESS=0, code=0, message='ok'
        ))

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
