from types import SimpleNamespace

from interfaces.action import MoveToDistance
from nav2_msgs.action import NavigateToPose

from mission_manager.node import MissionManager
from mission_manager.models import (
    AlignmentConfig,
    Arena,
    DepartureConfig,
    MapPose,
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
    manager._call_action = lambda client, goal, *_args: calls.append((client, goal))

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
