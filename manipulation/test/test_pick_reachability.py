from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from interfaces.action import PickObject
from interfaces.msg import AprilTagStampedDetection, ManipulationResult
from manipulation.errors import ObjectOutOfReach, PickRecoveryRequired
from manipulation.node import ManipulationServer
from manipulation.profiles import load_profiles
import pytest
from so_arm_101_moveit_config.movimento import FalhaDoMoveIt


PACKAGE = Path(__file__).parents[1]


def _detection(tag_id):
    detection = AprilTagStampedDetection()
    detection.header.frame_id = 'arm_base_link'
    detection.id = tag_id
    return detection


def test_pick_rejects_out_of_reach_tag_before_target_motion_or_retry():
    server = ManipulationServer.__new__(ManipulationServer)
    profiles = load_profiles(
        PACKAGE / 'config' / 'profiles.yaml',
        PACKAGE / 'config' / 'cargo_slots.yaml',
    )
    server._profiles = replace(
        profiles,
        pickup={
            'tabletop': replace(
                profiles.pickup['tabletop'], reachability_filter_enabled=True
            )
        },
    )
    server._feedback = lambda *_args: None
    server._gripper = lambda *_args: None
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    detection_calls = []

    def detect(_tag_id, _duration, **kwargs):
        detection_calls.append(True)
        kwargs['deteccoes_observadas'].extend([_detection(1), _detection(3)])
        return 0.0, -0.10, 0.10, 0.0

    server._motion = SimpleNamespace(
        obter_pose_da_april_tag=detect,
        executar_objetivo=lambda *_args: pytest.fail(
            'não deveria planejar movimento para uma tag fora do alcance'
        ),
    )

    observed = []

    def run(_action, _handle, _name, _tag_id, operation, **kwargs):
        observed.append(kwargs['observed_detections'])
        return operation()

    server._run = run
    goal = PickObject.Goal()
    goal.tag_id = 1
    goal.profile = 'tabletop'

    with pytest.raises(
        ObjectOutOfReach, match='fora da área de alcance'
    ) as captured:
        server._execute_pick(SimpleNamespace(request=goal))

    assert len(detection_calls) == 1
    assert [item.id for item in observed[0]] == [1, 3]
    assert (
        captured.value.recovery_reason
        == PickObject.Result.RECOVERY_OUT_OF_REACH
    )
    assert captured.value.detected_pose.pose.position.y == pytest.approx(-0.10)


def test_pick_exposes_detected_pose_when_moveit_returns_99999():
    server = ManipulationServer.__new__(ManipulationServer)
    server._profiles = load_profiles(
        PACKAGE / 'config' / 'profiles.yaml',
        PACKAGE / 'config' / 'cargo_slots.yaml',
    )
    server._feedback = lambda *_args: None
    server._gripper = lambda *_args: None
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    motion_calls = []

    def execute(*_args):
        motion_calls.append(True)
        raise FalhaDoMoveIt('MoveIt falhou com código 99999.', 99999)

    server._motion = SimpleNamespace(
        obter_pose_da_april_tag=lambda *_args, **_kwargs: (
            0.0, -0.20, 0.10, 0.0
        ),
        executar_objetivo=execute,
    )
    server._run = lambda _action, _handle, _name, _tag_id, operation, **_kwargs: operation()
    goal = PickObject.Goal()
    goal.tag_id = 1
    goal.profile = 'tabletop'

    with pytest.raises(PickRecoveryRequired) as captured:
        server._execute_pick(SimpleNamespace(request=goal))

    assert len(motion_calls) == 1
    assert captured.value.moveit_error_code == 99999
    assert (
        captured.value.recovery_reason
        == PickObject.Result.RECOVERY_MOVEIT_UNREACHABLE
    )
    assert captured.value.detected_pose.pose.position.y == pytest.approx(-0.20)


def test_pick_result_returns_all_observed_detections():
    server = ManipulationServer.__new__(ManipulationServer)
    statuses = []
    goal_handle = SimpleNamespace(
        succeed=lambda: statuses.append('succeeded'),
        canceled=lambda: statuses.append('canceled'),
        abort=lambda: statuses.append('aborted'),
    )

    result = server._make_result(
        PickObject,
        goal_handle,
        ManipulationResult.SUCCESS,
        'ok',
        1,
        observed_detections=[_detection(1), _detection(3)],
    )

    assert statuses == ['succeeded']
    assert [item.id for item in result.observed_detections] == [1, 3]
