import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from interfaces.action import PickObject
from manipulation.errors import PickRecoveryRequired
from manipulation.node import ManipulationServer
from manipulation.profiles import load_profiles
import pytest


def _server(position):
    server = ManipulationServer.__new__(ManipulationServer)
    package = Path(__file__).parents[1]
    server._profiles = load_profiles(
        package / 'config/profiles.yaml', package / 'config/cargo_slots.yaml')
    events = []
    server._feedback = lambda *_args: None
    server._gripper = lambda state, *_args: events.append(('gripper', state))
    server._arm_state = lambda state, *_args: events.append(('state', state))
    server._transfer_state = lambda *_args: events.append(('transfer',))
    server._record_effect = lambda *_args: None
    server._analyze_for_operation = lambda *_args, **_kwargs: ([], [], None)
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._run = lambda _action, _handle, _name, operation, **_kwargs: operation()
    server._motion = SimpleNamespace(
        pose_da_april_tag=lambda *_args: (*position, 0.0),
        executar_objetivo=lambda group, constraints, *_args, **kwargs:
            events.append(('motion', constraints[0], kwargs)),
        mover_para_posicoes_das_juntas=lambda _group, joints, _description, **_kwargs:
            events.append(('state', 'pick_shelf_front_ready', joints)),
    )
    goal = PickObject.Goal()
    goal.tag_id = 1
    goal.profile = 'shelf_front'
    goal.alignment_tag_y_m = -0.22
    goal.alignment_tolerance_x_m = 0.01
    goal.alignment_tolerance_y_m = 0.005
    return server, goal, events


def test_front_pick_requests_alignment_attempt_before_target_motion():
    position = (0.0, -0.22, 0.14)
    server, goal, events = _server(position)
    with pytest.raises(PickRecoveryRequired) as captured:
        server._execute_pick(SimpleNamespace(request=goal))
    assert captured.value.recovery_reason == PickObject.Result.RECOVERY_ALIGNMENT_REQUIRED
    assert not any(event[0] == 'motion' for event in events)
    assert ('gripper', 'grip') not in events


@pytest.mark.parametrize('position', [(0.0, -0.22, 0.14), (0.01, -0.28, 0.14), (0.08, -0.35, 0.14)])
@pytest.mark.parametrize('roll_deg', [0.0, 90.0])
def test_front_pick_respects_roll_y_offset_and_returns_in_reverse_order(position, roll_deg):
    server, goal, events = _server(position)
    profile = replace(server._profiles.pickup['shelf_front'],
                      link4_to_link5_deg=roll_deg, grasp_y_offset_m=0.015)
    server._profiles = replace(server._profiles, pickup={
        **server._profiles.pickup, 'shelf_front': profile})
    goal.alignment_completed = True
    server._execute_pick(SimpleNamespace(request=goal))
    motions = [event for event in events if event[0] == 'motion']
    assert len(motions) == 1
    poses = [event[1].position_constraints[0].constraint_region.primitive_poses[0]
             for event in motions]
    assert [pose.position.y for pose in poses] == pytest.approx([
        position[1] + 0.015])
    assert [pose.position.z for pose in poses] == pytest.approx([0.119])
    ready_events = [event for event in events if event[:2] == ('state', 'pick_shelf_front_ready')]
    assert events.index(ready_events[0]) < events.index(motions[0])
    for event in ready_events:
        assert event[2]['link4_to_link5'] == pytest.approx(math.radians(roll_deg))
    states = [event[1] for event in events if event[0] == 'state']
    assert states == ['detect_apriltags', 'home', 'pick_shelf_front_ready',
                      'pick_shelf_front_ready', 'home']
    assert events.index(motions[0]) < events.index(('gripper', 'grip'))
    for motion in motions:
        joint = motion[1].joint_constraints[0]
        assert len(motion[1].joint_constraints) == 2
        assert joint.joint_name == 'link3_to_link4'
        assert joint.position == pytest.approx(math.pi / 2.0)
        assert joint.tolerance_above == pytest.approx(math.radians(5.0))
        assert joint.tolerance_below == pytest.approx(math.radians(5.0))
        roll = motion[1].joint_constraints[1]
        assert roll.joint_name == 'link4_to_link5'
        assert roll.position == pytest.approx(math.radians(roll_deg))
        assert roll.tolerance_above == pytest.approx(math.radians(5.0))
        assert roll.tolerance_below == pytest.approx(math.radians(5.0))
        assert not motion[1].orientation_constraints
        assert not motion[2]
    closed_at = events.index(('gripper', 'grip'))
    assert events[closed_at + 1:] == [
        ready_events[1], ('state', 'home')]


def test_shelf_default_pose_is_known_to_moveit():
    server, _goal, _events = _server((0.0, -0.22, 0.14))
    server._validate_named_states(server._profiles)
