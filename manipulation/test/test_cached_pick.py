"""A mission-supplied detection skips capture, but never skips reach validation."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from interfaces.action import PickObject
from interfaces.msg import AprilTagStampedDetection, ManipulationResult
from manipulation.errors import ConfigurationError, ObjectOutOfReach
from manipulation.node import ManipulationServer
from manipulation.profiles import load_profiles
import pytest


PACKAGE = Path(__file__).parents[1]


def server_and_goal(position=(0.12, -0.18, 0.10)):
    server = ManipulationServer.__new__(ManipulationServer)
    profiles = load_profiles(PACKAGE / 'config/profiles.yaml', PACKAGE / 'config/cargo_slots.yaml')
    server._profiles = replace(profiles, pickup={
        'tabletop': replace(profiles.pickup['tabletop'], reachability_filter_enabled=True)})
    server._feedback = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    events = []
    server._gripper = lambda name, *_args: events.append(('gripper', name))
    server._arm_state = lambda name, *_args: events.append(('arm', name))
    server._transfer_state = lambda *_args: events.append(('transfer', 'observation'))
    goal = PickObject.Goal()
    goal.tag_id = 7
    goal.profile = 'tabletop'
    goal.use_observed_detection = True
    detection = AprilTagStampedDetection()
    detection.header.frame_id = 'arm_base_link'
    detection.id = goal.tag_id
    detection.pose.position.x, detection.pose.position.y, detection.pose.position.z = position
    detection.pose.orientation.w = 1.0
    goal.observed_detection = detection
    def pose(tags, tag, _duration):
        assert len(tags) == 1 and tags[0].id == tag
        p = tags[0].pose.position
        events.append(('pose', (p.x, p.y, p.z)))
        return p.x, p.y, p.z, 0.0
    def analyze(*_args, **_kwargs):
        events.append(('capture', True))
        return [detection], []
    server._motion = SimpleNamespace(
        pose_da_april_tag=pose, analisar_cena=analyze,
        executar_objetivo=lambda *_args: events.append(('motion', True)))
    def run(_action, _handle, _name, operation, **kwargs):
        operation()
        result = PickObject.Result()
        result.outcome.code = ManipulationResult.SUCCESS
        result.outcome.effect_known = True
        result.outcome.final_object_location = ManipulationResult.LOCATION_GRIPPER
        result.scene_observation = kwargs['scene_observation']
        result.observed_detections = kwargs['observed_detections']
        return result
    server._run = run
    return server, goal, events


@pytest.mark.parametrize('cached', [True, False])
def test_cached_pick_uses_profile_coordinates_without_capture(cached):
    server, goal, events = server_and_goal()
    goal.use_observed_detection = cached
    result = server._execute_pick(SimpleNamespace(request=goal))
    assert result.used_observed_detection is cached
    assert result.scene_observation.completed is (not cached)
    assert ('pose', (0.12, -0.18, 0.10)) in events
    assert sum(event[0] == 'motion' for event in events) == 3
    assert sum(event[0] == 'capture' for event in events) == int(not cached)
    assert sum(event[0] == 'arm' for event in events) == int(not cached)
    assert ('gripper', 'grip') in events


def test_cached_out_of_reach_detection_requests_recovery_without_capture():
    server, goal, events = server_and_goal((0.0, -0.10, 0.10))
    with pytest.raises(ObjectOutOfReach) as error:
        server._execute_pick(SimpleNamespace(request=goal))
    assert error.value.recovery_reason == PickObject.Result.RECOVERY_OUT_OF_REACH
    assert not any(event[0] in {'capture', 'arm', 'motion'} for event in events)
    assert ('gripper', 'grip') not in events


@pytest.mark.parametrize('invalid,match', [
    ('id', 'não corresponde'), ('frame', 'arm_base_link'),
    ('nan', 'não finito'), ('quaternion', 'quaternion nulo'),
])
def test_invalid_cached_detection_is_rejected_before_gripper_or_motion(invalid, match):
    server, goal, events = server_and_goal()
    detection = goal.observed_detection
    if invalid == 'id': detection.id = 99
    if invalid == 'frame': detection.header.frame_id = 'camera_link'
    if invalid == 'nan': detection.pose.position.x = float('nan')
    if invalid == 'quaternion': detection.pose.orientation.w = 0.0
    with pytest.raises(ConfigurationError, match=match):
        server._execute_pick(SimpleNamespace(request=goal))
    assert events == []


def test_standalone_pick_retry_reuses_the_first_capture_at_the_same_base():
    from so_arm_101_moveit_config.movimento import FalhaDoMoveIt
    server, goal, events = server_and_goal()
    goal.use_observed_detection = False
    profile = server._profiles.pickup['tabletop']
    server._profiles = replace(server._profiles, pickup={'tabletop': replace(profile, attempts=2)})
    server.get_logger = lambda: SimpleNamespace(warning=lambda *_args: None)
    def motion(*_args):
        events.append(('motion', True))
        if sum(event[0] == 'motion' for event in events) == 1:
            raise FalhaDoMoveIt('Falha transitória de planejamento', -1)
    server._motion.executar_objetivo = motion
    result = server._execute_pick(SimpleNamespace(request=goal))
    assert not result.used_observed_detection
    assert sum(event[0] == 'capture' for event in events) == 1
    assert sum(event[0] == 'arm' for event in events) == 1
    assert ('gripper', 'grip') in events
