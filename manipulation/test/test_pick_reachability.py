from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from interfaces.action import PickObject
from manipulation.errors import ObjectOutOfReach
from manipulation.node import ManipulationServer
from manipulation.profiles import load_profiles
from manipulation.state import ManipulationInventory


PACKAGE = Path(__file__).parents[1]


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
    server._inventory = ManipulationInventory([])
    server._feedback = lambda *_args: None
    server._gripper = lambda *_args: None
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    detection_calls = []

    def detect(_tag_id, _duration):
        detection_calls.append(True)
        return 0.0, -0.10, 0.10, 0.0

    server._motion = SimpleNamespace(
        obter_pose_da_april_tag=detect,
        executar_objetivo=lambda *_args: pytest.fail(
            'não deveria planejar movimento para uma tag fora do alcance'
        ),
    )

    def run(_action, _handle, _name, _tag_id, operation):
        return operation()

    server._run = run
    goal = PickObject.Goal()
    goal.tag_id = 1
    goal.profile = 'tabletop'

    with pytest.raises(ObjectOutOfReach, match='fora da área de alcance'):
        server._execute_pick(SimpleNamespace(request=goal))

    assert len(detection_calls) == 1
