from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from manipulation.errors import (
    ConfigurationError,
    FeatureUnavailable,
    PerceptionUnavailable,
)
from manipulation.node import ManipulationServer
from manipulation.profiles import PlacementProfile
from manipulation.state import EMPTY, ManipulationInventory
from interfaces.action import PlaceInContainer, PlaceOnTable


def _pose():
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.z = 0.10
    pose.pose.orientation.w = 1.0
    return pose


def _cartesian_profile():
    return PlacementProfile(
        name='explicit_pose',
        strategy='cartesian',
        enabled=True,
        named_state='',
        approach_height_m=0.08,
        retreat_height_m=0.10,
        reference_offset_xyz=(0.0, 0.0, 0.0),
        yaw_offset_deg=0.0,
        calibrated_reference=False,
    )


def test_common_release_commits_inventory_only_after_opening_gripper():
    server = ManipulationServer.__new__(ManipulationServer)
    server._inventory = ManipulationInventory(['left'])
    server._inventory.commit_pick(5)
    motions = []
    gripper = []
    server._motion = SimpleNamespace(
        executar_objetivo=lambda *args: motions.append(args)
    )
    server._feedback = lambda *args: None
    server._gripper = lambda state, description: gripper.append(state)
    server._publish_state = lambda: None
    server._safe = lambda: None

    message, location, placed_pose = server._release_at_pose(
        object(), PlaceOnTable, 5, _pose(), _cartesian_profile(), 'teste'
    )

    known, held, slots = server._inventory.snapshot()
    assert known is True
    assert held == EMPTY
    assert slots == {'left': EMPTY}
    assert gripper == ['open']
    assert len(motions) == 3
    assert placed_pose.pose.position.z == pytest.approx(0.10)
    assert 'depositado' in message
    assert location > 0


def test_gripper_failure_marks_inventory_unknown():
    server = ManipulationServer.__new__(ManipulationServer)
    server._inventory = ManipulationInventory(['left'])
    server._inventory.commit_pick(5)
    server._motion = SimpleNamespace(executar_objetivo=lambda *args: None)
    server._feedback = lambda *args: None
    server._gripper = lambda *args: (_ for _ in ()).throw(RuntimeError('falha'))
    server._publish_state = lambda: None

    with pytest.raises(RuntimeError, match='falha'):
        server._release_at_pose(
            object(), PlaceOnTable, 5, _pose(), _cartesian_profile(), 'teste'
        )

    assert server._inventory.snapshot()[0] is False


def _operation_only_server(tag_id=5):
    server = ManipulationServer.__new__(ManipulationServer)
    server._inventory = ManipulationInventory(['left'])
    server._inventory.commit_pick(tag_id)
    server._feedback = lambda *args: None
    server._profiles = SimpleNamespace(
        placements={
            'table': PlacementProfile(
                name='table',
                strategy='perception',
                enabled=True,
                named_state='',
                approach_height_m=0.08,
                retreat_height_m=0.08,
                reference_offset_xyz=(0.0, 0.0, 0.0),
                yaw_offset_deg=0.0,
                calibrated_reference=False,
            )
        }
    )

    def run(_action, _handle, _name, _tag_id, operation, **_kwargs):
        return operation()

    server._run = run
    return server


def test_table_container_analysis_fails_before_motion_while_detector_is_absent():
    server = _operation_only_server()
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 12.5
    goal.analyze_containers = True

    with pytest.raises(PerceptionUnavailable, match='contêineres'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_without_detectors_requires_nominal_pose_calibration():
    server = _operation_only_server()
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = -200.25

    with pytest.raises(FeatureUnavailable, match='release_x_m'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_without_detectors_uses_fixed_xy_and_height_plus_tcp_offset():
    server = _operation_only_server()
    server._profiles.placements['table'] = PlacementProfile(
        name='table',
        strategy='perception',
        enabled=True,
        named_state='',
        approach_height_m=0.08,
        retreat_height_m=0.08,
        reference_offset_xyz=(0.0, 0.0, 0.0),
        yaw_offset_deg=0.0,
        calibrated_reference=False,
        release_x_m=0.21,
        release_y_m=-0.04,
        release_yaw_deg=15.0,
        tcp_release_offset_cm=3.5,
    )
    captured = {}

    def release(_handle, _action, object_id, pose, _profile, _destination):
        captured['object_id'] = object_id
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 12.5

    server._execute_place_on_table(SimpleNamespace(request=goal))

    pose = captured['pose']
    assert captured['object_id'] == 5
    assert pose.pose.position.x == pytest.approx(0.21)
    assert pose.pose.position.y == pytest.approx(-0.04)
    assert pose.pose.position.z == pytest.approx(0.16)


def test_container_rejects_color_outside_enum_before_detection():
    server = _operation_only_server()
    goal = PlaceInContainer.Goal()
    goal.ws_height_cm = 10.0
    goal.container_color = 99

    with pytest.raises(ConfigurationError, match='Cor de contêiner inválida'):
        server._execute_place_in_container(SimpleNamespace(request=goal))
