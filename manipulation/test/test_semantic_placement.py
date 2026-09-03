import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from interfaces.action import PlaceInContainer, PlaceOnTable
from interfaces.msg import AprilTagStampedDetection, ManipulationResult

from manipulation.errors import (
    ConfigurationError,
    FeatureUnavailable,
    NoFreeSpace,
    PerceptionUnavailable,
)
from manipulation.node import ManipulationServer
from manipulation.profiles import PlacementProfile
import pytest


def _pose():
    pose = PoseStamped()
    pose.header.frame_id = 'arm_base_link'
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


def _search_profile(**overrides):
    values = {
        'name': 'table',
        'strategy': 'perception',
        'enabled': True,
        'named_state': '',
        'approach_height_m': 0.08,
        'retreat_height_m': 0.08,
        'reference_offset_xyz': (0.0, 0.0, 0.0),
        'yaw_offset_deg': 0.0,
        'calibrated_reference': False,
        'release_x_m': 0.0,
        'release_y_m': -0.20,
        'release_yaw_deg': 0.0,
        'tcp_release_offset_cm': -3.0,
        'free_space_min_distance_m': 0.08,
        'free_space_preferred_distance_m': 0.12,
        'reach_center_x_m': 0.0,
        'reach_center_y_m': 0.0,
        'reach_min_radius_m': 0.10,
        'reach_max_radius_m': 0.40,
        'search_x_min_m': -0.10,
        'search_x_max_m': 0.10,
        'search_y_min_m': -0.30,
        'search_y_max_m': -0.10,
        'search_step_m': 0.01,
    }
    values.update(overrides)
    return PlacementProfile(**values)


def _detection(tag_id, x, y):
    detection = AprilTagStampedDetection()
    detection.header.frame_id = 'arm_base_link'
    detection.id = tag_id
    detection.pose.position.x = x
    detection.pose.position.y = y
    return detection


def test_common_release_reports_physical_effect_only_after_opening_gripper():
    server = ManipulationServer.__new__(ManipulationServer)
    server._effect_known = True
    server._effect_location = ManipulationResult.LOCATION_UNKNOWN
    motions = []
    gripper = []
    server._motion = SimpleNamespace(
        executar_objetivo=lambda *args: motions.append(args)
    )
    server._feedback = lambda *args: None
    server._gripper = lambda state, description: gripper.append(state)
    server._safe = lambda _loaded: None

    message, location, placed_pose = server._release_at_pose(
        object(), PlaceOnTable, 5, _pose(), _cartesian_profile(), 'teste'
    )

    assert server._effect_known is True
    assert server._effect_location == ManipulationResult.LOCATION_DESTINATION
    assert gripper == ['open']
    assert len(motions) == 3
    assert placed_pose.pose.position.z == pytest.approx(0.10)
    assert 'depositado' in message
    assert location > 0


def test_gripper_failure_reports_physical_effect_as_unknown():
    server = ManipulationServer.__new__(ManipulationServer)
    server._effect_known = True
    server._effect_location = ManipulationResult.LOCATION_UNKNOWN
    server._motion = SimpleNamespace(executar_objetivo=lambda *args: None)
    server._feedback = lambda *args: None
    server._gripper = lambda *args: (_ for _ in ()).throw(RuntimeError('falha'))

    with pytest.raises(RuntimeError, match='falha'):
        server._release_at_pose(
            object(), PlaceOnTable, 5, _pose(), _cartesian_profile(), 'teste'
        )

    assert server._effect_known is False
    assert server._effect_location == ManipulationResult.LOCATION_LOST


def _operation_only_server(tag_id=5):
    server = ManipulationServer.__new__(ManipulationServer)
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
    goal.object_tag_id = 5
    goal.ws_height_cm = 12.5
    goal.analyze_containers = True

    with pytest.raises(PerceptionUnavailable, match='contêineres'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_without_detectors_requires_nominal_pose_calibration():
    server = _operation_only_server()
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
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
    goal.object_tag_id = 5
    goal.ws_height_cm = 12.5

    server._execute_place_on_table(SimpleNamespace(request=goal))

    pose = captured['pose']
    assert captured['object_id'] == 5
    assert pose.pose.position.x == pytest.approx(0.21)
    assert pose.pose.position.y == pytest.approx(-0.04)
    assert pose.pose.position.z == pytest.approx(0.16)


def test_table_search_starts_at_nominal_position():
    candidates = ManipulationServer._table_search_candidates(_search_profile())

    assert candidates[0] == pytest.approx((0.0, -0.20))
    assert all(y <= -0.10 + 1e-9 for _, y in candidates)


def test_table_search_keeps_only_rectangle_points_inside_reach_annulus():
    profile = _search_profile(
        reach_center_x_m=0.01,
        reach_center_y_m=-0.01,
        reach_min_radius_m=0.16,
        reach_max_radius_m=0.22,
    )

    candidates = ManipulationServer._table_search_candidates(profile)

    radii = [
        math.hypot(x - profile.reach_center_x_m, y - profile.reach_center_y_m)
        for x, y in candidates
    ]
    assert candidates
    assert min(radii) >= profile.reach_min_radius_m - 1e-9
    assert max(radii) <= profile.reach_max_radius_m + 1e-9


def test_table_search_rejects_disjoint_rectangle_and_reach_annulus():
    profile = _search_profile(
        search_x_min_m=0.0,
        search_x_max_m=0.0,
        search_y_min_m=-0.20,
        search_y_max_m=-0.20,
        reach_center_y_m=-0.20,
        reach_min_radius_m=0.10,
        reach_max_radius_m=0.20,
    )

    with pytest.raises(ConfigurationError, match='não contém candidatos'):
        ManipulationServer._table_search_candidates(profile)


def test_table_search_prefers_nearest_candidate_with_comfortable_clearance():
    profile = _search_profile()
    candidates = ManipulationServer._table_search_candidates(profile)

    selected = ManipulationServer._select_free_table_position(
        candidates,
        [(0.0, -0.20)],
        profile.free_space_min_distance_m,
        profile.free_space_preferred_distance_m,
    )

    assert selected != pytest.approx((0.0, -0.20))
    clearance = (selected[0] ** 2 + (selected[1] + 0.20) ** 2) ** 0.5
    assert 0.12 <= clearance < 0.13


def test_table_search_falls_back_to_minimum_clearance_when_needed():
    profile = _search_profile(
        search_x_min_m=0.0,
        search_x_max_m=0.0,
        search_y_min_m=-0.28,
        search_y_max_m=-0.12,
    )
    candidates = ManipulationServer._table_search_candidates(profile)

    selected = ManipulationServer._select_free_table_position(
        candidates,
        [(0.0, -0.20)],
        profile.free_space_min_distance_m,
        profile.free_space_preferred_distance_m,
    )

    clearance = (selected[0] ** 2 + (selected[1] + 0.20) ** 2) ** 0.5
    assert clearance == pytest.approx(0.08)
    assert clearance < profile.free_space_preferred_distance_m


def test_table_search_reports_no_free_space():
    profile = _search_profile(
        search_x_min_m=0.0,
        search_x_max_m=0.0,
        search_y_min_m=-0.20,
        search_y_max_m=-0.20,
    )
    candidates = ManipulationServer._table_search_candidates(profile)

    with pytest.raises(NoFreeSpace, match='Nenhuma posição'):
        ManipulationServer._select_free_table_position(
            candidates,
            [(0.0, -0.20)],
            profile.free_space_min_distance_m,
            profile.free_space_preferred_distance_m,
        )


def test_table_apriltag_analysis_ignores_object_held_by_gripper():
    server = _operation_only_server(tag_id=5)
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile()},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(
        obter_deteccoes_de_april_tags=lambda _duration: [
            _detection(5, 0.0, -0.20)
        ]
    )
    captured = {}

    def release(_handle, _action, _object_id, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = 10.0
    goal.analyze_apriltags = True

    server._execute_place_on_table(SimpleNamespace(request=goal))

    assert captured['pose'].pose.position.x == pytest.approx(0.0)
    assert captured['pose'].pose.position.y == pytest.approx(-0.20)


def test_table_apriltag_analysis_requires_complete_search_bounds_before_motion():
    server = _operation_only_server()
    server._profiles.placements['table'] = _search_profile(search_x_min_m=None)
    server._arm_state = lambda *_args: pytest.fail('não deveria mover o braço')
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = 10.0
    goal.analyze_apriltags = True

    with pytest.raises(FeatureUnavailable, match='search_x_min_m'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_container_rejects_color_outside_enum_before_detection():
    server = _operation_only_server()
    goal = PlaceInContainer.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = 10.0
    goal.container_color = 99

    with pytest.raises(ConfigurationError, match='Cor de contêiner inválida'):
        server._execute_place_in_container(SimpleNamespace(request=goal))
