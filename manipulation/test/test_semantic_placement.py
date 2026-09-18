import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from interfaces.action import PlaceInContainer, PlaceOnTable
from interfaces.msg import (
    AprilTagStampedDetection, ContainerStampedDetection, ManipulationResult,
)

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
        'usable_x_min_m': -0.40,
        'usable_x_max_m': 0.40,
        'usable_y_min_m': -0.45,
        'usable_y_max_m': 0.05,
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
    transfer_states = []
    server._transfer_state = lambda description: transfer_states.append(description)

    message, location, placed_pose = server._release_at_pose(
        object(), PlaceOnTable, 5, _pose(), _cartesian_profile(), 'teste'
    )

    assert server._effect_known is True
    assert server._effect_location == ManipulationResult.LOCATION_DESTINATION
    assert gripper == ['open']
    assert len(motions) == 3
    assert transfer_states == ['Preparando detect_apriltags após o depósito']
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
    server._table_height_in_arm_frame = lambda height: height
    return server


def test_floor_to_arm_height_uses_real_mount_and_allows_below_arm_plane():
    transform = TransformStamped()
    transform.transform.translation.z = -0.112
    transform.transform.rotation.w = 1.0
    assert ManipulationServer._height_via_transform(0.05, transform) == pytest.approx(-0.062)
    assert ManipulationServer._height_via_transform(0.10, transform) == pytest.approx(-0.012)
    assert ManipulationServer._height_via_transform(0.15, transform) == pytest.approx(0.038)
    transform.transform.rotation.x = 0.1
    with pytest.raises(PerceptionUnavailable, match='não são horizontais'):
        ManipulationServer._height_via_transform(0.10, transform)


def test_table_requires_calibrated_search_before_any_motion():
    server = _operation_only_server()
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = 12.5
    with pytest.raises(FeatureUnavailable, match='release_x_m'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_rejects_negative_height():
    server = _operation_only_server()
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = -200.25

    with pytest.raises(ConfigurationError, match='não negativa'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_uses_unified_scene_and_height_plus_tcp_offset():
    server = _operation_only_server()
    server._profiles.placements['table'] = _search_profile(
        release_x_m=0.02,
        release_y_m=-0.20,
        release_yaw_deg=15.0,
        tcp_release_offset_cm=3.5,
    )
    server._profiles.pickup_profile = lambda _name: SimpleNamespace(
        observation_state='detect_apriltags', cube_size_m=0.042)
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(
        analisar_cena=lambda *_args, **_kwargs: ([], []),
        ultimas_apriltags_rejeitadas=[], ultimos_containers_rejeitados=[])
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
    assert pose.pose.position.x == pytest.approx(0.02)
    assert pose.pose.position.y == pytest.approx(-0.20)
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
            observation_state='detect_apriltags', cube_size_m=0.042
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(
        analisar_cena=lambda *_args, **_kwargs: ([_detection(5, 0.0, -0.20)], []),
        ultimas_apriltags_rejeitadas=[], ultimos_containers_rejeitados=[],
    )
    captured = {}

    def release(_handle, _action, _object_id, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.object_tag_id = 5
    goal.ws_height_cm = 10.0

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


def test_rotated_rectangle_sat_distinguishes_corner_clearance():
    obstacle = ManipulationServer._rectangle(0.0, 0.0, 0.18, 0.10, math.pi/4)
    at_corner = ManipulationServer._rectangle(0.09, 0.0, 0.04, 0.04)
    away = ManipulationServer._rectangle(0.16, 0.0, 0.04, 0.04)
    assert ManipulationServer._polygons_overlap(at_corner, obstacle)
    assert not ManipulationServer._polygons_overlap(away, obstacle)


def test_container_selection_uses_color_recency_confidence_and_z_formula():
    item = ContainerStampedDetection()
    item.color = item.BLUE
    item.pose_valid = True
    item.confidence = 0.8
    item.header.stamp.sec = 9
    item.external_dimensions_m.z = 0.073
    selected = ManipulationServer._select_container_detection(
        [item], item.BLUE, 0.45, 2.0, 10_000_000_000)
    assert selected is item
    assert ManipulationServer._container_tcp_z(0.10, item, 0.0) == pytest.approx(0.173)
    with pytest.raises(PerceptionUnavailable, match='encontrados 0'):
        ManipulationServer._select_container_detection(
            [item], item.RED, 0.45, 2.0, 10_000_000_000)
    with pytest.raises(PerceptionUnavailable, match='encontrados 2'):
        ManipulationServer._select_container_detection(
            [item, item], item.BLUE, 0.45, 2.0, 10_000_000_000)
