import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from interfaces.action import PlaceInContainer, PlaceOnTable
from interfaces.msg import (
    AprilTagStampedDetection, ContainerStampedDetection,
    ManipulationFeedback, ManipulationResult, SceneObservation, TableSurfaceGrid,
)

from manipulation.errors import (
    ConfigurationError,
    FeatureUnavailable,
    NoFreeSpace,
    ObjectNotFound,
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
        'tcp_release_offset_cm': -3.0,
        'free_space_half_extent_x_m': 0.07,
        'free_space_half_extent_y_m': 0.04,
        'free_space_min_padding_m': 0.0,
        'free_space_preferred_padding_m': 0.03,
        'free_space_preferred_yaw_deg': 0.0,
        'free_space_alternate_yaw_deg': -90.0,
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


def _table_grid(default=TableSurfaceGrid.FREE):
    grid = TableSurfaceGrid()
    grid.header.frame_id = 'arm_base_link'
    grid.resolution_m = 0.01
    grid.x_min_m = -0.50
    grid.y_min_m = -0.60
    grid.width = 101
    grid.height = 101
    grid.cells = [default] * (grid.width * grid.height)
    return grid


def _set_grid_cell(grid, x, y, value):
    column = round((x - grid.x_min_m) / grid.resolution_m)
    row = round((y - grid.y_min_m) / grid.resolution_m)
    grid.cells[row * grid.width + column] = value


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
        object(), PlaceOnTable, _pose(), _cartesian_profile(), 'teste'
    )

    assert server._effect_known is True
    assert server._effect_location == ManipulationResult.LOCATION_DESTINATION
    assert gripper == ['open']
    assert len(motions) == 3
    assert transfer_states == ['Preparando detect_apriltags após o depósito']
    assert placed_pose.pose.position.z == pytest.approx(0.10)
    assert 'depositado' in message
    assert location > 0


def test_container_release_restricts_position_and_wrist_then_returns_directly():
    server = ManipulationServer.__new__(ManipulationServer)
    server._effect_known = True
    server._effect_location = ManipulationResult.LOCATION_UNKNOWN
    events = []
    feedback = []
    motions = []
    targets = []

    def move(*args):
        events.append('motion')
        motions.append(args)

    def publish_target(target):
        events.append('target')
        targets.append(target)

    server._motion = SimpleNamespace(
        executar_objetivo=move
    )
    server.container_target_publisher = SimpleNamespace(publish=publish_target)
    server._feedback = lambda _handle, _action, status, *_args: (
        feedback.append(status)
    )
    server._gripper = lambda *_args: events.append('open')
    server._transfer_state = lambda *_args: events.append('observation')

    _message, location, placed_pose = server._release_in_container(
        object(), _pose(), 'contêiner azul', -25.0,
    )

    assert events == ['target', 'motion', 'open', 'observation']
    assert ManipulationFeedback.APPROACHING not in feedback
    assert ManipulationFeedback.RETREATING not in feedback
    target_z = lambda motion: (
        motion[1][0].position_constraints[0]
        .constraint_region.primitive_poses[0].position.z
    )
    assert target_z(motions[0]) == pytest.approx(0.10)
    assert len(motions) == 1
    assert targets == [placed_pose]
    constraints = motions[0][1][0]
    assert motions[0][0] == 'arm_container'
    assert constraints.position_constraints[0].link_name == 'gripper_tcp_near'
    assert constraints.orientation_constraints == []
    assert len(constraints.joint_constraints) == 2
    wrist, elbow = constraints.joint_constraints
    assert wrist.joint_name == 'link4_to_link5'
    assert wrist.position == pytest.approx(math.pi / 2.0)
    assert wrist.tolerance_above == pytest.approx(math.radians(5.0))
    assert wrist.tolerance_below == pytest.approx(math.radians(5.0))
    assert elbow.joint_name == 'link3_to_link4'
    assert elbow.position == pytest.approx(math.radians(-25.0))
    assert elbow.tolerance_above == 0.0
    assert elbow.position - elbow.tolerance_below == pytest.approx(-math.pi)
    assert location == ManipulationResult.LOCATION_DESTINATION
    assert placed_pose.pose.position.z == pytest.approx(0.10)


def test_gripper_failure_reports_physical_effect_as_unknown():
    server = ManipulationServer.__new__(ManipulationServer)
    server._effect_known = True
    server._effect_location = ManipulationResult.LOCATION_UNKNOWN
    server._motion = SimpleNamespace(executar_objetivo=lambda *args: None)
    server._feedback = lambda *args: None
    server._gripper = lambda *args: (_ for _ in ()).throw(RuntimeError('falha'))

    with pytest.raises(RuntimeError, match='falha'):
        server._release_at_pose(
            object(), PlaceOnTable, _pose(), _cartesian_profile(), 'teste'
        )

    assert server._effect_known is False
    assert server._effect_location == ManipulationResult.LOCATION_LOST


def _operation_only_server():
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

    def run(_action, _handle, _name, operation, **_kwargs):
        return operation()

    server._run = run
    return server


def test_table_deposit_always_uses_one_combined_scene_request():
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile()},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    calls = []

    def analyze(duration, **kwargs):
        calls.append((duration, kwargs))
        return [], [], _table_grid()

    server._motion = SimpleNamespace(analisar_cena=analyze)
    server._release_at_pose = (
        lambda _handle, _action, pose, _profile, _destination:
        ('ok', 4, pose)
    )
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 12.5
    server._execute_place_on_table(SimpleNamespace(request=goal))

    duration, request = calls[0]
    assert duration == 2.0
    assert request['analisar_apriltags'] is False
    assert request['analisar_containers_hsv'] is False
    assert request['analisar_mesa_branca'] is True
    assert request['altura_mesa_m'] == pytest.approx(0.125)
    assert request['resolucao_grade_m'] == pytest.approx(0.01)
    assert request['mesa_x_min_m'] < -0.16
    assert request['mesa_x_max_m'] > 0.16
    assert request['mesa_y_min_m'] < -0.24
    assert request['mesa_y_max_m'] > -0.14


def test_table_fallback_skips_perception_and_uses_normal_table_profile():
    server = _operation_only_server()
    profile = _search_profile(
        tcp_release_offset_cm=-3.0,
        free_space_preferred_yaw_deg=-90.0,
    )
    server._profiles = SimpleNamespace(placements={'table': profile})
    server._arm_state = lambda *_args: pytest.fail(
        'fallback não deve preparar uma nova observação')
    server._motion = SimpleNamespace(
        analisar_cena=lambda *_args, **_kwargs: pytest.fail(
            'fallback não deve analisar a cena'))
    captured = {}

    def release(_handle, action, pose, selected_profile, destination):
        captured.update(
            action=action,
            pose=pose,
            profile=selected_profile,
            destination=destination,
        )
        return 'ok', ManipulationResult.LOCATION_DESTINATION, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 12.5
    goal.use_fallback_pose = True

    server._execute_place_on_table(SimpleNamespace(request=goal))

    pose = captured['pose']
    assert captured['action'] is PlaceOnTable
    assert captured['profile'] is profile
    assert 'fallback' in captured['destination']
    assert pose.pose.position.x == pytest.approx(0.0)
    assert pose.pose.position.y == pytest.approx(-0.20)
    assert pose.pose.position.z == pytest.approx(0.095)
    assert pose.pose.orientation.x == pytest.approx(0.5)
    assert pose.pose.orientation.y == pytest.approx(-0.5)
    assert pose.pose.orientation.z == pytest.approx(-0.5)
    assert pose.pose.orientation.w == pytest.approx(0.5)


def test_table_requires_release_orientation_calibration_before_detection():
    server = _operation_only_server()
    server._profiles.placements['table'] = _search_profile(
        free_space_preferred_yaw_deg=None)
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = -200.25

    with pytest.raises(FeatureUnavailable, match='preferred_yaw_deg'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_empty_scene_uses_fixed_xy_and_height_plus_tcp_offset():
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
        tcp_release_offset_cm=3.5,
        free_space_preferred_yaw_deg=15.0,
        free_space_alternate_yaw_deg=-75.0,
        reach_center_x_m=0.0,
        reach_center_y_m=0.0,
        reach_min_radius_m=0.10,
        reach_max_radius_m=0.40,
        search_x_min_m=0.21,
        search_x_max_m=0.21,
        search_y_min_m=-0.20,
        search_y_max_m=-0.20,
    )
    server._profiles.pickup_profile = lambda _name: SimpleNamespace(
        observation_state='detect_apriltags')
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(
        analisar_cena=lambda *_args, **_kwargs: ([], [], _table_grid()))
    captured = {}

    def release(_handle, _action, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 12.5

    server._execute_place_on_table(SimpleNamespace(request=goal))

    pose = captured['pose']
    assert pose.pose.position.x == pytest.approx(0.21)
    assert pose.pose.position.y == pytest.approx(-0.20)
    assert pose.pose.position.z == pytest.approx(0.16)


def test_table_search_grid_starts_at_configured_bounds():
    candidates = ManipulationServer._table_search_candidates(_search_profile())

    assert candidates[0] == pytest.approx((-0.10, -0.30))
    assert any(point == pytest.approx((0.10, -0.10)) for point in candidates)
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


def test_table_grid_requires_every_intersecting_footprint_cell_to_be_free():
    grid = _table_grid()
    assert ManipulationServer._table_grid_footprint_is_free(
        grid, 0.0, -0.20, 0.0, 0.07, 0.04, 0.0)

    _set_grid_cell(grid, 0.06, -0.20, TableSurfaceGrid.BLOCKED)
    assert not ManipulationServer._table_grid_footprint_is_free(
        grid, 0.0, -0.20, 0.0, 0.07, 0.04, 0.0)
    assert ManipulationServer._table_grid_footprint_is_free(
        grid, 0.0, -0.20, -90.0, 0.07, 0.04, 0.0)

    _set_grid_cell(grid, 0.0, -0.20, TableSurfaceGrid.UNKNOWN)
    assert not ManipulationServer._table_grid_footprint_is_free(
        grid, 0.0, -0.20, -90.0, 0.07, 0.04, 0.0)


def test_table_search_prefers_nearest_candidate_with_comfortable_clearance():
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.05), (0.0, 0.07)],
        [(0.0, 0.0)],
        0.07, 0.04, 0.03, (0.0,),
    )
    assert selected == pytest.approx((0.0, 0.07, 0.0))


def test_table_search_falls_back_to_minimum_clearance_when_needed():
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.05)],
        [(0.0, 0.0)],
        0.07, 0.04, 0.03, (0.0,),
    )
    assert selected == pytest.approx((0.0, 0.05, 0.0))


def test_table_search_never_falls_below_configured_minimum_padding():
    selected = ManipulationServer._select_free_table_position(
        [(0.06, 0.0), (0.08, 0.0)],
        [(0.0, 0.0)],
        0.05, 0.05, 0.05, (0.0,), minimum_padding_m=0.02,
    )

    assert selected == pytest.approx((0.08, 0.0, 0.0))


def test_table_search_reports_no_free_space():
    with pytest.raises(NoFreeSpace, match='Nenhuma pose'):
        ManipulationServer._select_free_table_position(
            [(0.0, 0.03)],
            [(0.0, 0.0)],
            0.07, 0.04, 0.03, (0.0,),
        )


def test_table_search_rotates_gripper_when_narrow_axis_fits():
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.0)],
        [(0.055, 0.0)],
        0.07, 0.04, 0.0, (0.0, -90.0),
    )
    assert selected == pytest.approx((0.0, 0.0, -90.0))


def test_table_search_shuffles_positions_and_prefers_first_yaw(monkeypatch):
    monkeypatch.setattr(
        'manipulation.node.random.shuffle', lambda candidates: candidates.reverse())
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.0), (0.04, 0.0), (0.08, 0.0)],
        [], 0.02, 0.02, 0.0, (-90.0, 0.0),
    )

    assert selected == pytest.approx((0.08, 0.0, -90.0))


def test_table_search_inflates_container_footprint_by_safety_distance():
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.0), (0.30, 0.0)],
        [(0.12, 0.0, 0.10)],
        0.05, 0.05, 0.0, (0.0,),
    )

    assert selected == pytest.approx((0.30, 0.0, 0.0))


def test_table_search_uses_oriented_rectangle_instead_of_its_diagonal_circle(
    monkeypatch,
):
    monkeypatch.setattr(
        'manipulation.node.random.shuffle', lambda _candidates: None)
    # The point is 8 cm from the container center across its narrow side.
    # Its distance to the 10.2 cm wide rectangle is 2.9 cm.
    selected = ManipulationServer._select_free_table_position(
        [(0.0, 0.08), (0.20, 0.0)],
        [(0.0, 0.0, 0.173, 0.102, 0.0, 0.0)],
        0.02, 0.02, 0.0, (0.0,),
    )
    assert selected == pytest.approx((0.0, 0.08, 0.0))


def test_table_search_rotates_container_and_supports_clearance_uncertainty():
    selected = ManipulationServer._select_free_table_position(
        [(0.08, 0.0), (0.0, 0.20)],
        [(0.0, 0.0, 0.173, 0.102, math.pi / 2.0, 0.02)],
        0.02, 0.02, 0.0, (0.0,),
    )
    assert selected == pytest.approx((0.0, 0.20, 0.0))


def test_table_deposit_reports_no_space_without_confirmed_white_surface():
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile()},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(analisar_cena=lambda *_args, **_kwargs: (
        [], [], _table_grid(TableSurfaceGrid.BLOCKED)
    ))
    server._release_at_pose = lambda *_args: ('ok', 4, _args[2])
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 10.0

    with pytest.raises(NoFreeSpace, match='mesa branca'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_table_deposit_uses_first_shuffled_candidate_free_in_grid(monkeypatch):
    monkeypatch.setattr(
        'manipulation.node.random.shuffle', lambda _candidates: None)
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile()},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(analisar_cena=lambda *_args, **_kwargs: (
        [], [], _table_grid()
    ))
    captured = {}

    def release(_handle, _action, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 10.0
    server._execute_place_on_table(SimpleNamespace(request=goal))

    assert captured['pose'].pose.position.x == pytest.approx(-0.10)
    assert captured['pose'].pose.position.y == pytest.approx(-0.30)


def test_table_deposit_applies_alternate_yaw_selected_by_free_space_search():
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile(
            search_x_min_m=0.0,
            search_x_max_m=0.0,
            search_y_min_m=-0.20,
            search_y_max_m=-0.20,
            free_space_min_padding_m=0.0,
            free_space_preferred_padding_m=0.0,
        )},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    grid = _table_grid()
    _set_grid_cell(grid, 0.06, -0.20, TableSurfaceGrid.BLOCKED)
    server._motion = SimpleNamespace(analisar_cena=lambda *_args, **_kwargs: (
        [], [], grid
    ))
    captured = {}

    def release(_handle, _action, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 10.0
    server._execute_place_on_table(SimpleNamespace(request=goal))

    orientation = captured['pose'].pose.orientation
    assert orientation.x == pytest.approx(0.5)
    assert orientation.y == pytest.approx(-0.5)
    assert orientation.z == pytest.approx(-0.5)
    assert orientation.w == pytest.approx(0.5)


def test_table_deposit_tests_profile_yaw_preference_first():
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'table': _search_profile(
            free_space_preferred_yaw_deg=-90.0,
            free_space_alternate_yaw_deg=0.0,
        )},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(analisar_cena=lambda *_args, **_kwargs: (
        [], [], _table_grid()
    ))
    captured = {}

    def release(_handle, _action, pose, _profile, _destination):
        captured['pose'] = pose
        return 'ok', 4, pose

    server._release_at_pose = release
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 10.0

    server._execute_place_on_table(SimpleNamespace(request=goal))

    orientation = captured['pose'].pose.orientation
    assert orientation.z == pytest.approx(-0.5)
    assert orientation.w == pytest.approx(0.5)


def test_table_apriltag_analysis_requires_complete_search_bounds_before_motion():
    server = _operation_only_server()
    server._profiles.placements['table'] = _search_profile(search_x_min_m=None)
    server._arm_state = lambda *_args: pytest.fail('não deveria mover o braço')
    goal = PlaceOnTable.Goal()
    goal.ws_height_cm = 10.0
    with pytest.raises(FeatureUnavailable, match='search_x_min_m'):
        server._execute_place_on_table(SimpleNamespace(request=goal))


def test_container_rejects_color_outside_enum_before_detection():
    server = _operation_only_server()
    goal = PlaceInContainer.Goal()
    goal.ws_height_cm = 10.0
    goal.container_color = 99

    with pytest.raises(ConfigurationError, match='Cor de contêiner inválida'):
        server._execute_place_in_container(SimpleNamespace(request=goal))


def _container_detection(color, x=0.02, y=-0.22, z=0.16):
    detection = ContainerStampedDetection()
    detection.header.frame_id = 'arm_base_link'
    detection.color = color
    detection.pose.position.x = x
    detection.pose.position.y = y
    detection.pose.position.z = z
    detection.pose.orientation.w = 1.0
    return detection


def _container_operation_server(detections):
    server = _operation_only_server()
    server._profiles = SimpleNamespace(
        placements={'container': PlacementProfile(
            name='container',
            strategy='perception',
            enabled=True,
            named_state='',
            approach_height_m=0.10,
            retreat_height_m=0.10,
            reference_offset_xyz=(0.0, 0.0, 0.07),
            yaw_offset_deg=90.0,
            calibrated_reference=True,
        )},
        pickup_profile=lambda _name: SimpleNamespace(
            observation_state='detect_apriltags'
        ),
    )
    server._arm_state = lambda *_args: None
    server.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    server._motion = SimpleNamespace(
        analisar_cena=lambda _duration, **_kwargs: ([], detections),
    )
    return server


def _container_goal(color, height_cm=12.5):
    goal = PlaceInContainer.Goal()
    goal.ws_height_cm = height_cm
    goal.container_color = color
    return SimpleNamespace(request=goal)


@pytest.mark.parametrize('color, height_cm, external_height_m', [
    (PlaceInContainer.Goal.RED, 12.5, 0.073),
    (PlaceInContainer.Goal.BLUE, 20.0, 0.090),
])
def test_hsv_container_release_uses_detected_top_and_offset(
    color, height_cm, external_height_m,
):
    selected = _container_detection(color)
    selected.pose.position.z = height_cm / 100.0 + external_height_m - 0.097
    selected.pose.orientation.w = 0.0
    other = _container_detection(
        PlaceInContainer.Goal.BLUE
        if color == PlaceInContainer.Goal.RED else PlaceInContainer.Goal.RED,
        x=-0.08,
    )
    server = _container_operation_server([other, selected])
    released = []
    server._release_in_container = lambda *args: released.append(
        args
    ) or (
        'ok', ManipulationResult.LOCATION_DESTINATION, args[1]
    )

    _message, _location, pose = server._execute_place_in_container(
        _container_goal(color, height_cm)
    )

    assert len(released) == 1
    assert pose.pose.position.x == pytest.approx(selected.pose.position.x)
    assert pose.pose.position.y == pytest.approx(selected.pose.position.y)
    assert pose.pose.position.z == pytest.approx(
        selected.pose.position.z + 0.07
    )
    assert pose.pose.orientation.w == pytest.approx(1.0)
    assert len(released[0]) == 4
    assert released[0][3] == pytest.approx(-10.0)


def test_partial_container_can_be_selected_as_deposit_target():
    target = _container_detection(PlaceInContainer.Goal.RED, x=0.04)
    target.partial = True
    server = _container_operation_server([target])
    server._vision_detector_masks = {
        'place_in_container': SceneObservation.CONTAINERS_HSV}
    feedback = []
    server._feedback = lambda *args: feedback.append(args[-1])
    server._release_in_container = lambda *args: ('ok', 4, args[1])

    _message, _location, pose = server._execute_place_in_container(
        _container_goal(PlaceInContainer.Goal.RED))

    assert pose.pose.position.x == pytest.approx(0.04)
    assert any('centro da parte visível' in message for message in feedback)




def test_missing_container_does_not_start_release():
    server = _container_operation_server([])
    released = []
    server._release_in_container = lambda *args: released.append(args)

    with pytest.raises(ObjectNotFound):
        server._execute_place_in_container(
            _container_goal(PlaceInContainer.Goal.RED)
        )
    assert released == []


def test_nearest_same_color_container_is_selected():
    farther = _container_detection(
        PlaceInContainer.Goal.RED, x=0.18, y=-0.22)
    nearest = _container_detection(
        PlaceInContainer.Goal.RED, x=0.03, y=-0.16)
    other_color = _container_detection(
        PlaceInContainer.Goal.BLUE, x=0.01, y=-0.05)
    server = _container_operation_server([farther, other_color, nearest])
    released = []
    server._release_in_container = lambda *args: released.append(args) or (
        'ok', ManipulationResult.LOCATION_DESTINATION, args[1]
    )

    _message, _location, pose = server._execute_place_in_container(
        _container_goal(PlaceInContainer.Goal.RED)
    )

    assert len(released) == 1
    assert pose.pose.position.x == pytest.approx(nearest.pose.position.x)
    assert pose.pose.position.y == pytest.approx(nearest.pose.position.y)


@pytest.mark.parametrize('invalid_field', ['x', 'y', 'z'])
def test_container_invalid_geometry_does_not_start_release(invalid_field):
    detection = _container_detection(PlaceInContainer.Goal.BLUE)
    if invalid_field == 'x':
        detection.pose.position.x = float('nan')
    elif invalid_field == 'y':
        detection.pose.position.y = float('inf')
    else:
        detection.pose.position.z = float('nan')
    server = _container_operation_server([detection])
    released = []
    server._release_in_container = lambda *args: released.append(args)

    with pytest.raises(PerceptionUnavailable, match='posição inválida'):
        server._execute_place_in_container(
            _container_goal(PlaceInContainer.Goal.BLUE)
        )
    assert released == []


def test_hsv_container_release_uses_top_height_in_arm_base_frame():
    detection = _container_detection(PlaceInContainer.Goal.BLUE)
    detection.pose.position.z = 0.076  # floor-to-arm offset already applied
    release = ManipulationServer._container_release_pose(
        detection, (0.0, 0.0, 0.015))
    assert release.pose.position.z == pytest.approx(0.091)


def test_hsv_partial_container_deposits_at_visible_center():
    target = _container_detection(PlaceInContainer.Goal.BLUE,
                                  x=0.19085, y=-0.23576, z=0.026)
    target.partial = True
    target.observation_count = 34
    server = _container_operation_server([target])
    feedback = []
    releases = []
    server._feedback = lambda *args: feedback.append(args[-1])
    server._release_in_container = lambda *args: releases.append(args[1]) or (
        'ok', 4, args[1])

    _message, _location, pose = server._execute_place_in_container(
        _container_goal(PlaceInContainer.Goal.BLUE, 5.0))

    assert len(releases) == 1
    assert pose.pose.position.x == pytest.approx(0.19085)
    assert pose.pose.position.y == pytest.approx(-0.23576)
    assert pose.pose.position.z == pytest.approx(0.096)  # top + profile offset
    assert any('centro da parte visível' in message for message in feedback)
