from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from mission_manager.errors import ConfigurationError
from mission_manager.generate_plan import write_validated
from mission_manager.loaders import load_arena, load_plan, validate_plan
from mission_manager.models import MapPose
from mission_manager.nav2_distances import DistanceCache, map_fingerprint, path_length
from mission_manager.plan_optimizer import Mission, Transport, import_commander, optimize
from mission_manager.scheduler import Scheduler


PACKAGE = Path(__file__).parents[1]


@pytest.fixture
def arena():
    return load_arena(PACKAGE / 'config/arena.yaml')


def team(sources, destinations, task_id='BTT1', containers=()):
    active = list(dict.fromkeys([*sources.values(), *destinations.values()]))
    raw = {'task_id': task_id, 'active_service_areas': active,
           'objects': [{'id': tag} for tag in sources], 'containers': list(containers)}
    for name, locations in [('start_state', sources), ('finish_state', destinations)]:
        raw[name] = {}
        for area in active:
            entry = {'obj_ids': [t for t, a in locations.items() if a == area]}
            colors = [c['color'] for c in containers if c['at'] == area]
            if colors:
                entry['cont_colors'] = colors
            raw[name][area] = entry
    return raw


def unit(a, b):
    return float(a != b)


def generate(tmp_path, arena, mission, **kwargs):
    result = optimize(mission, unit, plan_id='test_plan', **kwargs)
    path = tmp_path / 'test_plan.yaml'
    plan = write_validated(result.document, path, arena, ['left', 'right'])
    return result, plan


def test_shipped_area_ids_and_plan_targets_use_commander_spelling(arena):
    assert {'SH_1', 'WS_1', 'WS_2', 'WS_3', 'WS_4', 'WS_5', 'PP', 'ST'} <= arena.service_areas.keys()
    for path in (PACKAGE / 'config/plans').glob('*.yaml'):
        plan = load_plan(path)
        assert all(v.target == v.target.upper() for v in plan.visits)


def test_three_cubes_fit_two_slots_and_gripper(tmp_path, arena):
    raw = team({t: 'WS_1' for t in [1, 2, 3]}, {1: 'WS_2', 2: 'WS_2', 3: 'WS_3'})
    result, plan = generate(tmp_path, arena, import_commander(raw, arena))
    assert plan.initial_location == 'start' and plan.finish
    assert len(plan.visits) == 3
    assert result.distance_m == 4
    assert set(plan.visits[0].tasks[0].tag_ids) == {1, 2, 3}


def test_more_than_cargo_capacity_requires_repeat_source(tmp_path, arena):
    raw = team({t: 'WS_1' for t in range(1, 5)}, {t: 'WS_2' for t in range(1, 5)})
    _, plan = generate(tmp_path, arena, import_commander(raw, arena))
    assert [v.target for v in plan.visits] == ['WS_1', 'WS_2', 'WS_1', 'WS_2']


def test_same_shelf_is_transport_but_bmt_stationary_cube_is_ignored(tmp_path, arena):
    raw = team({1: 'SH_1', 2: 'WS_1'}, {1: 'SH_1', 2: 'WS_1'}, task_id='FINAL')
    mission = import_commander(raw, arena)
    assert [t.tag for t in mission.transports] == [1]
    _, plan = generate(tmp_path, arena, mission)
    assert len(plan.visits) == 1
    assert plan.visits[0].tasks[1].action == 'place_on_shelf'


def test_pp_requires_store_and_retrieve_and_reference_can_be_overridden(tmp_path, arena):
    raw = team({3: 'WS_1', 4: 'WS_1'}, {3: 'PP', 4: 'PP'}, task_id='ATT2')
    mission = import_commander(raw, arena, {3: 23, 4: 24})
    _, plan = generate(tmp_path, arena, mission)
    assert sorted(t.reference_tag_id for t in plan.visits[-1].tasks) == [23, 24]


def test_stack_places_bottom_before_upper_cube_without_invented_support(tmp_path, arena):
    raw = team({7: 'WS_1', 2: 'WS_2'}, {7: 'ST', 2: 'ST'}, task_id='FINAL')
    mission = import_commander(raw, arena)
    bottom = next(t for t in mission.transports if t.tag == 7)
    upper = next(t for t in mission.transports if t.tag == 2)
    assert bottom.action == 'place_on_table'
    assert upper.action == 'stack' and upper.support_tag_id == 7
    _, plan = generate(tmp_path, arena, mission)
    assert any(t.action == 'stack' and t.support_tag_id == 7 for v in plan.visits for t in v.tasks)


def test_two_containers_same_destination_route_is_valid_for_both_colors(tmp_path, arena):
    raw = team({1: 'WS_1', 2: 'WS_1'}, {1: 'WS_2', 2: 'WS_2'}, task_id='ATT1',
               containers=[{'at': 'WS_2', 'color': 'BLUE'}, {'at': 'WS_2', 'color': 'RED'}])
    _, plan = generate(tmp_path, arena, import_commander(raw, arena))
    assert len(plan.visits[-1].tasks) == 2
    for one in ('red', 'blue'):
        for two in ('red', 'blue'):
            scheduler = Scheduler(plan, ('left', 'right'))
            scheduler.set_tag_colors({1: one, 2: two})
            assert scheduler.feasible(scheduler.initial_state)


def test_amt_discovers_occupancy_instead_of_inventing_missing_cavities(tmp_path, arena):
    raw = team({2: 'PP', 1: 'PP'}, {1: 'PP', 2: 'PP'}, task_id='AMT')
    _, plan = generate(tmp_path, arena, import_commander(raw, arena))
    assert plan.visits[0].pp_start_state is None
    assert dict(plan.visits[0].pp_final_state) == {1: 1, 2: 2}


@pytest.mark.parametrize('mutation', [
    lambda r: r['objects'].append({'id': 1}),
    lambda r: r['start_state']['WS_1']['obj_ids'].append(2),
    lambda r: r['finish_state']['WS_2'].update(obj_ids=[]),
    lambda r: r.update(active_service_areas=['missing']),
    lambda r: r.update(task_id='judge'),
])
def test_malformed_team_export_rejected(arena, mutation):
    raw = team({1: 'WS_1'}, {1: 'WS_2'})
    mutation(raw)
    with pytest.raises(ConfigurationError):
        import_commander(raw, arena)


def test_finish_distance_participates_in_route_objective():
    mission = Mission((Transport(1, 'A', 'B', 'place_on_table'),
                       Transport(2, 'A', 'C', 'place_on_table')))
    # Equal area-to-area costs, but only B is near finish.
    def length(a, b):
        return 20.0 if (a, b) == ('C', 'finish') else 1.0
    result = optimize(mission, length, plan_id='route', objective='distance')
    assert [v['target'] for v in result.document['visits']] == ['A', 'C', 'B']
    assert result.distance_m == 4.0


def test_minimum_visits_and_minimum_distance_can_choose_different_routes():
    import math
    coordinates = {'A': (7, 9), 'B': (3, 12), 'C': (15, 4), 'D': (2, 2),
                   'start': (0, 12), 'finish': (17, 9)}
    mission = Mission(tuple(Transport(tag, source, destination, 'place_on_table')
                            for tag, source, destination in
                            [(1, 'A', 'D'), (2, 'C', 'B'), (3, 'B', 'A'), (4, 'C', 'A')]))
    distance = lambda a, b: math.dist(coordinates[a], coordinates[b])
    visits = optimize(mission, distance, plan_id='visits', objective='visits', cargo_capacity=1)
    meters = optimize(mission, distance, plan_id='meters', objective='distance', cargo_capacity=1)
    assert len(visits.document['visits']) == 4
    assert len(meters.document['visits']) == 6
    assert meters.distance_m < visits.distance_m


def test_unreachable_unused_pairs_do_not_prevent_a_viable_route():
    mission = Mission((Transport(1, 'A', 'B', 'place_on_table'),
                       Transport(2, 'A', 'C', 'place_on_table')))
    distance = lambda a, b: float('inf') if (a, b) == ('B', 'C') else 1.0
    result = optimize(mission, distance, plan_id='route')
    assert [v['target'] for v in result.document['visits']] == ['A', 'C', 'B']


def test_search_limit_never_returns_partial_plan():
    with pytest.raises(ConfigurationError, match='Limite'):
        optimize(Mission((Transport(1, 'A', 'B', 'place_on_table'),)), unit,
                 plan_id='route', max_states=1)


def test_validated_write_does_not_clobber_existing_or_publish_invalid(tmp_path, arena):
    path = tmp_path / 'test.yaml'
    path.write_text('original')
    doc = {'schema_version': 2, 'plan_id': 'test', 'visits': [{'target': 'WS_1'}]}
    with pytest.raises(FileExistsError):
        write_validated(doc, path, arena, ['left', 'right'])
    assert path.read_text() == 'original'
    doc['visits'][0]['target'] = 'missing'
    with pytest.raises(ConfigurationError):
        write_validated(doc, path, arena, ['left', 'right'], force=True)
    assert path.read_text() == 'original'
    assert not list(tmp_path.glob('.mission_plan_*'))


def test_cache_persists_and_is_directed_and_pose_and_context_sensitive(tmp_path):
    path = tmp_path / 'cache.json'
    a, b = MapPose(0.0, 0.0, 0.0), MapPose(1.0, 0.0, 0.0)
    cache = DistanceCache(path, {'map': 'one'})
    assert cache.get(a, b, lambda: 2.0) == 2.0
    def forbidden():
        pytest.fail('Cached pair requested again')
    cache = DistanceCache(path, {'map': 'one'})
    assert cache.get(a, b, forbidden) == 2.0
    assert cache.hits == 1
    assert cache.get(b, a, lambda: 3.0) == 3.0
    assert cache.get(a, replace(b, yaw_rad=1.0), lambda: 4.0) == 4.0
    assert DistanceCache(path, {'map': 'two'}).get(a, b, lambda: 5.0) == 5.0


def test_failed_distance_not_cached(tmp_path):
    cache = DistanceCache(tmp_path / 'cache.json', {})
    def failure():
        raise ConfigurationError('no path')
    with pytest.raises(ConfigurationError):
        cache.get(MapPose(0, 0, 0), MapPose(1, 0, 0), failure)
    assert not cache.path.exists()


def test_map_image_contents_invalidate_cache_identity(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': 0.05}))
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'first')
    first = map_fingerprint(path)
    image.write_bytes(b'second')
    assert map_fingerprint(path) != first


def test_path_length_uses_polyline_and_rejects_empty_or_partial_paths():
    def path(points):
        return SimpleNamespace(header=SimpleNamespace(frame_id='map'), poses=[
            SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))
            for x, y in points])
    start, finish = MapPose(0, 0, 0), MapPose(3, 4, 0)
    assert path_length(path([(0, 0), (3, 0), (3, 4)]), start, finish, 'map') == 7
    for points in ([], [(0, 0)], [(0, 0), (3, 0)]):
        with pytest.raises(ConfigurationError):
            path_length(path(points), start, finish, 'map')


def test_nav2_adapter_uses_explicit_start_real_messages_and_disk_cache(tmp_path, arena, monkeypatch):
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.parameter import Parameter
    from mission_manager.nav2_distances import Nav2Distances

    map_path = tmp_path / 'map.yaml'
    metadata = {'image': 'map.pgm', 'resolution': 0.05, 'origin': [0, 0, 0],
                'negate': 0, 'free_thresh': 0.196, 'occupied_thresh': 0.65}
    map_path.write_text(yaml.safe_dump(metadata))
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    calls = []
    replies = []
    grid = Costmap()
    grid.header.frame_id = 'map'
    grid.metadata.resolution = 0.05
    grid.metadata.size_x = grid.metadata.size_y = 1
    grid.data = [0]
    published = OccupancyGrid()
    published.header.frame_id = 'map'
    published.info.resolution = 0.05
    published.info.width = published.info.height = 1
    published.info.origin.orientation.w = 1.0
    published.data = [100]
    import copy
    mask = copy.deepcopy(published)
    mask.data = [0]
    config = yaml.safe_load((PACKAGE / 'config/mission_planner.yaml').read_text())
    source_parameters = {
        '/mission_planning/planner_server': config['planner_server']['ros__parameters'],
        '/mission_planning/global_costmap/global_costmap': config['global_costmap']['global_costmap']['ros__parameters'],
        '/mission_planning/keepout_costmap_filter_info_server': {'type': 0, 'base': 0.0, 'multiplier': 1.0},
    }
    def flatten(raw, prefix=''):
        result = {}
        for key, value in raw.items():
            name = prefix + key
            if isinstance(value, dict):
                result.update(flatten(value, name + '.'))
            else:
                result[name] = Parameter(name, value=value).get_parameter_value()
        return result
    parameters = {name: flatten(raw) for name, raw in source_parameters.items()}

    def future(value):
        return SimpleNamespace(result=lambda: value)

    class FakeAction:
        def __init__(self, *args):
            pass

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            assert isinstance(goal, ComputePathToPose.Goal)
            calls.append(goal)
            result = ComputePathToPose.Result()
            result.path.header.frame_id = 'map'
            result.path.poses = [goal.start, goal.goal]
            status = GoalStatus.STATUS_SUCCEEDED
            if replies:
                result.error_code = replies.pop()
                status = GoalStatus.STATUS_ABORTED
            return future(SimpleNamespace(accepted=True,
                          get_result_async=lambda: future(SimpleNamespace(status=status, result=result)),
                          cancel_goal_async=lambda: future(None)))

    monkeypatch.setattr('rclpy.action.ActionClient', FakeAction)
    monkeypatch.setattr(Nav2Distances, '_await', lambda self, f: f.result())
    def service(self, name, kind, request):
        assert name in {'/mission_planning/map_server/map',
                        '/mission_planning/keepout_filter_mask_server/map'}
        return SimpleNamespace(map=mask if 'keepout' in name else published)
    monkeypatch.setattr(Nav2Distances, '_service', service)
    monkeypatch.setattr(Nav2Distances, '_parameters',
                        lambda *args: pytest.fail('Remote yaml_filename must never be read'))
    monkeypatch.setattr(Nav2Distances, '_all_parameters', lambda self, name: parameters[name])
    node = SimpleNamespace(
        create_client=lambda *args: SimpleNamespace(wait_for_service=lambda **kwargs: True,
                            call_async=lambda req: future(SimpleNamespace(map=grid))),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        get_logger=lambda: SimpleNamespace(info=lambda *args: None, warning=lambda *args: None))
    cache = tmp_path / 'distances.json'
    distances = Nav2Distances(node, arena, map_path, cache, planner_id='GridBased')
    first = distances.distance('start', 'WS_1')
    request = calls[0]
    assert request.use_start and request.planner_id == 'GridBased'
    assert isinstance(request.start, PoseStamped)
    assert request.start.pose.position.x == arena.start.x_m
    assert request.goal.pose.position.x == arena.service_areas['WS_1'].pose.x_m
    assert distances.distance('start', 'WS_1') == first
    assert len(calls) == 1
    # A new CLI invocation with the same snapshot reuses the persistent pair.
    distances = Nav2Distances(node, arena, map_path, cache, planner_id='GridBased')
    assert distances.distance('start', 'WS_1') == first
    assert len(calls) == 1
    replies.append(ComputePathToPose.Result.NO_VALID_PATH)
    assert distances.distance('WS_1', 'WS_2') == float('inf')
    # Unreachable pairs are retried, never persisted as successful distances.
    assert distances.distance('WS_1', 'WS_2') > 0
    assert len(calls) == 3
    grid.data = [255]
    distances.verify_costmap()  # NavFn's working buffer is not cache identity.
    mask.data = [100]
    with pytest.raises(ConfigurationError, match='keepout'):
        distances.verify_costmap()
    changed = Nav2Distances(node, arena, map_path, cache, planner_id='GridBased')
    changed.distance('start', 'WS_1')
    assert len(calls) == 4  # A changed keepout mask cannot reuse the old cache.
    parameters['/mission_planning/global_costmap/global_costmap']['plugins'] = Parameter(
        'plugins', value=['static_layer', 'inflation_layer', 'obstacle_layer']).get_parameter_value()
    parameters['/mission_planning/global_costmap/global_costmap']['obstacle_layer.plugin'] = Parameter(
        'obstacle_layer.plugin', value='nav2_costmap_2d::ObstacleLayer').get_parameter_value()
    with pytest.raises(ConfigurationError, match='lidar'):
        Nav2Distances(node, arena, map_path, cache)
    other = tmp_path / 'other.yaml'
    other.write_text(yaml.safe_dump(dict(metadata, resolution=0.1)))
    with pytest.raises(ConfigurationError, match='difere do mapa'):
        Nav2Distances(node, arena, other, cache)


def test_navfn_tolerance_accepts_diagonal_but_reports_real_endpoint_failures():
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    start, goal = MapPose(0, 0, 0), MapPose(3, 4, 0)
    path = RosPath()
    path.header.frame_id = 'map'
    for x, y in [(0.05, 0.05), (3.10, 4.10)]:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        path.poses.append(pose)
    assert path_length(path, start, goal, 'map', 0.1, start_tolerance=2**0.5 * 0.05) > 0
    path.poses[-1].pose.position.x = 3.11
    with pytest.raises(ConfigurationError, match='X=11.00 cm'):
        path_length(path, start, goal, 'map', 0.1, start_tolerance=2**0.5 * 0.05)
    path.poses[-1].pose.position.x = 3.0
    path.poses[0].pose.position.x = 0.2
    with pytest.raises(ConfigurationError, match='desvio inicial='):
        path_length(path, start, goal, 'map', 0.1, start_tolerance=2**0.5 * 0.05)


@pytest.mark.parametrize('mismatch', [None, 'cell', 'origin', 'resolution', 'frame'])
def test_remote_map_comparison_checks_cells_geometry_and_vertical_flip(tmp_path, mismatch):
    from nav_msgs.msg import OccupancyGrid
    from mission_manager.nav2_distances import verify_map
    local = tmp_path / 'arena.yaml'
    local.write_text(yaml.safe_dump({'image': 'arena.pgm', 'resolution': 0.05,
                                    'origin': [1.0, -2.0, 0.0], 'negate': 0,
                                    'free_thresh': 0.196, 'occupied_thresh': 0.65}))
    (tmp_path / 'arena.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 255, 205, 0]))
    remote = OccupancyGrid()
    remote.header.frame_id = 'map'
    remote.info.width = remote.info.height = 2
    remote.info.resolution = 0.05
    remote.info.origin.position.x = 1.0
    remote.info.origin.position.y = -2.0
    remote.info.origin.orientation.w = 1.0
    remote.data = [-1, 100, 100, 0]
    if mismatch == 'cell':
        remote.data[0] = 0
    elif mismatch == 'origin':
        remote.info.origin.position.x += 0.1
    elif mismatch == 'resolution':
        remote.info.resolution = 0.1
    elif mismatch == 'frame':
        remote.header.frame_id = 'odom'
    if mismatch:
        with pytest.raises(ConfigurationError, match='difere do mapa'):
            verify_map(local, remote, 'map')
    else:
        verify_map(local, remote, 'map')
