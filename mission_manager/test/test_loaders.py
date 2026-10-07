from pathlib import Path

import pytest

from mission_manager.errors import ConfigurationError
from mission_manager.loaders import load_arena, load_plan, validate_plan


PACKAGE = Path(__file__).parents[1]


def test_stack_alignment_targets_are_independent(tmp_path):
    import yaml
    raw = yaml.safe_load(VALID_ARENA)
    raw['pickup_recovery'].update({
        'stack_preferred_tag_x_m': 0.03, 'stack_preferred_tag_y_m': -0.27,
        'shelf_preferred_tag_y_m': -0.32,
    })
    arena = load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))
    assert arena.pickup_recovery.stack_preferred_tag_x_m == pytest.approx(0.03)
    assert arena.pickup_recovery.stack_preferred_tag_y_m == pytest.approx(-0.27)
    assert arena.pickup_recovery.shelf_preferred_tag_y_m == pytest.approx(-0.32)
    assert arena.pickup_recovery.preferred_tag_y_m == pytest.approx(-0.22)


@pytest.mark.parametrize('field', ['stack_preferred_tag_x_m', 'stack_preferred_tag_y_m'])
@pytest.mark.parametrize('value', [float('nan'), True, '0.1'])
def test_stack_alignment_rejects_invalid_targets(tmp_path, field, value):
    import yaml
    raw = yaml.safe_load(VALID_ARENA)
    raw['pickup_recovery'][field] = value
    with pytest.raises(ConfigurationError, match=field):
        load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))


VALID_ARENA = """
schema_version: 1
frame_id: map
alignment_defaults:
  distance_mm: 200
  tolerance_mm: 10
  timeout_s: 10.0
departure_defaults:
  distance_mm: 250
  tolerance_mm: 15
  timeout_s: 8.0
  lateral_position_mm: 0
  max_alignment_error_mm: 100
  alignment_recovery_distance_mm: 80
  minimum_lateral_clearance_mm: 20
pickup_recovery:
  enabled: true
  minimum_wall_distance_mm: 30
  maximum_wall_distance_mm: 250
  minimum_lateral_position_mm: -275
  maximum_lateral_position_mm: 275
  preferred_tag_x_m: 0.0
  preferred_tag_y_m: -0.22
  wall_tolerance_mm: 5
  travel_tolerance_mm: 10
  timeout_s: 15.0
  max_reposition_attempts: 1
  search_positions_mm: [0, 250, -250]
start: {x_m: 0.0, y_m: 0.0, yaw_rad: 0.0}
finish: {x_m: 3.0, y_m: 0.0, yaw_rad: 3.14}
service_areas:
  ws_1:
    x_m: 1.0
    y_m: 2.0
    yaw_rad: 1.57
    height_cm: 10.0
    type: WS
  ws_3:
    x_m: 2.0
    y_m: 2.0
    yaw_rad: 1.57
    height_cm: 15.0
    type: SH
    alignment:
      distance_mm: 180
    departure:
      distance_mm: 300
      lateral_position_mm: -20
      max_alignment_error_mm: 75
      minimum_lateral_clearance_mm: 15
"""


def _write(tmp_path, name, content):
    path = tmp_path / name
    path.write_text(content, encoding='utf-8')
    return path


def test_arena_merges_partial_alignment_override(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))

    assert arena.service_areas['ws_1'].alignment.distance_mm == 200
    right = arena.service_areas['ws_3'].alignment
    assert right.distance_mm == 180
    assert right.tolerance_mm == 10
    assert right.timeout_s == pytest.approx(10.0)
    assert arena.service_areas['ws_1'].departure.distance_mm == 250
    departure = arena.service_areas['ws_3'].departure
    assert departure.distance_mm == 300
    assert departure.tolerance_mm == 15
    assert departure.timeout_s == pytest.approx(8.0)
    assert arena.service_areas['ws_1'].departure.lateral_position_mm == 0
    assert departure.lateral_position_mm == -20
    assert departure.max_alignment_error_mm == 75
    assert departure.alignment_recovery_distance_mm == 80
    assert departure.minimum_lateral_clearance_mm == 15
    assert arena.pickup_recovery.minimum_wall_distance_mm == 30
    assert arena.pickup_recovery.minimum_lateral_position_mm == -275
    assert arena.pickup_recovery.maximum_lateral_position_mm == 275
    assert arena.pickup_recovery.preferred_tag_y_m == pytest.approx(-0.22)
    assert arena.pickup_recovery.search_positions_mm == (0, 250, -250)


def test_shelf_place_alignment_has_independent_defaults_and_local_override(tmp_path):
    import yaml

    raw = yaml.safe_load(VALID_ARENA)
    arena = load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))
    assert arena.service_areas['ws_3'].shelf_place_alignment.distance_mm == 40
    raw['shelf_place_alignment_defaults'] = {'distance_mm': 65, 'tolerance_mm': 7}
    arena = load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))
    assert arena.service_areas['ws_3'].shelf_place_alignment.distance_mm == 65
    raw['service_areas']['ws_3']['shelf_place_alignment'] = {'distance_mm': 90}
    arena = load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))
    alignment = arena.service_areas['ws_3'].shelf_place_alignment
    assert alignment.distance_mm == 90
    assert alignment.tolerance_mm == 7
    assert alignment.timeout_s == 10.0
    assert arena.service_areas['ws_3'].alignment.distance_mm == 180
    assert arena.service_areas['ws_1'].shelf_place_alignment is None


@pytest.mark.parametrize('distance', [0, -1])
def test_shelf_place_alignment_rejects_invalid_distance(tmp_path, distance):
    import yaml

    raw = yaml.safe_load(VALID_ARENA)
    raw['service_areas']['ws_3']['shelf_place_alignment'] = {'distance_mm': distance}
    with pytest.raises(ConfigurationError):
        load_arena(_write(tmp_path, 'arena.yaml', yaml.safe_dump(raw)))



def test_table_place_positions_are_separate_from_pickup_positions(tmp_path):
    source = VALID_ARENA.replace(
        'start: {x_m:',
        'table_place_search_positions_mm: [0, 160, 250, -160, -250]\n'
        'start: {x_m:',
    )
    arena = load_arena(_write(tmp_path, 'arena.yaml', source))

    assert arena.table_place_search_positions_mm == (0, 160, 250, -160, -250)
    assert arena.pickup_recovery.search_positions_mm == (0, 250, -250)


def test_arena_refuses_uncalibrated_poses(tmp_path):
    source = VALID_ARENA.replace('x_m: 1.0', 'x_m: null')
    with pytest.raises(ConfigurationError, match='deve ser numérico'):
        load_arena(_write(tmp_path, 'arena.yaml', source))


def test_arena_rejects_unknown_fields(tmp_path):
    source = VALID_ARENA.replace('height_cm: 10.0', 'height_cm: 10.0\n    typo: 1')

    with pytest.raises(ConfigurationError, match='campos desconhecidos'):
        load_arena(_write(tmp_path, 'arena.yaml', source))


def test_arena_rejects_non_integer_departure_lateral_position(tmp_path):
    source = VALID_ARENA.replace(
        'lateral_position_mm: 0', 'lateral_position_mm: 0.5'
    )

    with pytest.raises(ConfigurationError, match='deve ser inteiro'):
        load_arena(_write(tmp_path, 'arena.yaml', source))


def test_arena_rejects_departure_recovery_without_alignment_limit(tmp_path):
    source = VALID_ARENA.replace(
        'max_alignment_error_mm: 100', 'max_alignment_error_mm: 0'
    )

    with pytest.raises(ConfigurationError, match='requer'):
        load_arena(_write(tmp_path, 'arena.yaml', source))


def test_arena_allows_legacy_departure_without_safety_overrides(tmp_path):
    source = VALID_ARENA.replace(
        '  max_alignment_error_mm: 100\n'
        '  alignment_recovery_distance_mm: 80\n'
        '  minimum_lateral_clearance_mm: 20\n',
        '',
    ).replace(
        '      max_alignment_error_mm: 75\n'
        '      minimum_lateral_clearance_mm: 15\n',
        '',
    )

    arena = load_arena(_write(tmp_path, 'arena.yaml', source))

    departure = arena.service_areas['ws_1'].departure
    assert departure.max_alignment_error_mm is None
    assert departure.alignment_recovery_distance_mm is None
    assert departure.minimum_lateral_clearance_mm is None


def test_arena_rejects_search_position_outside_lateral_limits(tmp_path):
    source = VALID_ARENA.replace(
        'search_positions_mm: [0, 250, -250]',
        'search_positions_mm: [0, 300, -250]',
    )

    with pytest.raises(ConfigurationError, match='fora dos limites laterais'):
        load_arena(_write(tmp_path, 'arena.yaml', source))


def _plan_yaml(task=''):
    return f"""schema_version: 2
plan_id: test
initial_location: ws_1
finish: false
visits:
  - id: visit_1
    target: ws_1
    tasks:
      - {{id: collect, action: pick, tag_id: 1}}
      - {{id: deliver, action: place_in_container, tag_id: 1, container_color: red}}
{task}
"""


@pytest.mark.parametrize('path', sorted((PACKAGE / 'config' / 'plans').glob('*.yaml')))
def test_installed_plans_are_complete_and_feasible(path):
    arena = load_arena(PACKAGE / 'config' / 'arena.yaml')
    plan = load_plan(path)
    validate_plan(plan, arena)
    assert plan.visits
    assert plan.total_steps >= len(plan.visits)


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_plan_accepts_container_deposit_by_color(tmp_path, color):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan = load_plan(_write(tmp_path, 'plan.yaml', _plan_yaml().replace('color: red', f'color: {color}')))
    validate_plan(plan, arena)
    assert plan.visits[0].tasks[1].container_color == color


def test_plan_rejects_v1_with_migration_message(tmp_path):
    with pytest.raises(ConfigurationError, match='Migre steps para visits'):
        load_plan(_write(tmp_path, 'plan.yaml', 'schema_version: 1\nplan_id: old\nsteps: []'))


@pytest.mark.parametrize('old,new,match', [
    ('color: red', 'color: green', 'red ou blue'),
    ('target: ws_1', 'target: missing', 'target desconhecido'),
    ('initial_location: ws_1', 'initial_location: missing', 'initial_location'),
    ('id: deliver', 'id: collect', 'IDs duplicados'),
    ('tag_id: 1, container_color', 'tag_id: 2, container_color', 'inviável'),
    ('action: pick, tag_id: 1', 'action: store, slot_id: left', 'action desconhecida'),
    ('action: pick, tag_id: 1', 'action: pick, tag_id: 1, slot_id: left', 'campos desconhecidos'),
    ('finish: false', 'finish: 1', 'booleano'),
])
def test_plan_validation(tmp_path, old, new, match):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    with pytest.raises(ConfigurationError, match=match):
        validate_plan(load_plan(_write(tmp_path, 'plan.yaml', _plan_yaml().replace(old, new))), arena)


def test_plan_rejects_support_in_own_stack(tmp_path):
    source = _plan_yaml().replace(
        'action: place_in_container, tag_id: 1, container_color: red',
        'action: stack, tag_ids: [1], support_tag_id: 1')
    with pytest.raises(ConfigurationError, match='suporte na própria pilha'):
        load_plan(_write(tmp_path, 'plan.yaml', source))


def test_arena_remains_v1(tmp_path):
    with pytest.raises(ConfigurationError, match='schema_version: 1'):
        load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA.replace('schema_version: 1', 'schema_version: 2')))


def test_empty_navigation_visit_and_finish_default(tmp_path):
    source = 'schema_version: 2\nplan_id: route\nvisits: [{id: route, target: ws_1}]\n'
    plan = load_plan(_write(tmp_path, 'plan.yaml', source))
    validate_plan(plan, load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA)))
    assert not plan.finish and plan.initial_location == 'start'


def test_generated_ids_include_location_and_normalized_parameters(tmp_path):
    source = '''schema_version: 2
plan_id: generated
visits:
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 2}
      - {action: place_in_container, tag_id: 2, container_color: RED}
      - {action: stack, tag_ids: [5, 4], support_tag_id: 14}
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 2}
'''
    path = _write(tmp_path, 'plan.yaml', source)
    plan = load_plan(path)
    assert plan == load_plan(path)
    assert [v.visit_id for v in plan.visits] == ['visit_ws_1', 'visit_ws_1_2']
    assert [t.step_id for t in plan.visits[0].tasks] == [
        'visit_ws_1_pick_2', 'visit_ws_1_place_in_container_2_red',
        'visit_ws_1_stack_4_5_on_14']
    assert plan.visits[1].tasks[0].step_id == 'visit_ws_1_2_pick_2'


def test_generated_ids_reserve_explicit_ids_and_distinguish_repeated_tasks(tmp_path):
    source = '''schema_version: 2
plan_id: generated
visits:
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 2}
      - {action: place_on_table, tag_id: 2}
      - {action: pick, tag_id: 2}
      - {action: place_on_table, tag_id: 2}
  - id: visit_ws_1
    target: ws_1
    tasks: []
'''
    plan = load_plan(_write(tmp_path, 'plan.yaml', source))
    validate_plan(plan, load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA)))
    assert plan.visits[0].visit_id == 'visit_ws_1_2'
    assert [t.step_id for t in plan.visits[0].tasks] == [
        'visit_ws_1_2_pick_2', 'visit_ws_1_2_place_on_table_2',
        'visit_ws_1_2_pick_2_2', 'visit_ws_1_2_place_on_table_2_2']
    assert plan.visits[1].visit_id == 'visit_ws_1'


def test_explicit_task_id_is_reserved_before_generating_previous_task(tmp_path):
    source = '''schema_version: 2
plan_id: generated
visits:
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 1}
      - {id: visit_ws_1_pick_1, action: place_on_table, tag_id: 1}
'''
    plan = load_plan(_write(tmp_path, 'plan.yaml', source))
    assert [t.step_id for t in plan.visits[0].tasks] == [
        'visit_ws_1_pick_1_2', 'visit_ws_1_pick_1']


def test_inconsistent_simples_plan_is_not_installed():
    assert not (PACKAGE / 'config' / 'plans' / 'simples.yaml').exists()


def test_advanced_transportation_v2_preserves_active_route_and_deliveries():
    plan = load_plan(PACKAGE / 'config/plans/advanced_transportation_test_i.yaml')
    validate_plan(plan, load_arena(PACKAGE / 'config/arena.yaml'))
    assert [visit.target for visit in plan.visits] == [
        'ws_3', 'ws_2', 'ws_1', 'ws_5', 'ws_6', 'sh_1', 'ws_1']
    assert [(task.tag_id, task.container_color) for task in plan.visits[1].tasks] == [
        (2, 'red'), (1, 'blue')]
    stack, = plan.visits[3].tasks
    assert stack.action == 'stack' and stack.support_tag_id == 14 and stack.tag_ids == (4, 5)
    assert [(task.action, task.tag_id) for task in plan.visits[5].tasks] == [
        ('place_on_shelf', 6), ('pick', 3)]
    assert plan.visits[6].tasks[0].tag_id == 3
    assert plan.finish and plan.total_steps == 20


@pytest.mark.parametrize('positions,distance', [
    ([0, 125, -125], 60),
    ([0, 0], 60),
    ([0, 375], 60),
    ([0], 20),
    ([0], True),
    ('invalid', 60),
])
def test_safety_search_configuration_limits(tmp_path, positions, distance):
    import yaml
    raw = yaml.safe_load(VALID_ARENA)
    raw['pickup_recovery'].update({
        'safety_search_positions_mm': positions,
        'safety_search_distance_mm': distance,
    })
    path = _write(tmp_path, 'arena.yaml', yaml.safe_dump(raw))
    if positions == [0, 125, -125]:
        config = load_arena(path).pickup_recovery
        assert config.safety_search_positions_mm == (0, 125, -125)
        assert config.safety_search_distance_mm == 60
    else:
        with pytest.raises(ConfigurationError, match='safety_search'):
            load_arena(path)


@pytest.mark.parametrize('reference', [42, -1, True, '42', None])
def test_precision_plan_reference_validation(tmp_path, reference):
    import yaml
    raw = {'schema_version': 2, 'plan_id': 'pp', 'visits': [
        {'target': 'ws_1', 'tasks': [{'action': 'pick', 'tag_id': 5}]},
        {'target': 'ws_3', 'tasks': [{'action': 'place_on_precision_table',
                                    'tag_id': 5, 'reference_tag_id': reference}]},
    ]}
    path = _write(tmp_path, 'plan.yaml', yaml.safe_dump(raw))
    if reference != 42 or isinstance(reference, bool):
        with pytest.raises(ConfigurationError, match='reference_tag_id'):
            load_plan(path)
        return
    from dataclasses import replace
    from mission_manager.scheduler import Scheduler
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan = load_plan(path)
    with pytest.raises(ConfigurationError, match='área PP'):
        validate_plan(plan, arena)
    arena.service_areas['ws_3'] = replace(arena.service_areas['ws_3'], area_type='PP')
    validate_plan(plan, arena)
    task = plan.visits[1].tasks[0]
    assert task.reference_tag_id == 42
    scheduler = Scheduler(plan, ('left', 'right'))
    assert 42 not in scheduler.tags  # Reference belongs to the table, not cargo.
    assert scheduler._rank(task, {('tag', 42): 300}, 100) == (1, 200)
    assert scheduler._rank(task, {('tag', 5): 300}, 100) is None
