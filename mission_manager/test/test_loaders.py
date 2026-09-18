from pathlib import Path

import pytest

from mission_manager.errors import ConfigurationError
from mission_manager.loaders import load_arena, load_plan, validate_plan


PACKAGE = Path(__file__).parents[1]


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


def test_package_arena_refuses_uncalibrated_poses():
    with pytest.raises(ConfigurationError, match='deve ser numérico'):
        load_arena(PACKAGE / 'config' / 'arena.yaml')


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


def test_example_plan_loads_and_validates_with_calibrated_arena(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan = load_plan(PACKAGE / 'config' / 'plans' / 'example_transport.yaml')

    validate_plan(plan, arena)
    assert plan.plan_id == 'example_transport'
    assert plan.steps[0].action == 'navigate'
    assert plan.steps[-1].action == 'finish'


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_plan_accepts_container_deposit_by_color(tmp_path, color):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan = load_plan(_write(tmp_path, 'container.yaml', f'''
schema_version: 1
plan_id: container
initial_location: ws_1
steps:
  - id: depositar
    action: place_in_container
    container_color: {color}
'''))

    validate_plan(plan, arena)
    assert plan.steps[0].action == 'place_in_container'
    assert plan.steps[0].container_color == color


def test_plan_rejects_unknown_container_color(tmp_path):
    plan_path = _write(tmp_path, 'container.yaml', '''
schema_version: 1
plan_id: container
initial_location: ws_1
steps:
  - id: depositar
    action: place_in_container
    container_color: green
''')

    with pytest.raises(ConfigurationError, match='red ou blue'):
        load_plan(plan_path)


def test_plan_rejects_unknown_target(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan_path = _write(
        tmp_path,
        'bad.yaml',
        """
schema_version: 1
plan_id: bad
steps:
  - {action: navigate, target: missing}
""",
    )

    with pytest.raises(ConfigurationError, match='target desconhecido'):
        validate_plan(load_plan(plan_path), arena)


def test_plan_rejects_manipulation_before_service_area(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan_path = _write(
        tmp_path,
        'bad.yaml',
        """
schema_version: 1
plan_id: bad
steps:
  - {action: pick, tag_id: 1}
""",
    )

    with pytest.raises(ConfigurationError, match='fora de uma área de serviço'):
        validate_plan(load_plan(plan_path), arena)


def test_plan_allows_manipulation_at_declared_initial_location(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan_path = _write(
        tmp_path,
        'positioned.yaml',
        """
schema_version: 1
plan_id: positioned
initial_location: ws_1
steps:
  - {action: pick, tag_id: 1}
""",
    )

    plan = load_plan(plan_path)
    validate_plan(plan, arena)

    assert plan.initial_location == 'ws_1'
    assert plan.steps[0].action == 'pick'


def test_plan_rejects_unknown_initial_location(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan_path = _write(
        tmp_path,
        'positioned.yaml',
        """
schema_version: 1
plan_id: positioned
initial_location: missing
steps:
  - {action: pick, tag_id: 1}
""",
    )

    with pytest.raises(ConfigurationError, match='initial_location'):
        validate_plan(load_plan(plan_path), arena)


def test_finish_must_be_last_step(tmp_path):
    arena = load_arena(_write(tmp_path, 'arena.yaml', VALID_ARENA))
    plan_path = _write(
        tmp_path,
        'bad.yaml',
        """
schema_version: 1
plan_id: bad
steps:
  - {action: finish}
  - {action: navigate, target: ws_1}
""",
    )

    with pytest.raises(ConfigurationError, match='último passo'):
        validate_plan(load_plan(plan_path), arena)


def test_plan_rejects_action_specific_extra_fields(tmp_path):
    plan_path = _write(
        tmp_path,
        'bad.yaml',
        """
schema_version: 1
plan_id: bad
steps:
  - {action: store, slot_id: left, tag_id: 5}
""",
    )

    with pytest.raises(ConfigurationError, match='campos desconhecidos'):
        load_plan(plan_path)
