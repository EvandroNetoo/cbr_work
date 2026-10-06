"""Strict YAML loaders for static arena geometry and visit-based plans."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
import re
from typing import Any, Callable

import yaml

from .errors import ConfigurationError
from .models import (
    AlignmentConfig,
    Arena,
    DepartureConfig,
    MapPose,
    PickupRecoveryConfig,
    Plan,
    SERVICE_AREA_TYPES,
    ServiceArea,
    Step,
    Visit,
)


PLAN_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]*$')


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigurationError(f'{context} deve ser um mapa YAML.')
    return value


def _only_keys(value: dict[str, Any], allowed: set[str], context: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ConfigurationError(
            f'{context} contém campos desconhecidos: {unknown}.'
        )


def _number(value: Any, context: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f'{context} deve ser numérico.')
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positivo e finito' if positive else 'finito'
        raise ConfigurationError(f'{context} deve ser {qualifier}.')
    return result


def _integer(value: Any, context: str, *, nonnegative: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f'{context} deve ser inteiro.')
    if nonnegative and value < 0:
        raise ConfigurationError(f'{context} não pode ser negativo.')
    return value


def _boolean(value: Any, context: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigurationError(f'{context} deve ser booleano.')
    return value


def _nonempty_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f'{context} deve ser texto não vazio.')
    return value.strip()


def _load_yaml(path: str | Path, schema_version: int = 1) -> dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(
            f"Não foi possível carregar '{source}': {error}"
        ) from error
    root = _mapping(data, str(source))
    if type(root.get('schema_version')) is not int or root.get('schema_version') != schema_version:
        hint = ' Migre steps para visits; store/retrieve agora são automáticos.' if schema_version == 2 else ''
        raise ConfigurationError(f"'{source}' deve usar schema_version: {schema_version}.{hint}")
    return root


def _pose(raw_value: Any, context: str) -> MapPose:
    raw = _mapping(raw_value, context)
    _only_keys(raw, {'x_m', 'y_m', 'yaw_rad'}, context)
    return MapPose(
        x_m=_number(raw.get('x_m'), f'{context}.x_m'),
        y_m=_number(raw.get('y_m'), f'{context}.y_m'),
        yaw_rad=_number(raw.get('yaw_rad'), f'{context}.yaw_rad'),
    )


def _distance_config_values(
    raw_value: Any,
    context: str,
    defaults: AlignmentConfig | DepartureConfig | None = None,
) -> tuple[int, int, float]:
    raw = _mapping(raw_value, context)
    _only_keys(raw, {'distance_mm', 'tolerance_mm', 'timeout_s'}, context)

    def selected(name: str) -> Any:
        if name in raw:
            return raw[name]
        if defaults is not None:
            return getattr(defaults, name)
        return None

    distance = _integer(
        selected('distance_mm'), f'{context}.distance_mm', nonnegative=True
    )
    tolerance = _integer(
        selected('tolerance_mm'), f'{context}.tolerance_mm', nonnegative=True
    )
    if distance == 0:
        raise ConfigurationError(f'{context}.distance_mm deve ser positivo.')
    if tolerance == 0:
        raise ConfigurationError(f'{context}.tolerance_mm deve ser positivo.')
    return (
        distance,
        tolerance,
        _number(
            selected('timeout_s'), f'{context}.timeout_s', positive=True
        ),
    )


def _alignment(
    raw_value: Any,
    context: str,
    defaults: AlignmentConfig | None = None,
) -> AlignmentConfig:
    return AlignmentConfig(*_distance_config_values(raw_value, context, defaults))


def _departure(
    raw_value: Any,
    context: str,
    defaults: DepartureConfig | None = None,
) -> DepartureConfig:
    raw = _mapping(raw_value, context)
    _only_keys(
        raw,
        {
            'distance_mm', 'tolerance_mm', 'timeout_s',
            'lateral_position_mm', 'max_alignment_error_mm',
            'alignment_recovery_distance_mm',
            'minimum_lateral_clearance_mm',
        },
        context,
    )
    distance_values = _distance_config_values(
        {
            key: raw[key]
            for key in ('distance_mm', 'tolerance_mm', 'timeout_s')
            if key in raw
        },
        context,
        defaults,
    )
    lateral_position = raw.get(
        'lateral_position_mm',
        defaults.lateral_position_mm if defaults is not None else None,
    )
    max_alignment_error = raw.get(
        'max_alignment_error_mm',
        defaults.max_alignment_error_mm if defaults is not None else None,
    )
    if max_alignment_error is not None:
        max_alignment_error = _integer(
            max_alignment_error,
            f'{context}.max_alignment_error_mm',
            nonnegative=True,
        )
    alignment_recovery_distance = raw.get(
        'alignment_recovery_distance_mm',
        (
            defaults.alignment_recovery_distance_mm
            if defaults is not None else None
        ),
    )
    if alignment_recovery_distance is not None:
        alignment_recovery_distance = _integer(
            alignment_recovery_distance,
            f'{context}.alignment_recovery_distance_mm',
            nonnegative=True,
        )
    minimum_lateral_clearance = raw.get(
        'minimum_lateral_clearance_mm',
        (
            defaults.minimum_lateral_clearance_mm
            if defaults is not None else None
        ),
    )
    if minimum_lateral_clearance is not None:
        minimum_lateral_clearance = _integer(
            minimum_lateral_clearance,
            f'{context}.minimum_lateral_clearance_mm',
            nonnegative=True,
        )
    if (
        alignment_recovery_distance is not None
        and alignment_recovery_distance > 0
        and max_alignment_error == 0
    ):
        raise ConfigurationError(
            f'{context}.alignment_recovery_distance_mm requer '
            'max_alignment_error_mm positivo.'
        )
    return DepartureConfig(
        *distance_values,
        lateral_position_mm=_integer(
            lateral_position, f'{context}.lateral_position_mm'
        ),
        max_alignment_error_mm=max_alignment_error,
        alignment_recovery_distance_mm=alignment_recovery_distance,
        minimum_lateral_clearance_mm=minimum_lateral_clearance,
    )


def _pickup_recovery(raw_value: Any, context: str) -> PickupRecoveryConfig:
    raw = _mapping(raw_value, context)
    _only_keys(
        raw,
        {
            'enabled', 'minimum_wall_distance_mm',
            'maximum_wall_distance_mm', 'minimum_lateral_position_mm',
            'maximum_lateral_position_mm', 'preferred_tag_x_m',
            'preferred_tag_y_m', 'wall_tolerance_mm', 'travel_tolerance_mm',
            'timeout_s', 'max_reposition_attempts', 'search_positions_mm',
            'safety_search_distance_mm', 'safety_search_positions_mm',
            'shelf_preferred_tag_x_m', 'shelf_preferred_tag_y_m',
            'stack_preferred_tag_x_m', 'stack_preferred_tag_y_m',
        },
        context,
    )
    minimum = _integer(
        raw.get('minimum_wall_distance_mm'),
        f'{context}.minimum_wall_distance_mm',
        nonnegative=True,
    )
    maximum = _integer(
        raw.get('maximum_wall_distance_mm'),
        f'{context}.maximum_wall_distance_mm',
        nonnegative=True,
    )
    minimum_lateral = _integer(
        raw.get('minimum_lateral_position_mm'),
        f'{context}.minimum_lateral_position_mm',
    )
    maximum_lateral = _integer(
        raw.get('maximum_lateral_position_mm'),
        f'{context}.maximum_lateral_position_mm',
    )
    wall_tolerance = _integer(
        raw.get('wall_tolerance_mm'),
        f'{context}.wall_tolerance_mm',
        nonnegative=True,
    )
    travel_tolerance = _integer(
        raw.get('travel_tolerance_mm'),
        f'{context}.travel_tolerance_mm',
        nonnegative=True,
    )
    attempts = _integer(
        raw.get('max_reposition_attempts'),
        f'{context}.max_reposition_attempts',
        nonnegative=True,
    )
    search_raw = raw.get('search_positions_mm', [0, 250, -250])
    if not isinstance(search_raw, list) or not search_raw:
        raise ConfigurationError(
            f'{context}.search_positions_mm deve ser uma lista não vazia.'
        )
    search_positions = tuple(
        _integer(value, f'{context}.search_positions_mm[{index}]')
        for index, value in enumerate(search_raw)
    )
    if len(set(search_positions)) != len(search_positions):
        raise ConfigurationError(
            f'{context}.search_positions_mm não pode conter posições repetidas.'
        )
    if 0 not in search_positions:
        raise ConfigurationError(
            f'{context}.search_positions_mm deve conter a origem 0.'
        )
    if minimum == 0:
        raise ConfigurationError(
            f'{context}.minimum_wall_distance_mm deve ser positivo.'
        )
    if maximum < minimum:
        raise ConfigurationError(
            f'{context}.maximum_wall_distance_mm deve ser maior ou igual '
            'ao mínimo.'
        )
    if maximum_lateral < minimum_lateral:
        raise ConfigurationError(
            f'{context}.maximum_lateral_position_mm deve ser maior ou igual '
            'ao mínimo lateral.'
        )
    outside_lateral_limits = [
        position for position in search_positions
        if not minimum_lateral <= position <= maximum_lateral
    ]
    if outside_lateral_limits:
        raise ConfigurationError(
            f'{context}.search_positions_mm contém posições fora dos limites '
            f'laterais [{minimum_lateral}, {maximum_lateral}]: '
            f'{outside_lateral_limits}.'
        )
    if wall_tolerance == 0 or travel_tolerance == 0:
        raise ConfigurationError(f'{context}.tolerâncias devem ser positivas.')
    safety_distance = _integer(raw.get('safety_search_distance_mm', 60),
                               f'{context}.safety_search_distance_mm')
    safety_raw = raw.get('safety_search_positions_mm', [])
    if not isinstance(safety_raw, list):
        raise ConfigurationError(f'{context}.safety_search_positions_mm deve ser uma lista.')
    safety_positions = tuple(_integer(value, f'{context}.safety_search_positions_mm')
                             for value in safety_raw)
    if len(set(safety_positions)) != len(safety_positions):
        raise ConfigurationError(f'{context}.safety_search_positions_mm contém repetições.')
    if safety_positions and not minimum <= safety_distance <= maximum:
        raise ConfigurationError(f'{context}.safety_search_distance_mm fora dos limites de parede.')
    if any(not minimum_lateral <= value <= maximum_lateral for value in safety_positions):
        raise ConfigurationError(f'{context}.safety_search_positions_mm fora dos limites laterais.')
    return PickupRecoveryConfig(
        safety_search_distance_mm=safety_distance,
        safety_search_positions_mm=safety_positions,
        stack_preferred_tag_x_m=_number(
            raw.get('stack_preferred_tag_x_m', 0.0),
            f'{context}.stack_preferred_tag_x_m'),
        stack_preferred_tag_y_m=_number(
            raw.get('stack_preferred_tag_y_m', -0.22),
            f'{context}.stack_preferred_tag_y_m'),
        shelf_preferred_tag_x_m=_number(
            raw.get('shelf_preferred_tag_x_m', 0.0),
            f'{context}.shelf_preferred_tag_x_m'),
        shelf_preferred_tag_y_m=_number(
            raw.get('shelf_preferred_tag_y_m', -0.22),
            f'{context}.shelf_preferred_tag_y_m'),
        enabled=_boolean(raw.get('enabled'), f'{context}.enabled'),
        minimum_wall_distance_mm=minimum,
        maximum_wall_distance_mm=maximum,
        minimum_lateral_position_mm=minimum_lateral,
        maximum_lateral_position_mm=maximum_lateral,
        preferred_tag_x_m=_number(
            raw.get('preferred_tag_x_m'), f'{context}.preferred_tag_x_m'
        ),
        preferred_tag_y_m=_number(
            raw.get('preferred_tag_y_m'), f'{context}.preferred_tag_y_m'
        ),
        wall_tolerance_mm=wall_tolerance,
        travel_tolerance_mm=travel_tolerance,
        timeout_s=_number(
            raw.get('timeout_s'), f'{context}.timeout_s', positive=True
        ),
        max_reposition_attempts=attempts,
        search_positions_mm=search_positions,
    )


def load_arena(path: str | Path) -> Arena:
    """Load calibrated map targets and merge per-area distance overrides."""
    root = _load_yaml(path)
    _only_keys(
        root,
        {
            'schema_version', 'frame_id', 'alignment_defaults',
            'departure_defaults', 'pickup_recovery',
            'table_place_search_positions_mm',
            'start', 'finish', 'service_areas',
            'shelf_place_alignment_defaults',
        },
        'arena',
    )
    frame_id = _nonempty_string(root.get('frame_id'), 'arena.frame_id')
    defaults = _alignment(
        root.get('alignment_defaults'), 'arena.alignment_defaults'
    )
    shelf_place_defaults = _alignment(
        root.get('shelf_place_alignment_defaults', {}),
        'arena.shelf_place_alignment_defaults', AlignmentConfig(40, 5, 10.0))
    departure_defaults = _departure(
        root.get('departure_defaults'), 'arena.departure_defaults'
    )
    pickup_recovery = _pickup_recovery(
        root.get('pickup_recovery'), 'arena.pickup_recovery'
    )
    table_positions_raw = root.get('table_place_search_positions_mm')
    table_positions = None
    if table_positions_raw is not None:
        context = 'arena.table_place_search_positions_mm'
        if not isinstance(table_positions_raw, list) or not table_positions_raw:
            raise ConfigurationError(f'{context} deve ser uma lista não vazia.')
        table_positions = tuple(
            _integer(value, f'{context}[{index}]')
            for index, value in enumerate(table_positions_raw)
        )
        if len(set(table_positions)) != len(table_positions):
            raise ConfigurationError(f'{context} não pode conter posições repetidas.')
        if 0 not in table_positions:
            raise ConfigurationError(f'{context} deve conter a origem 0.')
        outside = [
            position for position in table_positions
            if not pickup_recovery.minimum_lateral_position_mm <= position
            <= pickup_recovery.maximum_lateral_position_mm
        ]
        if outside:
            raise ConfigurationError(
                f'{context} contém posições fora dos limites laterais: {outside}.'
            )
    areas_raw = _mapping(root.get('service_areas'), 'arena.service_areas')
    areas: dict[str, ServiceArea] = {}
    for area_id, value in areas_raw.items():
        area_name = _nonempty_string(area_id, 'service area id')
        if area_name in {'start', 'finish'}:
            raise ConfigurationError(
                f"A área de serviço não pode se chamar '{area_name}'."
            )
        raw = _mapping(value, f'arena.service_areas.{area_name}')
        _only_keys(
            raw,
            {
                'x_m', 'y_m', 'yaw_rad', 'height_cm', 'type',
                'alignment', 'departure',
                'shelf_place_alignment',
            },
            f'arena.service_areas.{area_name}',
        )
        area_type = _nonempty_string(
            raw.get('type'), f'arena.service_areas.{area_name}.type'
        ).upper()
        if area_type not in SERVICE_AREA_TYPES:
            raise ConfigurationError(
                f"arena.service_areas.{area_name}.type deve ser WS, SH ou PP."
            )
        if 'shelf_place_alignment' in raw and area_type != 'SH':
            raise ConfigurationError(
                f'arena.service_areas.{area_name}.shelf_place_alignment exige type: SH.')
        pose = _pose(
            {key: raw.get(key) for key in ('x_m', 'y_m', 'yaw_rad')},
            f'arena.service_areas.{area_name}',
        )
        areas[area_name] = ServiceArea(
            area_id=area_name,
            pose=pose,
            height_cm=_number(
                raw.get('height_cm'),
                f'arena.service_areas.{area_name}.height_cm',
            ),
            area_type=area_type,
            shelf_place_alignment=(
                _alignment(
                    raw.get('shelf_place_alignment', {}),
                    f'arena.service_areas.{area_name}.shelf_place_alignment',
                    shelf_place_defaults)
                if area_type == 'SH' else None),
            alignment=_alignment(
                raw.get('alignment', {}),
                f'arena.service_areas.{area_name}.alignment',
                defaults,
            ),
            departure=_departure(
                raw.get('departure', {}),
                f'arena.service_areas.{area_name}.departure',
                departure_defaults,
            ),
        )
    return Arena(
        frame_id=frame_id,
        start=_pose(root.get('start'), 'arena.start'),
        finish=_pose(root.get('finish'), 'arena.finish'),
        alignment_defaults=defaults,
        departure_defaults=departure_defaults,
        pickup_recovery=pickup_recovery,
        service_areas=areas,
        shelf_place_alignment_defaults=shelf_place_defaults,
        table_place_search_positions_mm=table_positions,
    )


_TASK_FIELDS = {
    'pick': {'id', 'action', 'tag_id'},
    'place_on_table': {'id', 'action', 'tag_id'},
    'place_in_container': {'id', 'action', 'tag_id', 'container_color'},
    'stack': {'id', 'action', 'tag_ids', 'support_tag_id'},
    'place_on_shelf': {'id', 'action', 'tag_id'},
}


def _identifier(raw: Any, context: str) -> str:
    value = _nonempty_string(raw, context)
    if not PLAN_ID_PATTERN.fullmatch(value):
        raise ConfigurationError(f'{context} deve conter apenas letras, números, _ ou -.')
    return value


def _task(raw_value: Any, context: str) -> Step:
    raw = _mapping(raw_value, context)
    action = _nonempty_string(raw.get('action'), f'{context}.action')
    if action not in _TASK_FIELDS:
        raise ConfigurationError(f'{context}.action desconhecida: {action!r}.')
    _only_keys(raw, _TASK_FIELDS[action], context)
    task_id = _identifier(raw['id'], f'{context}.id') if 'id' in raw else ''
    if action == 'stack':
        values = raw.get('tag_ids')
        if not isinstance(values, list) or not values:
            raise ConfigurationError(f'{context}.tag_ids deve ser uma lista não vazia.')
        tags = tuple(_integer(value, f'{context}.tag_ids', nonnegative=True) for value in values)
        support = _integer(raw.get('support_tag_id'), f'{context}.support_tag_id', nonnegative=True)
        if len(set(tags)) != len(tags) or support in tags:
            raise ConfigurationError(f'{context}: tags duplicadas ou suporte na própria pilha.')
        return Step(task_id, action, support_tag_id=support, tag_ids=tags)
    tag = _integer(raw.get('tag_id'), f'{context}.tag_id', nonnegative=True)
    color = None
    if action == 'place_in_container':
        color = _nonempty_string(raw.get('container_color'), f'{context}.container_color').lower()
        if color not in {'red', 'blue'}:
            raise ConfigurationError(f'{context}.container_color deve ser red ou blue.')
    return Step(task_id, action, tag_id=tag, container_color=color)


def load_plan(path: str | Path) -> Plan:
    root = _load_yaml(path, schema_version=2)
    _only_keys(root, {'schema_version', 'plan_id', 'initial_location', 'visits', 'finish'}, 'plan')
    plan_id = _identifier(root.get('plan_id'), 'plan.plan_id')
    raw_visits = root.get('visits')
    if not isinstance(raw_visits, list) or not raw_visits:
        raise ConfigurationError('plan.visits deve ser uma lista não vazia.')
    visits = []
    # Reserve explicit IDs first so generated IDs cannot collide with later
    # declarations. Explicit IDs remain supported for existing plans.
    ids = set()
    for index, value in enumerate(raw_visits):
        context = f'plan.visits[{index}]'
        raw = _mapping(value, context)
        tasks = raw.get('tasks', [])
        if not isinstance(tasks, list):
            raise ConfigurationError(f'{context}.tasks deve ser uma lista.')
        entries = [(raw, context)] + [(_mapping(task, f'{context}.tasks[{i}]'),
                                      f'{context}.tasks[{i}]') for i, task in enumerate(tasks)]
        for entry, entry_context in entries:
            if 'id' in entry:
                identifier = _identifier(entry['id'], f'{entry_context}.id')
                if identifier in ids:
                    raise ConfigurationError(f'plan contém IDs duplicados: {identifier}.')
                ids.add(identifier)

    def allocate(base: str) -> str:
        identifier = base
        suffix = 2
        while identifier in ids:
            identifier = f'{base}_{suffix}'
            suffix += 1
        ids.add(identifier)
        return identifier

    for index, value in enumerate(raw_visits):
        context = f'plan.visits[{index}]'
        raw = _mapping(value, context)
        _only_keys(raw, {'id', 'target', 'tasks'}, context)
        target = _nonempty_string(raw.get('target'), f'{context}.target')
        visit_id = (_identifier(raw['id'], f'{context}.id') if 'id' in raw else
                    allocate('visit_' + re.sub(r'[^A-Za-z0-9_-]', '_', target)))
        values = raw.get('tasks', [])
        if not isinstance(values, list):
            raise ConfigurationError(f'{context}.tasks deve ser uma lista.')
        tasks = []
        for i, value in enumerate(values):
            task = _task(value, f'{context}.tasks[{i}]')
            if not task.step_id:
                parameters = (f'{"_".join(str(tag) for tag in sorted(task.tag_ids))}'
                              f'_on_{task.support_tag_id}' if task.action == 'stack'
                              else str(task.tag_id))
                if task.container_color:
                    parameters += f'_{task.container_color}'
                task = replace(task, step_id=allocate(f'{visit_id}_{task.action}_{parameters}'))
            tasks.append(task)
        visits.append(Visit(visit_id, target, tuple(tasks)))
    return Plan(plan_id, tuple(visits),
                _nonempty_string(root.get('initial_location', 'start'), 'plan.initial_location'),
                _boolean(root.get('finish', False), 'plan.finish'))


def validate_plan(plan: Plan, arena: Arena, cargo_slot_ids=('left', 'right'),
                  check_canceled: Callable[[], None] = lambda: None) -> None:
    if not arena.has_target(plan.initial_location):
        raise ConfigurationError('plan.initial_location referencia target desconhecido.')
    for visit in plan.visits:
        if not arena.has_target(visit.target):
            raise ConfigurationError(f"Visita '{visit.visit_id}': target desconhecido '{visit.target}'.")
        if visit.tasks and visit.target not in arena.service_areas:
            raise ConfigurationError(f"Visita '{visit.visit_id}': manipulação fora de uma área de serviço.")
    from .scheduler import Scheduler
    scheduler = Scheduler(plan, tuple(cargo_slot_ids), check_canceled)
    if not scheduler.feasible(scheduler.initial_state):
        raise ConfigurationError(
            'Missão inviável: verifique coletas, entregas, suportes, capacidade e garra na próxima visita.')
