"""Strict YAML loaders for static arena geometry and sequential plans."""

from __future__ import annotations

import math
from pathlib import Path
import re
from typing import Any

import yaml

from .errors import ConfigurationError
from .models import (
    Arena,
    AlignmentConfig,
    DepartureConfig,
    MapPose,
    Plan,
    PickupRecoveryConfig,
    SERVICE_AREA_TYPES,
    STEP_ACTIONS,
    ServiceArea,
    Step,
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


def _load_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(
            f"Não foi possível carregar '{source}': {error}"
        ) from error
    root = _mapping(data, str(source))
    if root.get('schema_version') != 1:
        raise ConfigurationError(f"'{source}' deve usar schema_version: 1.")
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
    return DepartureConfig(*_distance_config_values(raw_value, context, defaults))


def _pickup_recovery(raw_value: Any, context: str) -> PickupRecoveryConfig:
    raw = _mapping(raw_value, context)
    _only_keys(
        raw,
        {
            'enabled', 'minimum_wall_distance_mm',
            'maximum_wall_distance_mm', 'preferred_tag_x_m',
            'preferred_tag_y_m', 'wall_tolerance_mm',
            'travel_tolerance_mm', 'timeout_s',
            'max_reposition_attempts',
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
    if minimum == 0:
        raise ConfigurationError(
            f'{context}.minimum_wall_distance_mm deve ser positivo.'
        )
    if maximum < minimum:
        raise ConfigurationError(
            f'{context}.maximum_wall_distance_mm deve ser maior ou igual '
            'ao mínimo.'
        )
    if wall_tolerance == 0 or travel_tolerance == 0:
        raise ConfigurationError(f'{context}.tolerâncias devem ser positivas.')
    return PickupRecoveryConfig(
        enabled=_boolean(raw.get('enabled'), f'{context}.enabled'),
        minimum_wall_distance_mm=minimum,
        maximum_wall_distance_mm=maximum,
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
    )


def load_arena(path: str | Path) -> Arena:
    """Load calibrated map targets and merge per-area distance overrides."""
    root = _load_yaml(path)
    _only_keys(
        root,
        {
            'schema_version', 'frame_id', 'alignment_defaults',
            'departure_defaults', 'pickup_recovery',
            'start', 'finish', 'service_areas',
        },
        'arena',
    )
    frame_id = _nonempty_string(root.get('frame_id'), 'arena.frame_id')
    defaults = _alignment(
        root.get('alignment_defaults'), 'arena.alignment_defaults'
    )
    departure_defaults = _departure(
        root.get('departure_defaults'), 'arena.departure_defaults'
    )
    pickup_recovery = _pickup_recovery(
        root.get('pickup_recovery'), 'arena.pickup_recovery'
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
    )


_STEP_FIELDS = {
    'navigate': {'id', 'action', 'target'},
    'pick': {'id', 'action', 'tag_id'},
    'store': {'id', 'action', 'slot_id'},
    'retrieve': {'id', 'action', 'slot_id'},
    'place_on_table': {
        'id', 'action', 'analyze_apriltags', 'analyze_containers',
    },
    'place_in_container': {'id', 'action', 'container_color'},
    'stack': {'id', 'action', 'support_tag_id'},
    'place_on_shelf': {'id', 'action'},
    'finish': {'id', 'action'},
}


def _step(raw_value: Any, index: int) -> Step:
    context = f'plan.steps[{index}]'
    raw = _mapping(raw_value, context)
    action = _nonempty_string(raw.get('action'), f'{context}.action')
    if action not in STEP_ACTIONS:
        raise ConfigurationError(
            f'{context}.action desconhecida: {action!r}.'
        )
    _only_keys(raw, _STEP_FIELDS[action], context)
    step_id = _nonempty_string(
        raw.get('id', f'step_{index + 1:03d}'), f'{context}.id'
    )
    if not PLAN_ID_PATTERN.fullmatch(step_id):
        raise ConfigurationError(
            f'{context}.id deve conter apenas letras, números, _ ou -.'
        )

    if action == 'navigate':
        return Step(
            step_id, action,
            target=_nonempty_string(raw.get('target'), f'{context}.target'),
        )
    if action == 'pick':
        return Step(
            step_id, action,
            tag_id=_integer(raw.get('tag_id'), f'{context}.tag_id', nonnegative=True),
        )
    if action in {'store', 'retrieve'}:
        return Step(
            step_id, action,
            slot_id=_nonempty_string(raw.get('slot_id'), f'{context}.slot_id'),
        )
    if action == 'place_on_table':
        return Step(
            step_id,
            action,
            analyze_apriltags=_boolean(
                raw.get('analyze_apriltags', False),
                f'{context}.analyze_apriltags',
            ),
            analyze_containers=_boolean(
                raw.get('analyze_containers', False),
                f'{context}.analyze_containers',
            ),
        )
    if action == 'place_in_container':
        color = _nonempty_string(
            raw.get('container_color'), f'{context}.container_color'
        ).lower()
        if color not in {'red', 'blue'}:
            raise ConfigurationError(
                f'{context}.container_color deve ser red ou blue.'
            )
        return Step(step_id, action, container_color=color)
    if action == 'stack':
        return Step(
            step_id,
            action,
            support_tag_id=_integer(
                raw.get('support_tag_id'),
                f'{context}.support_tag_id',
                nonnegative=True,
            ),
        )
    return Step(step_id, action)


def load_plan(path: str | Path) -> Plan:
    """Load a sequential plan without embedding execution behavior in YAML."""
    root = _load_yaml(path)
    _only_keys(root, {'schema_version', 'plan_id', 'steps'}, 'plan')
    plan_id = _nonempty_string(root.get('plan_id'), 'plan.plan_id')
    if not PLAN_ID_PATTERN.fullmatch(plan_id):
        raise ConfigurationError(
            'plan.plan_id deve conter apenas letras, números, _ ou -.'
        )
    raw_steps = root.get('steps')
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ConfigurationError('plan.steps deve ser uma lista não vazia.')
    steps = tuple(_step(value, index) for index, value in enumerate(raw_steps))
    ids = [step.step_id for step in steps]
    if len(ids) != len(set(ids)):
        raise ConfigurationError('plan.steps contém IDs duplicados.')
    return Plan(plan_id=plan_id, steps=steps)


def validate_plan(plan: Plan, arena: Arena) -> None:
    """Validate static references without duplicating manipulation inventory."""
    current_location = 'start'
    for index, step in enumerate(plan.steps):
        if step.action == 'navigate':
            assert step.target is not None
            if not arena.has_target(step.target):
                raise ConfigurationError(
                    f"Passo '{step.step_id}' referencia target desconhecido: "
                    f"'{step.target}'."
                )
            current_location = step.target
            continue
        if step.action == 'finish':
            if index != len(plan.steps) - 1:
                raise ConfigurationError(
                    f"Passo '{step.step_id}': finish deve ser o último passo."
                )
            current_location = 'finish'
            continue
        if current_location not in arena.service_areas:
            raise ConfigurationError(
                f"Passo '{step.step_id}' executa manipulação fora de uma "
                'área de serviço.'
            )
