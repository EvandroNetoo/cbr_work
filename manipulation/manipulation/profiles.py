"""Strict loaders for manipulation and on-board cargo profiles."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .errors import ConfigurationError
from .base_wiggle import BaseWiggleProfile


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigurationError(f"{context} deve ser um mapa YAML.")
    return value


def _only_keys(value: dict[str, Any], allowed: set[str], context: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ConfigurationError(f'{context} contém campos desconhecidos: {unknown}.')


def _number(value: Any, context: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{context} deve ser numérico.")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positivo e finito' if positive else 'finito'
        raise ConfigurationError(f"{context} deve ser {qualifier}.")
    return result


@dataclass(frozen=True)
class PickupProfile:
    name: str
    observation_state: str
    approach_height_m: float
    cube_size_m: float
    yaw_offset_deg: float
    attempts: int
    strategy: str = 'top'
    pre_grasp_state: str = ''
    grasp_z_offset_m: float = 0.0
    grasp_y_offset_m: float = 0.0
    link3_to_link4_deg: float = 90.0
    link4_to_link5_deg: float = 0.0
    joint_tolerance_deg: float = 5.0
    reachability_filter_enabled: bool = False
    reach_center_x_m: float = 0.0
    reach_center_y_m: float = 0.0
    reach_min_radius_m: float | None = None
    reach_max_radius_m: float | None = None
    reach_x_min_m: float | None = None
    reach_x_max_m: float | None = None
    reach_y_min_m: float | None = None
    reach_y_max_m: float | None = None


@dataclass(frozen=True)
class PlacementProfile:
    name: str
    strategy: str
    enabled: bool
    named_state: str
    approach_height_m: float
    retreat_height_m: float
    reference_offset_xyz: tuple[float, float, float]
    yaw_offset_deg: float
    calibrated_reference: bool
    tcp_release_offset_cm: float | None = None
    free_space_half_extent_x_m: float = 0.07
    free_space_half_extent_y_m: float = 0.04
    free_space_min_padding_m: float = 0.0
    free_space_preferred_padding_m: float = 0.03
    free_space_preferred_yaw_deg: float | None = None
    free_space_alternate_yaw_deg: float | None = None
    reach_center_x_m: float = 0.0
    reach_center_y_m: float = 0.0
    reach_min_radius_m: float | None = None
    reach_max_radius_m: float | None = None
    search_x_min_m: float | None = None
    search_x_max_m: float | None = None
    search_y_min_m: float | None = None
    search_y_max_m: float | None = None
    search_step_m: float = 0.01
    link3_to_link4_max_deg: float = -10.0
    tilt_tolerance_deg: float | None = None
    base_wiggle: BaseWiggleProfile = BaseWiggleProfile()


@dataclass(frozen=True)
class CargoSlotProfile:
    slot_id: str
    store_state: str
    safe_state: str
    retrieve_state: str


@dataclass(frozen=True)
class ProfileSet:
    pickup: dict[str, PickupProfile]
    placements: dict[str, PlacementProfile]
    cargo_slots: dict[str, CargoSlotProfile]
    transport_empty_state: str
    transport_loaded_state: str

    def pickup_profile(self, name: str) -> PickupProfile:
        selected = name or 'tabletop'
        try:
            return self.pickup[selected]
        except KeyError as error:
            raise ConfigurationError(
                f"Perfil de coleta desconhecido: '{selected}'."
            ) from error


def _load_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(f"Não foi possível carregar '{source}': {error}") from error
    root = _mapping(data, str(source))
    if root.get('schema_version') != 1:
        raise ConfigurationError(f"'{source}' deve usar schema_version: 1.")
    return root


def _base_wiggle(raw_value: Any, context: str) -> BaseWiggleProfile:
    raw = _mapping(raw_value, context)
    defaults = BaseWiggleProfile()
    # Older YAMLs may still supply feedback settings; they are ignored.
    legacy = {'max_yaw_speed_rad_s', 'position_gain', 'yaw_gain',
              'position_tolerance_m', 'yaw_tolerance_rad', 'tracking_tolerance_m',
              'odom_timeout_s', 'return_timeout_s'}
    _only_keys(raw, set(defaults.__dataclass_fields__) | legacy, context)
    enabled = raw.get('enabled', False)
    cycles = raw.get('cycles', defaults.cycles)
    if not isinstance(enabled, bool):
        raise ConfigurationError(f'{context}.enabled deve ser booleano.')
    if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles <= 0:
        raise ConfigurationError(f'{context}.cycles deve ser inteiro positivo.')
    values = {'enabled': enabled, 'cycles': cycles}
    for field in defaults.__dataclass_fields__:
        if field not in values and field != 'sequence':
            values[field] = _number(raw.get(field, getattr(defaults, field)),
                                    f'{context}.{field}', positive=field in {'period_s', 'rate_hz'})
    for field in ('radius_m', 'max_speed_m_s', 'settle_s'):
        if values[field] < 0:
            raise ConfigurationError(f'{context}.{field} deve ser maior ou igual a zero.')
    if 'sequence' in raw:
        sequence = raw['sequence']
        if not isinstance(sequence, list) or not sequence:
            raise ConfigurationError(f'{context}.sequence deve ser uma lista não vazia.')
        stages = []
        for index, entry in enumerate(sequence):
            stage_context = f'{context}.sequence[{index}]'
            stage = _mapping(entry, stage_context)
            _only_keys(stage, (set(defaults.__dataclass_fields__) - {'enabled', 'sequence'}) | legacy,
                       stage_context)
            # Top-level timings/speed provide defaults; each stage can override them.
            stages.append(_base_wiggle({**values, **stage}, stage_context))
        values['sequence'] = tuple(stages)
    return BaseWiggleProfile(**values)


def load_profiles(profiles_path: str | Path, cargo_path: str | Path) -> ProfileSet:
    """Load both configuration files and reject incomplete or ambiguous profiles."""
    root = _load_yaml(profiles_path)
    _only_keys(
        root,
        {'schema_version', 'pickup', 'placements'},
        'profiles',
    )
    pickup_raw = _mapping(root.get('pickup'), 'pickup')
    placements_raw = _mapping(root.get('placements'), 'placements')

    pickup: dict[str, PickupProfile] = {}
    for name, raw_value in pickup_raw.items():
        raw = _mapping(raw_value, f'pickup.{name}')
        _only_keys(
            raw,
            {
                'observation_state', 'approach_height_m', 'cube_size_m',
                'yaw_offset_deg', 'attempts', 'reachability_filter_enabled',
                'reach_center_x_m', 'reach_center_y_m',
                'reach_min_radius_m', 'reach_max_radius_m',
                'reach_x_min_m', 'reach_x_max_m',
                'reach_y_min_m', 'reach_y_max_m',
                'strategy', 'pre_grasp_state',
                'grasp_z_offset_m', 'link3_to_link4_deg', 'joint_tolerance_deg',
                'link4_to_link5_deg',
                'grasp_y_offset_m',
            },
            f'pickup.{name}',
        )
        attempts = int(raw.get('attempts', 1))
        if attempts <= 0:
            raise ConfigurationError(f'pickup.{name}.attempts deve ser positivo.')
        reachability_filter_enabled = raw.get(
            'reachability_filter_enabled', False
        )
        if not isinstance(reachability_filter_enabled, bool):
            raise ConfigurationError(
                f'pickup.{name}.reachability_filter_enabled deve ser booleano.'
            )
        profile = PickupProfile(
            name=name,
            observation_state=str(raw.get('observation_state', '')),
            approach_height_m=_number(
                raw.get('approach_height_m'), f'pickup.{name}.approach_height_m', positive=True
            ),
            cube_size_m=_number(
                raw.get('cube_size_m'), f'pickup.{name}.cube_size_m', positive=True
            ),
            yaw_offset_deg=_number(
                raw.get('yaw_offset_deg', 90.0), f'pickup.{name}.yaw_offset_deg'
            ),
            attempts=attempts,
            strategy=str(raw.get('strategy', 'top')),
            pre_grasp_state=str(raw.get('pre_grasp_state', '')),
            grasp_z_offset_m=_number(
                raw.get('grasp_z_offset_m', 0.0),
                f'pickup.{name}.grasp_z_offset_m'),
            grasp_y_offset_m=_number(
                raw.get('grasp_y_offset_m', 0.0), f'pickup.{name}.grasp_y_offset_m'),
            link3_to_link4_deg=_number(
                raw.get('link3_to_link4_deg', 90.0),
                f'pickup.{name}.link3_to_link4_deg'),
            joint_tolerance_deg=_number(
                raw.get('joint_tolerance_deg', 5.0),
                f'pickup.{name}.joint_tolerance_deg', positive=True),
            link4_to_link5_deg=_number(
                raw.get('link4_to_link5_deg', 0.0),
                f'pickup.{name}.link4_to_link5_deg'),
            reachability_filter_enabled=reachability_filter_enabled,
            reach_center_x_m=_number(
                raw.get('reach_center_x_m', 0.0),
                f'pickup.{name}.reach_center_x_m',
            ),
            reach_center_y_m=_number(
                raw.get('reach_center_y_m', 0.0),
                f'pickup.{name}.reach_center_y_m',
            ),
            reach_min_radius_m=(
                None if raw.get('reach_min_radius_m') is None
                else _number(
                    raw['reach_min_radius_m'],
                    f'pickup.{name}.reach_min_radius_m', positive=True,
                )
            ),
            reach_max_radius_m=(
                None if raw.get('reach_max_radius_m') is None
                else _number(
                    raw['reach_max_radius_m'],
                    f'pickup.{name}.reach_max_radius_m', positive=True,
                )
            ),
            reach_x_min_m=(
                None if raw.get('reach_x_min_m') is None
                else _number(raw['reach_x_min_m'], f'pickup.{name}.reach_x_min_m')
            ),
            reach_x_max_m=(
                None if raw.get('reach_x_max_m') is None
                else _number(raw['reach_x_max_m'], f'pickup.{name}.reach_x_max_m')
            ),
            reach_y_min_m=(
                None if raw.get('reach_y_min_m') is None
                else _number(raw['reach_y_min_m'], f'pickup.{name}.reach_y_min_m')
            ),
            reach_y_max_m=(
                None if raw.get('reach_y_max_m') is None
                else _number(raw['reach_y_max_m'], f'pickup.{name}.reach_y_max_m')
            ),
        )
        pickup[name] = profile
        if profile.strategy not in {'top', 'front'}:
            raise ConfigurationError(f'pickup.{name}.strategy deve ser top ou front.')
        if profile.strategy == 'front' and not profile.pre_grasp_state:
            raise ConfigurationError(f'pickup.{name}.pre_grasp_state é obrigatório.')
        if not -180.0 < profile.link3_to_link4_deg < 180.0:
            raise ConfigurationError(f'pickup.{name}.link3_to_link4_deg inválido.')
        if not -180.0 < profile.link4_to_link5_deg < 180.0:
            raise ConfigurationError(f'pickup.{name}.link4_to_link5_deg inválido.')
        if profile.joint_tolerance_deg >= 90.0:
            raise ConfigurationError(f'pickup.{name}.joint_tolerance_deg deve ser menor que 90.')
        if not profile.observation_state:
            raise ConfigurationError(f'pickup.{name}.observation_state não pode ser vazio.')
        if profile.reachability_filter_enabled:
            values = (
                profile.reach_min_radius_m, profile.reach_max_radius_m,
                profile.reach_x_min_m, profile.reach_x_max_m,
                profile.reach_y_min_m, profile.reach_y_max_m,
            )
            if any(value is None for value in values):
                raise ConfigurationError(
                    f'pickup.{name} deve configurar os raios e os limites XY '
                    'quando reachability_filter_enabled estiver habilitada.'
                )
            if profile.reach_min_radius_m >= profile.reach_max_radius_m:
                raise ConfigurationError(
                    f'pickup.{name}.reach_min_radius_m deve ser menor que '
                    'reach_max_radius_m.'
                )
            if (
                profile.reach_x_min_m > profile.reach_x_max_m
                or profile.reach_y_min_m > profile.reach_y_max_m
            ):
                raise ConfigurationError(
                    f'pickup.{name} possui limites XY de alcance inválidos.'
                )

    placements: dict[str, PlacementProfile] = {}
    for name, raw_value in placements_raw.items():
        raw = _mapping(raw_value, f'placements.{name}')
        _only_keys(
            raw,
            {
                'strategy', 'enabled', 'named_state',
                'approach_height_m', 'retreat_height_m',
                'reference_offset_xyz', 'yaw_offset_deg',
                'calibrated_reference',
                'tcp_release_offset_cm',
                'free_space_half_extent_x_m', 'free_space_half_extent_y_m',
                'free_space_min_padding_m', 'free_space_preferred_padding_m',
                'free_space_preferred_yaw_deg',
                'free_space_alternate_yaw_deg',
                'reach_center_x_m', 'reach_center_y_m',
                'reach_min_radius_m', 'reach_max_radius_m',
                'search_x_min_m', 'search_x_max_m',
                'search_y_min_m', 'search_y_max_m', 'search_step_m',
                'link3_to_link4_max_deg',
                'tilt_tolerance_deg', 'base_wiggle',
            },
            f'placements.{name}',
        )
        strategy = str(raw.get('strategy', ''))
        if strategy not in {
            'cartesian', 'named_state', 'perception', 'tag_relative'
        }:
            raise ConfigurationError(
                f"placements.{name}.strategy inválida: '{strategy}'."
            )
        offset = raw.get('reference_offset_xyz', [0.0, 0.0, 0.0])
        if not isinstance(offset, list) or len(offset) != 3:
            raise ConfigurationError(
                f'placements.{name}.reference_offset_xyz deve ter três valores.'
            )
        profile = PlacementProfile(
            name=name,
            strategy=strategy,
            base_wiggle=_base_wiggle(raw.get('base_wiggle', {}), f'placements.{name}.base_wiggle'),
            enabled=bool(raw.get('enabled', False)),
            named_state=str(raw.get('named_state', '')),
            approach_height_m=_number(
                raw.get('approach_height_m', 0.08),
                f'placements.{name}.approach_height_m',
            ),
            retreat_height_m=_number(
                raw.get('retreat_height_m', 0.08),
                f'placements.{name}.retreat_height_m',
            ),
            reference_offset_xyz=tuple(
                _number(value, f'placements.{name}.reference_offset_xyz')
                for value in offset
            ),
            yaw_offset_deg=_number(
                raw.get('yaw_offset_deg', 90.0), f'placements.{name}.yaw_offset_deg'
            ),
            calibrated_reference=bool(raw.get('calibrated_reference', False)),
            tcp_release_offset_cm=(
                None if raw.get('tcp_release_offset_cm') is None
                else _number(
                    raw['tcp_release_offset_cm'],
                    f'placements.{name}.tcp_release_offset_cm',
                )
            ),
            free_space_half_extent_x_m=_number(
                raw.get('free_space_half_extent_x_m', 0.07),
                f'placements.{name}.free_space_half_extent_x_m',
                positive=True,
            ),
            free_space_half_extent_y_m=_number(
                raw.get('free_space_half_extent_y_m', 0.04),
                f'placements.{name}.free_space_half_extent_y_m',
                positive=True,
            ),
            free_space_min_padding_m=_number(
                raw.get('free_space_min_padding_m', 0.0),
                f'placements.{name}.free_space_min_padding_m',
            ),
            free_space_preferred_padding_m=_number(
                raw.get('free_space_preferred_padding_m', 0.03),
                f'placements.{name}.free_space_preferred_padding_m',
            ),
            free_space_preferred_yaw_deg=(
                None if raw.get('free_space_preferred_yaw_deg') is None
                else _number(
                    raw['free_space_preferred_yaw_deg'],
                    f'placements.{name}.free_space_preferred_yaw_deg',
                )
            ),
            free_space_alternate_yaw_deg=(
                None if raw.get('free_space_alternate_yaw_deg') is None
                else _number(
                    raw['free_space_alternate_yaw_deg'],
                    f'placements.{name}.free_space_alternate_yaw_deg',
                )
            ),
            reach_center_x_m=_number(
                raw.get('reach_center_x_m', 0.0),
                f'placements.{name}.reach_center_x_m',
            ),
            reach_center_y_m=_number(
                raw.get('reach_center_y_m', 0.0),
                f'placements.{name}.reach_center_y_m',
            ),
            reach_min_radius_m=(
                None if raw.get('reach_min_radius_m') is None
                else _number(
                    raw['reach_min_radius_m'],
                    f'placements.{name}.reach_min_radius_m',
                    positive=True,
                )
            ),
            reach_max_radius_m=(
                None if raw.get('reach_max_radius_m') is None
                else _number(
                    raw['reach_max_radius_m'],
                    f'placements.{name}.reach_max_radius_m',
                    positive=True,
                )
            ),
            search_x_min_m=(
                None if raw.get('search_x_min_m') is None
                else _number(raw['search_x_min_m'], f'placements.{name}.search_x_min_m')
            ),
            search_x_max_m=(
                None if raw.get('search_x_max_m') is None
                else _number(raw['search_x_max_m'], f'placements.{name}.search_x_max_m')
            ),
            search_y_min_m=(
                None if raw.get('search_y_min_m') is None
                else _number(raw['search_y_min_m'], f'placements.{name}.search_y_min_m')
            ),
            search_y_max_m=(
                None if raw.get('search_y_max_m') is None
                else _number(raw['search_y_max_m'], f'placements.{name}.search_y_max_m')
            ),
            search_step_m=_number(
                raw.get('search_step_m', 0.01),
                f'placements.{name}.search_step_m',
                positive=True,
            ),
            link3_to_link4_max_deg=_number(
                raw.get('link3_to_link4_max_deg', -10.0),
                f'placements.{name}.link3_to_link4_max_deg',
            ),
            tilt_tolerance_deg=(
                None if raw.get('tilt_tolerance_deg') is None
                else _number(
                    raw['tilt_tolerance_deg'],
                    f'placements.{name}.tilt_tolerance_deg',
                )
            ),
        )
        if profile.base_wiggle.enabled and name != 'precision_table':
            raise ConfigurationError('base_wiggle só pode ser habilitada em placements.precision_table.')
        for field in ('approach_height_m', 'retreat_height_m'):
            if getattr(profile, field) < 0.0:
                raise ConfigurationError(
                    f'placements.{name}.{field} deve ser maior ou igual a zero.'
                )
        if (
            profile.tilt_tolerance_deg is not None
            and not 0.0 <= profile.tilt_tolerance_deg <= 180.0
        ):
            raise ConfigurationError(
                f'placements.{name}.tilt_tolerance_deg deve estar entre 0 e 180 graus.'
            )
        if (
            not -180.0 < profile.link3_to_link4_max_deg < 180.0
        ):
            raise ConfigurationError(
                f'placements.{name}.link3_to_link4_max_deg deve estar entre '
                '-180 e 180 graus.')
        if profile.enabled and strategy == 'named_state' and not profile.named_state:
            raise ConfigurationError(
                f'placements.{name}.named_state é obrigatório quando habilitado.'
            )
        if profile.free_space_preferred_padding_m < 0.0:
            raise ConfigurationError(
                f'placements.{name}.free_space_preferred_padding_m deve ser '
                'maior ou igual a zero.'
            )
        if profile.free_space_min_padding_m < 0.0:
            raise ConfigurationError(
                f'placements.{name}.free_space_min_padding_m deve ser maior '
                'ou igual a zero.'
            )
        if (
            profile.free_space_preferred_padding_m
            < profile.free_space_min_padding_m
        ):
            raise ConfigurationError(
                f'placements.{name}.free_space_preferred_padding_m deve ser '
                'maior ou igual a free_space_min_padding_m.'
            )
        yaw_options = (
            profile.free_space_preferred_yaw_deg,
            profile.free_space_alternate_yaw_deg,
        )
        if (yaw_options[0] is None) != (yaw_options[1] is None):
            raise ConfigurationError(
                f'placements.{name}.free_space_preferred_yaw_deg e '
                'free_space_alternate_yaw_deg devem ser configurados juntos.'
            )
        if (
            yaw_options[0] is not None
            and yaw_options[1] is not None
            and abs((yaw_options[0] - yaw_options[1] + 180.0)
                    % 360.0 - 180.0) <= 1e-9
        ):
            raise ConfigurationError(
                f'placements.{name}.free_space_preferred_yaw_deg e '
                'free_space_alternate_yaw_deg devem ser diferentes.'
            )
        reach_radii = (profile.reach_min_radius_m, profile.reach_max_radius_m)
        if (reach_radii[0] is None) != (reach_radii[1] is None):
            raise ConfigurationError(
                f'placements.{name}.reach_min_radius_m e reach_max_radius_m '
                'devem ser configurados juntos.'
            )
        if (
            reach_radii[0] is not None
            and reach_radii[1] is not None
            and reach_radii[0] >= reach_radii[1]
        ):
            raise ConfigurationError(
                f'placements.{name}.reach_min_radius_m deve ser menor que '
                'reach_max_radius_m.'
            )
        placements[name] = profile

    required_strategies = {
        'table': 'perception',
        'explicit_pose': 'cartesian',
        'container': 'perception',
        'stack': 'tag_relative',
        'shelf': 'named_state',
    }
    missing_profiles = sorted(set(required_strategies) - set(placements))
    if missing_profiles:
        raise ConfigurationError(
            f'Perfis semânticos de depósito ausentes: {missing_profiles}.'
        )
    for name, expected_strategy in required_strategies.items():
        if placements[name].strategy != expected_strategy:
            raise ConfigurationError(
                f"placements.{name}.strategy deve ser '{expected_strategy}'."
            )

    if ('precision_table' in placements
            and placements['precision_table'].strategy != 'tag_relative'):
        raise ConfigurationError(
            "placements.precision_table.strategy deve ser 'tag_relative'.")

    cargo_root = _load_yaml(cargo_path)
    _only_keys(
        cargo_root,
        {
            'schema_version', 'cargo_slots', 'transport_empty_state',
            'transport_loaded_state',
        },
        'cargo',
    )
    cargo_raw = _mapping(cargo_root.get('cargo_slots'), 'cargo_slots')
    cargo_slots: dict[str, CargoSlotProfile] = {}
    for slot_id, raw_value in cargo_raw.items():
        raw = _mapping(raw_value, f'cargo_slots.{slot_id}')
        _only_keys(
            raw,
            {'store_state', 'safe_state', 'retrieve_state'},
            f'cargo_slots.{slot_id}',
        )
        store_state = str(raw.get('store_state', ''))
        safe_state = str(raw.get('safe_state', ''))
        retrieve_state = str(raw.get('retrieve_state', ''))
        if not store_state or not safe_state or not retrieve_state:
            raise ConfigurationError(
                f"O compartimento '{slot_id}' precisa de store_state, safe_state "
                'e retrieve_state.'
            )
        cargo_slots[slot_id] = CargoSlotProfile(
            slot_id, store_state, safe_state, retrieve_state
        )
    if not cargo_slots:
        raise ConfigurationError('Configure ao menos um compartimento de carga.')

    empty_state = str(cargo_root.get('transport_empty_state', ''))
    loaded_state = str(cargo_root.get('transport_loaded_state', ''))
    if not empty_state or not loaded_state:
        raise ConfigurationError('Estados de transporte não podem ser vazios.')
    return ProfileSet(
        pickup=pickup,
        placements=placements,
        cargo_slots=cargo_slots,
        transport_empty_state=empty_state,
        transport_loaded_state=loaded_state,
    )
