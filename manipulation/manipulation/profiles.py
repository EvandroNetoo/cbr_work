"""Strict loaders for manipulation and on-board cargo profiles."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .errors import ConfigurationError


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
    release_x_m: float | None = None
    release_y_m: float | None = None
    release_yaw_deg: float | None = None
    tcp_release_offset_cm: float | None = None
    free_space_min_distance_m: float = 0.08
    search_x_min_m: float | None = None
    search_x_max_m: float | None = None
    search_y_min_m: float | None = None
    search_y_max_m: float | None = None
    search_step_m: float = 0.01


@dataclass(frozen=True)
class CargoSlotProfile:
    slot_id: str
    store_state: str
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
                'yaw_offset_deg', 'attempts',
            },
            f'pickup.{name}',
        )
        attempts = int(raw.get('attempts', 1))
        if attempts <= 0:
            raise ConfigurationError(f'pickup.{name}.attempts deve ser positivo.')
        pickup[name] = PickupProfile(
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
        )
        if not pickup[name].observation_state:
            raise ConfigurationError(f'pickup.{name}.observation_state não pode ser vazio.')

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
                'release_x_m', 'release_y_m', 'release_yaw_deg',
                'tcp_release_offset_cm',
                'free_space_min_distance_m',
                'search_x_min_m', 'search_x_max_m',
                'search_y_min_m', 'search_y_max_m', 'search_step_m',
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
            enabled=bool(raw.get('enabled', False)),
            named_state=str(raw.get('named_state', '')),
            approach_height_m=_number(
                raw.get('approach_height_m', 0.08),
                f'placements.{name}.approach_height_m', positive=True,
            ),
            retreat_height_m=_number(
                raw.get('retreat_height_m', 0.08),
                f'placements.{name}.retreat_height_m', positive=True,
            ),
            reference_offset_xyz=tuple(
                _number(value, f'placements.{name}.reference_offset_xyz')
                for value in offset
            ),
            yaw_offset_deg=_number(
                raw.get('yaw_offset_deg', 90.0), f'placements.{name}.yaw_offset_deg'
            ),
            calibrated_reference=bool(raw.get('calibrated_reference', False)),
            release_x_m=(
                None if raw.get('release_x_m') is None
                else _number(raw['release_x_m'], f'placements.{name}.release_x_m')
            ),
            release_y_m=(
                None if raw.get('release_y_m') is None
                else _number(raw['release_y_m'], f'placements.{name}.release_y_m')
            ),
            release_yaw_deg=(
                None if raw.get('release_yaw_deg') is None
                else _number(
                    raw['release_yaw_deg'], f'placements.{name}.release_yaw_deg'
                )
            ),
            tcp_release_offset_cm=(
                None if raw.get('tcp_release_offset_cm') is None
                else _number(
                    raw['tcp_release_offset_cm'],
                    f'placements.{name}.tcp_release_offset_cm',
                )
            ),
            free_space_min_distance_m=_number(
                raw.get('free_space_min_distance_m', 0.08),
                f'placements.{name}.free_space_min_distance_m',
                positive=True,
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
        )
        if profile.enabled and strategy == 'named_state' and not profile.named_state:
            raise ConfigurationError(
                f'placements.{name}.named_state é obrigatório quando habilitado.'
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
        _only_keys(raw, {'store_state', 'retrieve_state'}, f'cargo_slots.{slot_id}')
        store_state = str(raw.get('store_state', ''))
        retrieve_state = str(raw.get('retrieve_state', ''))
        if not store_state or not retrieve_state:
            raise ConfigurationError(
                f"O compartimento '{slot_id}' precisa de store_state e retrieve_state."
            )
        cargo_slots[slot_id] = CargoSlotProfile(slot_id, store_state, retrieve_state)
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
