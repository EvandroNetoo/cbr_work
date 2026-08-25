"""Pure mission parsing, validation and cargo accounting."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


LEFT = "left"
RIGHT = "right"
GRIPPER = "gripper"
FAILED = "failed"
SLOTS = (LEFT, RIGHT)


class MissionConfigurationError(ValueError):
    """Raised before any physical command when a mission is invalid."""


@dataclass(frozen=True)
class Station:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Step:
    kind: str
    station: str | None = None
    tag_id: int | None = None
    placement: str | None = None


@dataclass(frozen=True)
class MissionDefinition:
    name: str
    steps: tuple[Step, ...]


@dataclass(frozen=True)
class MissionConfig:
    enabled: bool
    initial_pose: Station
    stations: dict[str, Station]
    placements: dict[str, str]
    missions: dict[str, MissionDefinition]


class Inventory:
    """Logical cargo state. It is only committed after successful actions."""

    def __init__(self) -> None:
        self.gripper: int | None = None
        self.slots: dict[str, int | None] = {LEFT: None, RIGHT: None}
        self.locations: dict[int, str] = {}

    def copy(self) -> "Inventory":
        other = Inventory()
        other.gripper = self.gripper
        other.slots = dict(self.slots)
        other.locations = dict(self.locations)
        return other

    def free_slot(self, *, excluding: str | None = None) -> str | None:
        return next(
            (slot for slot in SLOTS if slot != excluding and self.slots[slot] is None),
            None,
        )

    def add_to_gripper(self, tag_id: int) -> None:
        if self.gripper is not None:
            raise MissionConfigurationError("A garra já está ocupada.")
        if tag_id in self.locations:
            raise MissionConfigurationError(f"A tag {tag_id} já foi coletada.")
        self.gripper = tag_id
        self.locations[tag_id] = GRIPPER

    def store_gripper(self, slot: str) -> int:
        if slot not in SLOTS or self.slots[slot] is not None:
            raise MissionConfigurationError(f"Compartimento '{slot}' indisponível.")
        if self.gripper is None:
            raise MissionConfigurationError("Não há cubo na garra para guardar.")
        tag_id = self.gripper
        self.gripper = None
        self.slots[slot] = tag_id
        self.locations[tag_id] = slot
        return tag_id

    def retrieve(self, slot: str) -> int:
        if self.gripper is not None:
            raise MissionConfigurationError("A garra precisa estar vazia para retirar um cubo.")
        tag_id = self.slots.get(slot)
        if tag_id is None:
            raise MissionConfigurationError(f"Compartimento '{slot}' está vazio.")
        self.slots[slot] = None
        self.gripper = tag_id
        self.locations[tag_id] = GRIPPER
        return tag_id

    def deliver_gripper(self, expected_tag: int) -> None:
        if self.gripper != expected_tag:
            raise MissionConfigurationError(
                f"A garra contém {self.gripper}, não a tag {expected_tag}."
            )
        self.gripper = None
        del self.locations[expected_tag]

    def mark_failed(self, tag_id: int) -> None:
        self.locations[tag_id] = FAILED


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MissionConfigurationError(f"'{label}' precisa ser um mapeamento YAML.")
    return value


def _number(mapping: dict[str, Any], key: str, label: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigurationError(f"'{label}.{key}' precisa ser numérico.")
    return float(value)


def _station(name: str, value: Any) -> Station:
    data = _mapping(value, name)
    return Station(name, _number(data, "x", name), _number(data, "y", name),
                   _number(data, "yaw", name))


def _step(value: Any, index: int) -> Step:
    data = _mapping(value, f"steps[{index}]")
    if len(data) != 1:
        raise MissionConfigurationError(
            f"steps[{index}] deve conter exatamente uma ação."
        )
    kind, payload = next(iter(data.items()))
    if kind == "navigate":
        if not isinstance(payload, str) or not payload:
            raise MissionConfigurationError(f"steps[{index}].navigate inválido.")
        return Step(kind=kind, station=payload)
    details = _mapping(payload, f"steps[{index}].{kind}")
    if kind not in ("pick", "place"):
        raise MissionConfigurationError(f"Ação desconhecida '{kind}' em steps[{index}].")
    tag_id = details.get("tag_id")
    if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
        raise MissionConfigurationError(f"tag_id inválida em steps[{index}].")
    if kind == "pick":
        if set(details) != {"tag_id"}:
            raise MissionConfigurationError(f"Campos desconhecidos em steps[{index}].pick.")
        return Step(kind=kind, tag_id=tag_id)
    placement = details.get("placement")
    if not isinstance(placement, str) or not placement:
        raise MissionConfigurationError(f"placement inválido em steps[{index}].")
    if set(details) != {"tag_id", "placement"}:
        raise MissionConfigurationError(f"Campos desconhecidos em steps[{index}].place.")
    return Step(kind=kind, tag_id=tag_id, placement=placement)


def load_config(path: str | Path) -> MissionConfig:
    source = Path(path)
    try:
        root = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise MissionConfigurationError(f"Não foi possível ler {source}: {error}") from error
    root = _mapping(root, "raiz")
    enabled = root.get("enabled", False)
    if not isinstance(enabled, bool):
        raise MissionConfigurationError("'enabled' precisa ser true ou false.")
    initial = _station("initial_pose", root.get("initial_pose"))
    stations_data = _mapping(root.get("stations"), "stations")
    stations = {name: _station(name, value) for name, value in stations_data.items()}
    placements_data = _mapping(root.get("placements"), "placements")
    placements = {}
    for name, state in placements_data.items():
        if not isinstance(name, str) or not isinstance(state, str) or not state:
            raise MissionConfigurationError("Nomes de placement e estados devem ser strings.")
        placements[name] = state
    missions_data = _mapping(root.get("missions"), "missions")
    missions: dict[str, MissionDefinition] = {}
    for name, value in missions_data.items():
        details = _mapping(value, f"missions.{name}")
        raw_steps = details.get("steps")
        if not isinstance(raw_steps, list) or not raw_steps:
            raise MissionConfigurationError(f"A missão '{name}' não possui steps.")
        mission = MissionDefinition(name, tuple(_step(item, i) for i, item in enumerate(raw_steps)))
        validate_mission(mission, stations, placements)
        missions[name] = mission
    if not missions:
        raise MissionConfigurationError("É necessário configurar ao menos uma missão.")
    return MissionConfig(enabled, initial, stations, placements, missions)


def validate_mission(
    mission: MissionDefinition,
    stations: dict[str, Station],
    placements: dict[str, str],
) -> None:
    inventory = Inventory()
    collected: set[int] = set()
    at_station = False
    for index, step in enumerate(mission.steps):
        if step.kind == "navigate":
            if step.station not in stations:
                raise MissionConfigurationError(
                    f"Missão '{mission.name}', etapa {index}: estação '{step.station}' desconhecida."
                )
            at_station = True
            continue
        if not at_station:
            raise MissionConfigurationError(
                f"Missão '{mission.name}', etapa {index}: manipulação antes de navegar."
            )
        assert step.tag_id is not None
        if step.kind == "pick":
            if step.tag_id in collected:
                raise MissionConfigurationError(
                    f"Missão '{mission.name}': tag {step.tag_id} coletada mais de uma vez."
                )
            if inventory.gripper is not None:
                slot = inventory.free_slot()
                if slot is None:
                    raise MissionConfigurationError(
                        f"Missão '{mission.name}', etapa {index}: capacidade de três cubos excedida."
                    )
                inventory.store_gripper(slot)
            inventory.add_to_gripper(step.tag_id)
            collected.add(step.tag_id)
            continue
        if step.placement not in placements:
            raise MissionConfigurationError(
                f"Missão '{mission.name}', etapa {index}: placement '{step.placement}' desconhecido."
            )
        location = inventory.locations.get(step.tag_id)
        if location is None:
            raise MissionConfigurationError(
                f"Missão '{mission.name}', etapa {index}: tag {step.tag_id} não está carregada."
            )
        if location in SLOTS:
            if inventory.gripper is not None:
                free = inventory.free_slot(excluding=location)
                if free is None:
                    raise MissionConfigurationError(
                        f"Missão '{mission.name}', etapa {index}: não há onde guardar a tag "
                        f"{inventory.gripper} para retirar a tag {step.tag_id}."
                    )
                inventory.store_gripper(free)
            inventory.retrieve(location)
        inventory.deliver_gripper(step.tag_id)
