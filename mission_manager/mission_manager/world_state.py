"""Mission-owned state for the robot and its view of the world."""

from __future__ import annotations

import threading

from .errors import StateConflict


EMPTY = -1


class WorldState:
    """
    Thread-safe world state reset at the beginning of every mission.

    Gripper and cargo are the first state domain kept here.  Keeping the class
    in the mission package gives future state (objects, stations and arena
    facts) a single owner without coupling it to a physical capability server.
    """

    def __init__(self, cargo_slot_ids: list[str]) -> None:
        if any(not slot_id for slot_id in cargo_slot_ids):
            raise ValueError('cargo_slot_ids deve conter nomes não vazios.')
        if len(cargo_slot_ids) != len(set(cargo_slot_ids)):
            raise ValueError('cargo_slot_ids não pode conter nomes duplicados.')
        self._lock = threading.RLock()
        self._slot_ids = tuple(cargo_slot_ids)
        self.reset()

    def reset(self) -> None:
        """Start a new mission with an empty, known on-board inventory."""
        with self._lock:
            self._known = True
            self._gripper = EMPTY
            self._slots = {slot_id: EMPTY for slot_id in self._slot_ids}

    def snapshot(self) -> tuple[bool, int, dict[str, int]]:
        with self._lock:
            return self._known, self._gripper, dict(self._slots)

    def mark_unknown(self) -> None:
        with self._lock:
            self._known = False

    def require_gripper_object(self) -> int:
        with self._lock:
            self._require_known()
            if self._gripper == EMPTY:
                raise StateConflict('A garra está vazia.')
            return self._gripper

    def require_slot_object(self, slot_id: str) -> int:
        with self._lock:
            self._require_known()
            self._require_slot(slot_id)
            tag_id = self._slots[slot_id]
            if tag_id == EMPTY:
                raise StateConflict(f"Compartimento '{slot_id}' está vazio.")
            return tag_id

    def _require_known(self) -> None:
        if not self._known:
            raise StateConflict(
                'O estado físico da carga é incerto; faça recuperação manual antes '
                'de executar outra manipulação.'
            )

    @staticmethod
    def _require_object_id(tag_id: int) -> None:
        if tag_id < 0:
            raise StateConflict('O ID da AprilTag não pode ser negativo.')

    def _require_slot(self, slot_id: str) -> None:
        if slot_id not in self._slots:
            raise StateConflict(f"Compartimento desconhecido: '{slot_id}'.")

    def validate_pick(self, tag_id: int) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            if self._gripper != EMPTY:
                raise StateConflict(
                    f'A garra já contém o objeto {self._gripper}.'
                )
            if tag_id in self._slots.values():
                raise StateConflict(f'O objeto {tag_id} já está armazenado no robô.')

    def commit_pick(self, tag_id: int) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            self._gripper = tag_id

    def validate_store(self, tag_id: int, slot_id: str) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            self._require_slot(slot_id)
            if self._gripper != tag_id:
                raise StateConflict(
                    f'A garra contém {self._gripper}, não o objeto {tag_id}.'
                )
            if self._slots[slot_id] != EMPTY:
                raise StateConflict(
                    f"Compartimento '{slot_id}' está ocupado pelo objeto "
                    f'{self._slots[slot_id]}.'
                )

    def commit_store(self, tag_id: int, slot_id: str) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            self._require_slot(slot_id)
            self._gripper = EMPTY
            self._slots[slot_id] = tag_id

    def validate_retrieve(self, tag_id: int, slot_id: str) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            self._require_slot(slot_id)
            if self._gripper != EMPTY:
                raise StateConflict(
                    f'A garra já contém o objeto {self._gripper}.'
                )
            if self._slots[slot_id] != tag_id:
                raise StateConflict(
                    f"Compartimento '{slot_id}' contém {self._slots[slot_id]}, "
                    f'não o objeto {tag_id}.'
                )

    def commit_retrieve(self, tag_id: int, slot_id: str) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            self._require_slot(slot_id)
            self._slots[slot_id] = EMPTY
            self._gripper = tag_id

    def validate_place(self, tag_id: int) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            if self._gripper != tag_id:
                raise StateConflict(
                    f'A garra contém {self._gripper}, não o objeto {tag_id}.'
                )

    def commit_place(self) -> None:
        with self._lock:
            self._require_known()
            self._gripper = EMPTY
