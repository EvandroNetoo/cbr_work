"""Thread-safe logical state of the gripper and on-board cargo slots."""

from __future__ import annotations

import threading

from .errors import StateConflict


EMPTY = -1


class ManipulationInventory:
    """Transactional logical inventory updated at physical commit points."""

    def __init__(self, slot_ids: list[str]) -> None:
        self._lock = threading.RLock()
        self._known = True
        self._gripper = EMPTY
        self._slots = {slot_id: EMPTY for slot_id in slot_ids}

    def snapshot(self) -> tuple[bool, int, dict[str, int]]:
        with self._lock:
            return self._known, self._gripper, dict(self._slots)

    def mark_unknown(self) -> None:
        with self._lock:
            self._known = False

    def require_gripper_object(self) -> int:
        """Return the held object or reject an empty/unknown inventory."""
        with self._lock:
            self._require_known()
            if self._gripper == EMPTY:
                raise StateConflict('A garra está vazia.')
            return self._gripper

    def require_slot_object(self, slot_id: str) -> int:
        """Return the object in a cargo slot or reject an empty/unknown slot."""
        with self._lock:
            self._require_known()
            if slot_id not in self._slots:
                raise StateConflict(f"Compartimento desconhecido: '{slot_id}'.")
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
            if slot_id not in self._slots:
                raise StateConflict(f"Compartimento desconhecido: '{slot_id}'.")
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
            self._gripper = EMPTY
            self._slots[slot_id] = tag_id

    def validate_retrieve(self, tag_id: int, slot_id: str) -> None:
        with self._lock:
            self._require_known()
            self._require_object_id(tag_id)
            if slot_id not in self._slots:
                raise StateConflict(f"Compartimento desconhecido: '{slot_id}'.")
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
            self._gripper = EMPTY
