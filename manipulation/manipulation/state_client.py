"""Client facade for the manipulation state owned by mission_manager."""

from __future__ import annotations

import math
import threading
from typing import Any

from interfaces.srv import ManageManipulationState

from .errors import ServerUnavailable, StateConflict


EMPTY = -1


class MissionStateClient:
    """Expose the old inventory API while all data lives in mission_manager."""

    def __init__(
        self,
        node: Any,
        service_name: str,
        timeout_s: float,
        callback_group: Any,
    ) -> None:
        self._client = node.create_client(
            ManageManipulationState,
            service_name,
            callback_group=callback_group,
        )
        self._service_name = service_name
        self._timeout_s = float(timeout_s)
        self._cache_lock = threading.RLock()
        self._cached_snapshot = (False, EMPTY, {})

    def _request(
        self,
        command: int,
        tag_id: int = EMPTY,
        slot_id: str = '',
    ) -> tuple[bool, int, dict[str, int]]:
        if not math.isfinite(self._timeout_s) or self._timeout_s <= 0.0:
            raise ServerUnavailable('state_service_timeout_s deve ser positivo.')
        if not self._client.wait_for_service(timeout_sec=self._timeout_s):
            raise ServerUnavailable(
                f"Serviço de estado da missão '{self._service_name}' indisponível."
            )
        request = ManageManipulationState.Request()
        request.command = int(command)
        request.object_tag_id = int(tag_id)
        request.slot_id = slot_id
        future = self._client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(self._timeout_s):
            raise ServerUnavailable(
                f"Serviço de estado da missão '{self._service_name}' excedeu o timeout."
            )
        if future.exception() is not None:
            raise ServerUnavailable(
                f'Falha ao acessar o estado da missão: {future.exception()}'
            ) from future.exception()
        response = future.result()
        if response is None:
            raise ServerUnavailable('Serviço de estado retornou resposta vazia.')
        snapshot = self._snapshot_from_message(response.state)
        with self._cache_lock:
            self._cached_snapshot = snapshot
        if not response.success:
            raise StateConflict(response.message or 'Transição de estado rejeitada.')
        return snapshot

    @staticmethod
    def _snapshot_from_message(state: Any) -> tuple[bool, int, dict[str, int]]:
        return (
            bool(state.state_known),
            int(state.gripper_object_id),
            {slot.slot_id: int(slot.object_id) for slot in state.cargo_slots},
        )

    def snapshot(self) -> tuple[bool, int, dict[str, int]]:
        return self._request(ManageManipulationState.Request.GET_STATE)

    def cached_snapshot(self) -> tuple[bool, int, dict[str, int]]:
        with self._cache_lock:
            known, gripper, slots = self._cached_snapshot
            return known, gripper, dict(slots)

    def mark_unknown(self) -> None:
        self._mark_cache_unknown()
        self._request(ManageManipulationState.Request.MARK_UNKNOWN)

    def _mark_cache_unknown(self) -> None:
        with self._cache_lock:
            _, gripper, slots = self._cached_snapshot
            self._cached_snapshot = (False, gripper, slots)

    def _commit(
        self,
        command: int,
        tag_id: int = EMPTY,
        slot_id: str = '',
    ) -> None:
        """Commit a physical transition, preserving uncertainty on RPC failure."""
        try:
            self._request(command, tag_id, slot_id)
        except Exception:
            self._mark_cache_unknown()
            try:
                self._request(ManageManipulationState.Request.MARK_UNKNOWN)
            except Exception:
                pass
            raise

    def require_gripper_object(self) -> int:
        known, gripper, _ = self.snapshot()
        self._require_known(known)
        if gripper == EMPTY:
            raise StateConflict('A garra está vazia.')
        return gripper

    def require_slot_object(self, slot_id: str) -> int:
        known, _, slots = self.snapshot()
        self._require_known(known)
        if slot_id not in slots:
            raise StateConflict(f"Compartimento desconhecido: '{slot_id}'.")
        tag_id = slots[slot_id]
        if tag_id == EMPTY:
            raise StateConflict(f"Compartimento '{slot_id}' está vazio.")
        return tag_id

    @staticmethod
    def _require_known(known: bool) -> None:
        if not known:
            raise StateConflict(
                'O estado físico da carga é incerto; faça recuperação manual antes '
                'de executar outra manipulação.'
            )

    def validate_pick(self, tag_id: int) -> None:
        self._request(ManageManipulationState.Request.VALIDATE_PICK, tag_id)

    def commit_pick(self, tag_id: int) -> None:
        self._commit(ManageManipulationState.Request.COMMIT_PICK, tag_id)

    def validate_store(self, tag_id: int, slot_id: str) -> None:
        self._request(ManageManipulationState.Request.VALIDATE_STORE, tag_id, slot_id)

    def commit_store(self, tag_id: int, slot_id: str) -> None:
        self._commit(ManageManipulationState.Request.COMMIT_STORE, tag_id, slot_id)

    def validate_retrieve(self, tag_id: int, slot_id: str) -> None:
        self._request(
            ManageManipulationState.Request.VALIDATE_RETRIEVE, tag_id, slot_id
        )

    def commit_retrieve(self, tag_id: int, slot_id: str) -> None:
        self._commit(
            ManageManipulationState.Request.COMMIT_RETRIEVE, tag_id, slot_id
        )

    def validate_place(self, tag_id: int) -> None:
        self._request(ManageManipulationState.Request.VALIDATE_PLACE, tag_id)

    def commit_place(self) -> None:
        self._commit(ManageManipulationState.Request.COMMIT_PLACE)
