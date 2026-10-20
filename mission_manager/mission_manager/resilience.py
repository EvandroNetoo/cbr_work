"""Bounded retries and failure reporting without inventing physical effects."""

from types import SimpleNamespace

from .errors import ActionUncertain, MissionCanceled, RetrieveFailed, StepFailed, TaskNotFound
from .models import Step
from .world_state import EMPTY


class MissionResilience:
    def _retry_operation(self, operation, description, *, expected=None, key=None,
                         yield_missing=False, before_retry=None, passthrough=(),
                         max_attempts=None):
        """Return the final exception, or None after a confirmed success.

        A confirmed transfer wins over a failure retracting the arm. An
        ambiguous transfer must never be repeated or erased from inventory.
        """
        attempts = getattr(self, '_task_attempts', {})
        if key is None:
            key = object()
        limit = (getattr(self, '_operation_max_attempts', 2)
                 if max_attempts is None else max_attempts)
        while attempts.get(key, 0) < limit:
            self._check_canceled()
            retrying = attempts.get(key, 0) > 0
            attempts[key] = attempts.get(key, 0) + 1
            try:
                if retrying and before_retry is not None:
                    before_retry()
                operation()
                if expected is not None and not self._inventory_matches(expected):
                    raise StepFailed(f'{description}: efeito físico esperado não confirmado.')
                attempts.pop(key, None)
                return None
            except MissionCanceled:
                raise
            except ActionUncertain:
                self._unconfirmed_action_pending = True
                raise
            except Exception as error:
                if isinstance(error, passthrough):
                    raise
                self._check_canceled()
                if not self._world_state.snapshot()[0]:
                    raise StepFailed(
                        f'{description}: estado físico não confirmado; '
                        f'movimentos suspensos, servidor permanece ativo. {error}') from error
                if expected is not None and self._inventory_matches(expected):
                    self.get_logger().warning(
                        f'{description}: transferência confirmada antes da falha; '
                        f'ação não será repetida. {error}')
                    attempts.pop(key, None)
                    return None
                self.get_logger().warning(
                    f'{description}: tentativa {attempts[key]}/{limit} falhou: {error}')
                if attempts[key] >= limit:
                    attempts.pop(key, None)
                    return error
                if yield_missing and isinstance(error, TaskNotFound):
                    raise
        return StepFailed(f'{description}: tentativas esgotadas.')

    def _retry_retrieve(self, operation, step, *, expected, key=None):
        """Recover known cargo transfers; never skip a cube still on board."""
        def recover():
            # Recovery may move/open the empty gripper only while the cube is
            # still confirmed in its compartment. Unknown effects stop earlier.
            self._world_state.validate_retrieve(step.tag_id, step.slot_id)
            self._refresh_failed_step(step)
            self._prepare_for_navigation()

        error = self._retry_operation(
            operation, f'retrieve: tag {step.tag_id}, slot {step.slot_id}',
            expected=expected, key=key, before_retry=recover,
            max_attempts=getattr(self, '_retrieve_max_attempts', 4))
        if error is not None:
            raise RetrieveFailed(
                f'Retirada da tag {step.tag_id} do slot {step.slot_id} não confirmada '
                'após as tentativas de recuperação. Missão interrompida; carga '
                f'preservada, sem novas coletas ou navegação. Causa: {error}') from error

    def _refresh_failed_step(self, step):
        self._direct_pick_observation = None
        self._last_table_observation = None
        self._stack_alignment = None
        self._pp_aligned_reference = None
        self._last_stack_placement = None
        getattr(self, '_scene_observations', {}).clear()
        if step.action == 'pick':
            getattr(self, '_visited_search_positions', {}).pop(self._current_location, None)
        if step.action == 'place_in_container':
            getattr(self, '_container_search_positions', {}).pop(self._current_location, None)

    def _inventory_matches(self, state):
        known, held, cargo = self._world_state.snapshot()
        return known and held == state.gripper and tuple(cargo.values()) == state.slots

    def _record_mission_failure(self, step, error, skipped=()):
        if not hasattr(self, '_mission_failures'):
            self._mission_failures = []
        message = f'{step.step_id} ({step.action}, tag {step.tag_id}): {error}'
        if skipped:
            message += '; ignorados: ' + ', '.join(
                f'{task.step_id}/tag {task.tag_id}' for task in skipped)
        self._mission_failures.append((step.step_id, message))
        self.get_logger().error(message)

    def _abandon_scheduled_object(self, scheduler, state, step, error):
        if step.action == 'retrieve':
            raise RetrieveFailed(
                f'Retirada da tag {step.tag_id} do slot {step.slot_id} falhou; '
                f'o objeto não pode ser abandonado na carga. {error}') from error
        if step.action == 'store':
            scheduler.blocked_transfers.add((step.action, step.slot_id))
        source = (step.source_area or (self._current_location if step.action == 'pick'
                  else state.gripper_source if state.gripper == step.tag_id
                  else next((src for tag, src in zip(state.slots, state.slot_sources)
                             if tag == step.tag_id), None)))
        state, skipped = scheduler.abandon(state, step.tag_id, source)
        self._record_mission_failure(step, error, skipped)
        return state

    def _skip_visit_work(self, scheduler, state, error):
        for index, (vi, task, _group) in enumerate(scheduler.tasks):
            if vi == state.visit and not state.done & (1 << index):
                state = self._abandon_scheduled_object(scheduler, state, task, error)
        return state

    def _retain_failed_held_object(self, goal_handle, plan, visit):
        """Try each free cargo slot; retain a held object if storage also fails."""
        known, held, cargo = self._world_state.snapshot()
        if not known:
            raise StepFailed('Carga incerta após falha; movimentos suspensos.')
        if held == EMPTY:
            return
        for slot, tag in cargo.items():
            if tag != EMPTY:
                continue
            step = Step(f'{visit.visit_id}_retain', 'store', tag_id=held, slot_id=slot)
            self._report_scheduled_operation(goal_handle, plan, step, step.step_id,
                                             f'Preservando cubo {held} na carga')
            error = self._retry_operation(lambda: self._execute_step(step), step.step_id,
                                         expected=SimpleNamespace(gripper=EMPTY,
                                             slots=tuple(held if name == slot else value
                                                         for name, value in cargo.items())))
            if error is None:
                return
            self._record_mission_failure(step, error)

