"""ROS-independent finite-state search for a fixed route and flexible tasks.

Physical transfers are part of the search, so a locally legal pick is offered
only when the remaining mission can still finish. Observations rank choices;
they never change inventory or make missing objects logically disappear.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable, Mapping

from .errors import ConfigurationError
from .models import Plan, Step
from .world_state import EMPTY


@dataclass(frozen=True)
class State:
    visit: int
    done: int
    gripper: int
    slots: tuple[int, ...]
    # One entry per tag, containing the areas of all off-board copies.
    # Equal IDs in different areas represent interchangeable physical cubes.
    locations: tuple[tuple[str, ...], ...]
    tops: tuple[int, ...]
    regrasped: int = 0
    colors: tuple[str | None, ...] = ()
    gripper_source: str | None = None
    slot_sources: tuple[str | None, ...] = ()


@dataclass(frozen=True)
class Choice:
    step: Step
    next_state: State
    task_index: int | None = None
    task_id: str = ''


class Scheduler:
    def __init__(self, plan: Plan, slot_ids: tuple[str, ...],
                 check_canceled: Callable[[], None] = lambda: None):
        if any(not slot for slot in slot_ids) or len(set(slot_ids)) != len(slot_ids):
            raise ConfigurationError('cargo_slot_ids inválidos.')
        self.plan = plan
        self.slot_ids = slot_ids
        self.check_canceled = check_canceled
        self.tag_colors: dict[int | tuple[str | None, int], str] = {}
        self.tasks: list[tuple[int, Step, int]] = []
        self.masks = []
        self.groups: list[tuple[int, Step]] = []
        sources = {}
        for visit in plan.visits:
            for task in visit.tasks:
                if task.action == 'pick':
                    for tag in task.tag_ids or (task.tag_id,):
                        sources.setdefault(tag, set()).add(visit.target)
        self.sources = sources
        supports = {}
        for vi, visit in enumerate(plan.visits):
            mask = 0
            for task in visit.tasks:
                group = -1
                tags = task.tag_ids or task.possible_tag_ids or (task.tag_id,)
                if task.action == 'stack':
                    group = len(self.groups)
                    self.groups.append((vi, task))
                    tags = task.tag_ids
                    supports.setdefault(task.support_tag_id, visit.target)
                for tag in tags:
                    # A color filter applies independently to every physical
                    # copy, even when several source areas use the same ID.
                    areas = sorted(sources.get(tag, ())) if task.tag_color else [None]
                    for area in areas or [None]:
                        atomic = replace(task, tag_id=tag, tag_ids=(), source_area=area)
                        mask |= 1 << len(self.tasks)
                        self.tasks.append((vi, atomic, group))
            self.masks.append(mask)
        self.tags = tuple(sorted({t.tag_id for _, t, _ in self.tasks} | set(supports)))
        self.tag_indices = {tag: i for i, tag in enumerate(self.tags)}
        self.color_keys = tuple((area, tag) for tag in self.tags
                                for area in sorted(sources.get(tag, ())) or [None])
        self.color_indices = {key: i for i, key in enumerate(self.color_keys)}
        self.initial_state = State(0, 0, EMPTY, (EMPTY,) * len(slot_ids),
                                   tuple(tuple(sorted(sources[tag])) if tag in sources
                                         else (supports[tag],) if tag in supports else ()
                                         for tag in self.tags),
                                   tuple(task.support_tag_id for _, task in self.groups),
                                   colors=(None,) * len(self.color_keys),
                                   slot_sources=(None,) * len(slot_ids))
        self._memo: dict[State, bool] = {}
        self.skipped_mask = 0
        self.fallback_mask = 0
        self.blocked_transfers: set[tuple[str, str]] = set()
        self.recovering = False

    def object_pending(self, state: State, tag: int, source: str | None) -> bool:
        return any(not state.done & (1 << index) and task.tag_id == tag
                   and (task.source_area is None or task.source_area == source)
                   for index, (_vi, task, _group) in enumerate(self.tasks))

    def accept_fallback(self, state: State, choice: Choice) -> Choice:
        """A stack fallback becomes the new support; PP fallback leaves its slot empty."""
        self.recovering = True
        if choice.step.action == 'stack':
            # The normal next state already selects this cube as the group's top.
            self._memo.clear()
            return choice
        if choice.task_index is not None:
            self.skipped_mask |= 1 << choice.task_index
            self.fallback_mask |= 1 << choice.task_index
        self._memo.clear()
        return replace(choice, next_state=replace(choice.next_state, tops=state.tops))

    def retry_pending(self, state: State) -> State:
        """Reopen abandoned work without replaying confirmed physical transfers."""
        pending = self.skipped_mask & ~self.fallback_mask
        self.skipped_mask = self.fallback_mask
        self.blocked_transfers.clear()
        self._memo.clear()
        return replace(state, visit=0, done=state.done & ~pending)

    def abandon(self, state: State, tag: int, source: str | None = None) -> tuple[State, tuple[Step, ...]]:
        """Disable a failed object's remaining work and unavailable stack supports.

        Inventory is deliberately retained. A skipped task is not evidence of
        a physical transfer, nor of a new top on a stack.
        """
        self.recovering = True
        removed = []
        unavailable = {tag}
        while True:
            added = False
            for index, (vi, task, _group) in enumerate(self.tasks):
                bit = 1 << index
                if state.done & bit or vi < state.visit:
                    continue
                same_object = (task.tag_id in unavailable and (
                    task.tag_id != tag or source is None or (
                        task.source_area is None and (
                            task.action != 'pick' or len(self.sources.get(tag, ())) <= 1
                            or self.plan.visits[vi].target == source))
                    or task.source_area == source))
                dependent = task.action == 'stack' and task.support_tag_id in unavailable
                if not same_object and not dependent:
                    continue
                state = replace(state, done=state.done | bit)
                self.skipped_mask |= bit
                removed.append(task)
                if dependent:
                    unavailable.add(task.tag_id)
                added = True
            if not added:
                break
        self._memo.clear()
        return state, tuple(removed)

    def set_tag_colors(self, colors: Mapping[int | tuple[str | None, int], str]) -> None:
        if dict(colors) != self.tag_colors:
            self.tag_colors = dict(colors)
            self._memo.clear()

    def _color_index(self, task: Step) -> int:
        area = task.source_area
        if area is None:
            area = next(iter(sorted(self.sources.get(task.tag_id, ()))), None)
        return self.color_indices[(area, task.tag_id)]

    def _matches_source(self, state: State, task: Step) -> bool:
        return (len(self.sources.get(task.tag_id, ())) <= 1
                or task.source_area == state.gripper_source)

    def complete(self, state: State) -> bool:
        return state.visit == len(self.plan.visits)

    def _available_support(self, state: State, tag: int, target: str) -> bool:
        if target not in state.locations[self.tag_indices[tag]]:
            return False
        # A lower layer is no longer a free support after stacking above it.
        for gi, (vi, group) in enumerate(self.groups):
            if self.plan.visits[vi].target != target:
                continue
            used = {task.tag_id for index, (_v, task, g) in enumerate(self.tasks)
                    if g == gi and state.done & (1 << index) and not self.skipped_mask & (1 << index)}
            if used and tag in used | {group.support_tag_id} and state.tops[gi] != tag:
                return False
        return True

    def choices(self, state: State, *, speculative: bool = False) -> tuple[Choice, ...]:
        if self.complete(state):
            return ()
        visit = self.plan.visits[state.visit]
        colors = list(state.colors or (None,) * len(self.color_keys))
        for ti, key in enumerate(self.color_keys):
            color = self.tag_colors.get(key, self.tag_colors.get(
                (None, key[1]), self.tag_colors.get(key[1])))
            if color is not None:
                if colors[ti] is not None and colors[ti] != color:
                    return ()
                colors[ti] = color
        if state.done & self.masks[state.visit] == self.masks[state.visit]:
            # Transit-only visits keep their navigation but do not constrain
            # the held object's destination. Stop at the next manipulation
            # visit so a loaded gripper cannot block its pending work.
            next_work_visit = next((vi for vi in range(state.visit + 1, len(self.plan.visits))
                                    if self.masks[vi] & ~state.done or self.plan.visits[vi].pp_final_state is not None), len(self.plan.visits))
            next_tags = {task.tag_id for vi, task, _ in self.tasks
                         if vi == next_work_visit and task.action != 'pick'
                         and (task.tag_color is None or (
                             self._matches_source(state, task)
                             and colors[self._color_index(task)] in {None, task.tag_color}))}
            dynamic_ready = (visit.pp_final_state is None or visit.pp_start_state is not None
                             or (state.gripper == EMPTY and state.slots.count(EMPTY) >= 2))
            inactive_held = (self.recovering and not self.object_pending(
                state, state.gripper, state.gripper_source))
            if dynamic_ready and (state.gripper == EMPTY or state.gripper in next_tags or inactive_held):
                if state.visit + 1 < len(self.plan.visits) or (
                        (state.gripper == EMPTY or inactive_held) and all(
                            tag == EMPTY or tag not in self.tag_indices or (
                                self.recovering and not self.object_pending(state, tag, source))
                            for tag, source in zip(state.slots, state.slot_sources))):
                    return (Choice(Step(visit.visit_id, 'depart'),
                                   replace(state, visit=state.visit + 1), task_id=visit.visit_id),)
        result = []
        for index, (vi, task, group) in enumerate(self.tasks):
            if vi != state.visit or state.done & (1 << index):
                continue
            if visit.pp_start_state is not None:
                # Organization steps encode occupied/free PP slots. Do not let
                # perception ranking reorder them and place over another cube.
                earlier = self.masks[state.visit] & ((1 << index) - 1)
                if state.done & earlier != earlier:
                    continue
            tag = task.tag_id
            task_colors = list(colors)
            if task.tag_color is not None:
                ti = self._color_index(task)
                color = task_colors[ti]
                if color != task.tag_color:
                    if color is not None or speculative:
                        skipped_colors = list(task_colors)
                        skipped_colors[ti] = color or ('blue' if task.tag_color == 'red' else 'red')
                        result.append(Choice(replace(task, action='skip'),
                                             replace(state, done=state.done | (1 << index),
                                                     colors=tuple(skipped_colors)), index, task.step_id))
                    if color is not None or not speculative:
                        continue
                    task_colors[ti] = task.tag_color
            locations = list(state.locations)
            tops = list(state.tops)
            regrasped = state.regrasped
            if task.action == 'pick':
                if state.gripper != EMPTY or visit.target not in locations[self.tag_indices[tag]]:
                    continue
                if not self._available_support(state, tag, visit.target):
                    continue
                remaining = list(locations[self.tag_indices[tag]])
                remaining.remove(visit.target)
                locations[self.tag_indices[tag]] = tuple(remaining)
                regrasped &= ~(1 << self.tag_indices[tag])
                gripper = tag
                gripper_source = visit.target
            else:
                if state.gripper != tag or (task.tag_color and not self._matches_source(state, task)):
                    continue
                if (task.action == 'place_on_precision_table'
                        and not state.regrasped & (1 << self.tag_indices[tag])):
                    continue
                if group >= 0:
                    support = tops[group]
                    if not self._available_support(state, support, visit.target):
                        continue
                    task = replace(task, support_tag_id=support)
                    tops[group] = tag
                locations[self.tag_indices[tag]] = tuple(sorted(
                    (*locations[self.tag_indices[tag]], visit.target)))
                gripper = EMPTY
                gripper_source = None
            result.append(Choice(task, replace(state, done=state.done | (1 << index),
                                               gripper=gripper, locations=tuple(locations),
                                               tops=tuple(tops), regrasped=regrasped,
                                               colors=tuple(task_colors),
                                               gripper_source=gripper_source), index, task.step_id))
        if state.gripper != EMPTY:
            if EMPTY in state.slots:
                slot = next((i for i, tag in enumerate(state.slots)
                             if tag == EMPTY and ('store', self.slot_ids[i]) not in self.blocked_transfers), None)
                if slot is None:
                    return tuple(result)
                slots = list(state.slots)
                slots[slot] = state.gripper
                slot_sources = list(state.slot_sources or (None,) * len(state.slots))
                slot_sources[slot] = state.gripper_source
                result.append(Choice(Step('auto_store', 'store', tag_id=state.gripper,
                                          slot_id=self.slot_ids[slot]),
                                     replace(state, gripper=EMPTY, slots=tuple(slots),
                                             gripper_source=None, slot_sources=tuple(slot_sources))))
        else:
            for slot, tag in enumerate(state.slots):
                if (tag != EMPTY and tag in self.tag_indices
                        and ('retrieve', self.slot_ids[slot]) not in self.blocked_transfers
                        and (not self.recovering or self.object_pending(
                            state, tag, state.slot_sources[slot]))):
                    slots = list(state.slots)
                    slots[slot] = EMPTY
                    slot_sources = list(state.slot_sources or (None,) * len(state.slots))
                    gripper_source = slot_sources[slot]
                    slot_sources[slot] = None
                    result.append(Choice(Step('auto_retrieve', 'retrieve', tag_id=tag,
                                              slot_id=self.slot_ids[slot]),
                                         replace(state, gripper=tag, slots=tuple(slots),
                                                 gripper_source=gripper_source,
                                                 slot_sources=tuple(slot_sources),
                                                 regrasped=state.regrasped | (1 << self.tag_indices[tag]))))
        return tuple(result)

    def feasible(self, state: State) -> bool:
        """Memoized reachability with explicit cycle detection for cargo swaps."""
        if state in self._memo:
            return self._memo[state]
        parents = {state: None}
        furthest = state
        pending = [state]
        while pending:
            self.check_canceled()
            current = pending.pop()
            if (current.visit, current.done.bit_count()) > (furthest.visit, furthest.done.bit_count()):
                furthest = current
            if self.complete(current) or self._memo.get(current) is True:
                while current is not None:
                    self._memo[current] = True
                    current = parents[current]
                return True
            if self._memo.get(current) is False:
                continue
            # Stable task ordering also makes preflight search reproducible.
            for choice in reversed(self.choices(current, speculative=True)):
                child = choice.next_state
                if child not in parents:
                    parents[child] = current
                    pending.append(child)
        self._failure_state = furthest
        self._memo.update((visited, False) for visited in parents)
        return False

    def infeasibility_reason(self, state: State) -> str:
        """Explain an impossible plan with task IDs, tags and inventory evidence."""
        deliveries = {task.tag_id for _vi, task, _group in self.tasks if task.action != 'pick'}
        if not any(visit.pp_final_state is not None for visit in self.plan.visits):
            missing = [(vi, task) for vi, task, _group in self.tasks
                       if task.action == 'pick' and task.tag_id not in deliveries]
            if missing:
                details = '; '.join(
                    f"tag {task.tag_id} coletada na visita {vi + 1} "
                    f"'{self.plan.visits[vi].visit_id}' ({self.plan.visits[vi].target}), "
                    f"tarefa '{task.step_id}', sem nenhuma tarefa de entrega no plano"
                    for vi, task in missing)
                return (f'Missão inviável: {details}. Adicione a tag à entrega no destino '
                        'desejado ou remova sua coleta; a missão exige carga e garra vazias ao terminar.')
        self.feasible(state)
        blocked = getattr(self, '_failure_state', state)
        visit = self.plan.visits[blocked.visit]
        inventory = {slot: tag for slot, tag in zip(self.slot_ids, blocked.slots) if tag != EMPTY}
        held = 'vazia' if blocked.gripper == EMPTY else f'tag {blocked.gripper}'
        details = []
        for index, (vi, task, group) in enumerate(self.tasks):
            if vi != blocked.visit or blocked.done & (1 << index):
                continue
            tag = task.tag_id
            location = blocked.locations[self.tag_indices[tag]]
            if task.action == 'pick':
                if visit.target not in location:
                    reason = f'tag está em {location!r}, não em {visit.target!r}'
                elif not self._available_support(blocked, tag, visit.target):
                    reason = f'tag {tag} tem outro cubo empilhado acima'
                elif blocked.gripper != EMPTY:
                    reason = f'garra ocupada pela tag {blocked.gripper}'
                else:
                    reason = 'a coleta não permite concluir as tarefas restantes'
            elif group >= 0 and not self._available_support(blocked, blocked.tops[group], visit.target):
                support = blocked.tops[group]
                reason = (f'apoio {support} indisponível em {visit.target}; '
                          f'localização do apoio: {blocked.locations[self.tag_indices[support]]!r} '
                          '(pode estar coberto por outro cubo)')
            elif blocked.gripper != tag:
                reason = (f'tag {tag} está no slot {next(slot for slot, value in inventory.items() if value == tag)!r}'
                          if tag in inventory.values() else f'tag {tag} não está na carga; localização: {location!r}')
            elif task.action == 'place_on_precision_table' and not blocked.regrasped & (1 << self.tag_indices[tag]):
                reason = 'depósito PP exige guardar no slot e retirar novamente antes de depositar'
            else:
                reason = 'esta tarefa não permite concluir o restante da missão'
            details.append(f"tarefa '{task.step_id}' ({task.action}, tag {tag}): {reason}")
        if not details:
            remaining = [tag for tag in (blocked.gripper, *blocked.slots) if tag != EMPTY]
            if remaining:
                next_visit = next((v for v in range(blocked.visit + 1, len(self.plan.visits))
                                   if self.masks[v] or self.plan.visits[v].pp_final_state is not None), None)
                details.append(f'carga restante com tags {remaining}')
                if next_visit is not None:
                    details.append(f'garra não pode sair carregada para a próxima visita de trabalho '
                                   f'{next_visit + 1} ({self.plan.visits[next_visit].target}) '
                                   'sem uma entrega correspondente nessa visita')
                else:
                    details.append('não há próximas entregas para esvaziar a carga')
            if visit.pp_final_state is not None and visit.pp_start_state is None:
                details.append('organização PP exige garra vazia e pelo menos dois slots livres')
        capacity = (f'capacidade: {len(self.slot_ids)} slots; livres: {blocked.slots.count(EMPTY)}')
        return (f"Missão inviável: nenhuma sequência de operações conclui o plano. "
                f"Estado alcançável na visita {blocked.visit + 1} '{visit.visit_id}' ({visit.target}): "
                f"garra {held}; carga {inventory}; {capacity}. " + '; '.join(details) + '.')

    def viable_choices(self, state: State) -> tuple[Choice, ...]:
        choices = self.choices(state)
        viable = tuple(choice for choice in choices if self.feasible(choice.next_state))
        # After a failure, execute independent work even when retained cargo
        # prevents completing the original route's full set of deliveries.
        return viable or (choices if self.recovering else ())

    def select(self, state: State, observations: Mapping[tuple[str, int | str], float],
               lateral_position: float = 0.0, *, allow_unobserved: bool = False) -> Choice | None:
        """Rank legal tasks by held delivery, perception, travel, and stable IDs."""
        candidates = []
        for choice in self.viable_choices(state):
            step = choice.step
            if step.action in {'depart', 'skip'}:
                return choice
            if step.action == 'store':
                # Store only to enable pending work or satisfy departure policy.
                if any(c.step.action not in {'store', 'retrieve', 'skip'} for c in self.viable_choices(state)):
                    continue
                candidates.append(((3, 0, '', 0), choice))
                continue
            if step.action == 'retrieve':
                # Retrieve only for a pending, feasible delivery at this visit.
                deliveries = [c for c in self.viable_choices(choice.next_state)
                              if c.task_index is not None and c.step.action not in {'pick', 'skip'}]
                if not deliveries:
                    continue
                for delivery in deliveries:
                    rank = self._rank(delivery.step, observations, lateral_position)
                    if rank is not None or allow_unobserved:
                        rank = rank or (2, 0.0)
                        candidates.append(((rank[0], rank[1], delivery.task_id,
                                            delivery.step.tag_id), replace(choice, task_id=delivery.task_id)))
                continue
            if step.action != 'pick':
                candidates.append(((-1, 0.0, choice.task_id, step.tag_id), choice))
                continue
            rank = self._rank(step, observations, lateral_position)
            if rank is None and allow_unobserved:
                rank = (2, 0.0)
            if rank is not None:
                candidates.append(((rank[0], rank[1], choice.task_id, step.tag_id), choice))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    @staticmethod
    def _rank(step: Step, observations, lateral_position):
        if step.action in {'place_on_table', 'place_on_shelf'}:
            return (0, 0.0)
        if step.action == 'place_on_precision_table':
            key = ('reference', step.reference_tag_id)
        elif step.action == 'place_in_container':
            key = ('container', step.container_color)
        else:
            key = ('tag', step.support_tag_id if step.action == 'stack' else step.tag_id)
        if key not in observations:
            return None
        distance = abs(observations[key] - lateral_position)
        return (0 if distance <= 1.0 else 1, distance)
