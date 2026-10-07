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
    locations: tuple[str | None, ...]
    tops: tuple[int, ...]


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
        self.tasks: list[tuple[int, Step, int]] = []
        self.masks = []
        self.groups: list[tuple[int, Step]] = []
        sources = {}
        supports = {}
        for vi, visit in enumerate(plan.visits):
            mask = 0
            for task in visit.tasks:
                group = -1
                tags = (task.tag_id,)
                if task.action == 'stack':
                    group = len(self.groups)
                    self.groups.append((vi, task))
                    tags = task.tag_ids
                    supports.setdefault(task.support_tag_id, visit.target)
                for tag in tags:
                    atomic = replace(task, tag_id=tag, tag_ids=())
                    mask |= 1 << len(self.tasks)
                    self.tasks.append((vi, atomic, group))
                    if task.action == 'pick':
                        sources.setdefault(tag, visit.target)
            self.masks.append(mask)
        initial = dict(supports)
        initial.update(sources)
        self.tags = tuple(sorted({t.tag_id for _, t, _ in self.tasks} | set(supports)))
        self.tag_indices = {tag: i for i, tag in enumerate(self.tags)}
        self.initial_state = State(0, 0, EMPTY, (EMPTY,) * len(slot_ids),
                                   tuple(initial.get(tag) for tag in self.tags),
                                   tuple(task.support_tag_id for _, task in self.groups))
        self._memo: dict[State, bool] = {}

    def complete(self, state: State) -> bool:
        return state.visit == len(self.plan.visits)

    def _available_support(self, state: State, tag: int, target: str) -> bool:
        if state.locations[self.tag_indices[tag]] != target:
            return False
        # A lower layer is no longer a free support after stacking above it.
        for gi, (_vi, group) in enumerate(self.groups):
            used = {task.tag_id for index, (_v, task, g) in enumerate(self.tasks)
                    if g == gi and state.done & (1 << index)}
            if used and tag in used | {group.support_tag_id} and state.tops[gi] != tag:
                return False
        return True

    def choices(self, state: State) -> tuple[Choice, ...]:
        if self.complete(state):
            return ()
        visit = self.plan.visits[state.visit]
        if state.done & self.masks[state.visit] == self.masks[state.visit]:
            # Transit-only visits keep their navigation but do not constrain
            # the held object's destination. Stop at the next manipulation
            # visit so a loaded gripper cannot block its pending work.
            next_work_visit = next((vi for vi in range(state.visit + 1, len(self.plan.visits))
                                    if self.masks[vi]), len(self.plan.visits))
            next_tags = {task.tag_id for vi, task, _ in self.tasks
                         if vi == next_work_visit and task.action != 'pick'}
            if state.gripper == EMPTY or state.gripper in next_tags:
                if state.visit + 1 < len(self.plan.visits) or (
                        state.gripper == EMPTY and all(tag == EMPTY for tag in state.slots)):
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
            locations = list(state.locations)
            tops = list(state.tops)
            if task.action == 'pick':
                if state.gripper != EMPTY or locations[self.tag_indices[tag]] != visit.target:
                    continue
                if not self._available_support(state, tag, visit.target):
                    continue
                locations[self.tag_indices[tag]] = None
                gripper = tag
            else:
                if state.gripper != tag:
                    continue
                if group >= 0:
                    support = tops[group]
                    if not self._available_support(state, support, visit.target):
                        continue
                    task = replace(task, support_tag_id=support)
                    tops[group] = tag
                locations[self.tag_indices[tag]] = visit.target
                gripper = EMPTY
            result.append(Choice(task, replace(state, done=state.done | (1 << index),
                                               gripper=gripper, locations=tuple(locations),
                                               tops=tuple(tops)), index, task.step_id))
        if state.gripper != EMPTY:
            if EMPTY in state.slots:
                slot = state.slots.index(EMPTY)
                slots = list(state.slots)
                slots[slot] = state.gripper
                result.append(Choice(Step('auto_store', 'store', tag_id=state.gripper,
                                          slot_id=self.slot_ids[slot]),
                                     replace(state, gripper=EMPTY, slots=tuple(slots))))
        else:
            for slot, tag in enumerate(state.slots):
                if tag != EMPTY:
                    slots = list(state.slots)
                    slots[slot] = EMPTY
                    result.append(Choice(Step('auto_retrieve', 'retrieve', tag_id=tag,
                                              slot_id=self.slot_ids[slot]),
                                         replace(state, gripper=tag, slots=tuple(slots))))
        return tuple(result)

    def feasible(self, state: State) -> bool:
        """Memoized reachability with explicit cycle detection for cargo swaps."""
        if state in self._memo:
            return self._memo[state]
        parents = {state: None}
        pending = [state]
        while pending:
            self.check_canceled()
            current = pending.pop()
            if self.complete(current) or self._memo.get(current) is True:
                while current is not None:
                    self._memo[current] = True
                    current = parents[current]
                return True
            if self._memo.get(current) is False:
                continue
            # Stable task ordering also makes preflight search reproducible.
            for choice in reversed(self.choices(current)):
                child = choice.next_state
                if child not in parents:
                    parents[child] = current
                    pending.append(child)
        self._memo.update((visited, False) for visited in parents)
        return False

    def viable_choices(self, state: State) -> tuple[Choice, ...]:
        return tuple(choice for choice in self.choices(state) if self.feasible(choice.next_state))

    def select(self, state: State, observations: Mapping[tuple[str, int | str], float],
               lateral_position: float = 0.0, *, allow_unobserved: bool = False) -> Choice | None:
        """Rank legal tasks by held delivery, perception, travel, and stable IDs."""
        candidates = []
        for choice in self.viable_choices(state):
            step = choice.step
            if step.action == 'depart':
                return choice
            if step.action == 'store':
                # Store only to enable pending work or satisfy departure policy.
                if any(c.step.action not in {'store', 'retrieve'} for c in self.viable_choices(state)):
                    continue
                candidates.append(((3, 0, '', 0), choice))
                continue
            if step.action == 'retrieve':
                # Retrieve only for a pending, feasible delivery at this visit.
                deliveries = [c for c in self.viable_choices(choice.next_state)
                              if c.task_index is not None and c.step.action != 'pick']
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
            if rank is not None:
                candidates.append(((rank[0], rank[1], choice.task_id, step.tag_id), choice))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    @staticmethod
    def _rank(step: Step, observations, lateral_position):
        if step.action in {'place_on_table', 'place_on_shelf'}:
            return (0, 0.0)
        key = ('container', step.container_color) if step.action == 'place_in_container' else (
            'tag', step.reference_tag_id if step.action == 'place_on_precision_table'
            else step.support_tag_id if step.action == 'stack' else step.tag_id)
        if key not in observations:
            return None
        distance = abs(observations[key] - lateral_position)
        return (0 if distance <= 1.0 else 1, distance)
