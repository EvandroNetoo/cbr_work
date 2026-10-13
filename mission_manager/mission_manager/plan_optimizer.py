"""Commander import and exact route search, independent of ROS.

Search includes manipulation states: a cargo slot cannot substitute for a free
gripper, and a carried gripper object must be delivered at the next work visit.
Only navigation length is optimized; perception and lateral motions are unknown.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time
from typing import Callable

from .errors import ConfigurationError
from .models import Arena


@dataclass(frozen=True)
class Transport:
    tag: int
    source: str
    destination: str
    action: str
    container_color: str | None = None
    reference_tag_id: int | None = None
    support_tag_id: int | None = None
    conditional_colors: tuple[str, ...] = ()

    def delivery(self):
        task = {'action': self.action, 'tag_ids': [self.tag]}
        for key in ('container_color', 'reference_tag_id', 'support_tag_id'):
            value = getattr(self, key)
            if value is not None:
                task[key] = value
        return task

    def deliveries(self):
        if self.conditional_colors:
            return [{'action': 'place_in_container', 'possible_tag_ids': [self.tag],
                     'tag_color': color, 'container_color': color}
                    for color in self.conditional_colors]
        return [self.delivery()]


@dataclass(frozen=True)
class Mission:
    transports: tuple[Transport, ...]
    organization: tuple[str, dict[int, int]] | None = None

    @property
    def targets(self):
        if self.organization:
            return (self.organization[0],)
        return tuple(sorted({t.source for t in self.transports} |
                            {t.destination for t in self.transports}))


def import_commander(raw, arena: Arena, pp_references=None) -> Mission:
    """Read the team export only. Never infer hidden colors from the judge file."""
    if not isinstance(raw, dict) or raw.get('task_id') not in {
            'BMT', 'BTT1', 'BTT2', 'AMT', 'ATT1', 'ATT2', 'FINAL'}:
        raise ConfigurationError('Informe um YAML de equipe do Commander (task_id válido).')
    active = raw.get('active_service_areas')
    if (not isinstance(active, list) or not active
            or any(not isinstance(a, str) for a in active) or len(set(active)) != len(active)):
        raise ConfigurationError('active_service_areas inválidas.')
    if set(active) - arena.service_areas.keys():
        raise ConfigurationError(f'Áreas ausentes da arena: {sorted(set(active) - arena.service_areas.keys())}.')
    objects = raw.get('objects')
    if not isinstance(objects, list) or not objects:
        raise ConfigurationError('objects deve ser uma lista não vazia.')
    tags = []
    for obj in objects:
        if (not isinstance(obj, dict) or set(obj) != {'id'}
                or type(obj['id']) is not int or not 1 <= obj['id'] <= 30):
            raise ConfigurationError('Cada objeto deve conter somente id inteiro entre 1 e 30.')
        tags.append(obj['id'])
    if len(set(tags)) != len(tags):
        raise ConfigurationError('IDs de objetos duplicados.')

    states = []
    for name in ('start_state', 'finish_state'):
        state = raw.get(name)
        if not isinstance(state, dict) or set(state) != set(active):
            raise ConfigurationError(f'{name} deve declarar exatamente as áreas ativas.')
        locations = {}
        for area, entry in state.items():
            if (not isinstance(entry, dict) or set(entry) - {'obj_ids', 'cont_colors'}
                    or not isinstance(entry.get('obj_ids'), list)):
                raise ConfigurationError(f'{name}.{area} inválido.')
            for tag in entry['obj_ids']:
                if type(tag) is not int or tag not in tags or tag in locations:
                    raise ConfigurationError(f'{name}: objeto inválido ou duplicado: {tag}.')
                locations[tag] = area
            colors = entry.get('cont_colors', [])
            if (not isinstance(colors, list) or any(c not in ('RED', 'BLUE') for c in colors)
                    or len(set(colors)) != len(colors)):
                raise ConfigurationError(f'{name}.{area}.cont_colors inválido.')
        if set(locations) != set(tags):
            raise ConfigurationError(f'{name}: conjunto de objetos diferente de objects.')
        states.append(locations)
    sources, destinations = states
    containers = raw.get('containers', [])
    if not isinstance(containers, list):
        raise ConfigurationError('containers inválido.')
    by_area = {a: [] for a in active}
    used_colors = set()
    for container in containers:
        if (not isinstance(container, dict) or set(container) != {'color', 'at'}
                or container['color'] not in ('RED', 'BLUE') or container['at'] not in active
                or container['color'] in used_colors):
            raise ConfigurationError('Container inválido ou cor duplicada.')
        used_colors.add(container['color'])
        by_area[container['at']].append(container['color'])
    for state_name in ('start_state', 'finish_state'):
        for area in active:
            if set(raw[state_name][area].get('cont_colors', [])) != set(by_area[area]):
                raise ConfigurationError(f'{state_name}.{area}: containers inconsistentes.')

    references = {tag: tag for tag in tags}
    if set(pp_references or {}) - set(tags):
        raise ConfigurationError('--pp-reference menciona cubo ausente de objects.')
    references.update(pp_references or {})
    if any(type(k) is not int or type(v) is not int or v < 0 for k, v in references.items()):
        raise ConfigurationError('Referências PP inválidas.')
    if raw['task_id'] == 'AMT':
        if len(active) != 1 or arena.service_areas[active[0]].area_type != 'PP':
            raise ConfigurationError('AMT exige uma única área PP.')
        final = {references[tag]: tag for tag in tags}
        if len(final) != len(tags):
            raise ConfigurationError('Referências PP duplicadas.')
        # The export omits empty cavities, so discover occupancy at runtime.
        return Mission((), (active[0], final))

    transports = []
    for tag in sorted(tags):
        source, destination = sources[tag], destinations[tag]
        area_type = arena.service_areas[destination].area_type
        if source == destination and area_type != 'SH' and not by_area[destination]:
            # BMT's unsolicited fourth cube stays untouched.
            continue
        colors = by_area[destination]
        if colors:
            transports.append(Transport(tag, source, destination, 'place_in_container',
                                        container_color=colors[0].lower() if len(colors) == 1 else None,
                                        conditional_colors=tuple(c.lower() for c in colors)
                                        if len(colors) > 1 else ()))
        elif area_type == 'PP':
            transports.append(Transport(tag, source, destination, 'place_on_precision_table',
                                        reference_tag_id=references[tag]))
        elif area_type == 'SH':
            transports.append(Transport(tag, source, destination, 'place_on_shelf'))
        elif area_type == 'ST':
            # obj_ids preserves the bottom-to-top order supplied by Commander.
            pile = raw['finish_state'][destination]['obj_ids']
            index = pile.index(tag)
            transports.append(Transport(tag, source, destination,
                                        'stack' if index else 'place_on_table',
                                        support_tag_id=pile[index - 1] if index else None))
        else:
            transports.append(Transport(tag, source, destination, 'place_on_table'))
    if not transports:
        raise ConfigurationError('A missão não contém transportes executáveis.')
    pp = [(t.destination, t.reference_tag_id) for t in transports if t.reference_tag_id is not None]
    if len(set(pp)) != len(pp):
        raise ConfigurationError('Mais de um objeto destinado à mesma referência PP.')
    return Mission(tuple(transports))


@dataclass(frozen=True)
class SearchState:
    location: str
    picked: int = 0
    delivered: int = 0
    gripper: int = -1
    normalized: int = 0


@dataclass(frozen=True)
class OptimizationResult:
    document: dict
    distance_m: float
    expanded_states: int


def optimize(mission: Mission, distance: Callable[[str, str], float], *,
             plan_id: str, objective='visits_then_distance', cargo_capacity=2,
             time_limit=60.0, max_states=500000) -> OptimizationResult:
    """Dijkstra search; fail explicitly on limits, never label a heuristic optimal."""
    if objective not in {'visits', 'distance', 'visits_then_distance'}:
        raise ConfigurationError('Objetivo desconhecido.')
    if cargo_capacity < 1 or not math.isfinite(time_limit) or time_limit <= 0 or max_states < 1:
        raise ConfigurationError('Capacidade e limites devem ser positivos.')
    base = {'schema_version': 2, 'plan_id': plan_id, 'initial_location': 'start', 'finish': True}
    if mission.organization:
        target, final = mission.organization
        base['visits'] = [{'target': target, 'tasks': [{'final_state': final}]}]
        meters = distance('start', target) + distance(target, 'finish')
        if not math.isfinite(meters) or meters < 0:
            raise ConfigurationError('Rota AMT inacessível pelo Nav2.')
        return OptimizationResult(base, meters, 0)
    items = mission.transports
    count = len(items)
    all_done = (1 << count) - 1
    by_tag = {t.tag: i for i, t in enumerate(items)}
    distances = {}

    def length(a, b):
        if (a, b) not in distances:
            value = distance(a, b)
            if math.isnan(value) or value < 0:
                raise ConfigurationError(f'Distância inválida: {a} → {b}.')
            distances[a, b] = value
        return distances[a, b]

    def cost(visits, meters):
        return (meters, visits) if objective == 'distance' else (visits, meters)

    initial = SearchState('start')
    serial = itertools.count()
    pending = [(cost(0, 0.0), next(serial), initial)]
    best = {initial: cost(0, 0.0)}
    parents = {initial: (None, None)}
    deadline = time.monotonic() + time_limit
    expanded = 0
    terminal = None
    while pending:
        if time.monotonic() >= deadline or expanded >= max_states:
            raise ConfigurationError(
                f'Limite de otimização atingido ({expanded} estados). '
                'Aumente --time-limit/--max-states; nenhum plano parcial foi gravado.')
        current_cost, _, state = heapq.heappop(pending)
        if best[state] != current_cost:
            continue
        if state.location == 'finish':
            terminal = state
            break
        expanded += 1
        meters, visits = (current_cost if objective == 'distance' else current_cost[::-1])
        edges = []
        onboard = state.picked & ~state.delivered
        stored = onboard & ~(1 << state.gripper if state.gripper >= 0 else 0)
        if state.delivered == all_done:
            edges.append((SearchState('finish', state.picked, state.delivered),
                          ('finish', None), 0, length(state.location, 'finish')))
        elif state.gripper < 0:
            for i, item in enumerate(items):
                bit = 1 << i
                if item.source == state.location and not state.picked & bit:
                    edges.append((SearchState(state.location, state.picked | bit,
                                              state.delivered, i, state.normalized),
                                  ('pick', i), 0, 0.0))
                if stored & bit:
                    normalized = state.normalized | (bit if item.action == 'place_on_precision_table' else 0)
                    edges.append((SearchState(state.location, state.picked,
                                              state.delivered, i, normalized),
                                  ('retrieve', i), 0, 0.0))
        else:
            i = state.gripper
            bit = 1 << i
            item = items[i]
            if stored.bit_count() < cargo_capacity:
                edges.append((SearchState(state.location, state.picked, state.delivered,
                                          -1, state.normalized), ('store', i), 0, 0.0))
            support = by_tag.get(item.support_tag_id)
            supported = support is None or state.delivered & (1 << support)
            if (item.destination == state.location and supported
                    and (item.action != 'place_on_precision_table' or state.normalized & bit)):
                edges.append((SearchState(state.location, state.picked, state.delivered | bit,
                                          -1, state.normalized & ~bit), ('deliver', i), 0, 0.0))
        if state.delivered != all_done:
            targets = set()
            for i, item in enumerate(items):
                bit = 1 << i
                if not state.picked & bit:
                    targets.add(item.source)
                elif not state.delivered & bit:
                    targets.add(item.destination)
            for target in sorted(targets - {state.location}):
                if state.gripper >= 0 and items[state.gripper].destination != target:
                    continue
                edges.append((SearchState(target, state.picked, state.delivered,
                                          state.gripper, state.normalized),
                              ('navigate', target), 1, length(state.location, target)))
        for child, operation, add_visits, add_meters in edges:
            if not math.isfinite(add_meters):
                continue
            candidate = cost(visits + add_visits, meters + add_meters)
            if child not in best or candidate < best[child]:
                best[child] = candidate
                parents[child] = (state, operation)
                heapq.heappush(pending, (candidate, next(serial), child))
    if terminal is None:
        raise ConfigurationError('Nenhuma rota viável para a missão e as distâncias disponíveis.')
    operations = []
    cursor = terminal
    while parents[cursor][0] is not None:
        cursor, operation = parents[cursor]
        operations.append(operation)
    visits_out = []
    for action, value in reversed(operations):
        if action == 'navigate':
            visits_out.append({'target': value, 'tasks': []})
        elif action in {'pick', 'deliver'}:
            item = items[value]
            tasks = visits_out[-1]['tasks']
            additions = [{'action': 'pick', 'tag_ids': [item.tag]}] if action == 'pick' else item.deliveries()
            for task in additions:
                field = 'possible_tag_ids' if 'possible_tag_ids' in task else 'tag_ids'
                group = {k: v for k, v in task.items() if k != field}
                previous = next((t for t in tasks if {k: v for k, v in t.items() if k != field} == group), None)
                if previous is None:
                    tasks.append(task)
                else:
                    previous[field].extend(task[field])
    base['visits'] = visits_out
    total = best[terminal][0 if objective == 'distance' else 1]
    return OptimizationResult(base, total, expanded)
