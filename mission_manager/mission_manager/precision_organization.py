"""Compile a PP permutation into ordered picks and tag-relative placements.

A null PP slot is a legal temporary buffer. With a full table, one cube can
stay in the robot while its cycle is resolved; Scheduler inserts cargo
store/retrieve operations using the configured capacity.
"""
from __future__ import annotations

from .errors import ConfigurationError
from .models import Step


def organize_precision_slots(
    start: dict[int, int | None], final: dict[int, int | None],
) -> tuple[Step, ...]:
    if not start or set(start) != set(final):
        raise ConfigurationError('start_state e final_state devem ter os mesmos alojamentos, não vazios.')
    for name, state in (('start_state', start), ('final_state', final)):
        if any(isinstance(slot, bool) or not isinstance(slot, int) or slot < 0 for slot in state):
            raise ConfigurationError(f'{name}: IDs de alojamento devem ser inteiros não negativos.')
        cubes = [cube for cube in state.values() if cube is not None]
        if any(isinstance(cube, bool) or not isinstance(cube, int) or cube < 0 for cube in cubes):
            raise ConfigurationError(f'{name}: cubos devem ser IDs inteiros não negativos ou null.')
        if len(cubes) != len(set(cubes)):
            raise ConfigurationError(f'{name}: IDs de cubos duplicados.')
        if set(cubes) & set(state):
            raise ConfigurationError(f'{name}: tags de cubos e de alojamentos devem ser distintas.')
    if {c for c in start.values() if c is not None} != {c for c in final.values() if c is not None}:
        raise ConfigurationError('start_state e final_state devem conter os mesmos cubos; não é possível criar ou remover objetos.')

    slots = sorted(start)
    current = dict(start)
    destination = {cube: slot for slot, cube in final.items() if cube is not None}
    tasks = []
    parked = None

    def pick(source):
        cube = current[source]
        assert cube is not None
        tasks.append(Step('', 'pick', tag_id=cube, reference_tag_id=source))
        current[source] = None
        return cube

    def place(cube, target):
        assert current[target] is None
        tasks.append(Step('', 'place_on_precision_table', tag_id=cube, reference_tag_id=target))
        current[target] = cube

    while current != final or parked is not None:
        if parked is not None and current[destination[parked]] is None:
            place(parked, destination[parked])
            parked = None
            continue
        # Fill a cube's final destination whenever it is already empty.
        source = next((s for s in slots if current[s] is not None
                       and current[s] != final[s]
                       and current[destination[current[s]]] is None), None)
        if source is not None:
            cube = pick(source)
            place(cube, destination[cube])
            continue
        source = next(s for s in slots if current[s] is not None and current[s] != final[s])
        empty = next((s for s in slots if current[s] is None), None)
        cube = pick(source)
        if empty is not None:
            place(cube, empty)
        else:
            # No table buffer: the next pick forces an automatic cargo store.
            assert parked is None
            parked = cube
    return tuple(tasks)
