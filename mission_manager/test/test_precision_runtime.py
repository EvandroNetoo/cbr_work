"""Exercise unknown initial occupancy and overlapping IDs without moving hardware."""
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

from mission_manager.errors import StepFailed, ConfigurationError
from mission_manager.loaders import load_plan, load_arena, validate_plan
from mission_manager.models import PrecisionPerceptionConfig, Visit, Plan, TagObservation
from mission_manager.precision_perception import split_detections, slot_occupant
from mission_manager.precision_runtime import PrecisionRuntime
from mission_manager.world_state import EMPTY


def tag(identifier, x, z, y=0.0):
    return NS(id=identifier, header=NS(frame_id='arm_base_link'),
              pose=NS(position=NS(x=x, y=y, z=z)))


def test_overlapping_ids_are_independent_and_boundary_is_reference():
    config = PrecisionPerceptionConfig(slot_offset_y_m=0)
    refs, objects = split_detections([tag(1, 0, .035), tag(1, 0, .065)], config)
    assert refs[1].pose.position.z == .035
    assert objects[1].pose.position.z == .065
    assert slot_occupant(1, refs, objects, config) == 1
    assert slot_occupant(1, refs, {}, config) is None
    with pytest.raises(StepFailed, match='ausente'):
        slot_occupant(2, refs, {}, config)


def test_ambiguous_occupancy_and_invalid_frame_stop():
    config = PrecisionPerceptionConfig(slot_offset_y_m=0)
    refs, objects = split_detections([tag(1, 0, .02), tag(2, 0, .065), tag(3, .01, .065)], config)
    with pytest.raises(StepFailed, match='ambígua'):
        slot_occupant(1, refs, objects, config)
    wrong_frame = tag(1, 0, .02)
    wrong_frame.header.frame_id = 'camera'
    with pytest.raises(StepFailed, match='arm_base_link'):
        split_detections([wrong_frame], config)


def test_final_only_loader_and_pp_validation(tmp_path):
    path = tmp_path / 'plan.yaml'
    path.write_text('schema_version: 2\nplan_id: test\nvisits:\n- target: pp_1\n  tasks:\n  - final_state: {1: 1, 2: 2, 3: 3, 5: 4, 6: 5, 4: 6}\n')
    plan = load_plan(path)
    assert plan.visits[0].pp_start_state is None
    assert dict(plan.visits[0].pp_final_state)[1] == 1
    validate_plan(plan, load_arena(Path(__file__).parents[1] / 'config/arena.yaml'))
    with pytest.raises(ConfigurationError, match='dois compartimentos'):
        load_plan(path, cargo_capacity=1)
    path.write_text(path.read_text().replace('pp_1', 'ws_1'))
    with pytest.raises(ConfigurationError, match='exige área PP'):
        validate_plan(load_plan(path), load_arena(Path(__file__).parents[1] / 'config/arena.yaml'))


class Robot(PrecisionRuntime):
    def __init__(self, board):
        self.board = dict(board)
        self.held = EMPTY
        self.cargo = {'left': EMPTY, 'right': EMPTY}
        self.events = []
        self._completed_steps = 0
        self._delivery_outcomes = []
        self._current_location = 'pp_1'
        self._current_wall_distance_mm = 200
        self._current_lateral_position_mm = 0
        self._tag_observations = {}
        self._pp_reference_observations = {}
        config = PrecisionPerceptionConfig(slot_offset_y_m=0)
        self._arena = NS(precision_perception=config,
                         pickup_recovery=NS(search_positions_mm=(0, 100), safety_search_positions_mm=(),
                                            safety_search_distance_mm=100, wall_tolerance_mm=5, travel_tolerance_mm=5),
                         service_areas={'pp_1': NS(area_type='PP', alignment=NS(distance_mm=200))})
        self._world_state = NS(snapshot=lambda: (True, self.held, dict(self.cargo)))
    def _check_canceled(self): pass
    def _report_scheduled_operation(self, *args): pass
    def _precision_memory_destination(self, memory): return 200, 0
    def _move_to_table_position(self, *args): pass
    def _position_from_memory(self, tag_id): return self._tag_observations.get(('pp_1', tag_id))
    def _observe_visit(self):
        refs = {slot: tag(slot, slot * .1, .02) for slot in self.board}
        objects = {cube: tag(cube, slot * .1, .065) for slot, cube in self.board.items() if cube is not None}
        self._last_pp_scene = ('pp_1', 200, 0, refs, objects)
        self._tag_observations = {('pp_1', cube): TagObservation('pp_1', 200, 0, 200, 0, detection)
                                  for cube, detection in objects.items()}
        self._pp_reference_observations = {('pp_1', slot): TagObservation('pp_1', 200, 0, 200, 0, detection)
                                           for slot, detection in refs.items()}
    def _execute_step(self, step):
        self.events.append((step.action, step.tag_id, step.reference_tag_id, step.slot_id))
        if step.action == 'pick':
            assert self.held == EMPTY
            source = next(slot for slot, cube in self.board.items() if cube == step.tag_id)
            self.board[source] = None
            self.held = step.tag_id
        elif step.action == 'store':
            assert self.cargo[step.slot_id] == EMPTY
            self.cargo[step.slot_id], self.held = self.held, EMPTY
        elif step.action == 'retrieve':
            assert self.held == EMPTY
            self.held, self.cargo[step.slot_id] = self.cargo[step.slot_id], EMPTY
        else:
            assert self.board[step.reference_tag_id] is None
            self.board[step.reference_tag_id], self.held = self.held, EMPTY


def run(board, final):
    robot = Robot(board)
    visit = Visit('pp', 'pp_1', (), None, tuple(final.items()))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert all(robot.board[slot] == cube for slot, cube in final.items())
    assert robot.held == EMPTY and all(cube == EMPTY for cube in robot.cargo.values())
    # Every release follows a retrieve, and each retrieved cube was stored.
    stored = set()
    for i, (action, cube, _, _) in enumerate(robot.events):
        if action == 'store': stored.add(cube)
        if action == 'place_on_precision_table':
            assert cube in stored
            assert robot.events[i - 1][:2] == ('retrieve', cube)
    return robot


def test_full_seven_slots_cycle_with_equal_reference_ids():
    robot = run({1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7, 7: 1}, {i: i for i in range(1, 8)})
    assert sum(event[0] == 'place_on_precision_table' for event in robot.events) == 7


def test_seventh_slot_unspecified_and_full_initial_table():
    robot = run({1: 7, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6}, {i: i for i in range(1, 7)})
    assert robot.board[7] == 7


def test_initial_empty_slots_and_correct_objects_are_preserved():
    robot = run({1: 1, 2: 3, 3: None, 4: 2, 5: None, 6: None, 7: None}, {1: 1, 2: 2, 3: 3})
    assert not any(event[:2] == ('pick', 1) for event in robot.events)


def test_null_target_moves_surplus_to_unspecified_cavity():
    run({1: 2, 2: None, 3: None, 4: None, 5: None, 6: None, 7: None}, {1: None})


def test_absent_required_cube_stops_without_releasing():
    robot = Robot({1: None, 2: None, 3: None, 4: None, 5: None, 6: None, 7: None})
    visit = Visit('pp', 'pp_1', (), None, ((1, 1),))
    with pytest.raises(StepFailed, match='objeto 1 não encontrado'):
        robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert not robot.events


def test_all_references_prescribed_with_null_keeps_extra_cube_on_robot():
    robot = Robot({1: 7, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6})
    final = {i: i for i in range(1, 7)} | {7: None}
    visit = Visit('pp', 'pp_1', (), None, tuple(final.items()))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert robot.board == final
    assert robot.held == EMPTY
    assert 7 in robot.cargo.values()


def test_permutations_with_full_board_and_with_one_unknown_surplus():
    from itertools import permutations
    for permutation in permutations((1, 2, 3, 4)):
        run(dict(zip(range(1, 8), (*permutation, 5, 6, 7))), {i: i for i in range(1, 8)})
        run(dict(zip(range(1, 8), (*permutation, 7, 5, 6))), {i: i for i in range(1, 7)})


def test_missing_reference_is_unknown_even_with_object_seen():
    robot = Robot({1: 2, 2: None, 3: None, 4: None, 5: None, 6: None, 7: None})
    visit = Visit('pp', 'pp_1', (), None, ((8, 2),))
    with pytest.raises(StepFailed, match='referência 8 não encontrado'):
        robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert [event[0] for event in robot.events] == ['pick', 'store']
    assert robot.held == EMPTY and 2 in robot.cargo.values()


def test_capacity_exhaustion_never_picks_another_cube():
    robot = Robot({i: i for i in range(1, 8)})
    visit = Visit('pp', 'pp_1', (), None, tuple((i, None) for i in range(1, 8)))
    with pytest.raises(StepFailed, match='compartimento livre'):
        robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert sum(event[0] == 'pick' for event in robot.events) == 2
    assert robot.held == EMPTY and set(robot.cargo.values()) == {1, 2}


def test_scheduler_keeps_unknown_surplus_in_cargo_during_later_work():
    from dataclasses import replace
    from mission_manager.models import Step
    from mission_manager.scheduler import Scheduler
    plan = Plan('test', (Visit('ws', 'ws_1', (Step('p', 'pick', tag_id=1),
                                                  Step('d', 'place_on_table', tag_id=1))),))
    scheduler = Scheduler(plan, ('left', 'right'))
    state = replace(scheduler.initial_state, slots=(7, EMPTY))
    assert scheduler.feasible(state)
    while not scheduler.complete(state):
        choice = scheduler.select(state, {('tag', 1): 0}, allow_unobserved=True)
        assert choice.step.tag_id != 7
        assert choice.next_state.slots[0] == 7
        state = choice.next_state


def test_cube_is_stored_before_discovering_its_reference_without_release_confirmation():
    robot = Robot({1: 2, 2: None, 3: None, 4: None, 5: None, 6: None, 7: None})
    observe = robot._observe_visit
    timeline = []
    def scene():
        timeline.append('observe')
        observe()
        if len(timeline) == 1:
            robot._last_pp_scene[3].pop(2)
            robot._pp_reference_observations.pop(('pp_1', 2))
    robot._observe_visit = scene
    execute = robot._execute_step
    def step(value):
        timeline.append(value.action)
        execute(value)
    robot._execute_step = step
    visit = Visit('pp', 'pp_1', (), None, ((2, 2),))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert timeline == ['observe', 'pick', 'store', 'observe', 'retrieve', 'place_on_precision_table']
    assert robot.board[2] == 2


def test_occupied_destination_has_no_extra_analysis_between_pick_and_retrieve():
    robot = Robot({1: 2, 2: 1, 3: None, 4: None, 5: None, 6: None, 7: None})
    observe, execute = robot._observe_visit, robot._execute_step
    timeline = []
    def scene():
        timeline.append('observe')
        observe()
    def step(value):
        timeline.append((value.action, value.tag_id))
        execute(value)
    robot._observe_visit, robot._execute_step = scene, step
    visit = Visit('pp', 'pp_1', (), None, ((1, 1), (2, 2)))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    chosen = timeline[1][1]
    occupant = 3 - chosen
    assert timeline == ['observe', ('pick', chosen), ('store', chosen), 'observe',
                        ('pick', occupant), ('store', occupant), ('retrieve', chosen), ('place_on_precision_table', chosen),
                        'observe', ('retrieve', occupant), ('place_on_precision_table', occupant)]
    assert robot.board[1] == 1 and robot.board[2] == 2


def test_cached_reference_missing_after_alignment_searches_and_recovers():
    robot = Robot({i: None for i in range(1, 8)})
    robot._observe_visit()
    observe = robot._observe_visit
    moves = []
    snapshots = 0
    def scene():
        nonlocal snapshots
        snapshots += 1
        observe()
        if snapshots == 1:
            robot._last_pp_scene[3].pop(3)
    robot._last_pp_scene = None
    robot._observe_visit = scene
    robot._move_to_table_position = lambda wall, lateral, *_: moves.append((wall, lateral))
    assert robot._pp_seek_reference(3) is None
    assert moves == [(200, 100)]
    assert snapshots == 2


def test_reference_lost_after_each_alignment_exhausts_search_once():
    robot = Robot({i: None for i in range(1, 8)})
    robot._observe_visit()
    observe = robot._observe_visit
    moves = []
    def move(wall, lateral, *_):
        moves.append((wall, lateral))
        robot._current_wall_distance_mm, robot._current_lateral_position_mm = wall, lateral
        robot._last_pp_scene = None
    def scene():
        observe()
        # Visible at search lateral 100, never visible at the preferred destination 0.
        robot._last_pp_scene = ('pp_1', robot._current_wall_distance_mm,
                                robot._current_lateral_position_mm, *robot._last_pp_scene[3:])
        if robot._current_lateral_position_mm == 0:
            robot._last_pp_scene[3].pop(3)
    robot._last_pp_scene = None
    robot._observe_visit, robot._move_to_table_position = scene, move
    with pytest.raises(StepFailed, match='referência 3 não encontrado após busca'):
        robot._pp_seek_reference(3)
    assert moves == [(200, 100), (200, 0)]


def test_missing_occupant_detection_falls_back_to_object_search():
    robot = Robot({1: 2, **{i: None for i in range(2, 8)}})
    robot._observe_visit()
    robot._last_pp_scene[4].pop(2)
    robot._pp_owned_slots = set()
    assert robot._pp_pick_store(None, None, Visit('pp', 'pp_1', ()), 2, observed=True) == 'left'
    assert robot.cargo['left'] == 2 and robot.held == EMPTY


@pytest.mark.parametrize('full_cargo, needs_buffer', [(False, False), (True, False), (True, True)])
def test_final_occupancy_change_replans_without_losing_cargo(full_cargo, needs_buffer):
    from mission_manager.errors import PrecisionSlotOccupied
    board = ({1: 2, 2: 4, 3: 1, 4: 3} if needs_buffer else
             {1: 2, 2: 3, 3: 1})
    robot = Robot(board | {i: None for i in range(5, 8)})
    final = {i: i for i in range(1, 5 if needs_buffer else 4)}
    observe, execute = robot._observe_visit, robot._execute_step
    refused = False
    if not full_cargo:
        def scene():
            observe()
            if not refused:
                # A missed occupant makes the initial destination look vacant.
                robot._last_pp_scene[4].pop(2, None)
        robot._observe_visit = scene
    def step(value):
        nonlocal refused
        if value.action == 'place_on_precision_table' and not refused:
            refused = True
            if full_cargo:
                # Another cube becomes visible in the previously cleared cavity.
                source = next(slot for slot, cube in robot.board.items() if cube == 3)
                robot.board[source], robot.board[value.reference_tag_id] = None, 3
            assert robot.held == value.tag_id
            assert robot.board[value.reference_tag_id] is not None
            observe()
            raise PrecisionSlotOccupied('Cavidade PP ocupada; depósito recusado.')
        execute(value)
    robot._execute_step = step
    visit = Visit('pp', 'pp_1', (), None, tuple(final.items()))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert refused and all(robot.board[slot] == cube for slot, cube in final.items())
    assert robot.held == EMPTY and all(cube == EMPTY for cube in robot.cargo.values())
    assert robot._pp_inventory_pending is False
    assert len(robot._delivery_outcomes) == sum(e[0] == 'place_on_precision_table' for e in robot.events)
    if needs_buffer:
        assert any(action == 'place_on_precision_table' and final[target] != cube
                   for action, cube, target, _slot in robot.events)


def test_changed_final_scene_reopens_a_previously_correct_slot():
    from mission_manager.errors import PrecisionSlotOccupied
    robot = Robot({1: 2, 2: 1, 3: 3, **{i: None for i in range(4, 8)}})
    execute = robot._execute_step
    refused = False
    def step(value):
        nonlocal refused
        if value.action == 'place_on_precision_table' and not refused:
            refused = True
            robot.board[3], robot.board[value.reference_tag_id] = None, 3
            robot._observe_visit()
            raise PrecisionSlotOccupied('Novo ocupante na análise final.')
        execute(value)
    robot._execute_step = step
    visit = Visit('pp', 'pp_1', (), None, ((1, 1), (2, 2), (3, 3)))
    robot._run_precision_organization(None, Plan('test', (visit,)), visit)
    assert [robot.board[i] for i in (1, 2, 3)] == [1, 2, 3]
    assert robot.held == EMPTY and all(cube == EMPTY for cube in robot.cargo.values())
