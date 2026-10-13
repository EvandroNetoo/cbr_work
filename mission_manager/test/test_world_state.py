from mission_manager.errors import StateConflict
from mission_manager.world_state import EMPTY, WorldState
import pytest


def test_complete_pick_store_retrieve_place_transition():
    state = WorldState(['left'])

    state.validate_pick(5)
    state.commit_pick(5)
    state.validate_store(5, 'left')
    state.commit_store(5, 'left')
    assert state.snapshot() == (True, EMPTY, {'left': 5})

    state.validate_retrieve(5, 'left')
    state.commit_retrieve(5, 'left')
    state.validate_place(5)
    state.commit_place()
    assert state.snapshot() == (True, EMPTY, {'left': EMPTY})


def test_new_mission_reset_clears_gripper_slots_and_uncertainty():
    state = WorldState(['left', 'right'])
    state.commit_pick(7)
    state.commit_store(7, 'left')
    state.mark_unknown()

    state.reset()

    assert state.snapshot() == (
        True,
        EMPTY,
        {'left': EMPTY, 'right': EMPTY},
    )


def test_left_and_right_slots_keep_independent_objects():
    state = WorldState(['left', 'right'])
    state.commit_pick(1)
    state.validate_store(1, 'left')
    state.commit_store(1, 'left')
    state.commit_pick(2)
    state.validate_store(2, 'right')
    state.commit_store(2, 'right')

    assert state.snapshot() == (
        True,
        EMPTY,
        {'left': 1, 'right': 2},
    )

    state.validate_retrieve(2, 'right')
    state.commit_retrieve(2, 'right')
    assert state.snapshot() == (
        True,
        2,
        {'left': 1, 'right': EMPTY},
    )


def test_cannot_overwrite_an_occupied_slot():
    state = WorldState(['left'])
    state.commit_pick(1)
    state.commit_store(1, 'left')
    state.commit_pick(5)

    with pytest.raises(StateConflict, match='ocupado'):
        state.validate_store(5, 'left')


@pytest.mark.parametrize('first_slot', ['left', 'right'])
def test_equal_ids_in_separate_slots_can_be_placed_in_either_order(first_slot):
    state = WorldState(['left', 'right'])
    for slot in ('left', 'right'):
        state.validate_pick(1)
        state.commit_pick(1)
        state.validate_store(1, slot)
        state.commit_store(1, slot)
    assert state.snapshot() == (True, EMPTY, {'left': 1, 'right': 1})
    for slot in (first_slot, 'right' if first_slot == 'left' else 'left'):
        state.validate_retrieve(1, slot)
        state.commit_retrieve(1, slot)
        state.validate_place(1)
        state.commit_place()
    assert state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})


def test_duplicate_tag_colors_follow_their_own_slots():
    state = WorldState(['left', 'right'])
    state.remember_tag_color(3, 'red', 'WS_1')
    state.commit_pick(3, 'WS_1')
    state.commit_store(3, 'left')
    state.remember_tag_color(3, 'blue', 'WS_2')
    state.commit_pick(3, 'WS_2')
    assert state.gripper_color() == 'blue'
    state.commit_store(3, 'right')
    for slot, color in [('left', 'red'), ('right', 'blue')]:
        state.commit_retrieve(3, slot)
        assert state.gripper_color() == color
        state.commit_place()
        assert state.gripper_color() is None
    assert state.object_colors() == {('WS_1', 3): 'red', ('WS_2', 3): 'blue'}
    state.reset()
    assert state.object_colors() == {}


def test_actions_can_infer_objects_from_gripper_and_slot():
    state = WorldState(['left'])
    state.commit_pick(7)
    assert state.require_gripper_object() == 7
    state.commit_store(7, 'left')
    assert state.require_slot_object('left') == 7


def test_inference_rejects_empty_gripper_and_slot():
    state = WorldState(['left'])
    with pytest.raises(StateConflict, match='garra está vazia'):
        state.require_gripper_object()
    with pytest.raises(StateConflict, match='está vazio'):
        state.require_slot_object('left')


def test_cannot_retrieve_the_wrong_object():
    state = WorldState(['left'])
    state.commit_pick(1)
    state.commit_store(1, 'left')

    with pytest.raises(StateConflict, match='não o objeto 5'):
        state.validate_retrieve(5, 'left')


def test_unknown_state_is_explicit_and_persistent():
    state = WorldState(['left'])
    state.mark_unknown()
    assert state.snapshot()[0] is False
    with pytest.raises(StateConflict, match='incerto'):
        state.validate_pick(1)


@pytest.mark.parametrize('operation', ['pick', 'store', 'retrieve', 'place'])
def test_negative_object_id_is_rejected_for_every_operation(operation):
    state = WorldState(['left'])
    arguments = {
        'pick': (-1,),
        'store': (-1, 'left'),
        'retrieve': (-1, 'left'),
        'place': (-1,),
    }
    with pytest.raises(StateConflict, match='não pode ser negativo'):
        getattr(state, f'validate_{operation}')(*arguments[operation])
