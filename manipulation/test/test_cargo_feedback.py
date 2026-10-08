"""Keep the cargo APPROACHING feedback after each operation's safe waypoint."""
from types import SimpleNamespace

from interfaces.action import RetrieveObject, StoreObject
from interfaces.msg import ManipulationFeedback
import pytest

from manipulation.node import ManipulationServer


@pytest.mark.parametrize('operation', ['store', 'retrieve'])
def test_cargo_approach_feedback_follows_safe_arm_preparation(operation):
    server = ManipulationServer.__new__(ManipulationServer)
    server._profiles = SimpleNamespace(cargo_slots={
        'left': SimpleNamespace(store_state='deposit_cube_left',
                                safe_state='safe_cube_left', retrieve_state='pick_cube_left')})
    events = []
    server._transfer_state = lambda *_args: events.append('detect_apriltags')
    server._arm_state = lambda state, *_args: events.append(state)
    server._gripper = lambda state, *_args: events.append(state)
    server._feedback = lambda _handle, _action, phase, *_args: events.append(phase)
    server._motion = SimpleNamespace(mover_braco_e_garra_para_estados=
        lambda arm, gripper, *_args: events.extend([arm, gripper]))
    server._record_effect = lambda *_args: None
    server._run = lambda _action, _handle, _operation, execute: execute()
    action = StoreObject if operation == 'store' else RetrieveObject
    goal = action.Goal()
    goal.slot_id = 'left'
    getattr(server, '_execute_' + operation)(SimpleNamespace(request=goal))
    approach = events.index(ManipulationFeedback.APPROACHING)
    assert events.index('detect_apriltags') < approach
    if operation == 'store':
        assert approach < events.index('deposit_cube_left')
    else:
        assert events.index('safe_cube_left') < approach
        assert events.index('pre_grip') < approach < events.index('pick_cube_left')


@pytest.mark.parametrize('slot', ['left', 'right'])
def test_retrieve_waits_for_combined_preparation_before_descent(slot):
    from concurrent.futures import ThreadPoolExecutor
    import threading

    server = ManipulationServer.__new__(ManipulationServer)
    server._profiles = SimpleNamespace(cargo_slots={
        slot: SimpleNamespace(safe_state=f'safe_cube_{slot}', retrieve_state=f'pick_cube_{slot}')})
    preparing, release = threading.Event(), threading.Event()
    events = []

    def combined(arm, gripper, _description):
        events.append((arm, gripper))
        preparing.set()
        assert release.wait(2.0)

    server._motion = SimpleNamespace(mover_braco_e_garra_para_estados=combined)
    server._transfer_state = lambda *_args: events.append('detect_apriltags')
    server._arm_state = lambda state, *_args: events.append(state)
    server._gripper = lambda state, *_args: events.append(state)
    server._feedback = lambda _handle, _action, phase, *_args: events.append(phase)
    server._record_effect = lambda *_args: None
    server._run = lambda _action, _handle, _operation, execute: execute()
    goal = RetrieveObject.Goal()
    goal.slot_id = slot
    with ThreadPoolExecutor(max_workers=1) as executor:
        run = executor.submit(server._execute_retrieve, SimpleNamespace(request=goal))
        assert preparing.wait(1.0)
        assert (f'safe_cube_{slot}', 'pre_grip') in events
        assert ManipulationFeedback.APPROACHING not in events
        assert f'pick_cube_{slot}' not in events
        assert 'grip' not in events
        release.set()
        run.result(timeout=2.0)
    assert events.index((f'safe_cube_{slot}', 'pre_grip')) < events.index(f'pick_cube_{slot}')
    assert events.index(f'pick_cube_{slot}') < events.index('grip') < events.index(f'safe_cube_{slot}')


def test_immediate_store_retrieve_skips_observation_detour_and_reuses_pre_grip():
    server = ManipulationServer.__new__(ManipulationServer)
    server._profiles = SimpleNamespace(cargo_slots={
        'left': SimpleNamespace(store_state='deposit_cube_left',
                                safe_state='safe_cube_left', retrieve_state='pick_cube_left')})
    events = []
    server._transfer_state = lambda *_: events.append('detect_apriltags')
    server._arm_state = lambda state, *_: events.append(state)
    server._gripper = lambda state, *_: events.append(state)
    server._feedback = lambda *_: None
    server._record_effect = lambda *_: None
    server._motion = SimpleNamespace(mover_braco_e_garra_para_estados=
        lambda arm, gripper, *_: events.append((arm, gripper)))
    server._run = lambda _action, _handle, _name, operation: operation()
    server._execute_store(SimpleNamespace(request=StoreObject.Goal(
        slot_id='left', prepare_retrieve=True)))
    server._execute_retrieve(SimpleNamespace(request=RetrieveObject.Goal(slot_id='left')))
    assert events == [
        'detect_apriltags', 'deposit_cube_left', 'open',
        ('safe_cube_left', 'pre_grip'), 'pick_cube_left', 'grip', 'safe_cube_left',
    ]
    assert server._prepared_cargo_retrieve is None


def test_prepared_retrieve_is_not_reused_for_a_different_compartment():
    server = ManipulationServer.__new__(ManipulationServer)
    server._prepared_cargo_retrieve = 'left'
    server._profiles = SimpleNamespace(cargo_slots={
        'right': SimpleNamespace(safe_state='safe_cube_right', retrieve_state='pick_cube_right')})
    events = []
    server._transfer_state = lambda *_: events.append('detect_apriltags')
    server._arm_state = lambda state, *_: events.append(state)
    server._gripper = lambda state, *_: events.append(state)
    server._feedback = lambda *_: None
    server._record_effect = lambda *_: None
    server._motion = SimpleNamespace(mover_braco_e_garra_para_estados=
        lambda arm, gripper, *_: events.append((arm, gripper)))
    server._run = lambda _action, _handle, _name, operation: operation()
    server._execute_retrieve(SimpleNamespace(request=RetrieveObject.Goal(slot_id='right')))
    assert events[:2] == ['detect_apriltags', ('safe_cube_right', 'pre_grip')]
    assert server._prepared_cargo_retrieve is None
