"""Regression tests for the manipulation server executor lifecycle."""

from types import SimpleNamespace
import threading

import pytest
from interfaces.action import PickObject, StoreObject, RetrieveObject, PlaceOnShelf
from interfaces.msg import ManipulationResult

from manipulation import node as manipulation_node

from rclpy.exceptions import InvalidHandle


def test_spin_tolerates_entity_destroyed_during_wait_set_update(monkeypatch):
    """A transient invalid handle must not stop subsequent executor spins."""
    ok_states = iter((True, True, True, False))
    monkeypatch.setattr(manipulation_node.rclpy, 'ok', lambda: next(ok_states))

    spin_calls = []

    def spin_once():
        spin_calls.append(True)
        if len(spin_calls) == 1:
            raise InvalidHandle('destruction was requested')

    debug_messages = []
    executor = SimpleNamespace(spin_once=spin_once)
    node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(debug=debug_messages.append)
    )

    manipulation_node._spin_executor(executor, node)

    assert len(spin_calls) == 2
    assert debug_messages == [
        'Entidade ROS removida durante a atualização do executor: '
        'destruction was requested'
    ]


def test_callback_exception_keeps_executor_running(monkeypatch):
    states = iter((True, True, True, False))
    monkeypatch.setattr(manipulation_node.rclpy, 'ok', lambda: next(states))
    calls = []
    errors = []
    def spin():
        calls.append(True)
        if len(calls) == 1:
            raise RuntimeError('callback failure')
    manipulation_node._spin_executor(
        SimpleNamespace(spin_once=spin),
        SimpleNamespace(get_logger=lambda: SimpleNamespace(error=errors.append)))
    assert len(calls) == 2
    assert 'callback failure' in errors[0]


@pytest.mark.parametrize('action,operation,location', [
    (PickObject, 'pick', ManipulationResult.LOCATION_SOURCE),
    (StoreObject, 'store', ManipulationResult.LOCATION_GRIPPER),
    (RetrieveObject, 'retrieve', ManipulationResult.LOCATION_CARGO),
    (PlaceOnShelf, 'place_on_shelf', ManipulationResult.LOCATION_GRIPPER),
])
def test_operation_and_monitor_cleanup_failures_release_server(action, operation, location):
    server = manipulation_node.ManipulationServer.__new__(manipulation_node.ManipulationServer)
    server._lock = threading.RLock()
    server._cancel_event = threading.Event()
    server._busy = True
    server._set_active = lambda _: None
    server._ensure_moveit = lambda: None
    errors = []
    server.get_logger = lambda: SimpleNamespace(error=errors.append)
    def cleanup():
        raise RuntimeError('cleanup failure')
    server._motion = SimpleNamespace(iniciar_monitoramento_dos_estados=lambda: None,
                                    aguardar_primeiro_estado=lambda: True,
                                    parar_monitoramento_dos_estados=cleanup)
    def fail():
        raise RuntimeError('operation failure')
    aborted = []
    result = server._run(action, SimpleNamespace(abort=lambda: aborted.append(True)), operation, fail)
    assert result.outcome.code != ManipulationResult.SUCCESS
    assert result.outcome.effect_known
    assert result.outcome.final_object_location == location
    assert not server._busy and server._active_operation == ''
    assert aborted == [True]
    assert any('cleanup failure' in message for message in errors)


def test_exception_before_operation_returns_result_and_releases_guard():
    server = manipulation_node.ManipulationServer.__new__(manipulation_node.ManipulationServer)
    server._lock = threading.RLock()
    server._cancel_event = threading.Event()
    server._busy = True
    server.get_logger = lambda: SimpleNamespace(error=lambda *_: None)
    def bad_setup(_handle):
        raise ValueError('invalid setup')
    aborted = []
    callback = server._guard_execute(PickObject, bad_setup)
    result = callback(SimpleNamespace(abort=lambda: aborted.append(True)))
    assert result.outcome.code == ManipulationResult.INTERNAL_ERROR
    assert result.outcome.effect_known
    assert result.outcome.final_object_location == ManipulationResult.LOCATION_SOURCE
    assert not server._busy and not server._executing_guarded
    assert aborted == [True]


def test_guard_keeps_server_reserved_until_result_construction_finishes():
    server = manipulation_node.ManipulationServer.__new__(manipulation_node.ManipulationServer)
    server._lock = threading.RLock()
    server._cancel_event = threading.Event()
    server._busy = True
    server._set_active = lambda _: None
    def callback(handle):
        result = server._run(PickObject, handle, 'pick',
                             lambda: ('picked', ManipulationResult.LOCATION_GRIPPER),
                             requires_moveit=False)
        assert server._busy  # Another goal cannot overwrite this operation's effects yet.
        return result
    result = server._guard_execute(PickObject, callback)(SimpleNamespace(succeed=lambda: None))
    assert result.outcome.code == ManipulationResult.SUCCESS
    assert not server._busy
