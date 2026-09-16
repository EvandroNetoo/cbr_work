"""Regression tests for the manipulation server executor lifecycle."""

from types import SimpleNamespace

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
