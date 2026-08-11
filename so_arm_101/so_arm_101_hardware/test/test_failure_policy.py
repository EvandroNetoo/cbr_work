"""Unit tests for the physical driver's fail-fast communication policy."""

import pytest

from so_arm_101_hardware.hardware_node import SO101HardwareNode


class _Logger:
    def __init__(self):
        self.fatal_messages = []
        self.error_messages = []

    def fatal(self, message):
        self.fatal_messages.append(message)

    def error(self, message, **_kwargs):
        self.error_messages.append(message)


def _node(limit=5):
    node = SO101HardwareNode.__new__(SO101HardwareNode)
    node._max_consecutive_io_failures = limit
    node._read_failures = 0
    node._write_failures = 0
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def test_io_failure_is_tolerated_before_limit():
    node = _node()
    for _ in range(4):
        node._handle_io_failure('ler', OSError('temporário'), '_read_failures')
    assert node._read_failures == 4
    assert not node._logger.fatal_messages


def test_fifth_consecutive_io_failure_raises():
    node = _node()
    for _ in range(4):
        node._handle_io_failure('ler', OSError('temporário'), '_read_failures')
    with pytest.raises(RuntimeError):
        node._handle_io_failure('ler', OSError('permanente'), '_read_failures')
    assert node._read_failures == 5
    assert node._logger.fatal_messages


def test_successful_operation_resets_counter_contract():
    node = _node()
    node._read_failures = 4
    node._reset_io_failure('_read_failures')
    assert node._read_failures == 0
