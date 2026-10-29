"""O destino mostrado no OLED vem do passo real e é limpo em qualquer resultado."""

from types import SimpleNamespace

import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.models import Step
from mission_manager.node import MissionManager


@pytest.mark.parametrize('target', ['WS_5', 'SH_1', 'PP_67'])
@pytest.mark.parametrize('failure', [None, StepFailed, MissionCanceled])
def test_navigation_target_is_published_before_motion_and_cleared_afterwards(target, failure):
    manager = MissionManager.__new__(MissionManager)
    messages = []
    manager._navigation_target_publisher = SimpleNamespace(publish=lambda msg: messages.append(msg.data))

    def navigate(actual):
        assert actual == target
        assert messages == [target]
        if failure:
            raise failure('injected failure')

    manager._navigate = navigate
    step = Step('travel', 'navigate', target=target)
    if failure:
        with pytest.raises(failure):
            manager._execute_step(step)
    else:
        manager._execute_step(step)
    assert messages == [target, '']


def test_display_feedback_failure_does_not_stop_navigation():
    manager = MissionManager.__new__(MissionManager)
    warnings, motions = [], []

    def broken_publisher(message):
        raise RuntimeError('display feedback unavailable')

    manager._navigation_target_publisher = SimpleNamespace(publish=broken_publisher)
    manager.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    manager._navigate = motions.append
    manager._execute_step(Step('travel', 'navigate', target='WS_1'))
    assert motions == ['WS_1']
    assert len(warnings) == 2
