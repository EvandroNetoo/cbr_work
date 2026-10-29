"""A carinha permanece na última ação relevante, sem pilha nem restauração."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from interfaces.action import AnalyzeScene, FollowWall, PickObject
from interfaces.msg import ManipulationFeedback
from nav2_msgs.action import NavigateToPose
import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.live_operations import LiveOperations
from mission_manager.node import MissionManager
from mission_manager.world_state import WorldState


@pytest.fixture
def display_manager():
    manager = MissionManager.__new__(MissionManager)
    manager._active_world_operation = 'pick'
    manager._world_state = WorldState(['left', 'right'])
    manager.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    manager.get_parameter = lambda _: SimpleNamespace(value='arm_base_link')
    states = []
    manager._publish_world_state = lambda: states.append(manager._state_message().active_operation)
    return manager, states


def test_navigation_analysis_sideways_analysis_pick_does_not_flash_old_faces(display_manager):
    manager, states = display_manager
    phases, forwarded, dispatched = [], [], []

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        if isinstance(goal, PickObject.Goal):
            for phase in phases:
                feedback = PickObject.Feedback()
                feedback.status.phase = phase
                kwargs['feedback_callback'](SimpleNamespace(feedback=feedback))
        return object()

    manager._call_action_impl = call
    assert manager._state_message().active_operation == '', 'Agendar pick não começa a coleta.'
    manager._call_action(object(), NavigateToPose.Goal(), 'indo WS_1', 1)
    assert states == ['navigate']
    manager._call_action(object(), AnalyzeScene.Goal(), 'observação', 1)
    assert states == ['navigate', 'analyze_scene']
    manager._call_action(object(), FollowWall.Goal(), 'ajuste lateral', 1)
    manager._call_action(object(), AnalyzeScene.Goal(), 'nova observação', 1)
    assert states == ['navigate', 'analyze_scene']
    phases[:] = [ManipulationFeedback.VALIDATING, ManipulationFeedback.PREPARING,
                 ManipulationFeedback.OBSERVING, ManipulationFeedback.OBSERVING,
                 ManipulationFeedback.APPROACHING, ManipulationFeedback.GRASPING,
                 ManipulationFeedback.RECOVERING]
    manager._call_action(
        object(), PickObject.Goal(), 'coleta', 1,
        on_goal_dispatched=lambda: dispatched.append(True),
        feedback_callback=lambda msg: forwarded.append(msg.feedback.status.phase))
    assert states == ['navigate', 'analyze_scene', 'pick']
    assert forwarded == phases
    assert dispatched == [True]
    assert manager._state_message().active_operation == 'pick'


@pytest.mark.parametrize('failure', [MissionCanceled, StepFailed, RuntimeError])
def test_ending_child_action_keeps_last_face_and_ignores_late_feedback(display_manager, failure):
    manager, states = display_manager
    callbacks = []

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        callbacks.append(kwargs['feedback_callback'])
        observing = PickObject.Feedback()
        observing.status.phase = ManipulationFeedback.OBSERVING
        kwargs['feedback_callback'](SimpleNamespace(feedback=observing))
        raise failure('injected failure')

    manager._call_action_impl = call
    with pytest.raises(failure):
        manager._call_action(object(), PickObject.Goal(), 'coleta', 1)
    assert manager._live_operation_tracker().current() == 'analyze_scene'
    assert states == ['analyze_scene']
    late = PickObject.Feedback()
    late.status.phase = ManipulationFeedback.GRASPING
    callbacks[0](SimpleNamespace(feedback=late))
    assert states == ['analyze_scene']


@pytest.mark.parametrize('first_finished', ['navigation', 'analysis'])
def test_finishing_parallel_actions_never_restores_a_previous_face(first_finished):
    tracker = LiveOperations()
    navigation = tracker.pending('navigate')
    analysis = tracker.pending('analyze_scene')
    assert tracker.current() == ''
    assert tracker.dispatched(navigation)
    assert tracker.dispatched(analysis)
    assert tracker.current() == 'analyze_scene'
    first, second = (navigation, analysis) if first_finished == 'navigation' else (analysis, navigation)
    assert not tracker.finished(first)
    assert tracker.current() == 'analyze_scene'
    assert not tracker.finished(second)
    assert tracker.current() == 'analyze_scene'


def test_last_received_event_wins_and_repetition_or_preparation_keeps_the_face():
    tracker = LiveOperations()
    navigation = tracker.pending('navigate')
    transfer = tracker.pending('store')
    tracker.dispatched(navigation)
    assert tracker.dispatched(transfer)
    assert tracker.current() == 'store'
    assert tracker.feedback(transfer, 'analyze_scene')
    assert not tracker.feedback(transfer, 'analyze_scene')
    preparation = tracker.pending('prepare_manipulator')
    assert not tracker.dispatched(preparation)
    tracker.finished(preparation)
    tracker.finished(navigation)
    assert tracker.current() == 'analyze_scene'
    tracker.reset()
    assert tracker.current() == ''
    assert not tracker.feedback(transfer, 'store')


def test_feedback_before_dispatch_is_not_overwritten(display_manager):
    manager, states = display_manager

    def call(client, goal, description, timeout, validator, **kwargs):
        observing = PickObject.Feedback()
        observing.status.phase = ManipulationFeedback.OBSERVING
        kwargs['feedback_callback'](SimpleNamespace(feedback=observing))
        kwargs['on_goal_dispatched']()
        assert manager._live_operation_tracker().current() == 'analyze_scene'
        return object()

    manager._call_action_impl = call
    manager._call_action(object(), PickObject.Goal(), 'coleta', 1)
    assert states == ['analyze_scene']
