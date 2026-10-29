"""A carinha permanece na última ação relevante, sem pilha nem restauração."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from interfaces.action import (
    AnalyzeScene, FollowWall, PickObject, PlaceInContainer, PlaceOnPrecisionTable,
    PlaceOnShelf, PlaceOnTable, PrepareManipulator, RetrieveObject, StackObject, StoreObject,
)
from interfaces.msg import ManipulationFeedback
from nav2_msgs.action import NavigateToPose
import pytest

from mission_manager.errors import MissionCanceled, StepFailed
from mission_manager.live_operations import LiveOperations
from mission_manager.models import Step
from mission_manager.node import MissionManager
from mission_manager.world_state import WorldState


DECISION_ACTIONS = [
    (PlaceOnShelf, 'place_on_shelf'),
    (RetrieveObject, 'retrieve'),
    (StoreObject, 'store'),
]
PERCEPTION_ACTIONS = [
    (PickObject, 'pick'),
    (PlaceInContainer, 'place_in_container'),
    (PlaceOnPrecisionTable, 'place_on_precision_table'),
    (PlaceOnTable, 'place_on_table'),
    (StackObject, 'stack'),
]
MANIPULATION_ACTIONS = [*PERCEPTION_ACTIONS, *DECISION_ACTIONS]


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
@pytest.mark.parametrize('action, operation', MANIPULATION_ACTIONS)
def test_ending_child_action_keeps_last_face_and_ignores_late_feedback(
    display_manager, failure, action, operation,
):
    manager, states = display_manager
    callbacks = []

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        callbacks.append(kwargs['feedback_callback'])
        observing = action.Feedback()
        observing.status.phase = ManipulationFeedback.OBSERVING
        kwargs['feedback_callback'](SimpleNamespace(feedback=observing))
        raise failure('injected failure')

    manager._call_action_impl = call
    with pytest.raises(failure):
        manager._call_action(object(), action.Goal(), operation, 1)
    assert manager._live_operation_tracker().current() == 'analyze_scene'
    expected = ([operation] if (action, operation) in DECISION_ACTIONS else []) + ['analyze_scene']
    assert states == expected
    late = action.Feedback()
    late.status.phase = ManipulationFeedback.GRASPING
    callbacks[0](SimpleNamespace(feedback=late))
    assert states == expected


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


@pytest.mark.parametrize('action, operation', MANIPULATION_ACTIONS)
def test_feedback_before_dispatch_is_not_overwritten(display_manager, action, operation):
    manager, states = display_manager

    def call(client, goal, description, timeout, validator, **kwargs):
        observing = action.Feedback()
        observing.status.phase = ManipulationFeedback.OBSERVING
        kwargs['feedback_callback'](SimpleNamespace(feedback=observing))
        kwargs['on_goal_dispatched']()
        assert manager._live_operation_tracker().current() == 'analyze_scene'
        return object()

    manager._call_action_impl = call
    manager._call_action(object(), action.Goal(), 'manipulação', 1)
    assert states == ['analyze_scene']


@pytest.mark.parametrize('action, operation', DECISION_ACTIONS)
def test_decision_changes_face_before_alignment_preparation_or_dispatch(
    display_manager, action, operation,
):
    manager, states = display_manager
    tracker = manager._live_operation_tracker()
    tracker.dispatched(tracker.pending('analyze_scene'))
    manager._completed_steps = 0
    manager._feedback = lambda *_: None
    manager._report_scheduled_operation(
        object(), SimpleNamespace(total_steps=1), Step('task', operation), 'task', operation)
    assert states == [operation]

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        assert manager._state_message().active_operation == operation
        return object()

    manager._call_action_impl = call
    manager._call_action(object(), FollowWall.Goal(), 'alinhamento', 1)
    manager._call_action(object(), PrepareManipulator.Goal(), 'preparação', 1)
    manager._call_action(object(), action.Goal(), 'manipulação', 1)
    assert states == [operation], 'Preparação e conclusão não devem trocar o rosto escolhido.'

    manager._call_action_impl = lambda *args, **kwargs: kwargs['on_goal_dispatched']()
    manager._call_action(object(), AnalyzeScene.Goal(), 'nova análise independente', 1)
    assert states == [operation, 'analyze_scene'], 'Uma nova ação relevante deve substituir o rosto.'


@pytest.mark.parametrize('action, operation', DECISION_ACTIONS)
def test_actions_without_perception_start_early_but_any_observation_shows_analysis(
    display_manager, action, operation,
):
    manager, states = display_manager
    tracker = manager._live_operation_tracker()
    tracker.dispatched(tracker.pending('analyze_scene'))
    forwarded, dispatched = [], []
    phases = [
        (ManipulationFeedback.VALIDATING, [operation]),
        (ManipulationFeedback.PREPARING, [operation]),
        (ManipulationFeedback.OBSERVING, [operation, 'analyze_scene']),
        (ManipulationFeedback.OBSERVING, [operation, 'analyze_scene']),
        (ManipulationFeedback.PREPARING, [operation, 'analyze_scene']),
        (ManipulationFeedback.APPROACHING, [operation, 'analyze_scene', operation]),
        (ManipulationFeedback.GRASPING, [operation, 'analyze_scene', operation]),
        (ManipulationFeedback.RELEASING, [operation, 'analyze_scene', operation]),
        (ManipulationFeedback.RETREATING, [operation, 'analyze_scene', operation]),
        (ManipulationFeedback.RECOVERING, [operation, 'analyze_scene', operation]),
    ]

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        assert states == [operation], 'O rosto deve aparecer antes do primeiro feedback.'
        for phase, expected in phases:
            message = action.Feedback()
            message.status.phase = phase
            kwargs['feedback_callback'](SimpleNamespace(feedback=message))
            assert states == expected
        return object()

    manager._call_action_impl = call
    manager._call_action(
        object(), action.Goal(), operation, 1,
        on_goal_dispatched=lambda: dispatched.append(True),
        feedback_callback=lambda msg: forwarded.append(msg.feedback.status.phase))
    assert states == [operation, 'analyze_scene', operation]
    assert dispatched == [True]
    assert forwarded == [phase for phase, _ in phases]


@pytest.mark.parametrize('action, operation', PERCEPTION_ACTIONS)
def test_perception_keeps_analysis_until_destination_is_found_and_approach_begins(
    display_manager, action, operation,
):
    manager, states = display_manager
    tracker = manager._live_operation_tracker()
    tracker.dispatched(tracker.pending('navigate'))
    manager._completed_steps = 0
    manager._feedback = lambda *_: None
    manager._report_scheduled_operation(
        object(), SimpleNamespace(total_steps=1), Step('task', operation), 'task', operation)
    assert states == ['navigate'], 'Escolher a entrega não deve antecipar o rosto de depósito.'
    states.clear()
    phases = [
        (ManipulationFeedback.VALIDATING, []),
        (ManipulationFeedback.PREPARING, []),
        (ManipulationFeedback.OBSERVING, ['analyze_scene']),
        (ManipulationFeedback.OBSERVING, ['analyze_scene']),
        (ManipulationFeedback.PREPARING, ['analyze_scene']),
        (ManipulationFeedback.APPROACHING, ['analyze_scene', operation]),
        (ManipulationFeedback.RELEASING, ['analyze_scene', operation]),
        (ManipulationFeedback.OBSERVING, ['analyze_scene', operation, 'analyze_scene']),
        (ManipulationFeedback.APPROACHING, ['analyze_scene', operation, 'analyze_scene', operation]),
    ]

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        assert states == [], 'Despachar uma ação com percepção não inicia o depósito.'
        for phase, expected in phases:
            message = action.Feedback()
            message.status.phase = phase
            kwargs['feedback_callback'](SimpleNamespace(feedback=message))
            assert states == expected
        return object()

    manager._call_action_impl = call
    manager._call_action(object(), action.Goal(), operation, 1)
    assert manager._state_message().active_operation == operation
    assert states == ['analyze_scene', operation, 'analyze_scene', operation]


@pytest.mark.parametrize('action, operation', [
    (StackObject, 'stack'), (PlaceOnPrecisionTable, 'place_on_precision_table'),
    (PickObject, 'pick'),
])
def test_alignment_feedback_cannot_select_operation_before_final_execution(
    display_manager, action, operation,
):
    manager, states = display_manager
    goal = action.Goal()
    if action is PickObject:
        goal.profile = 'shelf_front'
        goal.alignment_completed = False
    else:
        goal.require_alignment = True
    tracker = manager._live_operation_tracker()
    tracker.dispatched(tracker.pending('navigate'))
    forwarded, callbacks = [], []
    phases = [ManipulationFeedback.OBSERVING, ManipulationFeedback.APPROACHING,
              ManipulationFeedback.GRASPING, ManipulationFeedback.RETREATING,
              ManipulationFeedback.RELEASING]
    aligning = [True]

    def call(client, goal, description, timeout, validator, **kwargs):
        kwargs['on_goal_dispatched']()
        callbacks.append(kwargs['feedback_callback'])
        for phase in phases:
            message = action.Feedback()
            message.status.phase = phase
            kwargs['feedback_callback'](SimpleNamespace(feedback=message))
            if aligning[0]:
                assert states == ['analyze_scene'], 'Alinhamento não pode mostrar execução do braço.'
        return object()

    manager._call_action_impl = call
    manager._call_action(
        object(), goal, 'localizar para alinhar', 1,
        feedback_callback=lambda msg: forwarded.append(msg.feedback.status.phase))
    assert states == ['analyze_scene']
    assert forwarded == phases, 'O filtro visual deve preservar os callbacks da execução.'

    manager._call_action_impl = lambda *args, **kwargs: kwargs['on_goal_dispatched']()
    manager._call_action(object(), FollowWall.Goal(), 'centralização da base', 1)
    assert states == ['analyze_scene']

    aligning[0] = False
    if action is PickObject:
        goal.alignment_completed = True
    else:
        goal.require_alignment = False
    manager._call_action_impl = call
    manager._call_action(object(), goal, 'confirmar alvo e executar braço', 1)
    assert states == ['analyze_scene', operation]

    late = action.Feedback()
    late.status.phase = ManipulationFeedback.APPROACHING
    callbacks[0](SimpleNamespace(feedback=late))
    assert states == ['analyze_scene', operation], 'Feedback antigo não deve interferir na execução final.'
