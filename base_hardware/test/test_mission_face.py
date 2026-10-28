"""Mission-to-OLED behavior, late joining DDS, and recovery without real I2C."""

import time
from pathlib import Path
from types import SimpleNamespace

from interfaces.msg import ManipulationState
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from base_hardware.mission_face import expression_for_mission, validate_expression
from base_hardware.mission_face_node import MissionFaceNode


class FakeFace:
    def __init__(self):
        self.frames = []
        self.cleaned = False
        self.fail = False
        self.tela = SimpleNamespace(display=SimpleNamespace(cleanup=self.cleanup))

    def atualizar(self, expression):
        if self.fail:
            raise OSError('OLED desconectado')
        self.frames.append(expression)

    def cleanup(self):
        self.cleaned = True


def state(operation='', known=True):
    message = ManipulationState()
    message.active_operation = operation
    message.state_known = known
    return message


@pytest.mark.parametrize('operation, expected', [
    (operation, 'comemorar' if operation == 'finish' else operation) for operation in (
        'navigate', 'finish', 'follow_wall', 'analyze_scene', 'pick', 'store',
        'retrieve', 'place_on_table', 'place_in_container', 'place_on_shelf',
        'place_on_precision_table', 'stack', 'organize_precision',
    )
] + [('', 'neutro'), ('future_operation', 'neutro')])

def test_mission_operation_selects_expression(operation, expected):
    assert expression_for_mission(operation, True) == expected


def test_default_yaml_and_mapping_select_celebration_for_finish():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'face.yaml').read_text())
    operations = config['mission_face_node']['ros__parameters']['expressions']['operations']
    assert operations['finish'] == expression_for_mission('finish', True) == 'comemorar'


def test_uncertain_inventory_overrides_running_expression():
    assert expression_for_mission('navigate', False) == 'triste'
    assert expression_for_mission('place_on_table', False) == 'triste'
    assert expression_for_mission('', False) == 'triste'


def test_invalid_face_name_identifies_yaml_parameter():
    with pytest.raises(ValueError, match=r'expressions.operations.navigate.*felizz.*correndo'):
        validate_expression('expressions.operations.navigate', 'felizz')


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    context = Context()
    rclpy.init(context=context)
    yield context
    context.try_shutdown()


def spin_until(executor, condition, timeout=4.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return
    assert condition(), 'O estado esperado não chegou pelo DDS.'


def test_yaml_changes_operation_idle_uncertainty_and_disconnect(ros_context, tmp_path):
    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'face.yaml').read_text())
    # Exercise the configured idle expression independently of the menu mode.
    config['mission_face_node']['ros__parameters']['show_menu_when_idle'] = False
    expressions = config['mission_face_node']['ros__parameters']['expressions']
    expressions.update(idle='feliz', unknown_state='surpreso',
                       unknown_operation='piscando', disconnected='bravo')
    expressions['operations']['navigate'] = 'dormindo'
    config_file = tmp_path / 'custom_face.yaml'
    config_file.write_text(yaml.safe_dump(config))
    face = FakeFace()
    node = MissionFaceNode(
        face_factory=lambda **kwargs: face, context=ros_context,
        cli_args=['--ros-args', '--params-file', str(config_file)])
    try:
        node._animate()
        assert face.frames[-1] == 'feliz'
        for message, expected in (
            (state('navigate'), 'dormindo'),
            (state('store'), 'store'),
            (state('navigate', known=False), 'surpreso'),
            (state('future_operation'), 'piscando'),
            (state(), 'feliz'),
        ):
            node._on_mission_state(message)
            node._animate()
            assert face.frames[-1] == expected
        node._check_publisher()
        node._animate()
        assert face.frames[-1] == 'bravo'
    finally:
        node.destroy_node()


def test_late_subscriber_animates_retained_state_and_returns_to_idle(ros_context):
    publisher_node = Node('test_mission_publisher', context=ros_context)
    face_node = None
    executor = SingleThreadedExecutor(context=ros_context)
    face = FakeFace()
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = publisher_node.create_publisher(ManipulationState, '/mission/state', qos)
        # Missão já em andamento quando a base entra no grafo.
        publisher.publish(state('navigate'))
        face_node = MissionFaceNode(face_factory=lambda **kwargs: face, context=ros_context)
        executor.add_node(face_node)
        spin_until(executor, lambda: face.frames.count('navigate') >= 5)
        assert face_node._subscription.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL

        publisher.publish(state('pick'))
        spin_until(executor, lambda: face.frames[-1:] == ['pick'])
        publisher.publish(state('place_on_table'))
        spin_until(executor, lambda: face.frames[-1:] == ['place_on_table'])
        publisher.publish(state('navigate', known=False))
        spin_until(executor, lambda: face.frames[-1:] == ['triste'])
        publisher.publish(state('navigate'))
        spin_until(executor, lambda: face.frames[-1:] == ['navigate'])
        # Sem heartbeat, uma operação longa continua animando normalmente.
        before = len(face.frames)
        spin_until(executor, lambda: len(face.frames) >= before + 25)
        assert face.frames[-1] == 'navigate'

        publisher_node.destroy_publisher(publisher)
        spin_until(executor, lambda: face.frames[-1:] == ['neutro'])
    finally:
        executor.shutdown()
        if face_node is not None:
            face_node.destroy_node()
        publisher_node.destroy_node()
    assert face.cleaned


def test_display_failure_retries_and_renders_latest_mission(ros_context, monkeypatch):
    import base_hardware.mission_face_node as module

    now = [100.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    calls = []
    first, recovered = FakeFace(), FakeFace()

    def factory(**kwargs):
        calls.append(kwargs)
        if len(calls) == 1:
            raise OSError('I2C ausente')
        return first if len(calls) == 2 else recovered

    node = MissionFaceNode(face_factory=factory, context=ros_context)
    try:
        node._animate()
        node._on_mission_state(state('navigate'))
        now[0] += 1
        node._animate()
        assert len(calls) == 1
        now[0] += 4
        node._animate()
        assert first.frames == ['navigate']
        assert calls[-1] == {'i2c_bus': 0, 'i2c_address': 0x3C, 'fps': 20.0}

        first.fail = True
        node._animate()
        assert first.cleaned
        node._on_mission_state(state('pick'))
        now[0] += 5
        node._animate()
        assert recovered.frames == ['pick']
    finally:
        node.destroy_node()
    assert recovered.cleaned


def test_menu_owns_idle_display_and_face_reserves_it_only_during_mission(ros_context):
    import fcntl
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool

    faces = []
    def factory(**kwargs):
        face = FakeFace()
        faces.append(face)
        return face
    node = MissionFaceNode(
        face_factory=factory, context=ros_context,
        parameter_overrides=[Parameter('show_menu_when_idle', value=True)])
    try:
        node._animate()
        assert faces == []
        with open('/tmp/cbr_mission_face.lock', 'r') as probe:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(probe, fcntl.LOCK_UN)
            node._on_mission_active(Bool(data=True))
            with pytest.raises(BlockingIOError):
                fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            node._animate()
            assert len(faces) == 1 and faces[0].frames
            node._on_mission_active(Bool(data=False))
            assert faces[0].cleaned
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(probe, fcntl.LOCK_UN)
            node._animate()
            assert len(faces) == 1
            node._on_mission_active(Bool(data=True))
            node._animate()
            assert len(faces) == 2
    finally:
        node.destroy_node()
    with open('/tmp/cbr_mission_face.lock', 'r') as probe:
        fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
