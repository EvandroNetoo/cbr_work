"""OLED da base: snapshot de missão via DDS e animação em um processo separado."""

import math
import time

from interfaces.msg import ManipulationState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .mission_face import OPERATION_EXPRESSIONS, expression_for_mission, validate_expression


def create_face(*, i2c_bus, i2c_address, fps):
    # A importação é tardia: dependência ou OLED ausente não derruba a base.
    from .face_animation import ExpressoesFaciais
    from .tela import Tela

    return ExpressoesFaciais(Tela(i2c_bus=i2c_bus, i2c_address=i2c_address), fps=fps)


class MissionFaceNode(Node):
    def __init__(self, face_factory=None, **kwargs):
        super().__init__('mission_face_node', **kwargs)
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('i2c_bus', 0)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('retry_interval_sec', 5.0)
        self._special_expressions = {}
        for state, default in (
            ('idle', 'neutro'), ('unknown_state', 'triste'),
            ('unknown_operation', 'neutro'), ('disconnected', 'neutro'),
        ):
            name = f'expressions.{state}'
            self.declare_parameter(name, default)
            self._special_expressions[state] = validate_expression(
                name, self.get_parameter(name).value)
        self._operation_expressions = {}
        for operation, default in OPERATION_EXPRESSIONS.items():
            name = f'expressions.operations.{operation}'
            self.declare_parameter(name, default)
            self._operation_expressions[operation] = validate_expression(
                name, self.get_parameter(name).value)
        self._fps = float(self.get_parameter('fps').value)
        self._retry_interval = float(self.get_parameter('retry_interval_sec').value)
        for name, value in (('fps', self._fps), ('retry_interval_sec', self._retry_interval)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} deve ser positivo e finito.')
        self._bus = int(self.get_parameter('i2c_bus').value)
        self._address = int(self.get_parameter('i2c_address').value)
        if self._bus < 0 or not 0x03 <= self._address <= 0x77:
            raise ValueError('Barramento ou endereço I2C inválido.')

        self._factory = face_factory or create_face
        self._face = None
        self._next_attempt = 0.0
        self._expression = self._special_expressions['idle']
        self._state_seen = False
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic = str(self.get_parameter('mission_state_topic').value)
        self._subscription = self.create_subscription(
            ManipulationState, topic, self._on_mission_state, qos)
        # A tela segue animando mesmo sem novas mensagens ou com /clock parado.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._animation_timer = self.create_timer(
            1.0 / self._fps, self._animate, clock=self._steady_clock)
        self._discovery_timer = self.create_timer(
            1.0, self._check_publisher, clock=self._steady_clock)
        self.get_logger().info(f'Tela da base acompanha {topic}; aguardando missão.')

    def _on_mission_state(self, message):
        self._state_seen = True
        self._set_expression(expression_for_mission(
            message.active_operation, message.state_known,
            operation_expressions=self._operation_expressions,
            idle=self._special_expressions['idle'],
            unknown_state=self._special_expressions['unknown_state'],
            unknown_operation=self._special_expressions['unknown_operation']))

    def _set_expression(self, expression):
        if expression != self._expression:
            self._expression = expression
            self.get_logger().info(f'Expressão da base: {expression}')

    def _check_publisher(self):
        # O tópico publica mudanças, não um heartbeat: silêncio não significa falha.
        if self._state_seen and self._subscription.get_publisher_count() == 0:
            self._state_seen = False
            self._set_expression(self._special_expressions['disconnected'])

    def _animate(self):
        if self._face is None:
            if time.monotonic() < self._next_attempt:
                return
            try:
                self._face = self._factory(
                    i2c_bus=self._bus, i2c_address=self._address, fps=self._fps)
                self.get_logger().info('Display OLED conectado.')
            except Exception as error:
                self._display_failed(error)
                return
        try:
            self._face.atualizar(self._expression)
        except Exception as error:
            self._display_failed(error)

    def _release_display(self):
        face, self._face = self._face, None
        if face is not None:
            try:
                face.tela.display.cleanup()
            except Exception as error:
                self.get_logger().warning(f'Falha ao liberar OLED: {error}')

    def _display_failed(self, error):
        self._release_display()
        self._next_attempt = time.monotonic() + self._retry_interval
        self.get_logger().warning(
            f'OLED indisponível: {error}. Nova tentativa em {self._retry_interval:g} s.')

    def destroy_node(self):
        self._animation_timer.cancel()
        self._discovery_timer.cancel()
        self._release_display()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFaceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
