"""Low-level LeRobot driver used by the ros2_control SystemInterface."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .lerobot_adapter import (
    LEROBOT_TO_ROS,
    connect_follower,
    make_follower,
    observation_to_ros,
    ros_to_action,
)


ROS_JOINT_ORDER = tuple(LEROBOT_TO_ROS.values())


def resolve_calibration_file(value: str) -> str:
    """Resolve nomes relativos da configuração sem fixar o workspace."""
    if not value:
        return ''
    path = Path(value)
    if path.is_absolute():
        return str(path)
    return str(
        Path(get_package_share_directory('so_arm_101_hardware'))
        / 'config' / path)


class SO101HardwareNode(Node):
    """Expose only the physical driver boundary to the ros2_control plugin."""

    def __init__(self):
        super().__init__('so101_hardware')
        self.declare_parameter('port', '')
        self.declare_parameter('robot_id', 'so101_follower')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('disable_torque', False)
        self.declare_parameter('use_degrees', False)
        # O controller_manager interpola a 30 Hz. Amostrar o último alvo a uma
        # taxa maior evita alias de fase entre dois timers de 30 Hz: um alvo
        # não fica esperando um ciclo inteiro nem é substituído pelo próximo.
        self.declare_parameter('write_rate_hz', 60.0)
        self.declare_parameter('read_rate_hz', 30.0)
        self.declare_parameter('idle_read_rate_hz', 2.0)
        self.declare_parameter('read_idle_timeout_sec', 1.0)
        self.declare_parameter('idle_velocity_threshold', 0.02)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('reconnect_timeout_sec', 30.0)
        self.declare_parameter('state_topic', '/so101_hardware/raw_joint_states')
        self.declare_parameter('command_topic', '/so101_hardware/command_positions')
        self._write_rate_hz = float(
            self.get_parameter('write_rate_hz').value)
        self._active_read_rate_hz = float(
            self.get_parameter('read_rate_hz').value)
        self._idle_read_rate_hz = float(
            self.get_parameter('idle_read_rate_hz').value)
        self._read_idle_timeout = float(
            self.get_parameter('read_idle_timeout_sec').value)
        self._idle_velocity_threshold = float(
            self.get_parameter('idle_velocity_threshold').value)
        for name, value in (
            ('write_rate_hz', self._write_rate_hz),
            ('read_rate_hz', self._active_read_rate_hz),
            ('idle_read_rate_hz', self._idle_read_rate_hz),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} deve ser positivo e finito.')
        if self._idle_read_rate_hz > self._active_read_rate_hz:
            raise ValueError(
                'idle_read_rate_hz não pode exceder read_rate_hz.')
        for name, value in (
            ('read_idle_timeout_sec', self._read_idle_timeout),
            ('idle_velocity_threshold', self._idle_velocity_threshold),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} deve ser não negativo e finito.')
        self._serial_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._port = self.get_parameter('port').value
        self._robot_id = self.get_parameter('robot_id').value
        self._calibration_file = resolve_calibration_file(
            self.get_parameter('calibration_file').value)
        self._use_degrees = bool(self.get_parameter('use_degrees').value)
        self._disable_torque = bool(
            self.get_parameter('disable_torque').value)
        self._latest_command = None
        self._last_received_command = None
        self._last_sent_command = None
        # Dois períodos do escritor mantêm a janela ativa após a última
        # mudança; depois disso o timer dorme para não consumir CPU em repouso.
        self._write_idle_timeout = 2.0 / self._write_rate_hz
        self._last_command_change_time = time.monotonic()
        self._read_is_idle = False
        self._last_positions = {}
        self._previous_positions = {}
        self._previous_read_time = None
        self._reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self._reconnect_timeout = float(
            self.get_parameter('reconnect_timeout_sec').value)
        for name, value in (
            ('reconnect_interval_sec', self._reconnect_interval),
            ('reconnect_timeout_sec', self._reconnect_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} deve ser positivo e finito.')
        self._next_reconnect_time = 0.0
        self._communication_failure_started_at = None

        # Esta ponte transporta somente o estado/alvo mais recente. BEST_EFFORT
        # evita que congestionamento DDS bloqueie o write() em tempo real do
        # controller_manager; alvos idênticos não são reenviados à serial.
        latest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.state_pub = self.create_publisher(
            JointState, self.get_parameter('state_topic').value, latest_qos)
        self.command_sub = None
        if not self._disable_torque:
            self.command_sub = self.create_subscription(
                Float64MultiArray,
                self.get_parameter('command_topic').value,
                self._command_callback,
                latest_qos,
            )
        self.calibrate_srv = self.create_service(Trigger, '~/calibrate', self._calibrate)
        self.follower = None

        try:
            self.follower = self._create_follower()
            connect_follower(
                self.follower, disable_torque=self._disable_torque)
            if self._disable_torque:
                self.get_logger().warning(
                    'SO-101 conectado com torque desabilitado; comandos de '
                    'movimento serão ignorados e o braço pode ser movido '
                    'manualmente.')
            else:
                self.get_logger().info(
                    'SO-101 conectado via LeRobot/Feetech.')
        except Exception as error:
            self._force_close_follower(self.follower)
            self.get_logger().fatal(f'Falha ao conectar o SO-101: {error}')
            raise RuntimeError(f'Falha ao conectar o SO-101: {error}') from error

        self.write_timer = self.create_timer(
            1.0 / self._write_rate_hz, self._write_cycle)
        self.write_timer.cancel()
        self.read_timer = self.create_timer(
            1.0 / self._active_read_rate_hz, self._read_cycle)

    @staticmethod
    def _command_signature(values):
        return tuple(values[name] for name in ROS_JOINT_ORDER)

    def _send_command(self, values):
        with self._serial_lock:
            self.follower.send_action(
                ros_to_action(values, use_degrees=self._use_degrees))
        self._last_sent_command = self._command_signature(values)

    @staticmethod
    def _period_ns(rate_hz):
        return int(1_000_000_000 / rate_hz)

    def _set_reading_active(self, active):
        idle = not active
        if idle == self._read_is_idle:
            return
        self._read_is_idle = idle
        rate = (
            self._idle_read_rate_hz
            if idle else self._active_read_rate_hz)
        self.read_timer.timer_period_ns = self._period_ns(rate)
        self.read_timer.reset()

    def _wake_write_timer(self):
        if self.write_timer.is_canceled():
            self.write_timer.reset()

    def _write_cycle(self):
        with self._command_lock:
            command = (
                dict(self._latest_command)
                if self._latest_command is not None else None)
        if command is None:
            self.write_timer.cancel()
            return
        if self._command_signature(command) != self._last_sent_command:
            try:
                self._send_command(command)
            except Exception as error:
                self._handle_io_failure('comandar', error)
                self._try_reconnect()
                return
        if time.monotonic() - self._last_command_change_time >= self._write_idle_timeout:
            self.write_timer.cancel()

    def _read_cycle(self):
        velocities = self._read_observation()
        if velocities is None:
            self._set_reading_active(True)
            return
        recently_commanded = (
            time.monotonic() - self._last_command_change_time
            < self._read_idle_timeout)
        moving = any(
            abs(velocity) > self._idle_velocity_threshold
            for velocity in velocities)
        self._set_reading_active(recently_commanded or moving)

    def _read_observation(self):
        try:
            with self._serial_lock:
                observation = self.follower.get_observation()
            positions = observation_to_ros(
                observation, use_degrees=self._use_degrees)
            self._last_positions.update(positions)
            if not all(name in self._last_positions for name in ROS_JOINT_ORDER):
                return None
            read_time = time.monotonic()
            if self._previous_read_time is None:
                velocities = [0.0 for _ in ROS_JOINT_ORDER]
            else:
                period = read_time - self._previous_read_time
                velocities = [
                    (self._last_positions[name] - self._previous_positions[name]) / period
                    if period > 0.0 else 0.0
                    for name in ROS_JOINT_ORDER
                ]
            self._previous_positions = {
                name: self._last_positions[name] for name in ROS_JOINT_ORDER}
            self._previous_read_time = read_time
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(ROS_JOINT_ORDER)
            message.position = [self._last_positions[name] for name in ROS_JOINT_ORDER]
            message.velocity = velocities
            self.state_pub.publish(message)
            return velocities
        except Exception as error:
            self._handle_io_failure('ler', error)
            self._try_reconnect()
            return None

    def _command_callback(self, message: Float64MultiArray):
        if self._disable_torque:
            return
        if len(message.data) != len(ROS_JOINT_ORDER):
            self.get_logger().error(
                f'Comando inválido: esperadas {len(ROS_JOINT_ORDER)} posições, '
                f'recebidas {len(message.data)}.', throttle_duration_sec=2.0)
            return
        if not all(math.isfinite(value) for value in message.data):
            self.get_logger().error(
                'Comando inválido: todas as posições devem ser finitas.',
                throttle_duration_sec=2.0)
            return
        values = dict(zip(ROS_JOINT_ORDER, message.data))
        signature = self._command_signature(values)
        if signature != self._last_received_command:
            self._last_received_command = signature
            self._last_command_change_time = time.monotonic()
            self._set_reading_active(True)
        with self._command_lock:
            self._latest_command = values
        if signature != self._last_sent_command:
            self._wake_write_timer()

    def _handle_io_failure(self, operation, error):
        now = time.monotonic()
        if self._communication_failure_started_at is None:
            self._communication_failure_started_at = now
        elapsed = now - self._communication_failure_started_at
        message = (
            f'Falha ao {operation} os motores; reconectando há '
            f'{elapsed:.1f}/{self._reconnect_timeout:.1f} s: {error}')
        if elapsed >= self._reconnect_timeout:
            self.get_logger().fatal(message)
            raise RuntimeError(message) from error
        self.get_logger().error(message, throttle_duration_sec=2.0)

    def _create_follower(self):
        return make_follower(
            self._port,
            self._robot_id,
            use_degrees=self._use_degrees,
            calibration_file=self._calibration_file,
        )

    @staticmethod
    def _force_close_follower(follower):
        """Close a broken SDK port without trying motor I/O first."""
        bus = getattr(follower, 'bus', None)
        port_handler = getattr(bus, 'port_handler', None)
        if port_handler is None:
            return
        try:
            if getattr(port_handler, 'is_open', False):
                port_handler.closePort()
        except Exception:
            # USB removal can make closePort fail too. The flags must still be
            # cleared so LeRobot does not consider the stale descriptor open.
            pass
        finally:
            port_handler.is_open = False
            port_handler.is_using = False

    def _try_reconnect(self):
        """Replace the stale LeRobot follower after a serial I/O failure."""
        now = time.monotonic()
        if now < self._next_reconnect_time:
            return False
        self._next_reconnect_time = now + self._reconnect_interval

        candidate = None
        try:
            with self._serial_lock:
                self._force_close_follower(self.follower)
                candidate = self._create_follower()
                connect_follower(
                    candidate, disable_torque=self._disable_torque)
                self.follower = candidate
        except Exception as error:
            self._force_close_follower(candidate)
            self.get_logger().error(
                f'Reconexão do SO-101 ainda indisponível: {error}',
                throttle_duration_sec=2.0,
            )
            return False

        self._communication_failure_started_at = None
        self._last_positions = {}
        self._previous_positions = {}
        self._previous_read_time = None
        self._last_sent_command = None
        self._next_reconnect_time = 0.0
        self._set_reading_active(True)
        if not self._disable_torque and self._latest_command is not None:
            self._wake_write_timer()
        self.get_logger().info('SO-101 reconectado após falha de comunicação.')
        return True

    def _calibrate(self, request, response):
        del request
        try:
            with self._serial_lock:
                self.follower.calibrate()
            response.success = True
            response.message = 'Calibração concluída.'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def destroy_node(self):
        try:
            with self._serial_lock:
                if self.follower is not None and self.follower.is_connected:
                    self.follower.disconnect()
                else:
                    self._force_close_follower(self.follower)
        except Exception as error:
            self.get_logger().error(f'Falha ao desconectar o braço: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SO101HardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Nó de hardware encerrado: {error}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
