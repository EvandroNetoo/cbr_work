"""Conversões ROS em torno da classe original ``ControleMotores``."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Mapping

from .controleMotores import (
    ControleMotores,
    ErroControleMotores,
    GrupoMotoresBrick,
    GrupoMotoresExpansao,
)
from .motores import Motores
from .placaControleMotor import PlacaControleMotor
from .portas import Portas


WHEEL_NAMES = (
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
)
MAX_WHEEL_VELOCITY = 7.0
MIN_EFFECTIVE_WHEEL_COMMAND = 2
BRICK_TICKS_PER_REVOLUTION = 1644
EXPANSION_TICKS_PER_REVOLUTION = 3288


class MotorCommunicationError(RuntimeError):
    pass


@dataclass(frozen=True)
class MariolaConfig:
    expansion_serial_port: int = Portas.SERIAL3
    serial_baud_rate: int = 250000
    expansion_timeout_sec: float = 0.005
    front_left_motor_id: int = 0
    front_right_motor_id: int = 7
    front_left_inverted: bool = True
    front_right_inverted: bool = False
    rear_left_inverted: bool = False
    rear_right_inverted: bool = True
    brick_ticks_per_revolution: int = BRICK_TICKS_PER_REVOLUTION
    expansion_ticks_per_revolution: int = EXPANSION_TICKS_PER_REVOLUTION
    max_wheel_velocity_rad_s: float = MAX_WHEEL_VELOCITY
    min_effective_wheel_command: int = MIN_EFFECTIVE_WHEEL_COMMAND

    def __post_init__(self) -> None:
        radians_per_second_to_command(
            0.0,
            self.max_wheel_velocity_rad_s,
            self.min_effective_wheel_command,
        )


@dataclass(frozen=True)
class WheelState:
    position: float
    velocity: float


def ticks_to_radians(ticks: int, ticks_per_revolution: int) -> float:
    if ticks_per_revolution <= 0:
        raise ValueError('ticks_per_revolution deve ser positivo.')
    return float(ticks) * 2.0 * math.pi / float(ticks_per_revolution)


def radians_per_second_to_command(
    value: float,
    max_velocity: float = MAX_WHEEL_VELOCITY,
    min_effective_command: int = MIN_EFFECTIVE_WHEEL_COMMAND,
) -> int:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('A velocidade da roda deve ser finita.')
    if not math.isfinite(max_velocity) or max_velocity <= 0.0:
        raise ValueError('O limite de velocidade deve ser positivo e finito.')
    if isinstance(min_effective_command, bool) or not isinstance(
            min_effective_command, int):
        raise ValueError('O comando mínimo efetivo deve ser um inteiro.')
    if not 0 <= min_effective_command <= 100:
        raise ValueError('O comando mínimo efetivo deve estar entre 0 e 100.')
    magnitude = abs(value)
    # A normalização proporcional feita pela interface ros2_control pode
    # terminar um ULP acima do limite (por exemplo, 7.000000000000001 para
    # 7.0). Isso ainda representa o próprio limite, não um excesso físico.
    if magnitude > math.nextafter(max_velocity, math.inf):
        raise ValueError(
            f'Velocidade {value} rad/s excede o limite de {max_velocity} rad/s.')
    if value == 0.0:
        return 0
    magnitude = min(magnitude, max_velocity)
    magnitude = max(
        min_effective_command,
        int(round(magnitude * 100.0 / max_velocity)),
    )
    return magnitude if value > 0.0 else -magnitude


def validate_complete_command(
    values: Mapping[str, float],
    max_velocity: float = MAX_WHEEL_VELOCITY,
    min_effective_command: int = MIN_EFFECTIVE_WHEEL_COMMAND,
) -> dict[str, float]:
    if set(values) != set(WHEEL_NAMES) or len(values) != len(WHEEL_NAMES):
        raise ValueError('O comando deve conter exatamente as quatro rodas conhecidas.')
    result = {name: float(values[name]) for name in WHEEL_NAMES}
    for value in result.values():
        radians_per_second_to_command(
            value, max_velocity, min_effective_command)
    return result


class MariolaBase:
    """Adapta a API nomeada original para posição e velocidade em SI."""

    def __init__(
        self,
        config: MariolaConfig | None = None,
        controle: ControleMotores | None = None,
    ):
        self._config = config or MariolaConfig()
        self._controle = controle
        self._serial_expansao = None
        self._last_positions = None
        self._last_read_time = None
        self._closed = False

        if self._controle is None:
            self._controle = self._criar_controle()

        if set(self._controle.nomes) != set(WHEEL_NAMES):
            raise ValueError('ControleMotores deve conter exatamente as quatro rodas.')

        try:
            self._controle.reseta_angulos_motores()
        except Exception:
            self.close(stop=False)
            raise

    def _criar_controle(self) -> ControleMotores:
        portas = Portas()
        self._serial_expansao = portas.abre_porta_serial(
            self._config.expansion_serial_port,
            self._config.serial_baud_rate,
            timeout=self._config.expansion_timeout_sec,
        )
        if self._serial_expansao is None:
            raise RuntimeError('Não foi possível abrir a serial da expansão.')

        motores_brick = Motores(True)
        motor_dianteiro_esquerdo = PlacaControleMotor(
            self._serial_expansao,
            id_equipamento=self._config.front_left_motor_id,
        )
        motor_dianteiro_direito = PlacaControleMotor(
            self._serial_expansao,
            id_equipamento=self._config.front_right_motor_id,
        )

        return ControleMotores(
            [
                GrupoMotoresBrick(
                    motores_brick,
                    nomes=('rear_left_wheel_joint', 'rear_right_wheel_joint'),
                    invertidos=(
                        self._config.rear_left_inverted,
                        self._config.rear_right_inverted,
                    ),
                ),
                GrupoMotoresExpansao(
                    {
                        'front_left_wheel_joint': motor_dianteiro_esquerdo,
                        'front_right_wheel_joint': motor_dianteiro_direito,
                    },
                    invertidos={
                        'front_left_wheel_joint': self._config.front_left_inverted,
                        'front_right_wheel_joint': self._config.front_right_inverted,
                    },
                ),
            ],
            freio_travado=False,
        )

    def write(self, velocities: Mapping[str, float]):
        values = validate_complete_command(
            velocities,
            self._config.max_wheel_velocity_rad_s,
            self._config.min_effective_wheel_command,
        )
        commands = {
            name: radians_per_second_to_command(
                values[name],
                self._config.max_wheel_velocity_rad_s,
                self._config.min_effective_wheel_command,
            )
            for name in WHEEL_NAMES
        }
        try:
            self._ensure_open()
            self._controle.definir_velocidades(**commands)
        except ErroControleMotores as error:
            raise MotorCommunicationError(str(error)) from error

    def read(self, now: float | None = None) -> dict[str, WheelState]:
        try:
            self._ensure_open()
            ticks = self._controle.angulos_motores()
        except ErroControleMotores as error:
            raise MotorCommunicationError(str(error)) from error

        positions = {
            name: ticks_to_radians(
                ticks[name],
                self._config.expansion_ticks_per_revolution
                if name.startswith('front_')
                else self._config.brick_ticks_per_revolution,
            )
            for name in WHEEL_NAMES
        }
        read_time = time.monotonic() if now is None else float(now)
        if self._last_positions is None or self._last_read_time is None:
            velocities = {name: 0.0 for name in WHEEL_NAMES}
        else:
            period = read_time - self._last_read_time
            velocities = {
                name: (positions[name] - self._last_positions[name]) / period
                if period > 0.0 else 0.0
                for name in WHEEL_NAMES
            }
        self._last_positions = positions
        self._last_read_time = read_time
        return {
            name: WheelState(positions[name], velocities[name]) for name in WHEEL_NAMES
        }

    def stop(self):
        try:
            self._ensure_open()
            self._controle.frear()
        except ErroControleMotores as error:
            raise MotorCommunicationError(str(error)) from error

    def close(self, *, stop: bool = True):
        if self._closed:
            return
        self._closed = True
        try:
            if self._controle is not None:
                self._controle.fechar(parar_motores=stop)
        finally:
            if self._serial_expansao is not None and self._serial_expansao.is_open:
                self._serial_expansao.close()

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError('O controle da base já foi fechado.')
