"""Protocolo serial enxuto da IMU embarcada na Mariola."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import struct
import time
from typing import Callable, Optional


ROS_IMU_COMMAND = 3
ROS_IMU_PACKET_SIZE = 40
UART_PORTS = {4: '/dev/ttyS4', 5: '/dev/ttyS2', 6: '/dev/ttyS5'}
USB_PORT_MATCH = {
    0: 'usb-0:1.1',
    1: 'usb-0:1.4',
    2: 'usb-0:1.3',
    3: 'usb-0:1.2',
}


@dataclass(frozen=True)
class ImuSample:
    orientation_wxyz: tuple[float, float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    linear_acceleration_xyz: tuple[float, float, float]


def resolve_serial_port(port: int, by_path: Path = Path('/dev/serial/by-path')) -> str:
    """Resolve a numeração SERIAL0..SERIAL6 usada pela Mariola."""
    if port in UART_PORTS:
        return UART_PORTS[port]
    match = USB_PORT_MATCH.get(port)
    if match is None:
        raise ValueError('serial_port deve estar entre 0 e 6.')
    try:
        candidates = sorted(by_path.iterdir())
    except OSError as error:
        raise RuntimeError(f'Erro ao listar {by_path}: {error}') from error
    for candidate in candidates:
        if match in candidate.name:
            return str(candidate.resolve())
    raise RuntimeError(f'Não foi encontrada a SERIAL{port} em {by_path}.')


class ImuDriver:
    """Executa uma transação serial somente quando ``read_sample`` é chamado."""

    def __init__(
        self,
        serial_port: int = 5,
        baud_rate: int = 115200,
        timeout_sec: float = 0.02,
        serial_instance=None,
        serial_factory: Optional[Callable] = None,
    ) -> None:
        if timeout_sec <= 0.0 or timeout_sec > 0.02:
            raise ValueError('timeout_sec deve estar no intervalo (0, 0.02].')
        self.timeout_sec = float(timeout_sec)
        if serial_instance is not None:
            self._serial = serial_instance
            return
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        device = resolve_serial_port(int(serial_port))
        self._serial = serial_factory(device, int(baud_rate), timeout=self.timeout_sec)

    def read_sample(self) -> ImuSample:
        """Solicita e valida um pacote ROS_IMU de dez float32 little-endian."""
        self._serial.reset_input_buffer()
        self._serial.write(bytes([ROS_IMU_COMMAND]))
        deadline = time.monotonic() + self.timeout_sec
        packet = bytearray()
        while len(packet) < ROS_IMU_PACKET_SIZE:
            if time.monotonic() >= deadline:
                break
            chunk = self._serial.read(ROS_IMU_PACKET_SIZE - len(packet))
            if chunk:
                packet.extend(chunk)
                continue
        if len(packet) != ROS_IMU_PACKET_SIZE:
            raise RuntimeError(
                f'Pacote incompleto da IMU: {len(packet)}/{ROS_IMU_PACKET_SIZE} bytes.')

        values = struct.unpack('<10f', packet)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError('A IMU retornou NaN ou infinito.')
        quaternion_norm = math.sqrt(sum(value * value for value in values[:4]))
        if not 0.9 <= quaternion_norm <= 1.1:
            raise RuntimeError(
                f'Quaternion inválido: norma {quaternion_norm:.6f}.')
        return ImuSample(
            orientation_wxyz=tuple(values[:4]),
            angular_velocity_xyz=tuple(values[4:7]),
            linear_acceleration_xyz=tuple(values[7:10]),
        )

    def close(self) -> None:
        serial_port = getattr(self, '_serial', None)
        if serial_port is not None and getattr(serial_port, 'is_open', True):
            serial_port.close()


class GyroBiasCalibrator:
    """Calcula uma média angular usando apenas amostras consecutivas paradas."""

    def __init__(self, sample_count: int = 40, stationary_max_rad_s: float = 0.05):
        if sample_count <= 0 or stationary_max_rad_s <= 0.0:
            raise ValueError('Parâmetros de calibração devem ser positivos.')
        self.sample_count = int(sample_count)
        self.stationary_max_rad_s = float(stationary_max_rad_s)
        self._samples: list[tuple[float, float, float]] = []
        self.bias: Optional[tuple[float, float, float]] = None

    def add(self, angular_velocity: tuple[float, float, float]) -> bool:
        norm = math.sqrt(sum(value * value for value in angular_velocity))
        if norm > self.stationary_max_rad_s:
            self._samples.clear()
            return False
        self._samples.append(angular_velocity)
        if len(self._samples) < self.sample_count:
            return False
        self.bias = tuple(
            sum(sample[axis] for sample in self._samples) / len(self._samples)
            for axis in range(3)
        )
        return True

    @property
    def collected_samples(self) -> int:
        return len(self._samples)

    def correct(self, angular_velocity: tuple[float, float, float]) -> tuple[float, float, float]:
        if self.bias is None:
            raise RuntimeError('Calibração do giroscópio ainda não concluída.')
        return tuple(value - offset for value, offset in zip(angular_velocity, self.bias))
