"""Aquisição do LiDAR XV-11 e formação de varreduras angulares coerentes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Protocol


PACKET_HEAD = 0xFA
PACKET_SIZE = 22
USB_PORT_MATCH = {
    0: 'usb-0:1.1',
    1: 'usb-0:1.4',
    2: 'usb-0:1.3',
    3: 'usb-0:1.2',
}
UART_PORT = {
    4: '/dev/ttyS4',
    5: '/dev/ttyS2',
    6: '/dev/ttyS5',
}


@dataclass(frozen=True)
class LidarConfig:
    serial_port: int = 1
    serial_baud_rate: int = 115200
    serial_read_chunk_size: int = 64
    serial_timeout_sec: float = 0.10
    data_timeout_sec: float = 5.0
    relay_gpio_chip: str = '/dev/gpiochip1'
    relay_pin: int = 266
    relay_active_low: bool = True
    angle_start_deg: int = 307
    angle_end_deg: int = 217
    valid_intervals_deg: tuple[int, ...] = (307, 67, 194, 217)
    range_min_m: float = 0.10
    range_max_m: float = 3.0

    def validate(self) -> None:
        if self.serial_port not in {*USB_PORT_MATCH, *UART_PORT}:
            raise ValueError('serial_port deve estar entre 0 e 6.')
        if self.serial_baud_rate <= 0:
            raise ValueError('serial_baud_rate deve ser positivo.')
        if not 1 <= self.serial_read_chunk_size <= 4096:
            raise ValueError(
                'serial_read_chunk_size deve estar entre 1 e 4096 bytes.')
        if not math.isfinite(self.serial_timeout_sec) or self.serial_timeout_sec <= 0.0:
            raise ValueError('serial_timeout_sec deve ser positivo e finito.')
        if not math.isfinite(self.data_timeout_sec) or self.data_timeout_sec <= 0.0:
            raise ValueError('data_timeout_sec deve ser positivo e finito.')
        if not self.relay_gpio_chip:
            raise ValueError('relay_gpio_chip não pode ser vazio.')
        if self.relay_pin < 0:
            raise ValueError('relay_pin não pode ser negativo.')
        if not 0 <= self.angle_start_deg <= 359:
            raise ValueError('angle_start_deg deve estar entre 0 e 359.')
        if not 0 <= self.angle_end_deg <= 359:
            raise ValueError('angle_end_deg deve estar entre 0 e 359.')
        if not self.valid_intervals_deg or len(self.valid_intervals_deg) % 2:
            raise ValueError(
                'valid_intervals_deg deve conter pares de início e fim.')
        if any(not 0 <= angle <= 359 for angle in self.valid_intervals_deg):
            raise ValueError(
                'Os ângulos de valid_intervals_deg devem estar entre 0 e 359.')

        published_angles = {
            (self.angle_start_deg + offset) % 360
            for offset in range(self.sample_count)
        }
        valid_angles = {
            angle
            for angle in range(360)
            if self.is_angle_valid(angle)
        }
        if not valid_angles.issubset(published_angles):
            raise ValueError(
                'Todos os intervalos válidos devem estar dentro do setor '
                'publicado por angle_start_deg e angle_end_deg.')
        if not 0.0 < self.range_min_m < self.range_max_m:
            raise ValueError('Os limites de alcance são inválidos.')

    @property
    def sample_count(self) -> int:
        return ((self.angle_end_deg - self.angle_start_deg) % 360) + 1

    def is_angle_valid(self, angle: int) -> bool:
        """Indica se o ângulo pertence a um dos intervalos inclusivos."""
        intervals = iter(self.valid_intervals_deg)
        return any(
            (angle - start) % 360 <= (end - start) % 360
            for start, end in zip(intervals, intervals)
        )


@dataclass(frozen=True)
class LidarScan:
    sequence: int
    ranges_m: tuple[float, ...]
    rpm: float
    start_monotonic_ns: int
    end_monotonic_ns: int


class Relay(Protocol):
    def turn_on(self) -> None: ...

    def turn_off(self) -> None: ...

    def close(self) -> None: ...


def resolve_serial_port(
    selector: int,
    by_path_directory: Path = Path('/dev/serial/by-path'),
) -> str:
    """Resolve a numeração SERIAL0..SERIAL6 usada pela Mariola."""
    if selector in UART_PORT:
        return UART_PORT[selector]
    if selector not in USB_PORT_MATCH:
        raise ValueError(f'Porta serial inválida: {selector}.')
    if not by_path_directory.is_dir():
        raise RuntimeError(f'Diretório serial ausente: {by_path_directory}.')

    marker = USB_PORT_MATCH[selector]
    matches = sorted(
        path for path in by_path_directory.iterdir()
        if marker in path.name
    )
    if not matches:
        raise RuntimeError(
            f'SERIAL{selector} não encontrada em {by_path_directory} '
            f'(identificador esperado: {marker}).')
    if len(matches) > 1:
        names = ', '.join(path.name for path in matches)
        raise RuntimeError(f'SERIAL{selector} é ambígua: {names}.')
    return str(matches[0])


class GpioRelay:
    """Aciona somente a linha GPIO usada pelo relé do LiDAR."""

    def __init__(self, chip_path: str, pin: int, active_low: bool):
        try:
            import gpiod
        except ImportError as error:
            raise RuntimeError('O módulo Python gpiod não está instalado.') from error

        self._pin = pin
        self._closed = False
        self._chip = None
        self._line = None

        if hasattr(gpiod, 'request_lines'):
            from gpiod.line import Direction, Value

            self._on_value = Value.INACTIVE if active_low else Value.ACTIVE
            self._off_value = Value.ACTIVE if active_low else Value.INACTIVE
            self._request = gpiod.request_lines(
                chip_path,
                consumer='lidar',
                config={
                    pin: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=self._off_value,
                    ),
                },
            )
        else:
            # Compatibilidade com os bindings libgpiod 1.x do Ubuntu.
            self._on_value = 0 if active_low else 1
            self._off_value = 1 if active_low else 0
            self._chip = gpiod.Chip(chip_path)
            self._line = self._chip.get_line(pin)
            self._line.request(
                consumer='lidar',
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[self._off_value],
            )
            self._request = None

    def turn_on(self) -> None:
        if self._request is not None:
            self._request.set_value(self._pin, self._on_value)
        else:
            self._line.set_value(self._on_value)

    def turn_off(self) -> None:
        if not self._closed:
            if self._request is not None:
                self._request.set_value(self._pin, self._off_value)
            else:
                self._line.set_value(self._off_value)

    def close(self) -> None:
        if self._closed:
            return
        try:
            self.turn_off()
        finally:
            if self._request is not None:
                self._request.release()
            else:
                self._line.release()
                if hasattr(self._chip, 'close'):
                    self._chip.close()
            self._closed = True


class LidarDriver:
    """Driver do protocolo Neato XV-11 com uma mensagem por rotação."""

    def __init__(
        self,
        config: LidarConfig,
        *,
        serial_connection=None,
        relay: Relay | None = None,
        start_thread: bool = True,
    ):
        config.validate()
        self.config = config
        self._relay = relay or GpioRelay(
            config.relay_gpio_chip,
            config.relay_pin,
            config.relay_active_low,
        )
        self._serial = serial_connection
        self._stop_event = threading.Event()
        self._thread = None
        self._read_error: Exception | None = None
        self._created_monotonic = time.monotonic()
        self._packet = bytearray(PACKET_SIZE)
        self._packet_index = 0
        self._waiting_head = True
        self._last_packet_monotonic = 0.0
        self._rpm = 0.0

        self._scan_lock = threading.Lock()
        self._ranges = [math.nan] * config.sample_count
        self._collecting = False
        self._previous_angle = None
        self._scan_start_ns = 0
        self._rpm_sum = 0.0
        self._rpm_samples = 0
        self._sequence = 0
        self._latest_scan: LidarScan | None = None
        self._last_consumed_sequence = 0

        try:
            if self._serial is None:
                self._serial = self._open_serial()
            self._relay.turn_on()
            if start_thread:
                self._thread = threading.Thread(
                    target=self._read_loop,
                    name='cbr-lidar-serial',
                    daemon=True,
                )
                self._thread.start()
        except Exception:
            self.close()
            raise

    def _open_serial(self):
        try:
            import serial
        except ImportError as error:
            raise RuntimeError('O módulo Python pyserial não está instalado.') from error

        port = resolve_serial_port(self.config.serial_port)
        try:
            return serial.Serial(
                port,
                self.config.serial_baud_rate,
                timeout=self.config.serial_timeout_sec,
            )
        except serial.SerialException as error:
            raise RuntimeError(f'Não foi possível abrir a serial {port}: {error}') from error

    @property
    def connected(self) -> bool:
        return (
            self._last_packet_monotonic > 0.0
            and time.monotonic() - self._last_packet_monotonic < 2.0
        )

    @property
    def rpm(self) -> float:
        return self._rpm

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data = self._serial.read(self.config.serial_read_chunk_size)
                if not data:
                    self._stop_event.wait(0.001)
                    continue
                for received in data:
                    self._process_byte(received)
            except Exception as error:
                if not self._stop_event.is_set():
                    self._read_error = error
                    self._stop_event.set()
                return

    def _process_byte(self, received: int) -> None:
        if self._waiting_head:
            if received != PACKET_HEAD:
                return
            self._packet_index = 0
            self._waiting_head = False

        self._packet[self._packet_index] = received
        self._packet_index += 1
        if self._packet_index >= PACKET_SIZE:
            self._waiting_head = True
            self._decode_packet(self._packet)

    @staticmethod
    def checksum_valid(packet, size: int = PACKET_SIZE - 2) -> bool:
        checksum = 0
        expected = packet[size] + (packet[size + 1] << 8)
        for index in range(size // 2):
            word = ((packet[index * 2 + 1] << 8) + packet[index * 2]) & 0xFFFF
            checksum = ((checksum << 1) + word) & 0xFFFFFFFF
        calculated = ((checksum & 0x7FFF) + (checksum >> 15)) & 0x7FFF
        return calculated == expected

    def _decode_packet(self, packet) -> None:
        base_angle = (packet[1] - 0xA0) * 4
        if not 0 <= base_angle < 360 or not self.checksum_valid(packet):
            return

        measured_rpm = ((packet[3] << 8) | packet[2]) / 64.0
        if self._rpm <= 0.0 or abs(measured_rpm - self._rpm) <= 100.0:
            self._rpm = measured_rpm
        else:
            self._rpm = self._rpm * 0.95 + measured_rpm * 0.05

        for reading_offset, packet_offset in enumerate((4, 8, 12, 16)):
            angle = (base_angle + reading_offset) % 360
            high_byte = packet[packet_offset + 1]
            invalid = bool(high_byte & 0x80)
            distance_mm = ((high_byte & 0x3F) << 8) | packet[packet_offset]
            if invalid:
                distance_m = math.nan
            elif distance_mm < self.config.range_min_m * 1000.0:
                distance_m = -math.inf
            elif distance_mm > self.config.range_max_m * 1000.0:
                distance_m = math.inf
            else:
                distance_m = distance_mm / 1000.0
            self._process_sample(angle, distance_m, time.monotonic_ns())

        self._last_packet_monotonic = time.monotonic()

    def _process_sample(self, angle: int, distance_m: float, now_ns: int) -> None:
        with self._scan_lock:
            crossed_start = False
            if self._previous_angle is not None:
                advance = (angle - self._previous_angle) % 360
                to_start = (
                    self.config.angle_start_deg - self._previous_angle
                ) % 360
                crossed_start = 0 < to_start <= advance
            elif angle == self.config.angle_start_deg:
                crossed_start = True

            if crossed_start:
                if self._collecting:
                    self._finish_scan(now_ns)
                self._start_scan(angle, now_ns)

            if self._collecting:
                index = (angle - self.config.angle_start_deg) % 360
                if index < self.config.sample_count:
                    if self.config.is_angle_valid(angle):
                        self._ranges[index] = distance_m
                    self._rpm_sum += max(self._rpm, 0.0)
                    self._rpm_samples += 1
                    if index == self.config.sample_count - 1:
                        self._finish_scan(now_ns)
                else:
                    # O último pacote do setor pode ter sido perdido por inteiro.
                    self._finish_scan(now_ns)

            self._previous_angle = angle

    def _start_scan(self, first_received_angle: int, now_ns: int) -> None:
        self._ranges = [math.nan] * self.config.sample_count
        self._rpm_sum = 0.0
        self._rpm_samples = 0
        self._collecting = True

        missing_degrees = (
            first_received_angle - self.config.angle_start_deg
        ) % 360
        delay_ns = 0
        if self._rpm > 0.0:
            delay_ns = round(
                (60.0 / self._rpm) * (missing_degrees / 360.0) * 1e9)
        self._scan_start_ns = now_ns - delay_ns

    def _finish_scan(self, now_ns: int) -> None:
        if not self._collecting:
            return
        average_rpm = (
            self._rpm_sum / self._rpm_samples if self._rpm_samples else 0.0)
        self._sequence += 1
        self._latest_scan = LidarScan(
            sequence=self._sequence,
            ranges_m=tuple(self._ranges),
            rpm=average_rpm,
            start_monotonic_ns=self._scan_start_ns,
            end_monotonic_ns=now_ns,
        )
        self._collecting = False

    def take_scan(self) -> LidarScan | None:
        """Retorna cada varredura concluída no máximo uma vez."""
        if self._read_error is not None:
            raise RuntimeError(
                f'A leitura serial do LiDAR foi interrompida: {self._read_error}'
            ) from self._read_error
        last_activity = self._last_packet_monotonic or self._created_monotonic
        if time.monotonic() - last_activity > self.config.data_timeout_sec:
            raise RuntimeError(
                f'O LiDAR não fornece pacotes válidos há mais de '
                f'{self.config.data_timeout_sec:.1f} s.')
        with self._scan_lock:
            if self._latest_scan is None:
                return None
            if self._latest_scan.sequence == self._last_consumed_sequence:
                return None
            self._last_consumed_sequence = self._latest_scan.sequence
            return self._latest_scan

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        if self._relay is not None:
            try:
                self._relay.close()
            finally:
                self._relay = None
