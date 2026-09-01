"""Aquisição sincronizada e filtrada de dois sensores VL53L0X."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import statistics

from .vl53 import SMBus, VL53L0X


@dataclass(frozen=True)
class SensorPairConfig:
    i2c_bus: int = 1
    mux_address: int = 0x70
    right_channel: int = 0
    right_offset_mm: int = 48
    left_channel: int = 1
    left_offset_mm: int = 106
    raw_min_mm: int = 30
    raw_max_mm: int = 2000
    median_window: int = 3
    ranging_timeout_ms: int = 200

    def __post_init__(self) -> None:
        for name, channel in (
            ('right_channel', self.right_channel),
            ('left_channel', self.left_channel),
        ):
            if not 0 <= channel <= 7:
                raise ValueError(f'{name} deve estar entre 0 e 7.')
        if self.left_channel == self.right_channel:
            raise ValueError('Os sensores esquerdo e direito devem usar canais diferentes.')
        if not 0x08 <= self.mux_address <= 0x77:
            raise ValueError('mux_address deve ser um endereço I2C válido.')
        if self.raw_min_mm < 0 or self.raw_max_mm <= self.raw_min_mm:
            raise ValueError('A faixa bruta de distância é inválida.')
        if self.median_window <= 0 or self.median_window % 2 == 0:
            raise ValueError('median_window deve ser ímpar e positivo.')
        if self.ranging_timeout_ms <= 0:
            raise ValueError('ranging_timeout_ms deve ser positivo.')

    @property
    def minimum_target_mm(self) -> int:
        return max(
            self.raw_min_mm - self.left_offset_mm,
            self.raw_min_mm - self.right_offset_mm,
        )

    @property
    def maximum_target_mm(self) -> int:
        return min(
            self.raw_max_mm - self.left_offset_mm,
            self.raw_max_mm - self.right_offset_mm,
        )


@dataclass(frozen=True)
class DistanceSample:
    raw_left_mm: int
    raw_right_mm: int
    left_mm: int
    right_mm: int

    @property
    def average_mm(self) -> float:
        return (float(self.left_mm) + float(self.right_mm)) / 2.0


class VL53SensorPair:
    """Possui um SMBus e duas instâncias do driver atrás do mesmo mux."""

    def __init__(
        self,
        config: SensorPairConfig,
        *,
        bus=None,
        sensor_factory=VL53L0X,
    ) -> None:
        self.config = config
        self._owns_bus = bus is None
        if bus is None:
            if SMBus is None:
                raise RuntimeError(
                    'smbus2 não está instalado; instale python3-smbus2.')
            bus = SMBus(config.i2c_bus)
        self._bus = bus
        self._closed = False
        self._left_samples = deque(maxlen=config.median_window)
        self._right_samples = deque(maxlen=config.median_window)

        common = {
            'bus': bus,
            'i2c_bus': config.i2c_bus,
            'mux_address': config.mux_address,
            'ranging_timeout_ms': config.ranging_timeout_ms,
        }
        self.right = None
        self.left = None
        try:
            self.right = sensor_factory(config.right_channel, **common)
            self.left = sensor_factory(config.left_channel, **common)
        except Exception:
            self.close()
            raise

    def reset_filter(self) -> None:
        self._left_samples.clear()
        self._right_samples.clear()

    def read(self) -> DistanceSample:
        if self._closed:
            raise RuntimeError('O par de sensores já foi fechado.')

        # As duas conversões começam antes da coleta para reduzir o skew temporal.
        self.right.solicita_leitura()
        self.left.solicita_leitura()
        raw_right = int(self.right.leitura_mm())
        raw_left = int(self.left.leitura_mm())
        self._validate_raw('direito', raw_right)
        self._validate_raw('esquerdo', raw_left)

        self._right_samples.append(raw_right)
        self._left_samples.append(raw_left)
        filtered_right = int(statistics.median(self._right_samples))
        filtered_left = int(statistics.median(self._left_samples))
        return DistanceSample(
            raw_left_mm=raw_left,
            raw_right_mm=raw_right,
            left_mm=filtered_left - self.config.left_offset_mm,
            right_mm=filtered_right - self.config.right_offset_mm,
        )

    def _validate_raw(self, side: str, value: int) -> None:
        if not self.config.raw_min_mm <= value <= self.config.raw_max_mm:
            raise ValueError(
                f'Leitura do sensor {side} fora da faixa válida: {value} mm.')

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        first_error = None
        for sensor in (self.right, self.left):
            if sensor is None:
                continue
            try:
                sensor.close()
            except Exception as error:  # Fechar o restante mesmo após uma falha.
                first_error = first_error or error
        try:
            self._bus.write_byte(self.config.mux_address, 0x00)
        except Exception as error:
            first_error = first_error or error
        finally:
            if self._owns_bus:
                try:
                    self._bus.close()
                except Exception as error:
                    first_error = first_error or error
        if first_error is not None:
            raise first_error
