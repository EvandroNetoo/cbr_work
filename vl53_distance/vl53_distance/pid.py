"""Controlador PID pequeno, determinístico e independente de ROS."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    derivative_filter_alpha: float
    output_limit: float

    def __post_init__(self) -> None:
        values = (
            self.kp, self.ki, self.kd, self.integral_limit,
            self.derivative_filter_alpha, self.output_limit,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Todos os parâmetros PID devem ser finitos.')
        if self.integral_limit < 0.0:
            raise ValueError('integral_limit não pode ser negativo.')
        if not 0.0 < self.derivative_filter_alpha <= 1.0:
            raise ValueError('derivative_filter_alpha deve estar em (0, 1].')
        if self.output_limit <= 0.0:
            raise ValueError('output_limit deve ser positivo.')


class PIDController:
    """PID com derivada filtrada, saturação e anti-windup condicional."""

    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: float | None = None
        self._derivative = 0.0

    def update(self, error: float, dt: float) -> float:
        error = float(error)
        dt = float(dt)
        if not math.isfinite(error):
            raise ValueError('O erro PID deve ser finito.')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('O período PID deve ser positivo e finito.')

        if self._previous_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self._previous_error) / dt
        alpha = self.config.derivative_filter_alpha
        self._derivative = (
            alpha * raw_derivative + (1.0 - alpha) * self._derivative)

        candidate_integral = self._clamp(
            self._integral + error * dt,
            self.config.integral_limit,
        )
        candidate_output = self._unclamped_output(error, candidate_integral)
        output = self._clamp(candidate_output, self.config.output_limit)

        # Não acumula integral quando ela empurraria ainda mais uma saída saturada.
        saturating_high = candidate_output > output and error > 0.0
        saturating_low = candidate_output < output and error < 0.0
        if not (saturating_high or saturating_low):
            self._integral = candidate_integral
            output = self._clamp(
                self._unclamped_output(error, self._integral),
                self.config.output_limit,
            )

        self._previous_error = error
        return output

    def _unclamped_output(self, error: float, integral: float) -> float:
        return (
            self.config.kp * error
            + self.config.ki * integral
            + self.config.kd * self._derivative
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit == 0.0:
            return 0.0
        return max(-limit, min(limit, value))
