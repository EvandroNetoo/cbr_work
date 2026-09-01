"""Cálculos puros do controle longitudinal e de alinhamento."""

from __future__ import annotations

from dataclasses import dataclass

from .pid import PIDController


@dataclass(frozen=True)
class ControlCommand:
    linear_velocity_mps: float
    angular_velocity_rad_s: float
    average_distance_mm: float
    distance_error_mm: float
    alignment_error_mm: float
    inside_tolerance: bool


class DistanceController:
    def __init__(
        self,
        linear_pid: PIDController,
        angular_pid: PIDController,
    ) -> None:
        self._linear_pid = linear_pid
        self._angular_pid = angular_pid

    def reset(self) -> None:
        self._linear_pid.reset()
        self._angular_pid.reset()

    def calculate(
        self,
        left_distance_mm: int,
        right_distance_mm: int,
        target_mm: int,
        tolerance_mm: int,
        dt: float,
    ) -> ControlCommand:
        average = (float(left_distance_mm) + float(right_distance_mm)) / 2.0
        distance_error = average - float(target_mm)
        alignment_error = float(right_distance_mm - left_distance_mm)
        lower = int(target_mm) - int(tolerance_mm)
        upper = int(target_mm) + int(tolerance_mm)
        inside = (
            lower <= int(left_distance_mm) <= upper
            and lower <= int(right_distance_mm) <= upper
        )

        if inside:
            linear = 0.0
            angular = 0.0
        else:
            linear = self._linear_pid.update(distance_error / 1000.0, dt)
            angular = self._angular_pid.update(alignment_error / 1000.0, dt)

        return ControlCommand(
            linear_velocity_mps=linear,
            angular_velocity_rad_s=angular,
            average_distance_mm=average,
            distance_error_mm=distance_error,
            alignment_error_mm=alignment_error,
            inside_tolerance=inside,
        )
