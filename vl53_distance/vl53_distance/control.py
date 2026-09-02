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


@dataclass(frozen=True)
class FollowWallCommand:
    linear_x_velocity_mps: float
    linear_y_velocity_mps: float
    angular_velocity_rad_s: float
    average_distance_mm: float
    wall_distance_error_mm: float
    alignment_error_mm: float
    traveled_distance_mm: float
    travel_error_mm: float
    inside_tolerance: bool


def limit_mecanum_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    wheel_linear_speed: float,
    kinematic_lever: float,
) -> tuple[float, float, float]:
    """Scale a planar command without changing its direction."""
    requested = (
        abs(float(linear_x))
        + abs(float(linear_y))
        + float(kinematic_lever) * abs(float(angular_z))
    )
    if requested <= wheel_linear_speed or requested == 0.0:
        return float(linear_x), float(linear_y), float(angular_z)
    scale = float(wheel_linear_speed) / requested
    return linear_x * scale, linear_y * scale, angular_z * scale


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


class FollowWallController:
    """Control wall distance, lateral displacement and parallel alignment."""

    def __init__(
        self,
        wall_pid: PIDController,
        travel_pid: PIDController,
        angular_pid: PIDController,
        *,
        wheel_linear_speed: float,
        kinematic_lever: float,
    ) -> None:
        self._wall_pid = wall_pid
        self._travel_pid = travel_pid
        self._angular_pid = angular_pid
        self._wheel_linear_speed = float(wheel_linear_speed)
        self._kinematic_lever = float(kinematic_lever)

    def reset(self) -> None:
        self._wall_pid.reset()
        self._travel_pid.reset()
        self._angular_pid.reset()

    def calculate(
        self,
        left_distance_mm: int,
        right_distance_mm: int,
        target_wall_mm: int,
        wall_tolerance_mm: int,
        traveled_distance_mm: float,
        target_travel_mm: int,
        travel_tolerance_mm: int,
        dt: float,
    ) -> FollowWallCommand:
        average = (float(left_distance_mm) + float(right_distance_mm)) / 2.0
        wall_error = average - float(target_wall_mm)
        alignment_error = float(right_distance_mm - left_distance_mm)
        travel_error = float(target_travel_mm) - float(traveled_distance_mm)
        lower = int(target_wall_mm) - int(wall_tolerance_mm)
        upper = int(target_wall_mm) + int(wall_tolerance_mm)
        wall_inside = (
            lower <= int(left_distance_mm) <= upper
            and lower <= int(right_distance_mm) <= upper
        )
        travel_inside = abs(travel_error) <= float(travel_tolerance_mm)

        if wall_inside:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self._wall_pid.update(wall_error / 1000.0, dt)
            angular_z = self._angular_pid.update(
                alignment_error / 1000.0, dt)

        if travel_inside:
            linear_y = 0.0
        else:
            # The public action uses positive=right, while REP-103 uses +Y=left.
            linear_y = -self._travel_pid.update(travel_error / 1000.0, dt)

        linear_x, linear_y, angular_z = limit_mecanum_command(
            linear_x,
            linear_y,
            angular_z,
            self._wheel_linear_speed,
            self._kinematic_lever,
        )
        return FollowWallCommand(
            linear_x_velocity_mps=linear_x,
            linear_y_velocity_mps=linear_y,
            angular_velocity_rad_s=angular_z,
            average_distance_mm=average,
            wall_distance_error_mm=wall_error,
            alignment_error_mm=alignment_error,
            traveled_distance_mm=float(traveled_distance_mm),
            travel_error_mm=travel_error,
            inside_tolerance=wall_inside and travel_inside,
        )
