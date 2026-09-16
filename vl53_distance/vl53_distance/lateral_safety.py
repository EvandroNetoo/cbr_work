"""Geometria pura para a protecao lateral baseada em LaserScan."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class PlanarTransform:
    """Parte XY de uma transformacao rigida 3D."""

    translation_x: float
    translation_y: float
    rotation_xx: float
    rotation_xy: float
    rotation_yx: float
    rotation_yy: float

    @classmethod
    def from_quaternion(
        cls,
        translation_x: float,
        translation_y: float,
        quaternion_x: float,
        quaternion_y: float,
        quaternion_z: float,
        quaternion_w: float,
    ) -> 'PlanarTransform':
        values = (
            translation_x,
            translation_y,
            quaternion_x,
            quaternion_y,
            quaternion_z,
            quaternion_w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Transformacao do LiDAR contem valores invalidos.')
        norm = math.sqrt(sum(value * value for value in values[2:]))
        if norm < 1e-9:
            raise ValueError('Transformacao do LiDAR contem quaternion nulo.')
        qx, qy, qz, qw = (value / norm for value in values[2:])
        return cls(
            translation_x=float(translation_x),
            translation_y=float(translation_y),
            rotation_xx=1.0 - 2.0 * (qy * qy + qz * qz),
            rotation_xy=2.0 * (qx * qy - qz * qw),
            rotation_yx=2.0 * (qx * qy + qz * qw),
            rotation_yy=1.0 - 2.0 * (qx * qx + qz * qz),
        )

    def apply(self, x: float, y: float) -> tuple[float, float]:
        return (
            self.translation_x + self.rotation_xx * x + self.rotation_xy * y,
            self.translation_y + self.rotation_yx * x + self.rotation_yy * y,
        )


@dataclass(frozen=True)
class LateralClearances:
    """Folgas desde as duas bordas laterais do footprint, em milimetros."""

    left_mm: float | None
    right_mm: float | None


def _minimum_cluster(
    samples: list[float],
    minimum_consecutive_points: int,
) -> float | None:
    best: float | None = None
    run: list[float] = []
    for value in samples + [math.nan]:
        if math.isfinite(value):
            run.append(value)
            continue
        if len(run) >= minimum_consecutive_points:
            candidate = min(run)
            best = candidate if best is None else min(best, candidate)
        run = []
    return best


def lateral_clearances_from_scan(
    ranges_m: Iterable[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    transform: PlanarTransform,
    footprint_half_length_m: float,
    footprint_half_width_m: float,
    longitudinal_margin_m: float,
    minimum_consecutive_points: int,
) -> LateralClearances:
    """Extrai os obstaculos nos corredores esquerdo e direito da base.

    Pontos fora da faixa longitudinal do footprint nao pertencem ao corredor
    lateral. Leituras isoladas sao descartadas para que um unico raio espurio
    nao interrompa a action.
    """
    numeric = (
        angle_min_rad,
        angle_increment_rad,
        range_min_m,
        range_max_m,
        footprint_half_length_m,
        footprint_half_width_m,
        longitudinal_margin_m,
    )
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError('Geometria do LiDAR deve conter valores finitos.')
    if angle_increment_rad == 0.0:
        raise ValueError('angle_increment do LiDAR nao pode ser zero.')
    if range_min_m < 0.0 or range_max_m <= range_min_m:
        raise ValueError('Limites de alcance do LiDAR sao invalidos.')
    if footprint_half_length_m <= 0.0 or footprint_half_width_m <= 0.0:
        raise ValueError('Dimensoes do footprint devem ser positivas.')
    if longitudinal_margin_m < 0.0:
        raise ValueError('Margem longitudinal nao pode ser negativa.')
    if minimum_consecutive_points <= 0:
        raise ValueError('Quantidade minima de pontos deve ser positiva.')

    corridor_x = footprint_half_length_m + longitudinal_margin_m
    left_samples: list[float] = []
    right_samples: list[float] = []
    for index, raw_range in enumerate(ranges_m):
        distance = float(raw_range)
        left_clearance = math.nan
        right_clearance = math.nan
        if math.isfinite(distance) and range_min_m <= distance <= range_max_m:
            angle = angle_min_rad + index * angle_increment_rad
            scan_x = distance * math.cos(angle)
            scan_y = distance * math.sin(angle)
            base_x, base_y = transform.apply(scan_x, scan_y)
            if -corridor_x <= base_x <= corridor_x:
                # Um retorno que ja entrou no footprint continua sendo uma
                # colisao (folga zero), em vez de desaparecer da selecao.
                if base_y > 0.0:
                    left_clearance = max(
                        0.0,
                        (base_y - footprint_half_width_m) * 1000.0,
                    )
                elif base_y < 0.0:
                    right_clearance = max(
                        0.0,
                        (-base_y - footprint_half_width_m) * 1000.0,
                    )
        left_samples.append(left_clearance)
        right_samples.append(right_clearance)

    return LateralClearances(
        left_mm=_minimum_cluster(
            left_samples, minimum_consecutive_points),
        right_mm=_minimum_cluster(
            right_samples, minimum_consecutive_points),
    )
