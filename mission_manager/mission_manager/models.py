"""Small immutable domain model shared by loaders and the executor."""

from __future__ import annotations

from dataclasses import dataclass


SERVICE_AREA_TYPES = frozenset({'WS', 'SH', 'PP'})
STEP_ACTIONS = frozenset({
    'navigate',
    'pick',
    'store',
    'retrieve',
    'place_on_table',
    'place_in_container',
    'stack',
    'place_on_shelf',
    'finish',
})


@dataclass(frozen=True)
class AlignmentConfig:
    distance_mm: int
    tolerance_mm: int
    timeout_s: float


@dataclass(frozen=True)
class MapPose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ServiceArea:
    area_id: str
    pose: MapPose
    height_cm: float
    area_type: str
    alignment: AlignmentConfig


@dataclass(frozen=True)
class Arena:
    frame_id: str
    start: MapPose
    finish: MapPose
    alignment_defaults: AlignmentConfig
    service_areas: dict[str, ServiceArea]

    def pose_for(self, target: str) -> MapPose:
        if target == 'start':
            return self.start
        if target == 'finish':
            return self.finish
        return self.service_areas[target].pose

    def has_target(self, target: str) -> bool:
        return target in {'start', 'finish'} or target in self.service_areas


@dataclass(frozen=True)
class Step:
    step_id: str
    action: str
    target: str | None = None
    tag_id: int | None = None
    slot_id: str | None = None
    analyze_apriltags: bool = False
    analyze_containers: bool = False
    container_color: str | None = None
    support_tag_id: int | None = None


@dataclass(frozen=True)
class Plan:
    plan_id: str
    steps: tuple[Step, ...]
