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
    'place_on_precision_table',
    'finish',
})


@dataclass(frozen=True)
class AlignmentConfig:
    distance_mm: int
    tolerance_mm: int
    timeout_s: float


@dataclass(frozen=True)
class DepartureConfig:
    distance_mm: int
    tolerance_mm: int
    timeout_s: float
    lateral_position_mm: int
    max_alignment_error_mm: int | None = None
    alignment_recovery_distance_mm: int | None = None
    minimum_lateral_clearance_mm: int | None = None


@dataclass(frozen=True)
class PickupRecoveryConfig:
    enabled: bool
    minimum_wall_distance_mm: int
    maximum_wall_distance_mm: int
    minimum_lateral_position_mm: int
    maximum_lateral_position_mm: int
    preferred_tag_x_m: float
    preferred_tag_y_m: float
    wall_tolerance_mm: int
    travel_tolerance_mm: int
    timeout_s: float
    max_reposition_attempts: int
    search_positions_mm: tuple[int, ...]
    safety_search_distance_mm: int = 60
    safety_search_positions_mm: tuple[int, ...] = ()
    shelf_preferred_tag_x_m: float = 0.0
    shelf_preferred_tag_y_m: float = -0.22
    stack_preferred_tag_x_m: float = 0.0
    stack_preferred_tag_y_m: float = -0.22
    precision_preferred_tag_x_m: float = 0.0
    precision_preferred_tag_y_m: float = -0.22


@dataclass(frozen=True)
class TagObservation:
    area_id: str
    wall_distance_mm: float
    lateral_position_mm: float
    pickup_wall_distance_mm: int
    pickup_lateral_position_mm: float
    detection: object


@dataclass(frozen=True)
class ContainerObservation:
    """Best known observation point for one container color in one area."""

    area_id: str
    color: int
    wall_distance_mm: float
    lateral_position_mm: float
    detection: object


@dataclass(frozen=True)
class TableObservation:
    area_id: str
    wall_distance_mm: float
    lateral_position_mm: float
    detected_tag_ids: frozenset[int]
    apriltags_observed: bool = True
    detected_container_colors: frozenset[int] = frozenset()
    containers_observed: bool = False


@dataclass(frozen=True)
class SlotMovement:
    """Known next base destination that may overlap a cargo transfer."""

    wall_distance_mm: int
    lateral_position_mm: float
    departure: bool = False


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
    departure: DepartureConfig
    shelf_place_alignment: AlignmentConfig | None = None


@dataclass(frozen=True)
class Arena:
    frame_id: str
    start: MapPose
    finish: MapPose
    alignment_defaults: AlignmentConfig
    departure_defaults: DepartureConfig
    pickup_recovery: PickupRecoveryConfig
    service_areas: dict[str, ServiceArea]
    table_place_search_positions_mm: tuple[int, ...] | None = None
    shelf_place_alignment_defaults: AlignmentConfig = AlignmentConfig(40, 5, 10.0)

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
    container_color: str | None = None
    support_tag_id: int | None = None
    tag_ids: tuple[int, ...] = ()
    reference_tag_id: int | None = None


@dataclass(frozen=True)
class Visit:
    visit_id: str
    target: str
    tasks: tuple[Step, ...]
    pp_start_state: tuple[tuple[int, int | None], ...] | None = None
    pp_final_state: tuple[tuple[int, int | None], ...] | None = None


@dataclass(frozen=True)
class Plan:
    plan_id: str
    visits: tuple[Visit, ...]
    initial_location: str = 'start'
    finish: bool = False

    @property
    def total_steps(self) -> int:
        return len(self.visits) + int(self.finish) + sum(
            len(task.tag_ids) if task.action == 'stack' else 1
            for visit in self.visits for task in visit.tasks)


@dataclass(frozen=True)
class DeliveryOutcome:
    task_id: str
    tag_id: int
    area_id: str
    requested_action: str
    actual_action: str
    container_color: str | None = None
    support_tag_id: int | None = None
    reference_tag_id: int | None = None
