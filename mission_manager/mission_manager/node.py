"""ROS 2 action server that executes validated mission steps sequentially."""

from __future__ import annotations

import copy
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from interfaces.action import (
    ExecuteMission,
    FollowWall,
    PickObject,
    PlaceInContainer,
    PlaceOnShelf,
    PlaceOnTable,
    PrepareManipulator,
    RetrieveObject,
    StackObject,
    StoreObject,
)
from interfaces.msg import (
    CargoSlotState, ManipulationResult, ManipulationState, SceneObservation,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .errors import ConfigurationError, MissionCanceled, StateConflict, StepFailed
from .loaders import load_arena, load_plan, PLAN_ID_PATTERN, validate_plan
from .models import (
    Arena,
    ContainerObservation,
    PickupRecoveryConfig,
    Plan,
    Step,
    TableObservation,
    TagObservation,
)
from .world_state import EMPTY, WorldState


class MissionManager(Node):
    """Own one mission at a time and compose existing semantic action servers."""

    def __init__(self) -> None:
        super().__init__('mission_manager')
        if (
            not hasattr(PickObject.Result(), 'observed_detections')
            or not hasattr(PickObject.Result(), 'scene_observation')
            or not hasattr(PrepareManipulator.Goal(), 'gripper_loaded')
            or not hasattr(PlaceOnTable.Goal(), 'use_fallback_pose')
        ):
            raise ConfigurationError(
                'As interfaces de manipulação instaladas estão desatualizadas; '
                'recompile interfaces antes de iniciar o mission_manager.'
            )
        share = Path(get_package_share_directory('mission_manager'))
        defaults = {
            'arena_file': str(share / 'config' / 'arena.yaml'),
            'plans_directory': str(share / 'config' / 'plans'),
            'execute_action': '/mission/execute',
            'state_topic': '/mission/state',
            'state_frame_id': 'arm_base_link',
            'cargo_slot_ids': ['left', 'right'],
            'navigate_action': '/navigate_to_pose',
            'wall_control_action': '/vl53/follow_wall',
            'follow_wall.max_alignment_error_mm': 100,
            'follow_wall.alignment_recovery_distance_mm': 100,
            'follow_wall.minimum_lateral_clearance_mm': 10,
            'prepare_action': '/manipulation/prepare',
            'pick_action': '/manipulation/pick',
            'store_action': '/manipulation/store',
            'retrieve_action': '/manipulation/retrieve',
            'place_on_table_action': '/manipulation/place_on_table',
            'place_in_container_action': '/manipulation/place_in_container',
            'stack_action': '/manipulation/stack',
            'place_on_shelf_action': '/manipulation/place_on_shelf',
            'server_timeout_s': 10.0,
            'navigation_timeout_s': 120.0,
            'manipulation_timeout_s': 120.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._wall_max_alignment_error_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.max_alignment_error_mm'))
        self._wall_alignment_recovery_distance_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.alignment_recovery_distance_mm'))
        self._wall_minimum_lateral_clearance_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.minimum_lateral_clearance_mm'))
        if (
            self._wall_alignment_recovery_distance_mm > 0
            and self._wall_max_alignment_error_mm == 0
        ):
            raise ConfigurationError(
                'follow_wall.alignment_recovery_distance_mm requer '
                'follow_wall.max_alignment_error_mm positivo.')

        self._callback_group = ReentrantCallbackGroup()
        self._cancel_event = threading.Event()
        self._lock = threading.RLock()
        self._busy = False
        self._status = 'idle'
        self._active_world_operation = ''
        self._current_step_index = 0
        self._current_location = 'start'
        self._current_wall_distance_mm: float | None = None
        self._current_lateral_position_mm = 0.0
        self._tag_observations: dict[tuple[str, int], TagObservation] = {}
        self._container_observations: dict[
            tuple[str, int], ContainerObservation
        ] = {}
        self._container_search_positions: dict[str, set[int]] = {}
        self._visited_search_positions: dict[str, set[int]] = {}
        self._last_table_observation: TableObservation | None = None
        self._active_child = None
        self._arena: Arena | None = None

        slot_ids = [
            str(value) for value in self.get_parameter('cargo_slot_ids').value
        ]
        try:
            self._world_state = WorldState(slot_ids)
        except ValueError as error:
            raise ConfigurationError(str(error)) from error

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_publisher = self.create_publisher(
            ManipulationState,
            str(self.get_parameter('state_topic').value),
            state_qos,
        )

        def client(action_type, parameter_name):
            return ActionClient(
                self,
                action_type,
                str(self.get_parameter(parameter_name).value),
                callback_group=self._callback_group,
            )

        self._navigate_client = client(NavigateToPose, 'navigate_action')
        self._wall_control_client = client(FollowWall, 'wall_control_action')
        self._prepare_client = client(PrepareManipulator, 'prepare_action')
        self._pick_client = client(PickObject, 'pick_action')
        self._store_client = client(StoreObject, 'store_action')
        self._retrieve_client = client(RetrieveObject, 'retrieve_action')
        self._place_table_client = client(PlaceOnTable, 'place_on_table_action')
        self._place_container_client = client(
            PlaceInContainer, 'place_in_container_action'
        )
        self._stack_client = client(StackObject, 'stack_action')
        self._place_shelf_client = client(PlaceOnShelf, 'place_on_shelf_action')

        self._server = ActionServer(
            self,
            ExecuteMission,
            str(self.get_parameter('execute_action').value),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )
        self._publish_world_state()
        self.get_logger().info(
            'Mission manager pronto; estado do mundo, arena e planos sob gestão.'
        )

    def _state_message(self) -> ManipulationState:
        known, gripper, slots = self._world_state.snapshot()
        message = ManipulationState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(
            self.get_parameter('state_frame_id').value
        )
        message.state_known = known
        message.gripper_object_id = gripper
        message.active_operation = self._active_world_operation
        for slot_id in sorted(slots):
            slot = CargoSlotState()
            slot.slot_id = slot_id
            slot.object_id = slots[slot_id]
            message.cargo_slots.append(slot)
        return message

    def _publish_world_state(self) -> None:
        self._state_publisher.publish(self._state_message())

    def _goal_callback(self, goal_request: ExecuteMission.Goal) -> GoalResponse:
        plan_id = str(goal_request.plan_id)
        with self._lock:
            if self._busy or not PLAN_ID_PATTERN.fullmatch(plan_id):
                return GoalResponse.REJECT
            self._cancel_event.clear()
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_event.set()
        self._cancel_active_child()
        return CancelResponse.ACCEPT

    def _cancel_active_child(self) -> None:
        with self._lock:
            child = self._active_child
        if child is not None:
            child.cancel_goal_async()

    def _check_canceled(self) -> None:
        if self._cancel_event.is_set():
            self._cancel_active_child()
            raise MissionCanceled('Missão cancelada pelo cliente.')

    def _wait_future(
        self,
        future: Any,
        timeout_s: float,
        *,
        check_cancel: bool = True,
    ) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if check_cancel:
                self._check_canceled()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError('Operação ROS excedeu o tempo limite.')
            completed.wait(min(0.05, remaining))
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _server_timeout(self) -> float:
        value = float(self.get_parameter('server_timeout_s').value)
        if not math.isfinite(value) or value <= 0.0:
            raise ConfigurationError('server_timeout_s deve ser positivo.')
        return value

    def _nonnegative_integer_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ConfigurationError(
                f'{name} deve ser um inteiro nao negativo.')
        return int(value)

    def _call_action(
        self,
        client: ActionClient,
        goal: Any,
        description: str,
        timeout_s: float,
        validate_result: Callable[[Any], str | None] | None = None,
        *,
        allow_unsuccessful_status: bool = False,
        accept_unsuccessful_result: Callable[[Any], bool] | None = None,
        on_goal_accepted: Callable[[], None] | None = None,
    ) -> Any:
        self._check_canceled()
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ConfigurationError(f'Timeout inválido para {description}.')
        if not client.wait_for_server(timeout_sec=self._server_timeout()):
            raise StepFailed(f'Servidor indisponível: {description}.')
        try:
            send_future = client.send_goal_async(goal)
            child = self._wait_future(
                send_future, self._server_timeout(), check_cancel=False
            )
            if child is None or not child.accepted:
                raise StepFailed(f'Goal rejeitado: {description}.')
            with self._lock:
                self._active_child = child
            if on_goal_accepted is not None:
                on_goal_accepted()
            self._check_canceled()
            result_wrapper = self._wait_future(
                child.get_result_async(), timeout_s
            )
        except MissionCanceled:
            raise
        except StepFailed:
            raise
        except TimeoutError as error:
            self._cancel_active_child()
            raise StepFailed(f'Timeout durante {description}.') from error
        except Exception as error:
            raise StepFailed(f'Falha de comunicação em {description}: {error}') from error
        finally:
            with self._lock:
                self._active_child = None
        if result_wrapper is None:
            raise StepFailed(f'{description} falhou sem resultado.')
        result = result_wrapper.result
        self._validate_action_status(
            result_wrapper.status,
            result,
            description,
            allow_unsuccessful_status=allow_unsuccessful_status,
            accept_unsuccessful_result=accept_unsuccessful_result,
        )
        if validate_result is not None:
            failure = validate_result(result)
            if failure:
                raise StepFailed(f'{description} falhou: {failure}')
        return result

    @staticmethod
    def _validate_action_status(
        status: int,
        result: Any,
        description: str,
        *,
        allow_unsuccessful_status: bool,
        accept_unsuccessful_result: Callable[[Any], bool] | None = None,
    ) -> None:
        if status == GoalStatus.STATUS_SUCCEEDED:
            return
        if allow_unsuccessful_status:
            outcome = getattr(result, 'outcome', None)
            if (
                outcome is not None
                and int(outcome.code) != int(outcome.SUCCESS)
            ):
                return
        if (
            accept_unsuccessful_result is not None
            and accept_unsuccessful_result(result)
        ):
            return
        raise StepFailed(f'{description} falhou com estado {status}.')

    @staticmethod
    def _manipulation_failure(result: Any) -> str | None:
        if result.outcome.code == result.outcome.SUCCESS:
            return None
        return result.outcome.message or f'código {result.outcome.code}'

    def _mark_world_unknown(self) -> None:
        self._world_state.mark_unknown()
        self._publish_world_state()

    def _reconcile_manipulation_result(
        self,
        operation: str,
        tag_id: int,
        slot_id: str,
        result: Any,
    ) -> None:
        """Apply only a physical effect explicitly confirmed by an action result."""
        outcome = result.outcome
        if not bool(outcome.effect_known):
            self._mark_world_unknown()
            return
        expected_locations = {
            'pick': ManipulationResult.LOCATION_GRIPPER,
            'store': ManipulationResult.LOCATION_CARGO,
            'retrieve': ManipulationResult.LOCATION_GRIPPER,
            'place': ManipulationResult.LOCATION_DESTINATION,
        }
        expected = expected_locations[operation]
        location = int(outcome.final_object_location)
        effect_reported = location == expected
        success = int(outcome.code) == int(ManipulationResult.SUCCESS)
        if success and not effect_reported:
            self._mark_world_unknown()
            raise StepFailed(
                'Action de manipulação declarou sucesso sem confirmar o efeito '
                f'físico esperado para {operation}.'
            )
        if not effect_reported:
            if location not in (
                ManipulationResult.LOCATION_UNKNOWN,
                ManipulationResult.LOCATION_SOURCE,
            ):
                self._mark_world_unknown()
            return

        if operation == 'pick':
            self._world_state.commit_pick(tag_id)
        elif operation == 'store':
            self._world_state.commit_store(tag_id, slot_id)
        elif operation == 'retrieve':
            self._world_state.commit_retrieve(tag_id, slot_id)
        else:
            self._world_state.commit_place()
        self._publish_world_state()

    def _call_manipulation_action(
        self,
        client: ActionClient,
        goal: Any,
        description: str,
        timeout_s: float,
        operation: str,
        tag_id: int,
        slot_id: str = '',
    ) -> Any:
        """Call a physical action and conservatively own its logical transition."""
        goal_accepted = False

        def note_acceptance() -> None:
            nonlocal goal_accepted
            goal_accepted = True

        try:
            result = self._call_action(
                client,
                goal,
                description,
                timeout_s,
                allow_unsuccessful_status=True,
                on_goal_accepted=note_acceptance,
            )
        except (MissionCanceled, StepFailed):
            # Without a result, the manager cannot know whether the gripper crossed
            # the irreversible point of the requested operation.
            if goal_accepted:
                self._mark_world_unknown()
            raise
        self._reconcile_manipulation_result(
            operation, tag_id, slot_id, result
        )
        self._remember_scene_observations(result)
        return result

    @staticmethod
    def _navigation_failure(result: NavigateToPose.Result) -> str | None:
        if result.error_code == NavigateToPose.Result.NONE:
            return None
        return result.error_msg or f'código {result.error_code}'

    @staticmethod
    def _wall_control_failure(result: FollowWall.Result) -> str | None:
        if result.has_valid_reading and result.has_valid_odometry:
            return None
        return result.message or 'sensores de distância ou odometria inválidos'

    def _accept_wall_control_abort(self, result: FollowWall.Result) -> bool:
        """Aceita somente as paradas de seguranca configuradas da FollowWall."""
        message = str(result.message)
        tolerated_prefixes = (
            'Desalinhamento de ',
            'Recuperação concluída após retorno lateral de ',
            'Obstaculo no lado ',
        )
        accepted = (
            bool(result.has_valid_reading)
            and bool(result.has_valid_odometry)
            and message.startswith(tolerated_prefixes)
        )
        if accepted:
            self.get_logger().warning(
                f'FollowWall interrompida por protecao; a missao continuara: '
                f'{message} Resultado parcial: parede='
                f'{result.final_average_distance_mm:.1f} mm, deslocamento '
                f'lateral={result.traveled_distance_mm:.1f} mm.')
        return accepted

    @staticmethod
    def _pickup_recovery_correction(
        current_wall_distance_mm: float,
        detected_x_m: float,
        detected_y_m: float,
        config: PickupRecoveryConfig,
    ) -> tuple[int, int]:
        target_wall = round(
            current_wall_distance_mm
            + 1000.0 * (detected_y_m - config.preferred_tag_y_m)
        )
        target_wall = max(
            config.minimum_wall_distance_mm,
            min(config.maximum_wall_distance_mm, target_wall),
        )
        travel = round(1000.0 * (config.preferred_tag_x_m - detected_x_m))
        if abs(target_wall - current_wall_distance_mm) <= config.wall_tolerance_mm:
            target_wall = round(current_wall_distance_mm)
        if abs(travel) <= config.travel_tolerance_mm:
            travel = 0
        return target_wall, travel

    def _duration(self, seconds: float):
        goal_duration = FollowWall.Goal().timeout
        total_nanoseconds = round(seconds * 1_000_000_000)
        goal_duration.sec, goal_duration.nanosec = divmod(
            total_nanoseconds, 1_000_000_000
        )
        return goal_duration

    def _control_wall(
        self,
        distance_mm: int,
        tolerance_mm: int,
        timeout_s: float,
        description: str,
        *,
        travel_distance_mm: int = 0,
        travel_tolerance_mm: int | None = None,
        max_alignment_error_mm: int | None = None,
        alignment_recovery_distance_mm: int | None = None,
        minimum_lateral_clearance_mm: int | None = None,
    ) -> FollowWall.Result:
        goal = FollowWall.Goal()
        goal.wall_distance_mm = int(distance_mm)
        goal.travel_distance_mm = int(travel_distance_mm)
        goal.wall_tolerance_mm = int(tolerance_mm)
        goal.travel_tolerance_mm = int(
            travel_tolerance_mm
            if travel_tolerance_mm is not None
            else tolerance_mm
        )
        has_lateral_travel = goal.travel_distance_mm != 0
        configured_max_alignment_error = (
            self._wall_max_alignment_error_mm
            if max_alignment_error_mm is None
            else max_alignment_error_mm
        )
        configured_recovery_distance = (
            self._wall_alignment_recovery_distance_mm
            if alignment_recovery_distance_mm is None
            else alignment_recovery_distance_mm
        )
        goal.max_alignment_error_mm = (
            configured_max_alignment_error if has_lateral_travel else 0)
        goal.alignment_recovery_distance_mm = (
            configured_recovery_distance if has_lateral_travel else 0)
        goal.minimum_lateral_clearance_mm = (
            self._wall_minimum_lateral_clearance_mm
            if minimum_lateral_clearance_mm is None
            else minimum_lateral_clearance_mm
        )
        goal.timeout = self._duration(timeout_s)
        self.get_logger().info(
            f'Iniciando FollowWall ({description}): parede alvo='
            f'{goal.wall_distance_mm}±{goal.wall_tolerance_mm} mm, '
            f'deslocamento lateral={goal.travel_distance_mm}±'
            f'{goal.travel_tolerance_mm} mm, folga lateral minima='
            f'{goal.minimum_lateral_clearance_mm} mm, timeout={timeout_s:.1f} s.'
        )
        result = self._call_action(
            self._wall_control_client,
            goal,
            description,
            timeout_s + 5.0,
            self._wall_control_failure,
            accept_unsuccessful_result=self._accept_wall_control_abort,
        )
        self.get_logger().info(
            f'FollowWall finalizada ({description}): parede final='
            f'{result.final_average_distance_mm:.1f} mm, deslocamento lateral '
            f'efetivo={result.traveled_distance_mm:.1f} mm; {result.message}'
        )
        return result

    def _navigation_timeout(self) -> float:
        return float(self.get_parameter('navigation_timeout_s').value)

    def _manipulation_timeout(self) -> float:
        return float(self.get_parameter('manipulation_timeout_s').value)

    def _prepare_for_navigation(self) -> None:
        known, gripper, _ = self._world_state.snapshot()
        if not known:
            raise StepFailed(
                'Estado da carga incerto; navegação automática bloqueada.'
            )
        goal = PrepareManipulator.Goal()
        goal.mode = PrepareManipulator.Goal.NAVIGATION
        goal.gripper_loaded = gripper != EMPTY
        self._call_action(
            self._prepare_client,
            goal,
            'preparação do manipulador para navegação',
            self._manipulation_timeout(),
            self._manipulation_failure,
        )

    def _prepare_for_pick_observation(self) -> None:
        goal = PrepareManipulator.Goal()
        goal.mode = PrepareManipulator.Goal.OBSERVATION
        goal.gripper_loaded = False
        self._call_action(
            self._prepare_client,
            goal,
            'preparação do manipulador para observar AprilTags',
            self._manipulation_timeout(),
            self._manipulation_failure,
        )

    def _update_table_position(self, result: FollowWall.Result) -> None:
        self._current_wall_distance_mm = float(
            result.final_average_distance_mm
        )
        self._current_lateral_position_mm += float(
            result.traveled_distance_mm
        )

    @staticmethod
    def _container_observation_quality(detection: Any) -> tuple[Any, ...]:
        """Prefer complete, well-supported and geometrically stable views."""
        def finite_or_infinity(value: Any) -> float:
            number = float(value)
            return number if math.isfinite(number) else math.inf

        return (
            not bool(detection.partial),
            int(detection.observation_count),
            -finite_or_infinity(detection.position_spread_m),
            -finite_or_infinity(detection.position_uncertainty_m),
            -finite_or_infinity(detection.pose_error),
        )

    def _remember_scene_observations(self, result: Any) -> None:
        """Merge one action's camera snapshot into mission-owned memory."""
        arena = getattr(self, '_arena', None)
        if arena is None or self._current_wall_distance_mm is None:
            return
        scene = getattr(result, 'scene_observation', None)
        if scene is not None and bool(scene.completed):
            detections = list(scene.apriltags)
            containers = list(scene.containers)
            apriltag_observation_completed = bool(
                int(scene.requested_detectors) & SceneObservation.APRILTAGS
            )
            container_observation_completed = bool(
                int(scene.requested_detectors) & SceneObservation.CONTAINERS
            )
        else:
            detections = list(getattr(result, 'observed_detections', []))
            containers = []
            apriltag_observation_completed = (
                hasattr(result, 'observed_detections')
                and (
                    bool(detections)
                    or result.outcome.code in {
                        ManipulationResult.SUCCESS,
                        ManipulationResult.OBJECT_NOT_FOUND,
                    }
                )
            )
            container_observation_completed = False
        config = arena.pickup_recovery
        if apriltag_observation_completed or container_observation_completed:
            self._last_table_observation = TableObservation(
                area_id=self._current_location,
                wall_distance_mm=self._current_wall_distance_mm,
                lateral_position_mm=self._current_lateral_position_mm,
                detected_tag_ids=frozenset(
                    int(detection.id) for detection in detections
                ),
                apriltags_observed=apriltag_observation_completed,
                detected_container_colors=frozenset(
                    int(detection.color) for detection in containers
                ),
                containers_observed=container_observation_completed,
            )
            visited = self._visited_search_positions.setdefault(
                self._current_location, set()
            )
            area = arena.service_areas[self._current_location]
            if (
                abs(
                    area.alignment.distance_mm
                    - self._current_wall_distance_mm
                )
                <= config.wall_tolerance_mm
            ):
                for position in config.search_positions_mm:
                    if (
                        abs(position - self._current_lateral_position_mm)
                        <= config.travel_tolerance_mm
                    ):
                        visited.add(position)

        if container_observation_completed:
            area = arena.service_areas[self._current_location]
            if (
                abs(
                    area.alignment.distance_mm
                    - self._current_wall_distance_mm
                )
                <= config.wall_tolerance_mm
            ):
                observed_positions = getattr(
                    self, '_container_search_positions', None)
                if observed_positions is None:
                    observed_positions = {}
                    self._container_search_positions = observed_positions
                positions = observed_positions.setdefault(
                    self._current_location, set())
                for position in config.search_positions_mm:
                    if (
                        abs(position - self._current_lateral_position_mm)
                        <= config.travel_tolerance_mm
                    ):
                        positions.add(position)

        for detection in detections:
            pose = detection.pose.position
            pickup_wall, pickup_travel = self._pickup_recovery_correction(
                self._current_wall_distance_mm,
                float(pose.x),
                float(pose.y),
                config,
            )
            observation = TagObservation(
                area_id=self._current_location,
                wall_distance_mm=self._current_wall_distance_mm,
                lateral_position_mm=self._current_lateral_position_mm,
                pickup_wall_distance_mm=pickup_wall,
                pickup_lateral_position_mm=self._clamp_lateral_position(
                    self._current_lateral_position_mm + pickup_travel
                ),
                detection=copy.deepcopy(detection),
            )
            self._tag_observations[
                (self._current_location, int(detection.id))
            ] = observation

        container_memory = getattr(self, '_container_observations', None)
        if container_memory is None:
            container_memory = {}
            self._container_observations = container_memory
        for detection in containers:
            color = int(detection.color)
            key = (self._current_location, color)
            previous = container_memory.get(key)
            if (
                previous is not None
                and self._container_observation_quality(previous.detection)
                > self._container_observation_quality(detection)
            ):
                continue
            container_memory[key] = ContainerObservation(
                area_id=self._current_location,
                color=color,
                wall_distance_mm=self._current_wall_distance_mm,
                lateral_position_mm=self._current_lateral_position_mm,
                detection=copy.deepcopy(detection),
            )

    def _remember_pick_observations(self, result: PickObject.Result) -> None:
        """Compatibility wrapper for callers of the former pick-only memory."""
        self._remember_scene_observations(result)

    def _forget_picked_tag(self, tag_id: int) -> None:
        for key in [key for key in self._tag_observations if key[1] == tag_id]:
            del self._tag_observations[key]

    def _clamp_lateral_position(self, position_mm: float) -> float:
        assert self._arena is not None
        config = self._arena.pickup_recovery
        return max(
            float(config.minimum_lateral_position_mm),
            min(float(config.maximum_lateral_position_mm), position_mm),
        )

    def _current_observation_excludes(self, tag_id: int) -> bool:
        if self._current_wall_distance_mm is None:
            return False
        observation = getattr(self, '_last_table_observation', None)
        if observation is None or observation.area_id != self._current_location:
            return False
        config = self._arena.pickup_recovery
        same_wall_distance = (
            abs(
                observation.wall_distance_mm
                - self._current_wall_distance_mm
            )
            <= config.wall_tolerance_mm
        )
        same_lateral_position = (
            abs(
                observation.lateral_position_mm
                - self._current_lateral_position_mm
            )
            <= config.travel_tolerance_mm
        )
        return (
            same_wall_distance
            and same_lateral_position
            and observation.apriltags_observed
            and tag_id not in observation.detected_tag_ids
        )

    def _current_observation_excludes_container(self, color: int) -> bool:
        if self._current_wall_distance_mm is None:
            return False
        observation = getattr(self, '_last_table_observation', None)
        if observation is None or observation.area_id != self._current_location:
            return False
        config = self._arena.pickup_recovery
        same_wall_distance = (
            abs(
                observation.wall_distance_mm
                - self._current_wall_distance_mm
            )
            <= config.wall_tolerance_mm
        )
        same_lateral_position = (
            abs(
                observation.lateral_position_mm
                - self._current_lateral_position_mm
            )
            <= config.travel_tolerance_mm
        )
        return (
            same_wall_distance
            and same_lateral_position
            and observation.containers_observed
            and int(color) not in observation.detected_container_colors
        )

    def _move_to_table_position(
        self,
        wall_distance_mm: int,
        lateral_position_mm: float,
        description: str,
    ) -> bool:
        assert self._arena is not None
        config = self._arena.pickup_recovery
        if self._current_wall_distance_mm is None:
            raise StepFailed(
                'Não há uma distância atual válida da parede para '
                'reposicionar o robô na mesa.'
            )
        requested_lateral_position_mm = float(lateral_position_mm)
        bounded_lateral_position_mm = self._clamp_lateral_position(
            requested_lateral_position_mm
        )
        if bounded_lateral_position_mm != requested_lateral_position_mm:
            self.get_logger().warning(
                f'Destino lateral {requested_lateral_position_mm:.0f} mm '
                f'limitado para {bounded_lateral_position_mm:.0f} mm; limites '
                f'permitidos: [{config.minimum_lateral_position_mm}, '
                f'{config.maximum_lateral_position_mm}] mm.'
            )
        travel = round(
            bounded_lateral_position_mm - self._current_lateral_position_mm
        )
        wall = int(wall_distance_mm)
        wall_is_current = (
            abs(wall - self._current_wall_distance_mm)
            <= config.wall_tolerance_mm
        )
        if abs(travel) <= config.travel_tolerance_mm:
            travel = 0
        if wall_is_current and travel == 0:
            self.get_logger().info(
                f'Reposicionamento dispensado ({description}): posição atual '
                f'já atende parede={self._current_wall_distance_mm:.1f} mm e '
                f'lateral={self._current_lateral_position_mm:.1f} mm.'
            )
            return False

        self.get_logger().info(
            f'Reposicionamento de mesa ({description}): posição atual '
            f'parede={self._current_wall_distance_mm:.1f} mm, lateral='
            f'{self._current_lateral_position_mm:.1f} mm; destino parede='
            f'{wall} mm, lateral={bounded_lateral_position_mm:.1f} mm; '
            f'percurso lateral solicitado={travel} mm.'
        )
        self._prepare_for_pick_observation()
        follow_result = self._control_wall(
            wall,
            config.wall_tolerance_mm,
            config.timeout_s,
            description,
            travel_distance_mm=travel,
            travel_tolerance_mm=config.travel_tolerance_mm,
        )
        self._update_table_position(follow_result)
        self.get_logger().info(
            f'Estado da mesa atualizado ({description}): parede='
            f'{self._current_wall_distance_mm:.1f} mm, lateral='
            f'{self._current_lateral_position_mm:.1f} mm '
            f'(avanço lateral medido={follow_result.traveled_distance_mm:.1f} '
            'mm).'
        )
        return True

    def _position_from_memory(
        self, tag_id: int
    ) -> TagObservation | None:
        observation = self._tag_observations.get(
            (self._current_location, tag_id)
        )
        if observation is None:
            return None
        self.get_logger().info(
            f'AprilTag {tag_id} já observada em {self._current_location}; '
            f'indo para parede={observation.pickup_wall_distance_mm} mm, '
            f'lateral={observation.pickup_lateral_position_mm:.0f} mm.'
        )
        self._move_to_table_position(
            observation.pickup_wall_distance_mm,
            observation.pickup_lateral_position_mm,
            f'retorno à posição armazenada da AprilTag {tag_id}',
        )
        return observation

    def _position_from_container_memory(self, color: int) -> bool:
        memory = getattr(self, '_container_observations', {})
        observation = memory.get((self._current_location, int(color)))
        if observation is None:
            return False
        color_name = {
            PlaceInContainer.Goal.RED: 'vermelho',
            PlaceInContainer.Goal.BLUE: 'azul',
        }.get(int(color), str(color))
        self.get_logger().info(
            f'Contêiner {color_name} já observado em '
            f'{self._current_location}; retornando ao ponto de observação '
            f'parede={observation.wall_distance_mm:.0f} mm, '
            f'lateral={observation.lateral_position_mm:.0f} mm. Uma nova '
            'detecção será feita antes do depósito.'
        )
        self._move_to_table_position(
            round(observation.wall_distance_mm),
            observation.lateral_position_mm,
            f'retorno ao contêiner {color_name} observado',
        )
        return True

    def _return_to_original_observation(
        self,
        tag_id: int,
        observation: TagObservation,
    ) -> bool:
        self.get_logger().info(
            f'AprilTag {tag_id} não reapareceu na posição estimada de '
            'coleta; retornando ao ponto original da observação: '
            f'parede={observation.wall_distance_mm:.0f} mm, '
            f'lateral={observation.lateral_position_mm:.0f} mm.'
        )
        return self._move_to_table_position(
            round(observation.wall_distance_mm),
            observation.lateral_position_mm,
            f'retorno ao ponto original da observação da AprilTag {tag_id}',
        )

    def _move_to_next_search_position(self, tag_id: int) -> bool:
        assert self._arena is not None
        config = self._arena.pickup_recovery
        visited = self._visited_search_positions.setdefault(
            self._current_location, set()
        )
        candidates = [
            position for position in config.search_positions_mm
            if position not in visited
        ]
        if not candidates:
            return False
        destination = min(
            candidates,
            key=lambda position: (
                abs(position - self._current_lateral_position_mm),
                config.search_positions_mm.index(position),
            ),
        )
        area = self._arena.service_areas[self._current_location]
        self.get_logger().info(
            f'AprilTag {tag_id} ainda não localizada; buscando em '
            f'lateral={destination} mm de {self._current_location}.'
        )
        self._move_to_table_position(
            area.alignment.distance_mm,
            destination,
            f'busca lateral da AprilTag {tag_id} em {destination} mm',
        )
        # O destino foi tentado mesmo quando FollowWall terminou por uma
        # protecao tolerada. Mantemos a posicao fisica medida, mas nao voltamos
        # a selecionar indefinidamente o mesmo extremo da busca.
        visited.add(destination)
        return True

    def _mark_current_search_position(self, visited: set[int]) -> None:
        """Mark the configured table-search point at the current pose."""
        assert self._arena is not None
        if self._current_wall_distance_mm is None:
            return
        config = self._arena.pickup_recovery
        area = self._arena.service_areas[self._current_location]
        if (
            abs(
                area.alignment.distance_mm - self._current_wall_distance_mm
            )
            > config.wall_tolerance_mm
        ):
            return
        for position in config.search_positions_mm:
            if (
                abs(position - self._current_lateral_position_mm)
                <= config.travel_tolerance_mm
            ):
                visited.add(position)

    def _move_to_next_place_search_position(
        self, step: Step, visited: set[int]
    ) -> bool:
        """Move to the nearest untried table-search point for this step."""
        assert self._arena is not None
        config = self._arena.pickup_recovery
        candidates = [
            position for position in config.search_positions_mm
            if position not in visited
        ]
        if not candidates:
            return False
        destination = min(
            candidates,
            key=lambda position: (
                abs(position - self._current_lateral_position_mm),
                config.search_positions_mm.index(position),
            ),
        )
        area = self._arena.service_areas[self._current_location]
        self.get_logger().info(
            f"Destino do passo '{step.step_id}' ainda não disponível; "
            f'buscando em lateral={destination} mm de '
            f'{self._current_location}.'
        )
        self._move_to_table_position(
            area.alignment.distance_mm,
            destination,
            f"busca de destino do passo '{step.step_id}' em {destination} mm",
        )
        # Consider the destination attempted even if FollowWall stopped at a
        # tolerated safety limit, matching the finite pickup-search behavior.
        visited.add(destination)
        return True

    def _default_place_goal(self, height_cm: float) -> PlaceOnTable.Goal:
        """Build the final deterministic table-placement request."""
        goal = PlaceOnTable.Goal()
        goal.ws_height_cm = float(height_cm)
        goal.use_fallback_pose = True
        return goal

    def _navigate(self, target: str) -> None:
        assert self._arena is not None
        pose = self._arena.pose_for(target)
        if (
            self._current_location in self._arena.service_areas
            and target != self._current_location
        ):
            departure = self._arena.service_areas[
                self._current_location
            ].departure
            departure_travel_mm = round(
                departure.lateral_position_mm
                - self._current_lateral_position_mm
            )
            self._control_wall(
                departure.distance_mm,
                departure.tolerance_mm,
                departure.timeout_s,
                f'recuo para sair de {self._current_location}',
                travel_distance_mm=departure_travel_mm,
                max_alignment_error_mm=departure.max_alignment_error_mm,
                alignment_recovery_distance_mm=(
                    departure.alignment_recovery_distance_mm
                ),
                minimum_lateral_clearance_mm=(
                    departure.minimum_lateral_clearance_mm
                ),
            )
            self._current_wall_distance_mm = None
            self._current_lateral_position_mm = 0.0
        self._prepare_for_navigation()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._arena.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x_m
        goal.pose.pose.position.y = pose.y_m
        goal.pose.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        self._call_action(
            self._navigate_client,
            goal,
            f'navegação até {target}',
            self._navigation_timeout(),
            self._navigation_failure,
        )
        self._current_wall_distance_mm = None
        self._current_lateral_position_mm = 0.0
        if target in self._arena.service_areas:
            alignment = self._arena.service_areas[target].alignment
            result = self._control_wall(
                alignment.distance_mm,
                alignment.tolerance_mm,
                alignment.timeout_s,
                f'alinhamento em {target}',
            )
            self._current_wall_distance_mm = float(
                result.final_average_distance_mm
            )
            self._current_lateral_position_mm = 0.0
        self._current_location = target

    def _recover_pick(self, result: PickObject.Result, step: Step) -> None:
        assert self._arena is not None
        config = self._arena.pickup_recovery
        if self._current_wall_distance_mm is None:
            raise StepFailed(
                'Não há uma distância atual válida da parede para recuperar '
                'a coleta.'
            )
        pose = result.detected_pose.pose.position
        target_wall, travel = self._pickup_recovery_correction(
            self._current_wall_distance_mm,
            float(pose.x),
            float(pose.y),
            config,
        )
        requested_lateral_position = self._current_lateral_position_mm + travel
        target_lateral_position = self._clamp_lateral_position(
            requested_lateral_position
        )
        bounded_travel = round(
            target_lateral_position - self._current_lateral_position_mm
        )
        lateral_limit_message = ''
        if target_lateral_position != requested_lateral_position:
            lateral_limit_message = (
                f' A correção lateral desejada de {travel} mm foi limitada '
                f'para {bounded_travel} mm pelo destino absoluto permitido '
                f'[{config.minimum_lateral_position_mm}, '
                f'{config.maximum_lateral_position_mm}] mm.'
            )
        if (
            bounded_travel == 0
            and target_wall == round(self._current_wall_distance_mm)
        ):
            raise StepFailed(
                f"passo '{step.step_id}' (pick) continua fora do alcance, "
                'mas a correção calculada não produziria movimento dentro das '
                'tolerâncias e dos limites configurados.'
            )
        self.get_logger().warning(
            f'Coleta da AprilTag {step.tag_id} fora do alcance em '
            f'x={pose.x:.3f}, y={pose.y:.3f} m. Reposicionando a base para '
            f'{target_wall} mm da parede e deslocando {bounded_travel} mm '
            f'(positivo=direita, negativo=esquerda).{lateral_limit_message}'
        )
        moved = self._move_to_table_position(
            target_wall,
            target_lateral_position,
            f"reposicionamento para repetir o passo '{step.step_id}'",
        )
        if not moved:
            raise StepFailed(
                f"passo '{step.step_id}' (pick) não produziu um novo "
                'reposicionamento.'
            )

    def _execute_pick(self, step: Step, timeout: float) -> None:
        assert self._arena is not None
        try:
            self._world_state.validate_pick(int(step.tag_id))
        except StateConflict as error:
            raise StepFailed(
                f"passo '{step.step_id}' (pick) bloqueado pelo estado: {error}"
            ) from error
        config = self._arena.pickup_recovery
        original_observation = None
        if config.enabled:
            original_observation = self._position_from_memory(
                int(step.tag_id)
            )
            if (
                original_observation is None
                and self._current_observation_excludes(int(step.tag_id))
            ):
                self.get_logger().info(
                    f'AprilTag {step.tag_id} ausente na última observação da '
                    'posição atual; evitando uma nova detecção no mesmo local.'
                )
                if not self._move_to_next_search_position(int(step.tag_id)):
                    raise StepFailed(
                        f"passo '{step.step_id}' (pick) falhou: AprilTag "
                        f'{step.tag_id} não apareceu na última observação e '
                        'todas as posições de busca já foram examinadas.'
                    )
        original_fallback_pending = original_observation is not None
        reposition_count = 0
        while True:
            goal = PickObject.Goal()
            goal.tag_id = int(step.tag_id)
            goal.profile = ''
            goal.ws_height_cm = float(
                self._arena.service_areas[self._current_location].height_cm
            )
            result = self._call_manipulation_action(
                self._pick_client,
                goal,
                f"passo '{step.step_id}' (pick)",
                timeout,
                'pick',
                int(step.tag_id),
            )
            failure = self._manipulation_failure(result)
            if failure is None:
                self._forget_picked_tag(int(step.tag_id))
                return
            if not result.outcome.effect_known:
                raise StepFailed(
                    f"passo '{step.step_id}' (pick) deixou o estado físico "
                    f'incerto: {failure}'
                )
            recoverable = (
                config.enabled
                and result.has_detected_pose
                and result.recovery_reason != PickObject.Result.RECOVERY_NONE
            )
            if recoverable:
                if reposition_count >= config.max_reposition_attempts:
                    raise StepFailed(
                        f"passo '{step.step_id}' (pick) falhou: {failure}"
                    )
                self._recover_pick(result, step)
                reposition_count += 1
                continue
            if (
                config.enabled
                and result.outcome.code == ManipulationResult.OBJECT_NOT_FOUND
            ):
                if original_fallback_pending:
                    original_fallback_pending = False
                    assert original_observation is not None
                    if self._return_to_original_observation(
                        int(step.tag_id), original_observation
                    ):
                        continue
                if self._move_to_next_search_position(int(step.tag_id)):
                    continue
            raise StepFailed(
                f"passo '{step.step_id}' (pick) falhou: {failure}"
            )

    def _execute_place_with_recovery(
        self,
        step: Step,
        client: ActionClient,
        goal: Any,
        tag_id: int,
        height_cm: float,
        timeout: float,
    ) -> None:
        """Retry perception-based placement across table search points."""
        visited: set[int] = set(
            getattr(self, '_container_search_positions', {}).get(
                self._current_location, set()
            )
            if step.action == 'place_in_container'
            else ()
        )
        positioned_from_memory = False
        if step.action == 'place_in_container':
            color = int(goal.container_color)
            positioned_from_memory = self._position_from_container_memory(
                color)
        while True:
            if (
                step.action == 'place_in_container'
                and not positioned_from_memory
                and self._current_observation_excludes_container(color)
            ):
                self.get_logger().info(
                    f"Contêiner do passo '{step.step_id}' ausente na última "
                    'observação da posição atual; evitando uma nova detecção '
                    'no mesmo local.'
                )
                self._mark_current_search_position(visited)
                if self._move_to_next_place_search_position(step, visited):
                    # FollowWall may be stopped before producing any physical
                    # displacement. Re-check the measured position before
                    # spending another camera session at the same viewpoint.
                    continue
                break
            positioned_from_memory = False
            result = self._call_manipulation_action(
                client,
                goal,
                f"passo '{step.step_id}' ({step.action})",
                timeout,
                'place',
                tag_id,
            )
            failure = self._manipulation_failure(result)
            if failure is None:
                return
            if not result.outcome.effect_known:
                raise StepFailed(
                    f"passo '{step.step_id}' ({step.action}) deixou o estado "
                    f'físico incerto: {failure}'
                )

            known, gripper, _slots = self._world_state.snapshot()
            if known and gripper == EMPTY:
                # The gripper was opened and the logical effect was confirmed;
                # A later return failure must not deposit a second time.
                self.get_logger().warning(
                    f"Passo '{step.step_id}' confirmou o depósito antes de "
                    f'falhar durante a finalização: {failure}. O fluxo da '
                    'missão continuará.'
                )
                return
            if not known or gripper != tag_id:
                raise StepFailed(
                    f"passo '{step.step_id}' ({step.action}) não pode ser "
                    f'repetido com segurança: {failure}'
                )

            self.get_logger().warning(
                f"Passo '{step.step_id}' falhou na posição lateral "
                f'{self._current_lateral_position_mm:.0f} mm: {failure}. '
                'Tentando outro ponto de observação.'
            )
            self._mark_current_search_position(visited)
            if self._move_to_next_place_search_position(step, visited):
                continue
            break

        fallback_goal = self._default_place_goal(height_cm)
        self.get_logger().warning(
            f"Nenhum destino utilizável para o passo '{step.step_id}' nas "
            'posições de busca; usando o fallback padrão de place_on_table.'
        )
        result = self._call_manipulation_action(
            self._place_table_client,
            fallback_goal,
            f"fallback do passo '{step.step_id}' (place_on_table)",
            timeout,
            'place',
            tag_id,
        )
        failure = self._manipulation_failure(result)
        if failure is None:
            return
        known, gripper, _slots = self._world_state.snapshot()
        if known and gripper == EMPTY:
            self.get_logger().warning(
                f"Fallback do passo '{step.step_id}' confirmou o depósito "
                f'antes de falhar durante a finalização: {failure}. O fluxo '
                'da missão continuará.'
            )
            return
        raise StepFailed(
            f"fallback do passo '{step.step_id}' (place_on_table) falhou: "
            f'{failure}'
        )

    def _execute_manipulation(self, step: Step) -> None:
        assert self._arena is not None
        area = self._arena.service_areas[self._current_location]
        timeout = self._manipulation_timeout()
        if step.action == 'pick':
            self._execute_pick(step, timeout)
            return
        try:
            if step.action == 'store':
                tag_id = self._world_state.require_gripper_object()
                slot_id = str(step.slot_id)
                self._world_state.validate_store(tag_id, slot_id)
                goal = StoreObject.Goal()
                goal.slot_id = slot_id
                client = self._store_client
                transition = 'store'
            elif step.action == 'retrieve':
                slot_id = str(step.slot_id)
                tag_id = self._world_state.require_slot_object(slot_id)
                self._world_state.validate_retrieve(tag_id, slot_id)
                goal = RetrieveObject.Goal()
                goal.slot_id = slot_id
                client = self._retrieve_client
                transition = 'retrieve'
            else:
                slot_id = ''
                tag_id = self._world_state.require_gripper_object()
                self._world_state.validate_place(tag_id)
                transition = 'place'
                if step.action == 'place_on_table':
                    goal = PlaceOnTable.Goal()
                    goal.ws_height_cm = float(area.height_cm)
                    client = self._place_table_client
                elif step.action == 'place_in_container':
                    goal = PlaceInContainer.Goal()
                    goal.ws_height_cm = float(area.height_cm)
                    goal.container_color = (
                        PlaceInContainer.Goal.RED
                        if step.container_color == 'red'
                        else PlaceInContainer.Goal.BLUE
                    )
                    client = self._place_container_client
                elif step.action == 'stack':
                    goal = StackObject.Goal()
                    goal.support_tag_id = int(step.support_tag_id)
                    goal.ws_height_cm = float(area.height_cm)
                    client = self._stack_client
                elif step.action == 'place_on_shelf':
                    goal = PlaceOnShelf.Goal()
                    client = self._place_shelf_client
                else:
                    raise ConfigurationError(
                        f'Operação não implementada: {step.action}.'
                    )
        except StateConflict as error:
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) bloqueado pelo estado: "
                f'{error}'
            ) from error

        if step.action in {'place_on_table', 'place_in_container'}:
            self._execute_place_with_recovery(
                step, client, goal, tag_id, float(area.height_cm), timeout
            )
            return

        result = self._call_manipulation_action(
            client,
            goal,
            f"passo '{step.step_id}' ({step.action})",
            timeout,
            transition,
            tag_id,
            slot_id,
        )
        failure = self._manipulation_failure(result)
        if failure is not None:
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) falhou: {failure}"
            )

    def _execute_step(self, step: Step) -> None:
        if step.action == 'navigate':
            assert step.target is not None
            self._navigate(step.target)
        elif step.action == 'finish':
            self._navigate('finish')
        else:
            self._execute_manipulation(step)

    def _load_goal_files(self, plan_id: str) -> tuple[Arena, Plan]:
        if not PLAN_ID_PATTERN.fullmatch(plan_id):
            raise ConfigurationError('plan_id possui formato inválido.')
        arena_path = Path(str(self.get_parameter('arena_file').value))
        plans_root = Path(
            str(self.get_parameter('plans_directory').value)
        ).resolve()
        plan_path = plans_root / f'{plan_id}.yaml'
        arena = load_arena(arena_path)
        plan = load_plan(plan_path)
        if plan.plan_id != plan_id:
            raise ConfigurationError(
                f"O arquivo solicitado como '{plan_id}' declara plan_id "
                f"'{plan.plan_id}'."
            )
        validate_plan(plan, arena)
        return arena, plan

    def _feedback(
        self,
        goal_handle: Any,
        index: int,
        total: int,
        step: Step,
        description: str,
    ) -> None:
        feedback = ExecuteMission.Feedback()
        feedback.current_step_index = index
        feedback.total_steps = total
        feedback.step_id = step.step_id
        feedback.operation = step.action
        feedback.description = description
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        goal_handle: Any,
        code: int,
        completed_steps: int,
        failed_step_id: str,
        message: str,
    ) -> ExecuteMission.Result:
        result = ExecuteMission.Result()
        result.code = code
        result.completed_steps = completed_steps
        result.failed_step_id = failed_step_id
        result.message = message
        if code == ExecuteMission.Result.SUCCESS:
            goal_handle.succeed()
        elif code == ExecuteMission.Result.CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_callback(self, goal_handle: Any) -> ExecuteMission.Result:
        self._status = 'running'
        self._current_location = 'start'
        self._current_wall_distance_mm = None
        self._current_lateral_position_mm = 0.0
        self._tag_observations.clear()
        self._container_observations.clear()
        self._container_search_positions.clear()
        self._visited_search_positions.clear()
        self._last_table_observation = None
        completed = 0
        failed_step = ''
        try:
            arena, plan = self._load_goal_files(str(goal_handle.request.plan_id))
            self._arena = arena
            self._current_location = plan.initial_location
            self._world_state.reset()
            self._publish_world_state()
            total = len(plan.steps)
            for index, step in enumerate(plan.steps):
                failed_step = step.step_id
                self._current_step_index = index
                self._active_world_operation = step.action
                self._publish_world_state()
                self._feedback(
                    goal_handle, index, total, step,
                    f'Executando {step.action}',
                )
                self._execute_step(step)
                completed += 1
            self._status = 'succeeded'
            return self._result(
                goal_handle,
                ExecuteMission.Result.SUCCESS,
                completed,
                '',
                f"Plano '{plan.plan_id}' concluído.",
            )
        except MissionCanceled as error:
            self._status = 'canceled'
            return self._result(
                goal_handle,
                ExecuteMission.Result.CANCELED,
                completed,
                failed_step,
                str(error),
            )
        except ConfigurationError as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.CONFIGURATION_ERROR,
                completed,
                failed_step,
                str(error),
            )
        except StepFailed as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.STEP_FAILED,
                completed,
                failed_step,
                str(error),
            )
        except Exception as error:
            self.get_logger().error(f'Falha interna na missão: {error}')
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.INTERNAL_ERROR,
                completed,
                failed_step,
                str(error),
            )
        finally:
            self._cancel_event.clear()
            self._arena = None
            self._active_world_operation = ''
            self._publish_world_state()
            with self._lock:
                self._busy = False
                self._active_child = None

    def destroy_node(self):
        self._cancel_event.set()
        self._cancel_active_child()
        self._server.destroy()
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    exit_code = 0
    try:
        node = MissionManager()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Falha fatal no mission manager: {error}')
        else:
            print(f'Falha fatal no mission manager: {error}')
        exit_code = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
