"""ROS 2 action server that executes validated visits with perception-ranked tasks."""

from __future__ import annotations

import copy
from concurrent.futures import ThreadPoolExecutor, wait
from contextlib import contextmanager
from dataclasses import replace
import math
import random
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from interfaces.action import (
    AnalyzeScene,
    ExecuteMission,
    FollowWall,
    PickObject,
    PlaceInContainer,
    PlaceOnShelf,
    PlaceOnTable,
    PlaceOnPrecisionTable,
    PrepareManipulator,
    RetrieveObject,
    StackObject,
    StoreObject,
)
from interfaces.msg import (
    CargoSlotState, ManipulationFeedback, ManipulationResult, ManipulationState,
    SceneObservation,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from .errors import ConfigurationError, MissionCanceled, StateConflict, StepFailed, TaskNotFound, PrecisionSlotOccupied
from .loaders import load_arena, load_plan, PLAN_ID_PATTERN, validate_plan
from .models import (
    Arena,
    ContainerObservation,
    DeliveryOutcome,
    PickupRecoveryConfig,
    Plan,
    ServiceArea,
    SlotMovement,
    Step,
    TableObservation,
    TagObservation,
)
from .world_state import EMPTY, WorldState
from .scheduler import Scheduler
from .precision_runtime import PrecisionRuntime


class MissionManager(PrecisionRuntime, Node):
    """Own one mission at a time and compose existing semantic action servers."""

    def __init__(self) -> None:
        super().__init__('mission_manager')
        if (
            not hasattr(PickObject.Result(), 'observed_detections')
            or not hasattr(PickObject.Result(), 'scene_observation')
            or not hasattr(PickObject.Goal(), 'alignment_completed')
            or not hasattr(PickObject.Goal(), 'classify_pp_tags')
            or not hasattr(AnalyzeScene.Goal(), 'classify_pp_tags')
            or not hasattr(PlaceOnPrecisionTable.Goal(), 'require_empty_slot')
            or not hasattr(PickObject.Goal(), 'use_observed_detection')
            or not hasattr(PickObject.Result(), 'used_observed_detection')
            or not hasattr(PickObject.Result, 'RECOVERY_ALIGNMENT_REQUIRED')
            or not hasattr(StackObject.Goal(), 'require_alignment')
            or not hasattr(StackObject.Result, 'RECOVERY_ALIGNMENT_REQUIRED')
            or not hasattr(PrepareManipulator.Goal(), 'gripper_loaded')
            or not hasattr(PlaceOnTable.Goal(), 'use_fallback_pose')
            or not hasattr(FollowWall.Goal(), 'alignment_error_ignore_duration')
        ):
            raise ConfigurationError(
                'As interfaces instaladas estão desatualizadas; '
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
            'follow_wall.alignment_error_ignore_sec': 2.0,
            'follow_wall.alignment_recovery_distance_mm': 100,
            'follow_wall.minimum_lateral_clearance_mm': 10,
            'deposit_lateral_retreat.threshold_mm': 100,
            'deposit_lateral_retreat.distance_mm': 50,
            'vision_action': '/vision/analyze_scene',
            'prepare_action': '/manipulation/prepare',
            'pick_action': '/manipulation/pick',
            'store_action': '/manipulation/store',
            'retrieve_action': '/manipulation/retrieve',
            'place_on_table_action': '/manipulation/place_on_table',
            'place_in_container_action': '/manipulation/place_in_container',
            'stack_action': '/manipulation/stack',
            'place_on_precision_table_action': '/manipulation/place_on_precision_table',
            'place_on_shelf_action': '/manipulation/place_on_shelf',
            'server_timeout_s': 10.0,
            'navigation_timeout_s': 120.0,
            'manipulation_timeout_s': 120.0,
            'camera_capture_service': '/camera/set_capture',
            'vision_led_hold_off_service': '/vision/hold_led_off',
            'vision_led_service': '/base_hardware/set_vision_led',
            'vision_resource_timeout_s': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._wall_max_alignment_error_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.max_alignment_error_mm'))
        self._wall_alignment_error_ignore_sec = (
            self._nonnegative_float_parameter(
                'follow_wall.alignment_error_ignore_sec'))
        self._wall_alignment_recovery_distance_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.alignment_recovery_distance_mm'))
        self._wall_minimum_lateral_clearance_mm = (
            self._nonnegative_integer_parameter(
                'follow_wall.minimum_lateral_clearance_mm'))
        self._deposit_lateral_retreat_threshold_mm = (
            self._nonnegative_integer_parameter(
                'deposit_lateral_retreat.threshold_mm'))
        self._deposit_lateral_retreat_distance_mm = (
            self._nonnegative_integer_parameter(
                'deposit_lateral_retreat.distance_mm'))
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
        self._stack_alignment = None
        self._current_wall_distance_mm: float | None = None
        self._current_lateral_position_mm = 0.0
        self._last_follow_wall_result: FollowWall.Result | None = None
        self._last_lateral_travel_direction = 0
        self._tag_observations: dict[tuple[str, int], TagObservation] = {}
        self._pp_reference_observations = {}
        self._pp_reference_views = {}
        self._last_pp_scene = None
        self._placed_tag_viewpoints: dict[tuple[str, int], tuple[int, float]] = {}
        self._container_observations: dict[
            tuple[str, int], ContainerObservation
        ] = {}
        self._container_search_positions: dict[str, set[int]] = {}
        self._visited_search_positions: dict[str, set[int]] = {}
        self._blocked_search_positions: dict[str, set[int]] = {}
        self._last_wall_control_protection_stop = False
        self._last_table_observation: TableObservation | None = None
        self._scene_observations: dict[tuple[str, int, int, bool], TableObservation] = {}
        self._direct_pick_observation: TableObservation | None = None
        self._active_children = {}
        self._departure_completed_for = None
        self._arena: Arena | None = None
        self._service_area_vision_active = False

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

        self._camera_capture_client = self.create_client(
            SetBool, str(self.get_parameter('camera_capture_service').value),
            callback_group=self._callback_group)
        self._vision_led_hold_off_client = self.create_client(
            SetBool, str(self.get_parameter('vision_led_hold_off_service').value),
            callback_group=self._callback_group)
        self._vision_led_client = self.create_client(
            SetBool, str(self.get_parameter('vision_led_service').value),
            callback_group=self._callback_group)
        self._navigate_client = client(NavigateToPose, 'navigate_action')
        self._wall_control_client = client(FollowWall, 'wall_control_action')
        self._vision_client = client(AnalyzeScene, 'vision_action')
        self._prepare_client = client(PrepareManipulator, 'prepare_action')
        self._pick_client = client(PickObject, 'pick_action')
        self._store_client = client(StoreObject, 'store_action')
        self._retrieve_client = client(RetrieveObject, 'retrieve_action')
        self._place_table_client = client(PlaceOnTable, 'place_on_table_action')
        self._place_container_client = client(
            PlaceInContainer, 'place_in_container_action'
        )
        self._stack_client = client(StackObject, 'stack_action')
        self._place_precision_client = client(PlaceOnPrecisionTable, 'place_on_precision_table_action')
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
        try:
            self._accepted_mission_files = self._load_goal_files(plan_id)
        except (ConfigurationError, MissionCanceled) as error:
            self.get_logger().warning(f'Missão {plan_id} rejeitada: {error}')
            with self._lock:
                self._busy = False
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_event.set()
        self._cancel_active_child()
        return CancelResponse.ACCEPT

    def _cancel_active_child(self) -> None:
        with self._lock:
            children = tuple(self._active_children.values())
        for child in children:
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

    def _nonnegative_float_parameter(self, name: str) -> float:
        raw = self.get_parameter(name).value
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ConfigurationError(f'{name} deve ser nao negativo.')
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            raise ConfigurationError(f'{name} deve ser nao negativo.')
        return value

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
        feedback_callback: Callable[[Any], None] | None = None,
    ) -> Any:
        self._check_canceled()
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ConfigurationError(f'Timeout inválido para {description}.')
        if not client.wait_for_server(timeout_sec=self._server_timeout()):
            raise StepFailed(f'Servidor indisponível: {description}.')
        child = None
        send_future = None
        result_received = False
        try:
            self._check_canceled()
            send_future = (
                client.send_goal_async(goal, feedback_callback=feedback_callback)
                if feedback_callback is not None else client.send_goal_async(goal)
            )
            child = self._wait_future(
                send_future, self._server_timeout(), check_cancel=False
            )
            if child is None or not child.accepted:
                raise StepFailed(f'Goal rejeitado: {description}.')
            with self._lock:
                self._active_children[id(child)] = child
            if on_goal_accepted is not None:
                on_goal_accepted()
            self._check_canceled()
            result_wrapper = self._wait_future(
                child.get_result_async(), timeout_s
            )
            result_received = True
            self._check_canceled()
        except MissionCanceled:
            raise
        except StepFailed:
            raise
        except TimeoutError as error:
            raise StepFailed(f'Timeout durante {description}.') from error
        except Exception as error:
            raise StepFailed(f'Falha de comunicação em {description}: {error}') from error
        finally:
            # Cancel even if acceptance arrives after the communication timeout.
            if child is None and send_future is not None:
                def cancel_late_goal(future):
                    if future.exception() is None:
                        late_child = future.result()
                        if late_child is not None and late_child.accepted:
                            late_child.cancel_goal_async()
                send_future.add_done_callback(cancel_late_goal)
            if child is not None and child.accepted:
                if not result_received:
                    child.cancel_goal_async()
                with self._lock:
                    self._active_children.pop(id(child), None)
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
            unchanged_locations = {
                'pick': ManipulationResult.LOCATION_SOURCE,
                'store': ManipulationResult.LOCATION_GRIPPER,
                'retrieve': ManipulationResult.LOCATION_CARGO,
                'place': ManipulationResult.LOCATION_GRIPPER,
            }
            if location == unchanged_locations[operation]:
                known, held, cargo = self._world_state.snapshot()
                unchanged = (held == tag_id if operation in {'store', 'place'} else
                             cargo.get(slot_id) == tag_id if operation == 'retrieve' else
                             held == EMPTY)
                if known and unchanged:
                    return
                self._mark_world_unknown()
                return
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
        *,
        feedback_callback: Callable[[Any], None] | None = None,
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
                **({'feedback_callback': feedback_callback}
                   if feedback_callback is not None else {}),
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
            self._last_wall_control_protection_stop = True
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

    def _pickup_config(self) -> PickupRecoveryConfig:
        assert self._arena is not None
        config = self._arena.pickup_recovery
        area = self._arena.service_areas.get(getattr(self, '_current_location', ''))
        if area is not None and area.area_type == 'SH':
            return replace(
                config, preferred_tag_x_m=config.shelf_preferred_tag_x_m,
                preferred_tag_y_m=config.shelf_preferred_tag_y_m)
        return config

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
        accept_safety_abort: bool = True,
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
        arena = getattr(self, '_arena', None)
        area = (
            arena.service_areas.get(getattr(self, '_current_location', ''))
            if arena is not None else None)
        ignore_alignment_sec = (
            area.alignment_error_ignore_sec
            if area is not None and area.alignment_error_ignore_sec is not None
            else self._wall_alignment_error_ignore_sec)
        if goal.max_alignment_error_mm <= 0:
            ignore_alignment_sec = 0.0
        goal.alignment_error_ignore_duration = self._duration(
            ignore_alignment_sec)
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
            f'{goal.minimum_lateral_clearance_mm} mm, ignorar desalinhamento='
            f'{ignore_alignment_sec:.1f} s, '
            f'timeout={timeout_s:.1f} s.'
        )
        self._last_wall_control_protection_stop = False
        result = self._call_action(
            self._wall_control_client,
            goal,
            description,
            timeout_s + 5.0,
            self._wall_control_failure,
            accept_unsuccessful_result=(
                self._accept_wall_control_abort if accept_safety_abort else None),
        )
        self._last_follow_wall_result = result
        if (
            goal.travel_distance_mm != 0
            and abs(float(result.traveled_distance_mm)) > 1.0
        ):
            self._last_lateral_travel_direction = (
                1 if result.traveled_distance_mm > 0 else -1)
        self.get_logger().info(
            f'FollowWall finalizada ({description}): parede final='
            f'{result.final_average_distance_mm:.1f} mm, deslocamento lateral '
            f'efetivo={result.traveled_distance_mm:.1f} mm; {result.message}'
        )
        return result

    def _retreat_from_lateral_wall_before_slot_access(
        self, slot_side: str, operation: str
    ) -> None:
        """Usa a folga do último alinhamento antes de acessar o slot."""
        direction = 1 if slot_side == 'right' else -1
        if operation == 'store':
            if direction != getattr(self, '_last_lateral_travel_direction', 0):
                return
        description = (
            'armazenamento' if operation == 'store' else 'retirada do slot')
        result = self._last_follow_wall_result
        threshold = self._deposit_lateral_retreat_threshold_mm
        distance = self._deposit_lateral_retreat_distance_mm
        if result is None or threshold == 0 or distance == 0:
            return
        if not result.has_fresh_lateral_scan:
            raise StepFailed(
                f'LiDAR lateral indisponivel apos alinhamento; {description} '
                'bloqueado porque nao foi possivel verificar a folga.')
        if direction > 0:
            valid = result.has_valid_right_lateral_clearance
            clearance = result.final_right_lateral_clearance_mm
            side = 'direito'
        else:
            valid = result.has_valid_left_lateral_clearance
            clearance = result.final_left_lateral_clearance_mm
            side = 'esquerdo'
        if not valid or clearance >= threshold:
            return
        if self._current_wall_distance_mm is None:
            raise StepFailed(
                'Distancia frontal desconhecida para recuo lateral antes '
                f'da operacao de {description}.')
        assert self._arena is not None
        config = self._arena.pickup_recovery
        requested_target = (
            self._current_lateral_position_mm - direction * distance)
        target = self._clamp_lateral_position(requested_target)
        if target != requested_target:
            raise StepFailed(
                f'Folga no lado {side} de {clearance:.1f} mm, mas o '
                'limite de posicao lateral impede o recuo completo antes '
                f'da operacao de {description}.')
        travel = round(target - self._current_lateral_position_mm)
        self.get_logger().warning(
            f'Folga no lado {side} de {clearance:.1f} mm abaixo de '
            f'{threshold} mm; recuando {abs(travel)} mm antes de {description}.')
        retreat_result = self._control_wall(
            round(self._current_wall_distance_mm),
            config.wall_tolerance_mm,
            config.timeout_s,
            f'recuo lateral antes de {description} ({side})',
            travel_distance_mm=travel,
            travel_tolerance_mm=config.travel_tolerance_mm,
            accept_safety_abort=False,
        )
        self._update_table_position(retreat_result)
        self._last_lateral_travel_direction = 0

    def _navigation_timeout(self) -> float:
        return float(self.get_parameter('navigation_timeout_s').value)

    def _manipulation_timeout(self) -> float:
        return float(self.get_parameter('manipulation_timeout_s').value)

    def _run_with_manipulator_prepare(
        self, prepare_arm: Callable[[], None], movement: Callable[[], Any]
    ) -> Any:
        """Overlap the selected arm preparation and wall control; wait for both."""
        known, _, _ = self._world_state.snapshot()
        if not known:
            raise StepFailed('Estado da carga incerto; navegação automática bloqueada.')
        with ThreadPoolExecutor(max_workers=2) as executor:
            prepare = executor.submit(prepare_arm)
            move = executor.submit(movement)
            wait((prepare, move))
            # An independent failure must not interrupt the other action.
            failures = [future.exception() for future in (prepare, move)
                        if future.exception() is not None]
            if failures:
                raise next((error for error in failures
                            if isinstance(error, MissionCanceled)), failures[0])
            return move.result()

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

    def _async_motion_config(self, area_id=None):
        arena = self._arena
        area = arena.service_areas.get(area_id or getattr(self, '_current_location', None))
        return (area.async_motion if area is not None and area.async_motion is not None
                else arena.async_motion_defaults)

    def _run_area_motion(self, prepare, movement, *, boundary=False, area_id=None):
        config = self._async_motion_config(area_id)
        enabled = config.approach_departure_enabled if boundary else config.table_mode == 'always'
        if enabled:
            return self._run_with_manipulator_prepare(prepare, movement)
        if boundary and not self._world_state.snapshot()[0]:
            raise StepFailed('Estado da carga incerto; movimento automático bloqueado.')
        prepare()
        return movement()

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
        self._direct_pick_observation = None
        self._stack_alignment = None
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
            finite_or_infinity(detection.mask_area_px),
        )

    def _remember_scene_observations(self, result: Any) -> None:
        """Merge one action's camera snapshot into mission-owned memory."""
        if (getattr(result, 'used_observed_detection', False)
                and not getattr(getattr(result, 'scene_observation', None), 'completed', False)):
            return
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
                int(scene.requested_detectors) & (
                    SceneObservation.CONTAINERS_HSV)
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
        references = {}
        if self._is_precision_area():
            from .precision_perception import split_detections
            references, objects = split_detections(detections, arena.precision_perception)
            if apriltag_observation_completed:
                self._last_pp_scene = (self._current_location, self._current_wall_distance_mm,
                                       self._current_lateral_position_mm, references, objects)
                self._pp_record_reference_view(references)
                self._pp_update_verified(references, objects)
            detections = list(objects.values())
        config = self._pickup_config()
        if abs(self._current_wall_distance_mm - config.safety_search_distance_mm) <= config.wall_tolerance_mm:
            for detector, completed in (('apriltags', apriltag_observation_completed), ('containers', container_observation_completed)):
                if completed:
                    for position in config.safety_search_positions_mm:
                        if abs(position - self._current_lateral_position_mm) <= config.travel_tolerance_mm:
                            self._safety_visited(detector).add(position)
        if apriltag_observation_completed or container_observation_completed:
            previous = self._scene_at_current_position()
            same_viewpoint = previous is not None and self._at_observation_point(previous)
            previous_tags = previous.detected_tag_ids if same_viewpoint else frozenset()
            previous_colors = previous.detected_container_colors if same_viewpoint else frozenset()
            self._last_table_observation = TableObservation(
                area_id=self._current_location,
                wall_distance_mm=self._current_wall_distance_mm,
                lateral_position_mm=self._current_lateral_position_mm,
                detected_tag_ids=(frozenset(int(detection.id) for detection in detections)
                                  if apriltag_observation_completed else previous_tags),
                apriltags_observed=(apriltag_observation_completed or
                                   bool(same_viewpoint and previous.apriltags_observed)),
                detected_container_colors=(frozenset(int(detection.color) for detection in containers)
                                           if container_observation_completed else previous_colors),
                containers_observed=(container_observation_completed or
                                     bool(same_viewpoint and previous.containers_observed)),
            )
            history = getattr(self, '_scene_observations', None)
            if history is None:
                history = self._scene_observations = {}
            key = (self._current_location, round(self._current_wall_distance_mm),
                   round(self._current_lateral_position_mm), bool(getattr(self, '_search_led_off', False)))
            history[key] = self._last_table_observation
            visited = self._visited_search_positions.setdefault(
                self._current_location, set()
            )
            area = arena.service_areas[self._current_location]
            if (
                apriltag_observation_completed and not getattr(self, '_search_phase', 0) and abs(
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
                not getattr(self, '_search_phase', 0) and abs(
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

        if not hasattr(self, '_pp_reference_observations'):
            self._pp_reference_observations = {}
        for detection in list(references.values()) + detections:
            color = {1: 'red', 2: 'blue'}.get(int(getattr(detection, 'color', 0)))
            if color is not None:
                self._world_state.remember_tag_color(int(detection.id), color)
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
            from .precision_perception import is_reference
            memory = (self._pp_reference_observations if self._is_precision_area()
                      and is_reference(detection, arena.precision_perception) else self._tag_observations)
            memory[(self._current_location, int(detection.id))] = observation

        if sum(area == self._current_location for area, _tag in self._pp_reference_observations) > 7:
            raise StepFailed('PP: mais de sete referências detectadas; confira a faixa Z e as tags da mesa.')

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
        history = getattr(self, '_scene_observations', {})
        for key, snapshot in tuple(history.items()):
            history[key] = replace(snapshot, detected_tag_ids=snapshot.detected_tag_ids - {tag_id})
        observation = getattr(self, '_last_table_observation', None)
        if observation is not None:
            self._last_table_observation = replace(
                observation, detected_tag_ids=observation.detected_tag_ids - {tag_id})
        for key in [key for key in self._tag_observations if key[1] == tag_id]:
            del self._tag_observations[key]
        for key in [key for key in getattr(self, '_placed_tag_viewpoints', {}) if key[1] == tag_id]:
            del self._placed_tag_viewpoints[key]

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
        self._direct_pick_observation = None
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

        self._stack_alignment = None
        self.get_logger().info(
            f'Reposicionamento de mesa ({description}): posição atual '
            f'parede={self._current_wall_distance_mm:.1f} mm, lateral='
            f'{self._current_lateral_position_mm:.1f} mm; destino parede='
            f'{wall} mm, lateral={bounded_lateral_position_mm:.1f} mm; '
            f'percurso lateral solicitado={travel} mm.'
        )
        self._last_wall_control_protection_stop = False
        follow_result = self._run_area_motion(self._prepare_for_pick_observation, lambda: self._control_wall(
            wall,
            config.wall_tolerance_mm,
            config.timeout_s,
            description,
            travel_distance_mm=travel,
            travel_tolerance_mm=config.travel_tolerance_mm,
        ))
        self._update_table_position(follow_result)
        self._remember_blocked_search_destination(bounded_lateral_position_mm, travel)
        self.get_logger().info(
            f'Estado da mesa atualizado ({description}): parede='
            f'{self._current_wall_distance_mm:.1f} mm, lateral='
            f'{self._current_lateral_position_mm:.1f} mm '
            f'(avanço lateral medido={follow_result.traveled_distance_mm:.1f} '
            'mm).'
        )
        return True

    def _remember_blocked_search_destination(self, lateral: float, travel: int) -> None:
        """Do not retry a search point that a protected partial movement missed."""
        config = self._arena.pickup_recovery
        destination_unreached = (
            travel > 0
            and self._current_lateral_position_mm
            < lateral - config.travel_tolerance_mm
        ) or (
            travel < 0
            and self._current_lateral_position_mm
            > lateral + config.travel_tolerance_mm
        )
        if (
            travel != 0
            and getattr(self, '_last_wall_control_protection_stop', False)
            and destination_unreached
            and lateral in config.search_positions_mm
        ):
            blocked_by_area = getattr(self, '_blocked_search_positions', None)
            if blocked_by_area is None:
                blocked_by_area = {}
                self._blocked_search_positions = blocked_by_area
            blocked_by_area.setdefault(self._current_location, set()).add(
                round(lateral))
            self.get_logger().warning(
                f'Destino lateral {lateral:.0f} mm '
                f'bloqueado em {self._current_location} apos protecao; '
                f'posicao medida={self._current_lateral_position_mm:.1f} mm.')

    def _at_observation_point(self, observation) -> bool:
        config = self._arena.pickup_recovery
        return (observation.area_id == self._current_location
                and self._current_wall_distance_mm is not None
                and abs(observation.wall_distance_mm - self._current_wall_distance_mm)
                    <= config.wall_tolerance_mm
                and abs(observation.lateral_position_mm - self._current_lateral_position_mm)
                    <= config.travel_tolerance_mm)

    def _scene_at_current_position(self) -> TableObservation | None:
        observation = getattr(self, '_last_table_observation', None)
        if observation is not None and self._at_observation_point(observation):
            return observation
        dark = bool(getattr(self, '_search_led_off', False))
        for key, observation in reversed(tuple(getattr(self, '_scene_observations', {}).items())):
            if key[3] == dark and self._at_observation_point(observation):
                return observation
        return None

    def _current_scene_observed(self) -> bool:
        observation = self._scene_at_current_position()
        return bool(observation is not None and observation.apriltags_observed)

    def _take_direct_pick_detection(self, tag_id: int):
        """Only the next action after explicit scene analysis may reuse its pose.

        Search/position history helps choose where to go; it never authorizes a
        direct grasp. Consuming this permission before the action also prevents
        a failed attempt or a return to the same coordinates from renewing it.
        """
        snapshot = getattr(self, '_direct_pick_observation', None)
        self._direct_pick_observation = None
        observation = self._tag_observations.get((self._current_location, tag_id))
        if (snapshot is not None and snapshot.apriltags_observed
                and tag_id in snapshot.detected_tag_ids
                and self._at_observation_point(snapshot)
                and observation is not None and self._at_observation_point(observation)):
            return copy.deepcopy(observation.detection)
        return None

    def _precision_alignment_config(self) -> PickupRecoveryConfig:
        config = self._arena.pickup_recovery
        return replace(
            config, preferred_tag_x_m=config.precision_preferred_tag_x_m,
            preferred_tag_y_m=config.precision_preferred_tag_y_m,
        )

    def _precision_memory_destination(self, observation: TagObservation) -> tuple[int, float]:
        # Translate from the measured observation point, using PP preferences.
        # The cached pickup destination uses different preferences.
        pose = observation.detection.pose.position
        wall, travel = self._pickup_recovery_correction(
            observation.wall_distance_mm, float(pose.x), float(pose.y),
            self._precision_alignment_config(),
        )
        return wall, self._clamp_lateral_position(observation.lateral_position_mm + travel)

    def _position_from_memory(self, tag_id: int) -> TagObservation | None:
        observation = self._tag_observations.get((self._current_location, tag_id))
        if observation is None:
            return None
        wall = observation.pickup_wall_distance_mm
        lateral = observation.pickup_lateral_position_mm
        self.get_logger().info(
            f'AprilTag {tag_id} já observada em {self._current_location}; '
            f'indo para alinhamento parede={wall} mm, lateral={lateral:.0f} mm.')
        self._move_to_table_position(wall, lateral,
                                     f'alinhamento configurado da AprilTag {tag_id} memorizada')
        return observation

    def _position_from_placed_tag_memory(self, tag_id: int) -> bool:
        viewpoint = getattr(self, '_placed_tag_viewpoints', {}).get(
            (self._current_location, tag_id))
        if viewpoint is None:
            return False
        wall, lateral = viewpoint
        self.get_logger().info(
            f'AprilTag {tag_id} empilhada em {self._current_location}; '
            f'retornando ao ponto de observação parede={wall} mm, '
            f'lateral={lateral:.0f} mm.'
        )
        self._move_to_table_position(
            wall, lateral, f'retorno ao empilhamento da AprilTag {tag_id}')
        return True

    def _remember_placed_tag_viewpoint(self, tag_id: int) -> None:
        if self._current_wall_distance_mm is None:
            return
        memory = getattr(self, '_placed_tag_viewpoints', None)
        if memory is None:
            memory = {}
            self._placed_tag_viewpoints = memory
        memory[(self._current_location, tag_id)] = (
            round(self._current_wall_distance_mm),
            self._current_lateral_position_mm,
        )
        # A observação feita antes da soltura não localiza o cubo na mesa.
        self._tag_observations.pop((self._current_location, tag_id), None)

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

    @contextmanager
    def _search_session(self):
        self._search_phase = 0
        self._safety_search_attempts = {}
        try:
            yield
        finally:
            try:
                self._restore_search_led()
            finally:
                self._search_phase = 0

    def _restore_search_led(self) -> None:
        if not getattr(self, '_search_led_off', False):
            return
        # Restore the physical LED before releasing vision's hold.
        self._set_vision_resource(self._vision_led_client, 'LED', True)
        self._set_vision_resource(
            self._vision_led_hold_off_client, 'bloqueio do LED', False)
        self._search_led_off = False
        self._last_table_observation = None

    def _safety_visited(self, detector: str) -> set[int]:
        if not hasattr(self, '_safety_search_history'):
            self._safety_search_history = {}
        key = (self._current_location, self._arena.pickup_recovery.safety_search_distance_mm,
               getattr(self, '_search_phase', 0) == 2, detector)
        return self._safety_search_history.setdefault(key, set())

    def _move_to_next_safety_position(self, detector: str) -> bool:
        config = self._arena.pickup_recovery
        if not config.safety_search_positions_mm:
            return False
        phase = max(1, getattr(self, '_search_phase', 0))
        while phase <= 2:
            if phase != getattr(self, '_search_phase', 0):
                self._search_phase = phase
                self._last_table_observation = None
                if phase == 2:
                    self._search_led_off = True
                    self._set_vision_resource(self._vision_led_hold_off_client, 'bloqueio do LED', True)
                    self._set_vision_resource(self._vision_led_client, 'LED', False)
                self.get_logger().info(f'Busca de segurança: parede={config.safety_search_distance_mm} mm, LED={"apagado" if phase == 2 else "ligado"}.')
            visited = self._safety_visited(detector)
            if not hasattr(self, '_safety_search_attempts'):
                self._safety_search_attempts = {}
            attempted = self._safety_search_attempts.setdefault((phase, detector), set())
            candidates = [p for p in config.safety_search_positions_mm if p not in visited and p not in attempted]
            if candidates:
                destination = min(candidates, key=lambda p: (abs(p - self._current_lateral_position_mm), config.safety_search_positions_mm.index(p)))
                self._move_to_table_position(config.safety_search_distance_mm, destination,
                                             f'busca de segurança de {detector}')
                attempted.add(destination)
                self._last_table_observation = None
                return True
            phase += 1
        self._restore_search_led()
        self._search_phase = 0
        return False

    def _move_to_next_search_position(self, tag_id: int) -> bool:
        assert self._arena is not None
        if getattr(self, '_search_phase', 0):
            return self._move_to_next_safety_position('apriltags')
        config = self._arena.pickup_recovery
        visited = self._visited_search_positions.setdefault(
            self._current_location, set()
        )
        blocked = getattr(self, '_blocked_search_positions', {}).get(
            self._current_location, set())
        candidates = [
            position for position in config.search_positions_mm
            if position not in visited and position not in blocked
        ]
        if not candidates:
            return self._move_to_next_safety_position('apriltags')
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

    def _mark_current_search_position(
        self, visited: set[int], positions: tuple[int, ...]
    ) -> None:
        """Mark the configured table-search point at the current pose."""
        if getattr(self, '_search_phase', 0):
            return
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
        for position in positions:
            if (
                abs(position - self._current_lateral_position_mm)
                <= config.travel_tolerance_mm
            ):
                visited.add(position)

    def _current_search_position_visited(
        self, positions_by_area: dict[str, set[int]] | None = None
    ) -> bool:
        """Whether the current WS point was analyzed for this detector."""
        config = self._arena.pickup_recovery
        if (getattr(self, '_search_phase', 0)
                or (config.safety_search_positions_mm
                    and self._current_wall_distance_mm is not None
                    and abs(self._current_wall_distance_mm - config.safety_search_distance_mm) <= config.wall_tolerance_mm
                    and abs(self._current_wall_distance_mm - self._arena.service_areas[self._current_location].alignment.distance_mm) > config.wall_tolerance_mm)):
            detector = 'apriltags' if positions_by_area is None else 'containers'
            return (self._current_wall_distance_mm is not None
                    and abs(self._current_wall_distance_mm - config.safety_search_distance_mm) <= config.wall_tolerance_mm
                    and any(abs(p - self._current_lateral_position_mm) <= config.travel_tolerance_mm
                            for p in self._safety_visited(detector)))
        if self._current_wall_distance_mm is None:
            return False
        config = self._arena.pickup_recovery
        area = self._arena.service_areas[self._current_location]
        if (abs(area.alignment.distance_mm - self._current_wall_distance_mm)
                > config.wall_tolerance_mm):
            return False
        if positions_by_area is None:
            positions_by_area = getattr(self, '_visited_search_positions', {})
        visited = positions_by_area.get(
            self._current_location, set())
        return any(
            position in visited
            and abs(position - self._current_lateral_position_mm)
            <= config.travel_tolerance_mm
            for position in config.search_positions_mm
        )

    def _move_to_next_place_search_position(
        self, step: Step, visited: set[int], positions: tuple[int, ...]
    ) -> bool:
        """Move to the nearest untried table-search point for this step."""
        assert self._arena is not None
        detector = 'containers' if step.action == 'place_in_container' else 'table'
        if getattr(self, '_search_phase', 0):
            return self._move_to_next_safety_position(detector)
        blocked = (
            getattr(self, '_blocked_search_positions', {}).get(
                self._current_location, set())
            if step.action == 'place_in_container' else set()
        )
        candidates = [
            position for position in positions
            if position not in visited and position not in blocked
        ]
        if not candidates:
            return self._move_to_next_safety_position(detector)
        destination = min(
            candidates,
            key=lambda position: (
                abs(position - self._current_lateral_position_mm),
                positions.index(position),
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

    def _set_vision_resource(self, client, name: str, enabled: bool) -> None:
        timeout = float(self.get_parameter('vision_resource_timeout_s').value)
        if not client.wait_for_service(timeout_sec=timeout):
            raise StepFailed(f'Serviço de {name} indisponível.')
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=timeout):
            raise StepFailed(f'Tempo limite ao controlar {name}.')
        try:
            response = future.result()
        except Exception as error:
            raise StepFailed(f'Falha ao controlar {name}: {error}') from error
        if response is None or not response.success:
            message = response.message if response is not None else 'sem resposta'
            raise StepFailed(f'Falha ao controlar {name}: {message}')

    def _activate_service_area_vision(self) -> None:
        if self._service_area_vision_active:
            return
        # Mark active before the calls so a timeout still gets cleanup.
        self._service_area_vision_active = True
        try:
            self._set_vision_resource(self._camera_capture_client, 'câmera', True)
            self._set_vision_resource(self._vision_led_client, 'LED', True)
        except Exception:
            try:
                self._deactivate_service_area_vision()
            except StepFailed as error:
                self.get_logger().error(str(error))
            raise

    def _deactivate_service_area_vision(self) -> None:
        if not self._service_area_vision_active:
            return
        failures = []
        for client, name in (
            (self._camera_capture_client, 'câmera'),
            (self._vision_led_client, 'LED'),
        ):
            try:
                self._set_vision_resource(client, name, False)
            except StepFailed as error:
                failures.append(str(error))
        if failures:
            raise StepFailed('; '.join(failures))
        self._service_area_vision_active = False

    def _depart_service_area(self, *, slot_overlap: bool = False) -> None:
        self._deactivate_service_area_vision()
        departure = self._arena.service_areas[self._current_location].departure
        result = self._control_wall(
            departure.distance_mm, departure.tolerance_mm, departure.timeout_s,
            f'recuo para sair de {self._current_location}',
            travel_distance_mm=round(
                departure.lateral_position_mm - self._current_lateral_position_mm),
            max_alignment_error_mm=departure.max_alignment_error_mm,
            alignment_recovery_distance_mm=(
                0 if slot_overlap else departure.alignment_recovery_distance_mm),
            minimum_lateral_clearance_mm=departure.minimum_lateral_clearance_mm,
            accept_safety_abort=True,
        )
        if slot_overlap:
            self._update_table_position(result)
            reached = (
                abs(self._current_wall_distance_mm - departure.distance_mm)
                <= departure.tolerance_mm
                and abs(self._current_lateral_position_mm - departure.lateral_position_mm)
                <= departure.tolerance_mm
            )
            self._departure_completed_for = self._current_location if reached else None

    def _navigate(self, target: str) -> None:
        self._direct_pick_observation = None
        self._stack_alignment = None
        assert self._arena is not None
        pose = self._arena.pose_for(target)
        if (
            self._current_location in self._arena.service_areas
            and target != self._current_location
        ):
            if getattr(self, '_departure_completed_for', None) == self._current_location:
                self._prepare_for_navigation()
            else:
                self._run_area_motion(
                    self._prepare_for_navigation, self._depart_service_area, boundary=True)
            self._departure_completed_for = None
            self._current_wall_distance_mm = None
            self._current_lateral_position_mm = 0.0
        else:
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
        self._last_follow_wall_result = None
        self._last_lateral_travel_direction = 0
        if target in self._arena.service_areas:
            self._activate_service_area_vision()
            alignment = self._arena.service_areas[target].alignment
            result = self._run_area_motion(
                self._prepare_for_pick_observation,
                lambda: self._control_wall(
                    alignment.distance_mm,
                    alignment.tolerance_mm,
                    alignment.timeout_s,
                    f'alinhamento em {target}',
                ), boundary=True, area_id=target,
            )
            self._current_wall_distance_mm = float(
                result.final_average_distance_mm
            )
            self._current_lateral_position_mm = 0.0
        self._current_location = target

    def _recover_pick(
        self, result: Any, step: Step, *, alignment_only: bool = False,
        config: PickupRecoveryConfig | None = None,
    ) -> None:
        assert self._arena is not None
        config = config or self._pickup_config()
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
            if alignment_only:
                return
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) continua fora do alcance, "
                'mas a correção calculada não produziria movimento dentro das '
                'tolerâncias e dos limites configurados.'
            )
        self.get_logger().warning(
            f'Alinhamento para {step.action} da AprilTag '
            f'{step.reference_tag_id if step.action == "place_on_precision_table" else step.support_tag_id if step.action == "stack" else step.tag_id} em '
            f'x={pose.x:.3f}, y={pose.y:.3f} m. Reposicionando a base para '
            f'{target_wall} mm da parede e deslocando {bounded_travel} mm '
            f'(positivo=direita, negativo=esquerda).{lateral_limit_message}'
        )
        moved = self._move_to_table_position(
            target_wall,
            target_lateral_position,
            f"reposicionamento para repetir o passo '{step.step_id}'",
        )
        if not moved and not alignment_only:
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) não produziu um novo "
                'reposicionamento.'
            )

    def _confirm_pick_color(self, step: Step, detection):
        """Resolve a required color while the cube is still visible on its source."""
        tag = int(step.tag_id)
        if tag not in getattr(self, '_required_color_tags', set()):
            return detection
        if tag in self._world_state.tag_colors():
            return detection
        config = self._pickup_config()
        max_attempts = max(1, config.max_reposition_attempts)
        for attempt in range(max_attempts + 1):
            self._check_canceled()
            if detection is None:
                self._observe_visit()
                detection = self._take_direct_pick_detection(tag)
            if detection is None:
                raise TaskNotFound(f'AprilTag {tag} não visível para confirmar a cor.')
            color = {1: 'red', 2: 'blue'}.get(int(getattr(detection, 'color', 0)))
            if color is not None:
                self._world_state.remember_tag_color(tag, color)
                return detection
            pose = detection.pose.position
            aligned = (abs(pose.x - config.preferred_tag_x_m) <= config.travel_tolerance_mm / 1000.0
                       and abs(pose.y - config.preferred_tag_y_m) <= config.wall_tolerance_mm / 1000.0)
            if aligned:
                color = random.choice(('red', 'blue'))
                self._world_state.remember_tag_color(tag, color)
                self.get_logger().warning(
                    f'AprilTag {tag} alinhada com cor UNKNOWN; cor sorteada: {color}.')
                return detection
            if attempt == max_attempts:
                break
            self._recover_pick(SimpleNamespace(detected_pose=SimpleNamespace(pose=detection.pose)),
                               step, alignment_only=True)
            self._observe_visit()
            detection = self._take_direct_pick_detection(tag)
        raise StepFailed(f'AprilTag {tag}: não foi possível alinhar para confirmar a cor.')

    def _execute_pick(self, step: Step, timeout: float) -> None:
        if getattr(self, '_flexible_pick', False):
            self._execute_pick_impl(step, timeout)
        else:
            with self._search_session():
                self._execute_pick_impl(step, timeout)

    def _execute_pick_impl(self, step: Step, timeout: float) -> None:
        assert self._arena is not None
        try:
            self._world_state.validate_pick(int(step.tag_id))
        except StateConflict as error:
            raise StepFailed(
                f"passo '{step.step_id}' (pick) bloqueado pelo estado: {error}"
            ) from error
        config = self._pickup_config()
        area = self._arena.service_areas[self._current_location]
        shelf_pick = area.area_type == 'SH'
        alignment_completed = False
        direct_detection = self._take_direct_pick_detection(int(step.tag_id))
        if getattr(self, '_pp_direct_pick_tag', None) == int(step.tag_id) and direct_detection is None:
            self._observe_visit()
            self._pp_find_object(int(step.tag_id))
            direct_detection = self._take_direct_pick_detection(int(step.tag_id))
        self._pp_skip_confirmed_pick(int(step.tag_id))
        original_observation = None
        pp_search_attempts = set()
        if config.enabled and direct_detection is None:
            original_observation = self._position_from_memory(int(step.tag_id))
            if (
                original_observation is None
                and (
                    self._current_observation_excludes(int(step.tag_id))
                    or self._current_search_position_visited()
                )
            ):
                self.get_logger().info(
                    f'AprilTag {step.tag_id} não localizada nas observações '
                    'da posição atual; evitando uma nova detecção no mesmo local.'
                )
                searching = (self._pp_move_to_next_search_point(int(step.tag_id), reference=False,
                                                               attempted=pp_search_attempts)
                             if area.area_type == 'PP'
                             else self._move_to_next_search_position(int(step.tag_id)))
                if not searching:
                    raise StepFailed(
                        f"passo '{step.step_id}' (pick) falhou: AprilTag "
                        f'{step.tag_id} não apareceu nas observações e '
                        'todas as posições de busca já foram examinadas '
                        'ou bloqueadas por proteção.'
                    )
        reposition_count = 0
        while True:
            direct_detection = self._confirm_pick_color(step, direct_detection)
            self._pp_skip_confirmed_pick(int(step.tag_id))
            goal = PickObject.Goal()
            goal.tag_id = int(step.tag_id)
            goal.profile = 'shelf_front' if shelf_pick else ''
            self._configure_pp_goal(goal)
            if direct_detection is not None:
                goal.use_observed_detection = True
                goal.observed_detection = direct_detection
            goal.alignment_completed = alignment_completed
            # Keep compatibility metadata populated for existing action clients.
            goal.alignment_tag_x_m = config.preferred_tag_x_m
            goal.alignment_tag_y_m = config.preferred_tag_y_m
            goal.alignment_tolerance_x_m = config.travel_tolerance_mm / 1000.0
            goal.alignment_tolerance_y_m = config.wall_tolerance_mm / 1000.0
            goal.ws_height_cm = float(area.height_cm)
            result = self._call_manipulation_action(
                self._pick_client,
                goal,
                f"passo '{step.step_id}' (pick)",
                timeout,
                'pick',
                int(step.tag_id),
            )
            direct_detection = None
            failure = self._manipulation_failure(result)
            if failure is None:
                self._forget_picked_tag(int(step.tag_id))
                return
            if not result.outcome.effect_known:
                raise StepFailed(
                    f"passo '{step.step_id}' (pick) deixou o estado físico "
                    f'incerto: {failure}'
                )
            if (
                shelf_pick
                and result.recovery_reason == PickObject.Result.RECOVERY_ALIGNMENT_REQUIRED
            ):
                if not result.has_detected_pose:
                    raise StepFailed('Não há pose detectada para tentar alinhar a base na SH.')
                self._recover_pick(result, step, alignment_only=True)
                alignment_completed = True
                continue
            recoverable = (
                config.enabled
                and result.has_detected_pose
                and result.recovery_reason != PickObject.Result.RECOVERY_NONE
            )
            if recoverable:
                self._pp_skip_confirmed_pick(int(step.tag_id))
                if reposition_count >= config.max_reposition_attempts:
                    raise StepFailed(
                        f"passo '{step.step_id}' (pick) falhou: {failure}"
                    )
                self._recover_pick(result, step)
                alignment_completed = False
                reposition_count += 1
                if (area.area_type == 'PP' and
                        getattr(self, '_pp_skip_correct_pick_tag', None) == int(step.tag_id)):
                    # Inspect the full scene before a new physical pick. Otherwise
                    # manipulation learns the reference only inside its next pick,
                    # and the manager receives that image after the cube was taken.
                    self._observe_visit()
                    self._pp_skip_confirmed_pick(int(step.tag_id))
                    self._pp_find_object(int(step.tag_id))
                    self._pp_skip_confirmed_pick(int(step.tag_id))
                    direct_detection = self._take_direct_pick_detection(int(step.tag_id))
                continue
            if (getattr(self, '_flexible_pick', False)
                    and (area.area_type != 'PP' or not config.enabled)
                    and result.outcome.code == ManipulationResult.OBJECT_NOT_FOUND):
                self._tag_observations.pop((self._current_location, int(step.tag_id)), None)
                raise TaskNotFound(f"AprilTag {step.tag_id} não encontrada nesta observação.")
            if (
                config.enabled
                and result.outcome.code == ManipulationResult.OBJECT_NOT_FOUND
            ):
                searching = (self._pp_move_to_next_search_point(int(step.tag_id), reference=False,
                                                               attempted=pp_search_attempts)
                             if area.area_type == 'PP'
                             else self._move_to_next_search_position(int(step.tag_id)))
                if searching:
                    alignment_completed = False
                    continue
            raise StepFailed(
                f"passo '{step.step_id}' (pick) falhou: {failure}"
            )

    def _execute_place_with_recovery(self, step, client, goal, tag_id, height_cm, timeout) -> None:
        with self._search_session():
            self._execute_place_with_recovery_impl(step, client, goal, tag_id, height_cm, timeout)

    def _execute_place_with_recovery_impl(
        self,
        step: Step,
        client: ActionClient,
        goal: Any,
        tag_id: int,
        height_cm: float,
        timeout: float,
    ) -> None:
        """Retry perception-based placement across table search points."""
        # AprilTag observations do not analyze free table space. Table placement
        # must try every configured point for this step. Container placement may
        # reuse only positions where container detection actually ran.
        visited: set[int] = (
            set(getattr(self, '_container_search_positions', {}).get(
                self._current_location, set()))
            if step.action == 'place_in_container' else set()
        )
        positions = (
            self._arena.table_place_search_positions_mm
            if step.action == 'place_on_table'
            and self._arena.table_place_search_positions_mm is not None
            else self._arena.pickup_recovery.search_positions_mm
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
                and (
                    self._current_observation_excludes_container(color)
                    or self._current_search_position_visited(
                        getattr(self, '_container_search_positions', {}))
                )
            ):
                self.get_logger().info(
                    f"Contêiner do passo '{step.step_id}' não localizado "
                    'nas observações da posição atual; evitando uma nova '
                    'detecção no mesmo local.'
                )
                self._mark_current_search_position(visited, positions)
                if self._move_to_next_place_search_position(step, visited, positions):
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
            self._mark_current_search_position(visited, positions)
            if self._move_to_next_place_search_position(step, visited, positions):
                continue
            break

        self._last_delivery_action = 'place_on_table_fallback'
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

    def _stack_is_aligned(self, support_tag_id: int) -> bool:
        """Reuse alignment only for the next cube on this stack at the same base pose."""
        alignment = getattr(self, '_stack_alignment', None)
        return alignment is not None and (
            alignment[0] == self._current_location
            and support_tag_id in alignment[1]
            and alignment[2:] == (
                self._current_wall_distance_mm, self._current_lateral_position_mm,
            )
        )

    def _remember_stack_alignment(self, tag_id: int, support_tag_id: int) -> None:
        supports = {support_tag_id, tag_id}
        if self._stack_is_aligned(support_tag_id):
            supports.update(self._stack_alignment[1])
        self._stack_alignment = (
            self._current_location, frozenset(supports),
            self._current_wall_distance_mm, self._current_lateral_position_mm,
        )

    def _execute_stack_with_search(self, step, goal, tag_id, timeout) -> None:
        with self._search_session():
            self._execute_stack_with_search_impl(step, goal, tag_id, timeout)

    def _execute_stack_with_search_impl(
        self, step: Step, goal: StackObject.Goal, tag_id: int, timeout: float,
    ) -> None:
        self._execute_tag_placement_with_search(
            step, goal, tag_id, timeout, self._stack_client, StackObject,
            int(goal.support_tag_id), precision=False,
        )

    def _execute_precision_with_search(self, step, goal, tag_id, timeout):
        with self._search_session():
            self._execute_tag_placement_with_search(
                step, goal, tag_id, timeout, self._place_precision_client,
                PlaceOnPrecisionTable, int(goal.reference_tag_id), precision=True,
            )

    def _execute_tag_placement_with_search(
        self, step, goal, tag_id, timeout, client, action_type,
        reference_tag_id: int, *, precision: bool,
    ):
        """Use known tag locations, then search only unexamined viewpoints."""
        assert self._arena is not None
        config = replace(
            self._arena.pickup_recovery,
            preferred_tag_x_m=(self._arena.pickup_recovery.precision_preferred_tag_x_m
                               if precision else self._arena.pickup_recovery.stack_preferred_tag_x_m),
            preferred_tag_y_m=(self._arena.pickup_recovery.precision_preferred_tag_y_m
                               if precision else self._arena.pickup_recovery.stack_preferred_tag_y_m),
        )
        original_observation = None
        alignment_completed = (self._pp_destination_is_aligned(reference_tag_id) if precision
                               else self._stack_is_aligned(reference_tag_id))
        if config.enabled and not alignment_completed:
            placed_viewpoint = (False if precision else
                                self._position_from_placed_tag_memory(reference_tag_id))
            if not placed_viewpoint:
                if precision:
                    original_observation = getattr(self, '_pp_reference_observations', {}).get(
                        (self._current_location, reference_tag_id))
                    if original_observation is not None:
                        wall, lateral = self._precision_memory_destination(original_observation)
                        self._move_to_table_position(
                            wall, lateral,
                            f'alinhamento PP pela AprilTag {reference_tag_id} memorizada',
                        )
                        alignment_completed = (
                            self._current_wall_distance_mm is not None
                            and abs(self._current_wall_distance_mm - wall) <= config.wall_tolerance_mm
                            and abs(self._current_lateral_position_mm - lateral) <= config.travel_tolerance_mm
                        )
                else:
                    original_observation = self._position_from_memory(reference_tag_id)
            if (
                not placed_viewpoint
                and original_observation is None
                and (
                    (not precision and self._current_observation_excludes(reference_tag_id))
                    or (not precision and self._current_search_position_visited())
                )
            ):
                self.get_logger().info(
                    f'AprilTag {reference_tag_id} não localizada nas '
                    'observações da posição atual; evitando nova detecção '
                    'no mesmo local.'
                )
                if not self._move_to_next_search_position(reference_tag_id):
                    raise StepFailed(
                        f"passo '{step.step_id}' ({step.action}) falhou: AprilTag "
                        f'{reference_tag_id} não apareceu nas observações e '
                        'todas as posições de busca já foram examinadas '
                        'ou bloqueadas por proteção.'
                    )
        original_fallback_pending = original_observation is not None
        pp_search_attempts = set()
        pp_retry_count = 0
        while True:
            goal.require_alignment = not alignment_completed
            result = self._call_manipulation_action(
                client, goal,
                f"passo '{step.step_id}' ({step.action})", timeout, 'place', tag_id,
            )
            failure = self._manipulation_failure(result)
            if failure is None:
                self._remember_placed_tag_viewpoint(tag_id)
                if not precision:
                    self._remember_stack_alignment(tag_id, reference_tag_id)
                return
            if not result.outcome.effect_known:
                raise StepFailed(
                    f"passo '{step.step_id}' ({step.action}) deixou o estado físico "
                    f'incerto: {failure}'
                )

            known, gripper, _slots = self._world_state.snapshot()
            if known and gripper == EMPTY:
                self._remember_placed_tag_viewpoint(tag_id)
                if not precision:
                    self._remember_stack_alignment(tag_id, reference_tag_id)
                self.get_logger().warning(
                    f"Passo '{step.step_id}' confirmou o depósito antes "
                    f'de falhar durante a finalização: {failure}. O fluxo da '
                    'missão continuará.'
                )
                return
            if not known or gripper != tag_id:
                raise StepFailed(
                    f"passo '{step.step_id}' ({step.action}) não pode ser repetido "
                    f'com segurança: {failure}'
                )
            if (precision and getattr(self, '_pp_organizing', False)
                    and result.outcome.code == ManipulationResult.NO_FREE_SPACE):
                # The final scene has already been merged by the action wrapper.
                # Let the organizer store the held cube and clear/replan this slot.
                raise PrecisionSlotOccupied(failure)
            if result.recovery_reason == action_type.Result.RECOVERY_ALIGNMENT_REQUIRED:
                if alignment_completed or not result.has_detected_pose:
                    if not precision or pp_retry_count >= 2:
                        raise StepFailed('Solicitação de alinhamento do depósito sem pose válida ou repetida.')
                    pp_retry_count += 1
                    self._check_canceled()
                    if not result.has_detected_pose:
                        continue  # Request a fresh snapshot rather than use an invalid pose.
                self._recover_pick(result, step, alignment_only=True, config=config)
                alignment_completed = True
                continue
            if (precision and pp_retry_count < 2 and result.outcome.code in {
                    ManipulationResult.PERCEPTION_UNAVAILABLE, ManipulationResult.MOTION_FAILED,
                    ManipulationResult.BUSY, ManipulationResult.SERVER_UNAVAILABLE}):
                pp_retry_count += 1
                self._check_canceled()
                self.get_logger().warning(
                    f'PP: repetindo depósito com cubo confirmado na garra '
                    f'(tentativa de recuperação {pp_retry_count}/2): {failure}')
                continue
            if (
                config.enabled
                and result.outcome.code == ManipulationResult.OBJECT_NOT_FOUND
            ):
                self._mark_current_search_position(
                    self._visited_search_positions.setdefault(
                        self._current_location, set()),
                    config.search_positions_mm,
                )
                if original_fallback_pending:
                    original_fallback_pending = False
                    assert original_observation is not None
                    if self._return_to_original_observation(
                        reference_tag_id, original_observation
                    ):
                        alignment_completed = False
                        continue
                searching = (self._pp_move_to_next_search_point(reference_tag_id, reference=True,
                                                               attempted=pp_search_attempts)
                             if precision else self._move_to_next_search_position(reference_tag_id))
                if searching:
                    alignment_completed = False
                    continue
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) falhou: {failure}"
            )

    def _align_for_shelf_placement(self, area: ServiceArea) -> None:
        assert self._arena is not None
        alignment = (
            area.shelf_place_alignment or self._arena.shelf_place_alignment_defaults
        )
        result = self._control_wall(
            alignment.distance_mm, alignment.tolerance_mm, alignment.timeout_s,
            f'alinhamento antes do depósito em {area.area_id}',
            accept_safety_abort=False,
        )
        self._update_table_position(result)
        if (
            not result.has_valid_reading
            or self._current_wall_distance_mm is None
            or not math.isfinite(self._current_wall_distance_mm)
            or abs(self._current_wall_distance_mm - alignment.distance_mm)
            > alignment.tolerance_mm
        ):
            raise StepFailed('Distância de alinhamento para depósito na SH não confirmada.')

    def _restore_shelf_observation_distance(self, area: ServiceArea) -> None:
        alignment = area.alignment
        result = self._run_area_motion(
            self._prepare_for_pick_observation,
            lambda: self._control_wall(
                alignment.distance_mm, alignment.tolerance_mm, alignment.timeout_s,
                f'retorno à distância padrão após depósito em {area.area_id}',
                accept_safety_abort=False,
            )
        )
        self._update_table_position(result)
        if (
            not result.has_valid_reading
            or self._current_wall_distance_mm is None
            or not math.isfinite(self._current_wall_distance_mm)
            or abs(self._current_wall_distance_mm - alignment.distance_mm)
            > alignment.tolerance_mm
        ):
            raise StepFailed('Retorno à distância padrão da SH não confirmado.')

    def _next_slot_movement(self, scheduler, choice, plan: Plan) -> SlotMovement | None:
        """Look ahead for base motion without assuming that cargo already moved."""
        observations = self._scheduler_observations()
        following = scheduler.select(
            choice.next_state, observations, self._current_lateral_position_mm)
        config = self._arena.pickup_recovery
        if following is None:
            # A fresh observation must precede any new destination decision.
            if (choice.step.action != 'store' or not config.enabled
                    or not self._current_scene_observed()
                    or getattr(self, '_search_phase', 0)):
                return None
            visited = self._visited_search_positions.get(self._current_location, set())
            blocked = self._blocked_search_positions.get(self._current_location, set())
            candidates = [p for p in config.search_positions_mm
                          if p not in visited and p not in blocked]
            if not candidates:
                return None
            lateral = min(candidates, key=lambda p: (
                abs(p - self._current_lateral_position_mm),
                config.search_positions_mm.index(p)))
            return SlotMovement(
                self._arena.service_areas[self._current_location].alignment.distance_mm,
                lateral)
        step = following.step
        if step.action == 'depart':
            next_visit = following.next_state.visit
            target = (plan.visits[next_visit].target if next_visit < len(plan.visits)
                      else 'finish' if plan.finish else None)
            if target is None or target == self._current_location:
                return None
            departure = self._arena.service_areas[self._current_location].departure
            return SlotMovement(departure.distance_mm, departure.lateral_position_mm, True)
        if not config.enabled:
            return None
        if step.action == 'place_in_container':
            color = (PlaceInContainer.Goal.RED if step.container_color == 'red'
                     else PlaceInContainer.Goal.BLUE)
            memory = self._container_observations.get((self._current_location, color))
            return (SlotMovement(round(memory.wall_distance_mm), memory.lateral_position_mm)
                    if memory is not None else None)
        if step.action not in {'pick', 'stack', 'place_on_precision_table'}:
            return None
        tag = (step.reference_tag_id if step.action == 'place_on_precision_table'
               else step.support_tag_id if step.action == 'stack' else step.tag_id)
        if step.action == 'stack':
            if self._stack_is_aligned(tag):
                return None
            viewpoint = self._placed_tag_viewpoints.get((self._current_location, tag))
            if viewpoint is not None:
                return SlotMovement(*viewpoint)
        memory = ((self._pp_reference_observations if step.action == 'place_on_precision_table'
                   else self._tag_observations).get((self._current_location, tag)))
        if step.action == 'place_on_precision_table' and memory is not None:
            return SlotMovement(*self._precision_memory_destination(memory))
        return (SlotMovement(memory.pickup_wall_distance_mm, memory.pickup_lateral_position_mm)
                if memory is not None else None)

    def _slot_movement_is_opposite(self, slot_id: str, movement: SlotMovement) -> bool:
        # FollowWall defines positive travel as right and negative travel as left.
        slot_direction = {'left': -1, 'right': 1}.get(slot_id)
        if slot_direction is None or self._current_wall_distance_mm is None:
            return False
        lateral = (movement.lateral_position_mm if movement.departure
                   else self._clamp_lateral_position(movement.lateral_position_mm))
        travel = round(lateral - self._current_lateral_position_mm)
        tolerance = (self._arena.service_areas[self._current_location].departure.tolerance_mm
                     if movement.departure else self._arena.pickup_recovery.travel_tolerance_mm)
        return abs(travel) > tolerance and travel * slot_direction < 0

    def _slot_movement_can_overlap(self, slot_id, movement):
        if self._current_wall_distance_mm is None:
            return False
        config = self._async_motion_config()
        area = self._arena.service_areas[self._current_location]
        if movement.departure:
            if not config.approach_departure_enabled:
                return False
            tolerance = area.departure.tolerance_mm
            lateral = movement.lateral_position_mm
        else:
            if config.table_mode == 'disabled':
                return False
            if config.table_mode == 'opposite_sides':
                return self._slot_movement_is_opposite(slot_id, movement)
            tolerance = self._arena.pickup_recovery.travel_tolerance_mm
            lateral = self._clamp_lateral_position(movement.lateral_position_mm)
        return (abs(lateral - self._current_lateral_position_mm) > tolerance or
                abs(movement.wall_distance_mm - self._current_wall_distance_mm) >
                (area.departure.tolerance_mm if movement.departure else self._arena.pickup_recovery.wall_tolerance_mm))

    def _execute_slot_with_movement(
        self, step, client, goal, timeout, tag_id, movement: SlotMovement,
    ) -> Any:
        ready = threading.Event()
        finished = threading.Event()
        results = []

        def feedback(message):
            # Both cargo actions publish APPROACHING only after their safe waypoint.
            if message.feedback.status.phase == ManipulationFeedback.APPROACHING:
                ready.set()

        def transfer():
            try:
                result = self._call_manipulation_action(
                    client, goal, f"passo '{step.step_id}' ({step.action})",
                    timeout, step.action, tag_id, step.slot_id,
                    feedback_callback=feedback,
                )
                failure = self._manipulation_failure(result)
                if failure is not None:
                    raise StepFailed(f"passo '{step.step_id}' ({step.action}) falhou: {failure}")
                if not self._world_state.snapshot()[0]:
                    raise StepFailed('Efeito da transferência no slot não confirmado; carga incerta.')
                results.append(result)
            finally:
                finished.set()

        def move():
            while not ready.wait(0.05):
                self._check_canceled()
                if finished.is_set():
                    return
            self._check_canceled()
            if finished.is_set():
                return
            if movement.departure:
                self._depart_service_area(slot_overlap=True)
            else:
                config = self._arena.pickup_recovery
                lateral = self._clamp_lateral_position(movement.lateral_position_mm)
                travel = round(lateral - self._current_lateral_position_mm)
                result = self._control_wall(
                    movement.wall_distance_mm, config.wall_tolerance_mm, config.timeout_s,
                    f'deslocamento durante {step.action} no slot {step.slot_id}',
                    travel_distance_mm=travel,
                    travel_tolerance_mm=config.travel_tolerance_mm,
                    alignment_recovery_distance_mm=0,
                    accept_safety_abort=True,
                )
                self._update_table_position(result)
                self._remember_blocked_search_destination(lateral, travel)

        self._run_with_manipulator_prepare(transfer, move)
        return results[0]

    def _execute_manipulation(
        self, step: Step, slot_movement: SlotMovement | None = None
    ) -> None:
        if step.action != 'pick':
            self._direct_pick_observation = None
        if step.action not in {'stack', 'retrieve', 'store'}:
            self._stack_alignment = None
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
                goal.prepare_retrieve = step.prepare_retrieve
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
                if step.tag_id is not None and tag_id != step.tag_id:
                    raise StateConflict(f'A entrega exige {step.tag_id}, mas a garra contém {tag_id}.')
                if step.possible_tag_ids and (
                    tag_id not in step.possible_tag_ids
                    or self._world_state.tag_colors().get(tag_id) != step.tag_color
                ):
                    raise StateConflict('Entrega exige uma tag candidata com a cor solicitada confirmada.')
                self._last_delivery_action = step.action
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
                elif step.action == 'place_on_precision_table':
                    if area.area_type != 'PP':
                        raise ConfigurationError('place_on_precision_table exige área PP.')
                    goal = PlaceOnPrecisionTable.Goal()
                    goal.reference_tag_id = int(step.reference_tag_id)
                    self._configure_pp_goal(goal)
                    goal.require_empty_slot = getattr(self, '_pp_organizing', False)
                    goal.held_tag_id = int(tag_id)
                    goal.occupancy_radius_m = self._arena.precision_perception.occupancy_radius_m
                    goal.ws_height_cm = float(area.height_cm)
                    client = self._place_precision_client
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

        if transition == 'place' and area.area_type == 'SH' and step.action != 'stack':
            self._align_for_shelf_placement(area)

        if step.action in {'place_on_table', 'place_in_container'}:
            self._execute_place_with_recovery(
                step, client, goal, tag_id, float(area.height_cm), timeout
            )
            return

        if step.action == 'stack':
            self._execute_stack_with_search(step, goal, tag_id, timeout)
            return
        if step.action == 'place_on_precision_table':
            self._execute_precision_with_search(step, goal, tag_id, timeout)
            return

        if transition in {'store', 'retrieve'}:
            self._retreat_from_lateral_wall_before_slot_access(slot_id, transition)
        if (slot_movement is not None and transition in {'store', 'retrieve'}
                and self._slot_movement_can_overlap(slot_id, slot_movement)):
            result = self._execute_slot_with_movement(
                step, client, goal, timeout, tag_id, slot_movement)
        else:
            result = self._call_manipulation_action(
                client, goal, f"passo '{step.step_id}' ({step.action})",
                timeout, transition, tag_id, slot_id,
            )
        failure = self._manipulation_failure(result)
        if failure is not None:
            raise StepFailed(
                f"passo '{step.step_id}' ({step.action}) falhou: {failure}"
            )
        if step.action == 'place_on_shelf' and area.area_type == 'SH':
            self._restore_shelf_observation_distance(area)

    def _execute_step(
        self, step: Step, slot_movement: SlotMovement | None = None
    ) -> None:
        if step.action != 'pick':
            self._direct_pick_observation = None
        if step.action in {'navigate', 'finish'}:
            self._stack_alignment = None
            self._pp_aligned_reference = None
        if step.action == 'navigate':
            assert step.target is not None
            self._navigate(step.target)
        elif step.action == 'finish':
            self._navigate('finish')
        else:
            if slot_movement is None:
                self._execute_manipulation(step)
            else:
                self._execute_manipulation(step, slot_movement)

    def _observe_visit(self) -> None:
        """Observe without requesting any particular object or physical pick."""
        known, gripper, _slots = self._world_state.snapshot()
        if not known or gripper != EMPTY:
            raise StepFailed('Observação de seleção requer garra vazia e carga conhecida.')
        self._prepare_for_pick_observation()
        goal = AnalyzeScene.Goal()
        goal.requested_detectors = AnalyzeScene.Goal.APRILTAGS | AnalyzeScene.Goal.CONTAINERS_HSV
        self._configure_pp_goal(goal)
        if self._is_precision_area():
            goal.requested_detectors = AnalyzeScene.Goal.APRILTAGS
        goal.duration = self._duration(2.0)
        goal.work_surface_height_m = self._arena.service_areas[self._current_location].height_cm / 100.0
        result = self._call_action(self._vision_client, goal, 'observação da visita',
                                   self._manipulation_timeout())
        if result.frames_processed == 0 or result.frames_with_base_transform == 0:
            raise StepFailed('Observação sem frames válidos no referencial da base.')
        scene = SceneObservation()
        scene.completed = True
        scene.requested_detectors = goal.requested_detectors
        scene.apriltags = result.best_apriltags_base
        scene.containers = result.best_containers_base
        # Negative observations invalidate memories at this same viewpoint.
        visible_object_ids = {int(tag.id) for tag in scene.apriltags}
        if self._is_precision_area():
            from .precision_perception import split_detections
            _refs, _objects = split_detections(scene.apriltags, self._arena.precision_perception)
            visible_object_ids = set(_objects)
        for key, memory in list(self._tag_observations.items()):
            if (key[0] == self._current_location
                    and abs(memory.wall_distance_mm - self._current_wall_distance_mm) <= 1
                    and abs(memory.lateral_position_mm - self._current_lateral_position_mm) <= 1
                    and key[1] not in visible_object_ids):
                del self._tag_observations[key]
        for key, memory in list(self._container_observations.items()):
            if (key[0] == self._current_location
                    and abs(memory.wall_distance_mm - self._current_wall_distance_mm) <= 1
                    and abs(memory.lateral_position_mm - self._current_lateral_position_mm) <= 1
                    and key[1] not in {int(container.color) for container in scene.containers}):
                del self._container_observations[key]
        class ObservationResult:
            scene_observation = scene
        self._remember_scene_observations(ObservationResult())
        self._direct_pick_observation = self._last_table_observation

    def _scheduler_observations(self):
        observations = {('tag', tag): memory.lateral_position_mm
                        for (area, tag), memory in self._tag_observations.items()
                        if area == self._current_location}
        observations.update({('reference', tag): memory.lateral_position_mm
                             for (area, tag), memory in
                             getattr(self, '_pp_reference_observations', {}).items()
                             if area == self._current_location})
        observations.update({('container', 'red' if color == PlaceInContainer.Goal.RED else 'blue'):
                             memory.lateral_position_mm
                             for (area, color), memory in self._container_observations.items()
                             if area == self._current_location})
        observations.update({('tag', tag): lateral
                             for (area, tag), (_wall, lateral) in self._placed_tag_viewpoints.items()
                             if area == self._current_location})
        return observations

    def _report_scheduled_operation(self, goal_handle, plan, step, task_id, description):
        self._failed_step_id = task_id
        self._current_step_index = self._completed_steps
        self._active_world_operation = step.action
        self._publish_world_state()
        self._feedback(goal_handle, self._completed_steps, max(plan.total_steps, self._completed_steps + 1),
                       replace(step, step_id=task_id), description)

    def _run_plan(self, goal_handle, plan: Plan) -> None:
        scheduler = Scheduler(plan, tuple(self._world_state.snapshot()[2]), self._check_canceled)
        self._required_color_tags = {tag for visit in plan.visits for task in visit.tasks
                                     for tag in task.possible_tag_ids}
        state = scheduler.initial_state
        if not scheduler.feasible(state):
            raise ConfigurationError('Missão inviável para a carga configurada.')
        while not scheduler.complete(state):
            visit = plan.visits[state.visit]
            nav = Step(visit.visit_id, 'navigate', target=visit.target)
            self._report_scheduled_operation(goal_handle, plan, nav, visit.visit_id,
                                             f'Navegando para {visit.target}')
            self._execute_step(nav)
            self._completed_steps += 1
            if visit.pp_final_state is not None and visit.pp_start_state is None:
                with self._search_session():
                    self._run_precision_organization(goal_handle, plan, visit)
                known, held, cargo = self._world_state.snapshot()
                if not known or held != EMPTY:
                    raise StepFailed('PP: estado da carga não confirmado ao encerrar a visita.')
                state = replace(state, visit=state.visit + 1,
                                slots=tuple(cargo[slot] for slot in scheduler.slot_ids))
                if not scheduler.feasible(state):
                    raise StepFailed('Carga restante após PP impede as próximas tarefas.')
                continue
            pp_slots = dict(visit.pp_start_state) if visit.pp_start_state is not None else None
            with self._search_session():
                while state.visit < len(plan.visits) and plan.visits[state.visit] is visit:
                    self._check_canceled()
                    known, gripper, slots = self._world_state.snapshot()
                    if (not known or gripper != state.gripper
                            or tuple(slots[slot] for slot in scheduler.slot_ids) != state.slots):
                        raise StepFailed('Carga física confirmada diverge do escalonador.')
                    scheduler.set_tag_colors(self._world_state.tag_colors())
                    viable = scheduler.viable_choices(state)
                    if not viable:
                        raise StepFailed(f'Visita {visit.visit_id} bloqueada pela carga ou suportes.')
                    choice = scheduler.select(state, self._scheduler_observations(),
                                              self._current_lateral_position_mm)
                    if choice is None:
                        if gripper != EMPTY:
                            raise StepFailed(f'Visita {visit.visit_id}: garra bloqueada.')
                        if not self._current_scene_observed():
                            self._observe_visit()
                            continue
                        self._failed_step_id = min((c.task_id for c in viable if c.task_id),
                                                   default=visit.visit_id)
                        pending_pick = next((c.step.tag_id for c in viable if c.step.action == 'pick'), 0)
                        if (self._arena.pickup_recovery.enabled
                                and self._move_to_next_search_position(pending_pick)):
                            self._observe_visit()
                            continue
                        # Permit retrieving a delivery whose detector found no target;
                        # its existing recovery/fallback handles that case. Never pick unseen tags.
                        choice = scheduler.select(state, self._scheduler_observations(),
                                                  self._current_lateral_position_mm,
                                                  allow_unobserved=True)
                        if choice is None:
                            raise StepFailed(f'Visita {visit.visit_id}: objetos pendentes não encontrados após busca.')
                    if choice.step.action == 'skip':
                        state = choice.next_state
                        continue
                    if choice.step.action == 'depart':
                        if pp_slots is not None and pp_slots != dict(visit.pp_final_state):
                            raise StepFailed(f'Visita {visit.visit_id}: organização PP não alcançou final_state.')
                        state = choice.next_state
                        break
                    task_id = choice.task_id or next(
                        (task.step_id for index, (vi, task, _group) in enumerate(scheduler.tasks)
                         if vi == state.visit and not state.done & (1 << index)), visit.visit_id)
                    description = f'{choice.step.action}: tag {choice.step.tag_id} em {visit.target}'
                    self._report_scheduled_operation(goal_handle, plan, choice.step, task_id, description)
                    self._flexible_pick = choice.step.action == 'pick'
                    try:
                        step = replace(choice.step, step_id=task_id)
                        if pp_slots is not None:
                            if step.action == 'pick' and pp_slots.get(step.reference_tag_id) != step.tag_id:
                                raise StepFailed(f'Organização PP: cubo {step.tag_id} não está no alojamento esperado.')
                            if step.action == 'place_on_precision_table' and (
                                step.reference_tag_id not in pp_slots or pp_slots[step.reference_tag_id] is not None
                            ):
                                raise StepFailed(f'Organização PP: alojamento {step.reference_tag_id} não está vazio.')
                        if step.action == 'store':
                            following = scheduler.select(
                                choice.next_state, self._scheduler_observations(),
                                self._current_lateral_position_mm, allow_unobserved=True)
                            if (following is not None and following.step.action == 'retrieve'
                                    and following.step.slot_id == step.slot_id):
                                step = replace(step, prepare_retrieve=True)
                        movement = (self._next_slot_movement(scheduler, choice, plan)
                                    if step.action in {'store', 'retrieve'} else None)
                        if movement is None:
                            self._execute_step(step)
                        else:
                            self._execute_step(step, slot_movement=movement)
                    except TaskNotFound:
                        continue
                    finally:
                        self._flexible_pick = False
                    known, gripper, slots = self._world_state.snapshot()
                    expected = choice.next_state
                    if (not known or gripper != expected.gripper
                            or tuple(slots[slot] for slot in scheduler.slot_ids) != expected.slots):
                        raise StepFailed(f'Tarefa {task_id}: efeito físico esperado não confirmado.')
                    if pp_slots is not None:
                        # Update slot occupancy only after confirming the physical transfer.
                        if choice.step.action == 'pick':
                            pp_slots[choice.step.reference_tag_id] = None
                        elif choice.step.action == 'place_on_precision_table':
                            pp_slots[choice.step.reference_tag_id] = choice.step.tag_id
                    if choice.task_index is not None:
                        self._completed_steps += 1
                        if choice.step.action != 'pick':
                            actual_action = getattr(self, '_last_delivery_action', choice.step.action)
                            self._delivery_outcomes.append(DeliveryOutcome(
                                task_id, choice.step.tag_id, visit.target, choice.step.action,
                                actual_action,
                                choice.step.container_color if actual_action == 'place_in_container' else None,
                                choice.step.support_tag_id if actual_action == 'stack' else None,
                                choice.step.reference_tag_id if actual_action == 'place_on_precision_table' else None))
                            self._feedback(goal_handle, self._completed_steps - 1, plan.total_steps,
                                           replace(choice.step, step_id=task_id),
                                           f'Entrega confirmada: tag {choice.step.tag_id}, {actual_action} em {visit.target}')
                    state = choice.next_state
        if plan.finish:
            step = Step('finish', 'finish')
            self._report_scheduled_operation(goal_handle, plan, step, 'finish', 'Navegando para finish')
            self._execute_step(step)
            self._completed_steps += 1

    def _load_goal_files(self, plan_id: str) -> tuple[Arena, Plan]:
        if not PLAN_ID_PATTERN.fullmatch(plan_id):
            raise ConfigurationError('plan_id possui formato inválido.')
        arena_path = Path(str(self.get_parameter('arena_file').value))
        plans_root = Path(
            str(self.get_parameter('plans_directory').value)
        ).resolve()
        plan_path = plans_root / f'{plan_id}.yaml'
        arena = load_arena(arena_path)
        plan = load_plan(plan_path, cargo_capacity=len(self._world_state.snapshot()[2]))
        if plan.plan_id != plan_id:
            raise ConfigurationError(
                f"O arquivo solicitado como '{plan_id}' declara plan_id "
                f"'{plan.plan_id}'."
            )
        validate_plan(plan, arena, tuple(self._world_state.snapshot()[2]), self._check_canceled)
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
        feedback.total_steps = max(total, index + 1)
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
        self._stack_alignment = None
        self._status = 'running'
        self._departure_completed_for = None
        self._service_area_vision_active = False
        self._current_location = 'start'
        self._current_wall_distance_mm = None
        self._current_lateral_position_mm = 0.0
        self._last_follow_wall_result = None
        self._last_lateral_travel_direction = 0
        self._tag_observations.clear()
        self._pp_reference_observations = {}
        self._last_pp_scene = None
        self._pp_aligned_reference = None
        self._pp_reference_views = {}
        self._pp_direct_pick_tag = None
        self._placed_tag_viewpoints.clear()
        self._container_observations.clear()
        self._safety_search_history = {}
        self._search_phase = 0
        self._search_led_off = False
        self._container_search_positions.clear()
        self._visited_search_positions.clear()
        self._blocked_search_positions.clear()
        self._last_wall_control_protection_stop = False
        self._last_table_observation = None
        self._scene_observations = {}
        self._direct_pick_observation = None
        completed = 0
        self._completed_steps = 0
        self._failed_step_id = ''
        self._delivery_outcomes = []
        self._flexible_pick = False
        try:
            if getattr(self, '_pp_inventory_pending', False):
                known, held, cargo = self._world_state.snapshot()
                if not known or held != EMPTY or any(tag != EMPTY for tag in cargo.values()):
                    raise StepFailed('Carga remanescente da organização PP: recupere a carga antes de iniciar outra missão.')
                self._pp_inventory_pending = False
            accepted_files = getattr(self, '_accepted_mission_files', None)
            self._accepted_mission_files = None
            arena, plan = (accepted_files if accepted_files is not None else
                           self._load_goal_files(str(goal_handle.request.plan_id)))
            self._arena = arena
            self._current_location = plan.initial_location
            if self._current_location in arena.service_areas:
                self._activate_service_area_vision()
            self._world_state.reset()
            self._publish_world_state()
            self._run_plan(goal_handle, plan)
            completed = self._completed_steps
            self._deactivate_service_area_vision()
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
                self._completed_steps,
                self._failed_step_id,
                str(error),
            )
        except ConfigurationError as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.CONFIGURATION_ERROR,
                self._completed_steps,
                self._failed_step_id,
                str(error),
            )
        except StepFailed as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.STEP_FAILED,
                self._completed_steps,
                self._failed_step_id,
                str(error),
            )
        except Exception as error:
            self.get_logger().error(f'Falha interna na missão: {error}')
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.INTERNAL_ERROR,
                self._completed_steps,
                self._failed_step_id,
                str(error),
            )
        finally:
            try:
                self._deactivate_service_area_vision()
            except StepFailed as error:
                self.get_logger().error(f'Falha ao desligar visão após missão: {error}')
            self._cancel_event.clear()
            self._arena = None
            self._active_world_operation = ''
            self._publish_world_state()
            with self._lock:
                self._busy = False
                self._active_children = {}

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
