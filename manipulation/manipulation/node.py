"""Serialized ROS 2 action servers for all manipulation capabilities."""

from __future__ import annotations

import copy
import math
import threading
from pathlib import Path
from typing import Any, Callable

from ament_index_python.packages import get_package_share_directory
from interfaces.action import (
    PickObject,
    PlaceAtPose,
    PlaceInContainer,
    PlaceOnShelf,
    PlaceOnTable,
    PrepareManipulator,
    RetrieveObject,
    StackObject,
    StoreObject,
)
from interfaces.msg import (
    CargoSlotState,
    ManipulationFeedback,
    ManipulationResult,
    ManipulationState,
)
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from so_arm_101_moveit_config.configuracao import (
    ACELERACAO_MAXIMA,
    ACELERACAO_MAXIMA_DA_GARRA,
    ESTADOS_DOS_GRUPOS,
    GRUPO_BRACO,
    GRUPO_GARRA,
    REFERENCIAL_BASE,
    TOLERANCIA_DA_JUNTA_DA_GARRA,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_MAXIMA,
    VELOCIDADE_MAXIMA_DA_GARRA,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt, OperacaoCancelada
from so_arm_101_moveit_config.restricoes import (
    criar_pose,
    normalizar_angulo_de_pegada,
    restricoes_de_pegada,
    restricoes_de_pre_pegada,
)

from .errors import (
    ConfigurationError,
    FeatureUnavailable,
    NoFreeSpace,
    ObjectNotFound,
    PerceptionUnavailable,
    ServerUnavailable,
    StateConflict,
)
from .profiles import PlacementProfile, ProfileSet, load_profiles
from .state import EMPTY, ManipulationInventory


_ERROR_CODES = {
    ConfigurationError: ManipulationResult.CONFIGURATION_ERROR,
    ObjectNotFound: ManipulationResult.OBJECT_NOT_FOUND,
    ServerUnavailable: ManipulationResult.SERVER_UNAVAILABLE,
    PerceptionUnavailable: ManipulationResult.PERCEPTION_UNAVAILABLE,
    NoFreeSpace: ManipulationResult.NO_FREE_SPACE,
    FeatureUnavailable: ManipulationResult.FEATURE_UNAVAILABLE,
    StateConflict: ManipulationResult.STATE_CONFLICT,
}


class ManipulationServer(Node):
    """Owns the arm/gripper resource and exposes semantic manipulation actions."""

    def __init__(self) -> None:
        """Load calibrated profiles and create the serialized action servers."""
        super().__init__('manipulation_server')
        share = Path(get_package_share_directory('manipulation'))
        defaults = {
            'profiles_file': str(share / 'config' / 'profiles.yaml'),
            'cargo_slots_file': str(share / 'config' / 'cargo_slots.yaml'),
            'move_group_action': '/move_action',
            'apriltag_action': '/apriltags/analyze',
            'joint_states_topic': '/joint_states',
            'state_topic': 'manipulation/state',
            'pick_action': 'manipulation/pick',
            'store_action': 'manipulation/store',
            'retrieve_action': 'manipulation/retrieve',
            'place_on_table_action': 'manipulation/place_on_table',
            'place_in_container_action': 'manipulation/place_in_container',
            'stack_action': 'manipulation/stack',
            'place_on_shelf_action': 'manipulation/place_on_shelf',
            'place_at_pose_action': 'manipulation/place_at_pose',
            'prepare_action': 'manipulation/prepare',
            'moveit_server_timeout_s': 15.0,
            'apriltag_analysis_duration_s': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        profiles_path = (
            str(self.get_parameter('profiles_file').value)
            or defaults['profiles_file']
        )
        cargo_path = (
            str(self.get_parameter('cargo_slots_file').value)
            or defaults['cargo_slots_file']
        )
        self._profiles = load_profiles(profiles_path, cargo_path)
        self._validate_named_states(self._profiles)
        self._inventory = ManipulationInventory(list(self._profiles.cargo_slots))

        self._callback_group = ReentrantCallbackGroup()
        self._busy = False
        self._active_operation = ''
        self._lock = threading.RLock()
        self._cancel_event = threading.Event()
        self._motion = ExecutorDoMoveIt(
            self,
            cancelamento_solicitado=self._cancel_event.is_set,
            callback_group=self._callback_group,
            move_group_action=str(self.get_parameter('move_group_action').value),
            apriltag_action=str(self.get_parameter('apriltag_action').value),
            joint_states_topic=str(self.get_parameter('joint_states_topic').value),
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_publisher = self.create_publisher(
            ManipulationState, str(self.get_parameter('state_topic').value), qos
        )

        common = {
            'goal_callback': self._goal_callback,
            'cancel_callback': self._cancel_callback,
            'callback_group': self._callback_group,
        }
        self._servers = [
            ActionServer(
                self, PickObject, str(self.get_parameter('pick_action').value),
                execute_callback=self._execute_pick, **common,
            ),
            ActionServer(
                self, StoreObject, str(self.get_parameter('store_action').value),
                execute_callback=self._execute_store, **common,
            ),
            ActionServer(
                self, RetrieveObject, str(self.get_parameter('retrieve_action').value),
                execute_callback=self._execute_retrieve, **common,
            ),
            ActionServer(
                self, PlaceOnTable,
                str(self.get_parameter('place_on_table_action').value),
                execute_callback=self._execute_place_on_table, **common,
            ),
            ActionServer(
                self, PlaceInContainer,
                str(self.get_parameter('place_in_container_action').value),
                execute_callback=self._execute_place_in_container, **common,
            ),
            ActionServer(
                self, StackObject, str(self.get_parameter('stack_action').value),
                execute_callback=self._execute_stack, **common,
            ),
            ActionServer(
                self, PlaceOnShelf,
                str(self.get_parameter('place_on_shelf_action').value),
                execute_callback=self._execute_place_on_shelf, **common,
            ),
            ActionServer(
                self, PlaceAtPose,
                str(self.get_parameter('place_at_pose_action').value),
                execute_callback=self._execute_place_at_pose, **common,
            ),
            ActionServer(
                self, PrepareManipulator, str(self.get_parameter('prepare_action').value),
                execute_callback=self._execute_prepare, **common,
            ),
        ]
        self._publish_state()
        self.get_logger().info(
            'Manipulação pronta: coleta, carga e depósitos semânticos.'
        )

    @staticmethod
    def _validate_named_states(profiles: ProfileSet) -> None:
        arm_states = ESTADOS_DOS_GRUPOS.get(GRUPO_BRACO, {})
        gripper_states = ESTADOS_DOS_GRUPOS.get(GRUPO_GARRA, {})
        required_arm = {
            profiles.transport_empty_state,
            profiles.transport_loaded_state,
            *(profile.observation_state for profile in profiles.pickup.values()),
            *(slot.store_state for slot in profiles.cargo_slots.values()),
            *(slot.retrieve_state for slot in profiles.cargo_slots.values()),
            *(
                profile.named_state
                for profile in profiles.placements.values()
                if profile.enabled and profile.strategy == 'named_state'
            ),
        }
        missing = sorted(state for state in required_arm if state not in arm_states)
        if missing:
            raise ConfigurationError(
                f'Estados do braço ausentes no SRDF: {missing}.'
            )
        required_gripper = {'open', 'pre_grip', 'grip'}
        missing_gripper = sorted(required_gripper - set(gripper_states))
        if missing_gripper:
            raise ConfigurationError(
                f'Estados da garra ausentes no SRDF: {missing_gripper}.'
            )

    def _goal_callback(self, _request: Any) -> GoalResponse:
        with self._lock:
            if self._busy:
                self.get_logger().warning(
                    'Goal de manipulação rejeitado: manipulador ocupado.'
                )
                return GoalResponse.REJECT
            self._busy = True
            self._active_operation = 'accepted'
            self._cancel_event.clear()
        self._publish_state()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_event.set()
        self._motion.cancelar_objetivo_ativo()
        return CancelResponse.ACCEPT

    def _set_active(self, operation: str) -> None:
        with self._lock:
            self._active_operation = operation
        self._publish_state()

    def _publish_state(self) -> None:
        known, gripper, slots = self._inventory.snapshot()
        message = ManipulationState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = REFERENCIAL_BASE
        message.state_known = known
        message.gripper_object_id = gripper
        message.active_operation = self._active_operation
        for slot_id in sorted(slots):
            slot = CargoSlotState()
            slot.slot_id = slot_id
            slot.object_id = slots[slot_id]
            message.cargo_slots.append(slot)
        self._state_publisher.publish(message)

    def _feedback(
        self,
        goal_handle: Any,
        action_type: Any,
        phase: int,
        progress: float,
        description: str,
    ) -> None:
        feedback = action_type.Feedback()
        feedback.status.phase = phase
        feedback.status.progress = float(progress)
        feedback.status.description = description
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(description)

    def _arm_state(self, state: str, description: str) -> None:
        self._motion.mover_para_estado(
            GRUPO_BRACO,
            state,
            description,
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )

    def _gripper(self, state: str, description: str) -> None:
        self._motion.mover_para_estado(
            GRUPO_GARRA,
            state,
            description,
            tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
        )

    def _safe(self) -> None:
        _, gripper, _ = self._inventory.snapshot()
        state = (
            self._profiles.transport_loaded_state
            if gripper != EMPTY
            else self._profiles.transport_empty_state
        )
        self._arm_state(state, 'Recolhendo o manipulador para transporte')

    def _ensure_moveit(self) -> None:
        timeout = float(self.get_parameter('moveit_server_timeout_s').value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ConfigurationError('moveit_server_timeout_s deve ser positivo.')
        if not self._motion.cliente_do_move_group.wait_for_server(timeout_sec=timeout):
            raise ServerUnavailable(
                f"Action '{self.get_parameter('move_group_action').value}' indisponível."
            )

    @staticmethod
    def _location_for_inventory(tag_id: int, inventory: ManipulationInventory) -> int:
        if tag_id == EMPTY:
            return ManipulationResult.LOCATION_UNKNOWN
        _, gripper, slots = inventory.snapshot()
        if gripper == tag_id:
            return ManipulationResult.LOCATION_GRIPPER
        if tag_id in slots.values():
            return ManipulationResult.LOCATION_CARGO
        return ManipulationResult.LOCATION_UNKNOWN

    def _make_result(
        self,
        action_type: Any,
        goal_handle: Any,
        code: int,
        message: str,
        tag_id: int,
        final_location: int | None = None,
        placed_pose: Any | None = None,
    ) -> Any:
        result = action_type.Result()
        result.outcome.object_tag_id = int(tag_id)
        result.outcome.code = int(code)
        known, _, _ = self._inventory.snapshot()
        result.outcome.state_known = known
        result.outcome.final_object_location = int(
            final_location
            if final_location is not None
            else self._location_for_inventory(tag_id, self._inventory)
        )
        result.outcome.message = message
        if placed_pose is not None and hasattr(result, 'placed_pose'):
            result.placed_pose = copy.deepcopy(placed_pose)
        if code == ManipulationResult.SUCCESS:
            goal_handle.succeed()
        elif code == ManipulationResult.CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _run(
        self,
        action_type: Any,
        goal_handle: Any,
        operation_name: str,
        tag_id: int,
        operation: Callable[[], tuple[str, int] | tuple[str, int, Any]],
        *,
        requires_moveit: bool = True,
    ) -> Any:
        self._set_active(operation_name)
        try:
            if requires_moveit:
                self._ensure_moveit()
            outcome = operation()
            message, location = outcome[:2]
            placed_pose = outcome[2] if len(outcome) == 3 else None
            return self._make_result(
                action_type, goal_handle, ManipulationResult.SUCCESS,
                message, tag_id, location, placed_pose,
            )
        except OperacaoCancelada as error:
            self._cancel_event.clear()
            return self._make_result(
                action_type, goal_handle, ManipulationResult.CANCELED,
                f'{error} O braço foi mantido na posição em que parou.', tag_id,
            )
        except Exception as error:
            code = ManipulationResult.MOTION_FAILED
            for error_type, mapped_code in _ERROR_CODES.items():
                if isinstance(error, error_type):
                    code = mapped_code
                    break
            if (
                action_type is PickObject
                and 'não encontrada' in str(error).lower()
            ):
                code = ManipulationResult.OBJECT_NOT_FOUND
            self.get_logger().error(f'{operation_name} falhou: {error}')
            return self._make_result(action_type, goal_handle, code, str(error), tag_id)
        finally:
            self._cancel_event.clear()
            with self._lock:
                self._busy = False
                self._active_operation = ''
            self._publish_state()

    def _execute_pick(self, goal_handle: Any) -> PickObject.Result:
        tag_id = int(goal_handle.request.tag_id)

        def operation() -> tuple[str, int]:
            self._inventory.validate_pick(tag_id)
            profile = self._profiles.pickup_profile(goal_handle.request.profile)
            last_error: Exception | None = None
            for attempt in range(1, profile.attempts + 1):
                grasp_committed = False
                try:
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.PREPARING,
                        0.05, f'Preparando coleta em mesa ({attempt}/{profile.attempts})',
                    )
                    self._gripper('open', 'Abrindo a garra')
                    self._arm_state(profile.observation_state, 'Posicionando câmera sobre a mesa')
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.OBSERVING,
                        0.20, f'Localizando AprilTag {tag_id}',
                    )
                    duration = float(
                        self.get_parameter('apriltag_analysis_duration_s').value
                    )
                    x, y, tag_z, yaw = self._motion.obter_pose_da_april_tag(
                        tag_id, duration
                    )
                    grasp_z = tag_z - profile.cube_size_m
                    grasp_yaw = (
                        normalizar_angulo_de_pegada(yaw) + profile.yaw_offset_deg
                    )
                    grasp_pose = criar_pose(x, y, grasp_z, grasp_yaw)
                    approach_pose = criar_pose(
                        x, y, grasp_z + profile.approach_height_m, grasp_yaw
                    )
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.APPROACHING,
                        0.40, 'Aproximando do objeto sobre a mesa',
                    )
                    self._motion.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pre_pegada(approach_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._motion.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pegada(grasp_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.GRASPING,
                        0.68, 'Fechando a garra no objeto',
                    )
                    try:
                        self._gripper('grip', 'Fechando a garra')
                    except Exception:
                        self._inventory.mark_unknown()
                        self._publish_state()
                        raise
                    self._inventory.commit_pick(tag_id)
                    self._publish_state()
                    grasp_committed = True
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.RETREATING,
                        0.82, 'Retirando o objeto da mesa',
                    )
                    self._motion.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pre_pegada(approach_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._safe()
                    return (
                        f'Objeto {tag_id} coletado da mesa.',
                        ManipulationResult.LOCATION_GRIPPER,
                    )
                except OperacaoCancelada:
                    raise
                except Exception as error:
                    last_error = error
                    if not self._inventory.snapshot()[0]:
                        raise
                    if grasp_committed:
                        self._inventory.mark_unknown()
                        self._publish_state()
                        raise
                    if attempt < profile.attempts:
                        self.get_logger().warning(
                            f'Tentativa de coleta {attempt} falhou: {error}'
                        )
            assert last_error is not None
            if 'não encontrada' in str(last_error).lower():
                raise ObjectNotFound(str(last_error)) from last_error
            raise last_error

        return self._run(PickObject, goal_handle, 'pick', tag_id, operation)

    def _execute_store(self, goal_handle: Any) -> StoreObject.Result:
        slot_id = str(goal_handle.request.slot_id)
        _, tag_id, _ = self._inventory.snapshot()

        def operation() -> tuple[str, int]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_store(object_tag_id, slot_id)
            slot = self._profiles.cargo_slots.get(slot_id)
            if slot is None:
                raise ConfigurationError(f"Compartimento não configurado: '{slot_id}'.")
            self._feedback(
                goal_handle, StoreObject, ManipulationFeedback.APPROACHING,
                0.30, f"Levando objeto ao compartimento '{slot_id}'",
            )
            self._arm_state(slot.store_state, 'Posicionando sobre o compartimento')
            self._feedback(
                goal_handle, StoreObject, ManipulationFeedback.RELEASING,
                0.65, 'Liberando objeto no compartimento',
            )
            try:
                self._gripper('open', 'Abrindo a garra')
            except Exception:
                self._inventory.mark_unknown()
                self._publish_state()
                raise
            self._inventory.commit_store(object_tag_id, slot_id)
            self._publish_state()
            self._safe()
            return (
                f"Objeto {object_tag_id} armazenado em '{slot_id}'.",
                ManipulationResult.LOCATION_CARGO,
            )

        return self._run(StoreObject, goal_handle, 'store', tag_id, operation)

    def _execute_retrieve(self, goal_handle: Any) -> RetrieveObject.Result:
        slot_id = str(goal_handle.request.slot_id)
        _, _, slots = self._inventory.snapshot()
        tag_id = slots.get(slot_id, EMPTY)

        def operation() -> tuple[str, int]:
            object_tag_id = self._inventory.require_slot_object(slot_id)
            self._inventory.validate_retrieve(object_tag_id, slot_id)
            slot = self._profiles.cargo_slots.get(slot_id)
            if slot is None:
                raise ConfigurationError(f"Compartimento não configurado: '{slot_id}'.")
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.PREPARING,
                0.10, 'Preparando a abertura da garra para retirar o objeto',
            )
            self._gripper('pre_grip', 'Posicionando a garra em pre_grip')
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.APPROACHING,
                0.25, f"Aproximando do compartimento '{slot_id}'",
            )
            self._arm_state(slot.store_state, 'Indo para a pose de armazenamento')
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.APPROACHING,
                0.45, 'Descendo até o objeto armazenado',
            )
            self._arm_state(slot.retrieve_state, 'Indo para a pose de retirada')
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.GRASPING,
                0.65, 'Fechando a garra no objeto armazenado',
            )
            try:
                self._gripper('grip', 'Fechando a garra')
            except Exception:
                self._inventory.mark_unknown()
                self._publish_state()
                raise
            self._inventory.commit_retrieve(object_tag_id, slot_id)
            self._publish_state()
            try:
                self._feedback(
                    goal_handle, RetrieveObject, ManipulationFeedback.RETREATING,
                    0.80, 'Elevando o objeto do compartimento',
                )
                self._arm_state(slot.store_state, 'Retornando à pose de armazenamento')
                self._safe()
            except Exception:
                self._inventory.mark_unknown()
                self._publish_state()
                raise
            return (
                f"Objeto {object_tag_id} retirado de '{slot_id}'.",
                ManipulationResult.LOCATION_GRIPPER,
            )

        return self._run(RetrieveObject, goal_handle, 'retrieve', tag_id, operation)

    @staticmethod
    def _validate_target_pose(pose) -> None:
        if pose.header.frame_id != REFERENCIAL_BASE:
            raise ConfigurationError(
                f"target_pose deve estar em '{REFERENCIAL_BASE}', não em "
                f"'{pose.header.frame_id}'."
            )
        values = (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ConfigurationError('target_pose contém valor não finito.')
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
        if quaternion_norm < 1e-6:
            raise ConfigurationError('target_pose possui quaternion nulo.')

    @staticmethod
    def _table_search_candidates(
        profile: PlacementProfile,
    ) -> list[tuple[float, float]]:
        """Build a bounded grid ordered from the nominal release position."""
        bounds = (
            profile.search_x_min_m,
            profile.search_x_max_m,
            profile.search_y_min_m,
            profile.search_y_max_m,
        )
        if any(value is None for value in bounds):
            raise FeatureUnavailable(
                'Preencha search_x_min_m, search_x_max_m, search_y_min_m e '
                'search_y_max_m no perfil table antes de analisar AprilTags.'
            )
        if profile.release_x_m is None or profile.release_y_m is None:
            raise FeatureUnavailable(
                'A posição nominal release_x_m/release_y_m não foi configurada.'
            )
        x_min, x_max, y_min, y_max = (float(value) for value in bounds)
        nominal_x = float(profile.release_x_m)
        nominal_y = float(profile.release_y_m)
        step = float(profile.search_step_m)
        if x_min > x_max or y_min > y_max:
            raise ConfigurationError(
                'Os limites mínimos da busca na mesa devem ser menores ou iguais '
                'aos limites máximos.'
            )
        if y_max > -0.10:
            raise ConfigurationError(
                'search_y_max_m não pode ser maior que -0.10 m.'
            )
        if not (x_min <= nominal_x <= x_max and y_min <= nominal_y <= y_max):
            raise ConfigurationError(
                'A pose nominal da mesa deve estar dentro da região de busca.'
            )

        negative_x = math.floor((nominal_x - x_min) / step + 1e-9)
        positive_x = math.floor((x_max - nominal_x) / step + 1e-9)
        negative_y = math.floor((nominal_y - y_min) / step + 1e-9)
        positive_y = math.floor((y_max - nominal_y) / step + 1e-9)
        xs = [
            nominal_x + index * step
            for index in range(-negative_x, positive_x + 1)
        ]
        ys = [
            nominal_y + index * step
            for index in range(-negative_y, positive_y + 1)
        ]
        candidates = [(x, y) for x in xs for y in ys]
        candidates.sort(key=lambda point: (
            (point[0] - nominal_x) ** 2 + (point[1] - nominal_y) ** 2,
            abs(point[1] - nominal_y),
            abs(point[0] - nominal_x),
            point[0],
            point[1],
        ))
        return candidates

    @staticmethod
    def _select_free_table_position(
        candidates: list[tuple[float, float]],
        obstacles: list[tuple[float, float]],
        minimum_distance_m: float,
    ) -> tuple[float, float]:
        """Return the first nominal-outward candidate clear of every tag."""
        for candidate_x, candidate_y in candidates:
            if all(
                math.hypot(candidate_x - obstacle_x, candidate_y - obstacle_y)
                + 1e-9 >= minimum_distance_m
                for obstacle_x, obstacle_y in obstacles
            ):
                return candidate_x, candidate_y
        raise NoFreeSpace(
            'Nenhuma posição da região de busca mantém a distância mínima de '
            f'{minimum_distance_m:.3f} m das AprilTags.'
        )

    def _placement_profile(self, name: str, capability: str) -> PlacementProfile:
        profile = self._profiles.placements.get(name)
        if profile is None:
            raise ConfigurationError(f"Perfil de depósito ausente: '{name}'.")
        if not profile.enabled:
            raise FeatureUnavailable(
                f"{capability} ainda não está habilitado/calibrado."
            )
        return profile

    def _release_at_pose(
        self,
        goal_handle: Any,
        action_type: Any,
        tag_id: int,
        release_pose: Any,
        profile: PlacementProfile,
        destination: str,
    ) -> tuple[str, int, Any]:
        """Execute the common approach, release and retreat transaction."""
        self._inventory.validate_place(tag_id)
        approach_pose = copy.deepcopy(release_pose)
        approach_pose.pose.position.z += profile.approach_height_m
        retreat_pose = copy.deepcopy(release_pose)
        retreat_pose.pose.position.z += profile.retreat_height_m
        self._feedback(
            goal_handle, action_type, ManipulationFeedback.APPROACHING,
            0.40, f'Aproximando do destino: {destination}',
        )
        self._motion.executar_objetivo(
            GRUPO_BRACO, restricoes_de_pre_pegada(approach_pose),
            VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
        )
        self._motion.executar_objetivo(
            GRUPO_BRACO, restricoes_de_pegada(release_pose),
            VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
        )
        self._feedback(
            goal_handle, action_type, ManipulationFeedback.RELEASING,
            0.72, f'Liberando objeto: {destination}',
        )
        try:
            self._gripper('open', 'Abrindo a garra no destino')
        except Exception:
            self._inventory.mark_unknown()
            self._publish_state()
            raise
        self._inventory.commit_place()
        self._publish_state()
        self._feedback(
            goal_handle, action_type, ManipulationFeedback.RETREATING,
            0.86, 'Recuando do destino',
        )
        self._motion.executar_objetivo(
            GRUPO_BRACO, restricoes_de_pre_pegada(retreat_pose),
            VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
        )
        self._safe()
        return (
            f'Objeto {tag_id} depositado: {destination}.',
            ManipulationResult.LOCATION_DESTINATION,
            release_pose,
        )

    def _execute_place_on_table(self, goal_handle: Any) -> PlaceOnTable.Result:
        _, tag_id, _ = self._inventory.snapshot()

        def operation() -> tuple[str, int, Any]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_place(object_tag_id)
            height_cm = float(goal_handle.request.ws_height_cm)
            if bool(goal_handle.request.analyze_containers):
                raise PerceptionUnavailable(
                    'A análise de contêineres ainda não foi implementada.'
                )
            profile = self._placement_profile('table', 'Depósito nominal na mesa')
            calibration = (
                profile.release_x_m,
                profile.release_y_m,
                profile.release_yaw_deg,
                profile.tcp_release_offset_cm,
            )
            if any(value is None for value in calibration):
                raise FeatureUnavailable(
                    'Preencha release_x_m, release_y_m, release_yaw_deg e '
                    'tcp_release_offset_cm no perfil table antes do depósito nominal.'
                )
            release_x_m, release_y_m, release_yaw_deg, tcp_offset_cm = calibration
            if bool(goal_handle.request.analyze_apriltags):
                candidates = self._table_search_candidates(profile)
                observation = self._profiles.pickup_profile(
                    'tabletop'
                ).observation_state
                self._feedback(
                    goal_handle, PlaceOnTable, ManipulationFeedback.OBSERVING,
                    0.10, 'Preparando a câmera para analisar as AprilTags da mesa',
                )
                self._arm_state(observation, 'Preparando câmera para depósito na mesa')
                duration = float(
                    self.get_parameter('apriltag_analysis_duration_s').value
                )
                try:
                    detections = self._motion.obter_deteccoes_de_april_tags(duration)
                except OperacaoCancelada:
                    raise
                except RuntimeError as error:
                    raise PerceptionUnavailable(str(error)) from error
                obstacles: list[tuple[float, float]] = []
                for detection in detections:
                    if int(detection.id) == object_tag_id:
                        continue
                    x = float(detection.pose.position.x)
                    y = float(detection.pose.position.y)
                    if not math.isfinite(x) or not math.isfinite(y):
                        raise PerceptionUnavailable(
                            f'AprilTag {detection.id} possui posição XY inválida.'
                        )
                    obstacles.append((x, y))
                release_x_m, release_y_m = self._select_free_table_position(
                    candidates,
                    obstacles,
                    float(profile.free_space_min_distance_m),
                )
                self._feedback(
                    goal_handle, PlaceOnTable, ManipulationFeedback.OBSERVING,
                    0.30,
                    f'Posição livre selecionada a partir da nominal: '
                    f'x={release_x_m:.3f}, y={release_y_m:.3f} m; '
                    f'{len(obstacles)} obstáculo(s)',
                )
            else:
                self._feedback(
                    goal_handle, PlaceOnTable, ManipulationFeedback.OBSERVING,
                    0.10,
                    f'Usando posição nominal na mesa de {height_cm:g} cm',
                )
            release_pose = criar_pose(
                float(release_x_m),
                float(release_y_m),
                (height_cm + float(tcp_offset_cm)) / 100.0,
                float(release_yaw_deg),
            )
            return self._release_at_pose(
                goal_handle, PlaceOnTable, object_tag_id, release_pose, profile,
                f'mesa com altura de {height_cm:g} cm',
            )

        return self._run(
            PlaceOnTable, goal_handle, 'place_on_table', tag_id, operation,
        )

    def _execute_place_in_container(
        self, goal_handle: Any
    ) -> PlaceInContainer.Result:
        _, tag_id, _ = self._inventory.snapshot()

        def operation() -> tuple[str, int]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_place(object_tag_id)
            color = int(goal_handle.request.container_color)
            colors = {
                PlaceInContainer.Goal.RED: 'vermelho',
                PlaceInContainer.Goal.BLUE: 'azul',
            }
            if color not in colors:
                raise ConfigurationError(f'Cor de contêiner inválida: {color}.')
            height_cm = float(goal_handle.request.ws_height_cm)
            self._feedback(
                goal_handle, PlaceInContainer, ManipulationFeedback.OBSERVING,
                0.10,
                f'Buscando contêiner {colors[color]} sobre WS de {height_cm:g} cm',
            )
            raise PerceptionUnavailable(
                'A detecção de contêineres ainda não foi implementada. '
                'Nenhum movimento foi executado.'
            )

        return self._run(
            PlaceInContainer, goal_handle, 'place_in_container', tag_id, operation,
            requires_moveit=False,
        )

    def _execute_stack(self, goal_handle: Any) -> StackObject.Result:
        _, tag_id, _ = self._inventory.snapshot()
        support_tag_id = int(goal_handle.request.support_tag_id)

        def operation() -> tuple[str, int, Any]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_place(object_tag_id)
            if support_tag_id < 0 or support_tag_id == object_tag_id:
                raise ConfigurationError(
                    'support_tag_id deve identificar outro objeto não negativo.'
                )
            profile = self._placement_profile('stack', 'Empilhamento')
            if not profile.calibrated_reference:
                raise FeatureUnavailable(
                    "O offset do perfil 'stack' ainda não foi calibrado."
                )
            observation = self._profiles.pickup_profile('tabletop').observation_state
            self._feedback(
                goal_handle, StackObject, ManipulationFeedback.OBSERVING,
                0.15,
                f'Localizando cubo de apoio {support_tag_id} pela AprilTag',
            )
            self._arm_state(observation, 'Preparando câmera para empilhamento')
            duration = float(self.get_parameter('apriltag_analysis_duration_s').value)
            try:
                x, y, z, yaw = self._motion.obter_pose_da_april_tag(
                    support_tag_id, duration
                )
            except RuntimeError as error:
                if 'não encontrada' in str(error).lower():
                    raise ObjectNotFound(str(error)) from error
                raise
            dx, dy, dz = profile.reference_offset_xyz
            release_pose = criar_pose(
                x + dx,
                y + dy,
                z + dz,
                normalizar_angulo_de_pegada(yaw) + profile.yaw_offset_deg,
            )
            return self._release_at_pose(
                goal_handle, StackObject, object_tag_id, release_pose, profile,
                f'empilhamento sobre o objeto {support_tag_id}',
            )

        return self._run(StackObject, goal_handle, 'stack', tag_id, operation)

    def _execute_place_on_shelf(
        self, goal_handle: Any
    ) -> PlaceOnShelf.Result:
        _, tag_id, _ = self._inventory.snapshot()

        def operation() -> tuple[str, int]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_place(object_tag_id)
            profile = self._placement_profile('shelf', 'Depósito na prateleira')
            if profile.strategy != 'named_state' or not profile.named_state:
                raise FeatureUnavailable(
                    "O perfil 'shelf' ainda não possui pose fixa calibrada."
                )
            self._feedback(
                goal_handle, PlaceOnShelf, ManipulationFeedback.APPROACHING,
                0.40, 'Movendo para a pose fixa da prateleira',
            )
            self._arm_state(profile.named_state, 'Posicionando sobre a prateleira')
            self._feedback(
                goal_handle, PlaceOnShelf, ManipulationFeedback.RELEASING,
                0.72, 'Liberando objeto na prateleira',
            )
            try:
                self._gripper('open', 'Abrindo a garra na prateleira')
            except Exception:
                self._inventory.mark_unknown()
                self._publish_state()
                raise
            self._inventory.commit_place()
            self._publish_state()
            self._safe()
            return (
                f'Objeto {object_tag_id} depositado na prateleira.',
                ManipulationResult.LOCATION_DESTINATION,
            )

        return self._run(
            PlaceOnShelf, goal_handle, 'place_on_shelf', tag_id, operation
        )

    def _execute_place_at_pose(self, goal_handle: Any) -> PlaceAtPose.Result:
        _, tag_id, _ = self._inventory.snapshot()

        def operation() -> tuple[str, int, Any]:
            object_tag_id = self._inventory.require_gripper_object()
            self._inventory.validate_place(object_tag_id)
            release_pose = copy.deepcopy(goal_handle.request.release_pose)
            self._validate_target_pose(release_pose)
            profile = self._placement_profile('explicit_pose', 'Depósito em pose')
            return self._release_at_pose(
                goal_handle, PlaceAtPose, object_tag_id, release_pose, profile,
                'pose explícita',
            )

        return self._run(
            PlaceAtPose, goal_handle, 'place_at_pose', tag_id, operation
        )

    def _execute_prepare(self, goal_handle: Any) -> PrepareManipulator.Result:
        mode = int(goal_handle.request.mode)

        def operation() -> tuple[str, int]:
            if mode in (
                PrepareManipulator.Goal.NAVIGATION,
                PrepareManipulator.Goal.SAFE_HOLD,
            ):
                self._safe()
                description = 'Manipulador recolhido para navegação.'
            elif mode == PrepareManipulator.Goal.OBSERVATION:
                profile = self._profiles.pickup_profile('tabletop')
                self._arm_state(profile.observation_state, 'Preparando observação da mesa')
                description = 'Manipulador preparado para observar a mesa.'
            else:
                raise ConfigurationError(f'Modo de preparação inválido: {mode}.')
            return description, ManipulationResult.LOCATION_UNKNOWN

        return self._run(
            PrepareManipulator, goal_handle, 'prepare', EMPTY, operation
        )

    def destroy_node(self):
        """Cancel child work before destroying the ROS node."""
        self._cancel_event.set()
        self._motion.cancelar_objetivo_ativo()
        for server in self._servers:
            server.destroy()
        return super().destroy_node()


def main(args=None) -> int:
    """Run the manipulation server in an executor that can service child actions."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    exit_code = 0
    try:
        node = ManipulationServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Falha fatal no servidor de manipulação: {error}')
        else:
            print(f'Falha fatal no servidor de manipulação: {error}')
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
