"""Serialized ROS 2 action servers for all manipulation capabilities."""

from __future__ import annotations

import copy
import math
from pathlib import Path
import threading
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
from interfaces.msg import ManipulationFeedback, ManipulationResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
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
from so_arm_101_moveit_config.movimento import (
    ExecutorDoMoveIt,
    FalhaDoMoveIt,
    OperacaoCancelada,
)
from so_arm_101_moveit_config.restricoes import (
    criar_pose,
    normalizar_angulo_de_pegada,
    restricoes_de_pegada,
    restricoes_de_deposito_em_container,
    restricoes_de_pre_pegada,
)

from .errors import (
    ConfigurationError,
    FeatureUnavailable,
    NoFreeSpace,
    ObjectNotFound,
    ObjectOutOfReach,
    PerceptionUnavailable,
    PickRecoveryRequired,
    ServerUnavailable,
)
from .profiles import load_profiles, PickupProfile, PlacementProfile, ProfileSet


EMPTY = ManipulationResult.EMPTY


_ERROR_CODES = {
    ConfigurationError: ManipulationResult.CONFIGURATION_ERROR,
    ObjectNotFound: ManipulationResult.OBJECT_NOT_FOUND,
    ServerUnavailable: ManipulationResult.SERVER_UNAVAILABLE,
    PerceptionUnavailable: ManipulationResult.PERCEPTION_UNAVAILABLE,
    NoFreeSpace: ManipulationResult.NO_FREE_SPACE,
    FeatureUnavailable: ManipulationResult.FEATURE_UNAVAILABLE,
}


class ManipulationServer(Node):
    """Owns the arm/gripper resource and exposes semantic manipulation actions."""

    def __init__(self) -> None:
        """Load calibrated profiles and create the serialized action servers."""
        super().__init__('manipulation_server')
        if (
            not hasattr(PickObject.Result(), 'observed_detections')
            or not hasattr(StoreObject.Goal(), 'object_tag_id')
            or not hasattr(PrepareManipulator.Goal(), 'gripper_loaded')
        ):
            raise ConfigurationError(
                'As interfaces de manipulação instaladas estão desatualizadas; '
                'recompile interfaces antes de iniciar manipulation.'
            )
        share = Path(get_package_share_directory('manipulation'))
        defaults = {
            'profiles_file': str(share / 'config' / 'profiles.yaml'),
            'cargo_slots_file': str(share / 'config' / 'cargo_slots.yaml'),
            'move_group_action': '/move_action',
            'vision_action': '/vision/analyze',
            'execute_trajectory_action': '/execute_trajectory',
            'joint_states_topic': '/joint_states',
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
            'vision_analysis_duration_s': 2.0,
            'work_surface_height_frame': 'base_footprint',
            'height_transform_timeout_s': 0.5,
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

        self._callback_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._busy = False
        self._active_operation = ''
        self._effect_known = True
        self._effect_location = ManipulationResult.LOCATION_UNKNOWN
        self._lock = threading.RLock()
        self._cancel_event = threading.Event()
        self._motion = ExecutorDoMoveIt(
            self,
            cancelamento_solicitado=self._cancel_event.is_set,
            callback_group=self._callback_group,
            move_group_action=str(self.get_parameter('move_group_action').value),
            vision_action=str(self.get_parameter('vision_action').value),
            execute_trajectory_action=str(
                self.get_parameter('execute_trajectory_action').value),
            joint_states_topic=str(self.get_parameter('joint_states_topic').value),
            monitorar_estados_continuamente=False,
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
        self.get_logger().info(
            'Manipulação pronta; actions físicas independentes do estado da missão.'
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
            *(slot.safe_state for slot in profiles.cargo_slots.values()),
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
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_event.set()
        self._motion.cancelar_objetivo_ativo()
        return CancelResponse.ACCEPT

    def _set_active(self, operation: str) -> None:
        with self._lock:
            self._active_operation = operation

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

    def _safe(self, gripper_loaded: bool) -> None:
        state = (
            self._profiles.transport_loaded_state
            if gripper_loaded
            else self._profiles.transport_empty_state
        )
        self._arm_state(state, 'Recolhendo o manipulador para transporte')

    def _transfer_state(self, description: str = 'Indo para detect_apriltags'):
        profile = self._profiles.pickup_profile('tabletop')
        self._arm_state(profile.observation_state, description)

    def _ensure_moveit(self) -> None:
        timeout = float(self.get_parameter('moveit_server_timeout_s').value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ConfigurationError('moveit_server_timeout_s deve ser positivo.')
        if not self._motion.cliente_do_move_group.wait_for_server(timeout_sec=timeout):
            raise ServerUnavailable(
                f"Action '{self.get_parameter('move_group_action').value}' indisponível."
            )

    def _record_effect(self, location: int) -> None:
        """Record a completed physical load transition for the action result."""
        self._effect_location = int(location)

    def _mark_effect_unknown(self) -> None:
        """Report that an interrupted gripper transition has an ambiguous effect."""
        self._effect_known = False
        self._effect_location = ManipulationResult.LOCATION_LOST

    def _make_result(
        self,
        action_type: Any,
        goal_handle: Any,
        code: int,
        message: str,
        tag_id: int,
        final_location: int | None = None,
        placed_pose: Any | None = None,
        failure: Exception | None = None,
        observed_detections: list[Any] | None = None,
    ) -> Any:
        result = action_type.Result()
        result.outcome.object_tag_id = int(tag_id)
        result.outcome.code = int(code)
        result.outcome.effect_known = getattr(self, '_effect_known', True)
        result.outcome.final_object_location = int(
            final_location
            if final_location is not None
            else getattr(
                self, '_effect_location', ManipulationResult.LOCATION_UNKNOWN
            )
        )
        result.outcome.message = message
        if placed_pose is not None and hasattr(result, 'placed_pose'):
            result.placed_pose = copy.deepcopy(placed_pose)
        if action_type is PickObject and isinstance(failure, PickRecoveryRequired):
            result.recovery_reason = failure.recovery_reason
            result.has_detected_pose = True
            result.detected_pose = copy.deepcopy(failure.detected_pose)
            result.moveit_error_code = failure.moveit_error_code
        if action_type is PickObject and observed_detections is not None:
            result.observed_detections = copy.deepcopy(observed_detections)
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
        observed_detections: list[Any] | None = None,
    ) -> Any:
        self._set_active(operation_name)
        self._effect_known = True
        self._effect_location = ManipulationResult.LOCATION_UNKNOWN
        monitorando_estados = False
        try:
            if requires_moveit:
                self._ensure_moveit()
                self._motion.iniciar_monitoramento_dos_estados()
                monitorando_estados = True
                if not self._motion.aguardar_primeiro_estado():
                    self.get_logger().warning(
                        'Nenhum /joint_states novo recebido antes da operação.'
                    )
            outcome = operation()
            message, location = outcome[:2]
            placed_pose = outcome[2] if len(outcome) == 3 else None
            return self._make_result(
                action_type, goal_handle, ManipulationResult.SUCCESS,
                message, tag_id, location, placed_pose,
                observed_detections=observed_detections,
            )
        except OperacaoCancelada as error:
            self._cancel_event.clear()
            return self._make_result(
                action_type, goal_handle, ManipulationResult.CANCELED,
                f'{error} O braço foi mantido na posição em que parou.', tag_id,
                observed_detections=observed_detections,
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
            return self._make_result(
                action_type, goal_handle, code, str(error), tag_id,
                failure=error, observed_detections=observed_detections,
            )
        finally:
            self._cancel_event.clear()
            if monitorando_estados:
                self._motion.parar_monitoramento_dos_estados()
            with self._lock:
                self._busy = False
                self._active_operation = ''

    def _execute_pick(self, goal_handle: Any) -> PickObject.Result:
        tag_id = int(goal_handle.request.tag_id)
        observed_detections: list[Any] = []

        def remember(detections: list[Any]) -> None:
            by_id = {int(item.id): item for item in observed_detections}
            by_id.update({int(item.id): item for item in detections})
            observed_detections[:] = [
                by_id[item_id] for item_id in sorted(by_id)
            ]

        def operation() -> tuple[str, int]:
            if tag_id < 0:
                raise ConfigurationError('tag_id não pode ser negativo.')
            profile = self._profiles.pickup_profile(goal_handle.request.profile)
            last_error: Exception | None = None
            for attempt in range(1, profile.attempts + 1):
                grasp_committed = False
                detected_pose = None
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
                        self.get_parameter('vision_analysis_duration_s').value
                    )
                    attempt_detections: list[Any] = []
                    try:
                        x, y, tag_z, yaw = self._motion.obter_pose_da_april_tag(
                            tag_id,
                            duration,
                            deteccoes_observadas=attempt_detections,
                        )
                    finally:
                        remember(attempt_detections)
                    detected_pose = criar_pose(x, y, tag_z, yaw)
                    if profile.reachability_filter_enabled:
                        reach_filter = self._pickup_reach_filter(profile)
                        if not reach_filter(x, y):
                            radius = math.hypot(
                                x - profile.reach_center_x_m,
                                y - profile.reach_center_y_m,
                            )
                            raise ObjectOutOfReach(
                                f'AprilTag {tag_id} detectada em x={x:.3f}, '
                                f'y={y:.3f} m (raio={radius:.3f} m), fora da '
                                'área de alcance configurada para a coleta.',
                                detected_pose,
                                PickObject.Result.RECOVERY_OUT_OF_REACH,
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
                        self._mark_effect_unknown()
                        raise
                    self._record_effect(ManipulationResult.LOCATION_GRIPPER)
                    grasp_committed = True
                    self._feedback(
                        goal_handle, PickObject, ManipulationFeedback.RETREATING,
                        0.82, 'Retirando o objeto da mesa',
                    )
                    self._motion.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pre_pegada(approach_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._transfer_state(
                        'Retornando para detect_apriltags após a coleta'
                    )
                    return (
                        f'Objeto {tag_id} coletado da mesa.',
                        ManipulationResult.LOCATION_GRIPPER,
                    )
                except OperacaoCancelada:
                    if grasp_committed:
                        self._mark_effect_unknown()
                    raise
                except ObjectOutOfReach:
                    raise
                except FalhaDoMoveIt as error:
                    if (
                        not grasp_committed
                        and detected_pose is not None
                        and error.error_code == 99999
                    ):
                        raise PickRecoveryRequired(
                            f'{error} A base pode ser reposicionada usando a '
                            'pose detectada da AprilTag.',
                            detected_pose,
                            PickObject.Result.RECOVERY_MOVEIT_UNREACHABLE,
                            error.error_code,
                        ) from error
                    last_error = error
                    if grasp_committed:
                        self._mark_effect_unknown()
                        raise
                    if attempt < profile.attempts:
                        self.get_logger().warning(
                            f'Tentativa de coleta {attempt} falhou: {error}'
                        )
                except Exception as error:
                    last_error = error
                    if grasp_committed:
                        self._mark_effect_unknown()
                        raise
                    if attempt < profile.attempts:
                        self.get_logger().warning(
                            f'Tentativa de coleta {attempt} falhou: {error}'
                        )
            assert last_error is not None
            if 'não encontrada' in str(last_error).lower():
                raise ObjectNotFound(str(last_error)) from last_error
            raise last_error

        return self._run(
            PickObject,
            goal_handle,
            'pick',
            tag_id,
            operation,
            observed_detections=observed_detections,
        )

    def _execute_store(self, goal_handle: Any) -> StoreObject.Result:
        slot_id = str(goal_handle.request.slot_id)
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            slot = self._profiles.cargo_slots.get(slot_id)
            if slot is None:
                raise ConfigurationError(f"Compartimento não configurado: '{slot_id}'.")
            self._transfer_state(
                'Garantindo detect_apriltags antes do compartimento'
            )
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
                self._mark_effect_unknown()
                raise
            self._record_effect(ManipulationResult.LOCATION_CARGO)
            self._transfer_state(
                'Retornando do compartimento para detect_apriltags'
            )
            return (
                f"Objeto {tag_id} armazenado em '{slot_id}'.",
                ManipulationResult.LOCATION_CARGO,
            )

        return self._run(StoreObject, goal_handle, 'store', tag_id, operation)

    def _execute_retrieve(self, goal_handle: Any) -> RetrieveObject.Result:
        slot_id = str(goal_handle.request.slot_id)
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            slot = self._profiles.cargo_slots.get(slot_id)
            if slot is None:
                raise ConfigurationError(f"Compartimento não configurado: '{slot_id}'.")
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.PREPARING,
                0.10, f"Indo para a pose segura do compartimento '{slot_id}'",
            )
            self._arm_state(slot.safe_state, 'Indo para a pose segura de retirada')
            self._feedback(
                goal_handle, RetrieveObject, ManipulationFeedback.PREPARING,
                0.25, 'Preparando a abertura da garra para retirar o objeto',
            )
            self._gripper('pre_grip', 'Posicionando a garra em pre_grip')
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
                self._mark_effect_unknown()
                raise
            self._record_effect(ManipulationResult.LOCATION_GRIPPER)
            try:
                self._feedback(
                    goal_handle, RetrieveObject, ManipulationFeedback.RETREATING,
                    0.80, 'Retornando à pose segura com o objeto',
                )
                self._arm_state(
                    slot.safe_state, 'Retornando à pose segura de retirada'
                )
            except Exception:
                self._mark_effect_unknown()
                raise
            return (
                f"Objeto {tag_id} retirado de '{slot_id}'.",
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
    def _reach_filter(
        *,
        x_min: float | None,
        x_max: float | None,
        y_min: float | None,
        y_max: float | None,
        center_x: float,
        center_y: float,
        min_radius: float | None,
        max_radius: float | None,
        context: str,
    ) -> Callable[[float, float], bool]:
        if None in (x_min, x_max, y_min, y_max):
            raise FeatureUnavailable(
                f'Preencha os limites XY de alcance em {context}.'
            )
        if min_radius is None or max_radius is None:
            raise FeatureUnavailable(
                f'Preencha reach_min_radius_m e reach_max_radius_m em {context}.'
            )
        if x_min > x_max or y_min > y_max:
            raise ConfigurationError(
                f'Os limites XY de alcance em {context} são inválidos.'
            )
        if min_radius >= max_radius:
            raise ConfigurationError(
                'reach_min_radius_m deve ser menor que reach_max_radius_m em '
                f'{context}.'
            )

        def contains(x: float, y: float) -> bool:
            radius = math.hypot(x - center_x, y - center_y)
            return (
                x_min - 1e-9 <= x <= x_max + 1e-9
                and y_min - 1e-9 <= y <= y_max + 1e-9
                and radius + 1e-9 >= min_radius
                and radius <= max_radius + 1e-9
            )

        return contains

    @staticmethod
    def _pickup_reach_filter(
        profile: PickupProfile,
    ) -> Callable[[float, float], bool]:
        """Build the independent RL intersection CP/CL filter for pickup."""
        return ManipulationServer._reach_filter(
            x_min=profile.reach_x_min_m,
            x_max=profile.reach_x_max_m,
            y_min=profile.reach_y_min_m,
            y_max=profile.reach_y_max_m,
            center_x=profile.reach_center_x_m,
            center_y=profile.reach_center_y_m,
            min_radius=profile.reach_min_radius_m,
            max_radius=profile.reach_max_radius_m,
            context=f'pickup.{profile.name}',
        )

    @staticmethod
    def _table_reach_filter(
        profile: PlacementProfile,
    ) -> Callable[[float, float], bool]:
        """Return the common RL intersection CP/CL membership test."""
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
        if (
            profile.reach_min_radius_m is None
            or profile.reach_max_radius_m is None
        ):
            raise FeatureUnavailable(
                'Preencha reach_min_radius_m e reach_max_radius_m no perfil '
                'table antes de analisar AprilTags.'
            )
        if (
            profile.search_y_max_m is not None
            and profile.search_y_max_m > -0.10
        ):
            raise ConfigurationError(
                'search_y_max_m não pode ser maior que -0.10 m.'
            )
        return ManipulationServer._reach_filter(
            x_min=profile.search_x_min_m,
            x_max=profile.search_x_max_m,
            y_min=profile.search_y_min_m,
            y_max=profile.search_y_max_m,
            center_x=profile.reach_center_x_m,
            center_y=profile.reach_center_y_m,
            min_radius=profile.reach_min_radius_m,
            max_radius=profile.reach_max_radius_m,
            context='placements.table',
        )

    @staticmethod
    def _table_search_candidates(
        profile: PlacementProfile,
    ) -> list[tuple[float, float]]:
        """Build a bounded grid ordered from the nominal release position."""
        reach_filter = ManipulationServer._table_reach_filter(profile)
        bounds = (
            profile.search_x_min_m,
            profile.search_x_max_m,
            profile.search_y_min_m,
            profile.search_y_max_m,
        )
        if profile.release_x_m is None or profile.release_y_m is None:
            raise FeatureUnavailable(
                'A posição nominal release_x_m/release_y_m não foi configurada.'
            )
        x_min, x_max, y_min, y_max = (float(value) for value in bounds)
        nominal_x = float(profile.release_x_m)
        nominal_y = float(profile.release_y_m)
        step = float(profile.search_step_m)
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
        candidates = []
        for x in xs:
            for y in ys:
                if reach_filter(x, y):
                    candidates.append((x, y))
        if not candidates:
            raise ConfigurationError(
                'A interseção entre a região retangular de busca e a faixa '
                'de alcance do braço não contém candidatos.'
            )
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
        preferred_distance_m: float,
    ) -> tuple[float, float]:
        """Prefer comfortable clearance, falling back to the safe minimum."""
        if preferred_distance_m < minimum_distance_m:
            raise ConfigurationError(
                'A distância preferencial deve ser maior ou igual à mínima.'
            )
        for required_distance_m in (preferred_distance_m, minimum_distance_m):
            for candidate_x, candidate_y in candidates:
                if all(
                    math.hypot(candidate_x - obstacle_x, candidate_y - obstacle_y)
                    + 1e-9 >= required_distance_m
                    for obstacle_x, obstacle_y in obstacles
                ):
                    return candidate_x, candidate_y
        raise NoFreeSpace(
            'Nenhuma posição da região de busca mantém a distância mínima de '
            f'{minimum_distance_m:.3f} m das AprilTags.'
        )

    @staticmethod
    def _rectangle(
        x: float, y: float, size_x: float, size_y: float, yaw: float = 0.0,
    ) -> list[tuple[float, float]]:
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return [
            (x + cosine*dx - sine*dy, y + sine*dx + cosine*dy)
            for dx, dy in (
                (-size_x/2, -size_y/2), (size_x/2, -size_y/2),
                (size_x/2, size_y/2), (-size_x/2, size_y/2),
            )
        ]

    @staticmethod
    def _polygons_overlap(
        first: list[tuple[float, float]], second: list[tuple[float, float]],
    ) -> bool:
        """Separating-axis test for convex footprints; touching is collision."""
        for polygon in (first, second):
            for index, point in enumerate(polygon):
                following = polygon[(index + 1) % len(polygon)]
                axis = (-(following[1] - point[1]), following[0] - point[0])
                norm = math.hypot(*axis)
                if norm <= 1e-12:
                    continue
                axis = (axis[0]/norm, axis[1]/norm)
                projected_a = [x*axis[0] + y*axis[1] for x, y in first]
                projected_b = [x*axis[0] + y*axis[1] for x, y in second]
                if max(projected_a) < min(projected_b) or max(projected_b) < min(projected_a):
                    return False
        return True

    @staticmethod
    def _yaw_from_pose(pose: Any) -> float:
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z),
        )

    @classmethod
    def _select_safe_table_position(
        cls,
        candidates: list[tuple[float, float]],
        tags: list[Any],
        containers: list[Any],
        profile: PlacementProfile,
        cube_size_m: float,
    ) -> tuple[float, float]:
        """Select a fully-contained footprint using oriented rectangles."""
        usable = (
            profile.usable_x_min_m, profile.usable_x_max_m,
            profile.usable_y_min_m, profile.usable_y_max_m)
        if any(value is None for value in usable):
            raise FeatureUnavailable(
                'Meça e configure usable_x_min_m, usable_x_max_m, '
                'usable_y_min_m e usable_y_max_m no frame do braço antes '
                'de depositar sobre a mesa.')
        bounds = tuple(map(float, usable))
        # The vertical approach has the same XY sweep as this conservative
        # union of the carried cube and the gripper envelope.
        footprint_x = max(cube_size_m, profile.gripper_footprint_x_m)
        footprint_y = max(cube_size_m, profile.gripper_footprint_y_m)
        clearance = profile.free_space_min_distance_m
        obstacles: list[list[tuple[float, float]]] = []
        tag_centers: list[tuple[float, float]] = []
        for tag in tags:
            x, y = float(tag.pose.position.x), float(tag.pose.position.y)
            tag_centers.append((x, y))
            inflation = clearance + profile.obstacle_uncertainty_m
            obstacles.append(cls._rectangle(
                x, y, cube_size_m + 2*inflation,
                cube_size_m + 2*inflation, cls._yaw_from_pose(tag.pose)))
        for container in containers:
            x, y = float(container.pose.position.x), float(container.pose.position.y)
            inflation = (clearance + profile.obstacle_uncertainty_m
                         + float(container.position_uncertainty_m))
            obstacles.append(cls._rectangle(
                x, y,
                float(container.external_dimensions_m.x) + 2*inflation,
                float(container.external_dimensions_m.y) + 2*inflation,
                cls._yaw_from_pose(container.pose)))
        for x, y in candidates:
            candidate = cls._rectangle(x, y, footprint_x, footprint_y)
            if any(not (bounds[0] <= px <= bounds[1] and bounds[2] <= py <= bounds[3])
                   for px, py in candidate):
                continue
            if any(math.hypot(x-tx, y-ty) + 1e-9 < clearance
                   for tx, ty in tag_centers):
                continue
            if not any(cls._polygons_overlap(candidate, obstacle)
                       for obstacle in obstacles):
                return x, y
        raise NoFreeSpace(
            'Nenhuma posição mantém o footprint completo do cubo/garra, a '
            'folga dos cubos e os retângulos orientados dos containers.')

    def _placement_profile(self, name: str, capability: str) -> PlacementProfile:
        profile = self._profiles.placements.get(name)
        if profile is None:
            raise ConfigurationError(f"Perfil de depósito ausente: '{name}'.")
        if not profile.enabled:
            raise FeatureUnavailable(
                f'{capability} ainda não está habilitado/calibrado.'
            )
        return profile

    @staticmethod
    def _select_container_detection(
        detections: list[Any], color: int, minimum_confidence: float,
        maximum_age_s: float, now_ns: int,
    ) -> Any:
        candidates = []
        for item in detections:
            stamp_ns = (int(item.header.stamp.sec)*1_000_000_000
                        + int(item.header.stamp.nanosec))
            age = (int(now_ns) - stamp_ns) * 1e-9
            if (int(item.color) == color and bool(item.pose_valid)
                    and not bool(item.partial)
                    and float(item.confidence) >= minimum_confidence
                    and 0.0 <= age <= maximum_age_s):
                candidates.append(item)
        if len(candidates) != 1:
            raise PerceptionUnavailable(
                'Esperado exatamente um container recente e seguro da cor '
                f'solicitada; encontrados {len(candidates)}.')
        return candidates[0]

    @staticmethod
    def _container_tcp_z(
        table_height_m: float, detection: Any, placement_offset_m: float,
    ) -> float:
        return (float(table_height_m)
                + float(detection.external_dimensions_m.z)
                + float(placement_offset_m))

    @staticmethod
    def _height_via_transform(height_m: float, transform: Any) -> float:
        """Convert a horizontal floor-referenced plane to the arm Z axis."""
        q = transform.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-9:
            raise PerceptionUnavailable('TF da altura da mesa tem rotação inválida.')
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        axis_x = 2.0*(x*z + y*w)
        axis_y = 2.0*(y*z - x*w)
        axis_z = 1.0 - 2.0*(x*x + y*y)
        if max(abs(axis_x), abs(axis_y)) > 0.001 or axis_z < 0.999:
            raise PerceptionUnavailable(
                'O piso e arm_base_link não são horizontais entre si; '
                'uma altura única não descreve o plano da mesa.')
        converted = float(transform.transform.translation.z) + height_m*axis_z
        if not math.isfinite(converted):
            raise PerceptionUnavailable('Altura convertida da mesa não é finita.')
        return converted

    def _table_height_in_arm_frame(self, floor_height_m: float) -> float:
        """WS heights from missions are above ground, not above the arm mount."""
        source = str(self.get_parameter('work_surface_height_frame').value)
        timeout = float(self.get_parameter('height_transform_timeout_s').value)
        if not source or not math.isfinite(timeout) or timeout <= 0.0:
            raise ConfigurationError('Configure o frame do piso e timeout TF positivo.')
        try:
            transform = self._tf_buffer.lookup_transform(
                REFERENCIAL_BASE, source, Time(), timeout=Duration(seconds=timeout))
        except TransformException as error:
            raise PerceptionUnavailable(
                f'TF {source} → {REFERENCIAL_BASE} indisponível: {error}') from error
        return self._height_via_transform(floor_height_m, transform)

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
            self._mark_effect_unknown()
            raise
        self._record_effect(ManipulationResult.LOCATION_DESTINATION)
        self._feedback(
            goal_handle, action_type, ManipulationFeedback.RETREATING,
            0.86, 'Recuando do destino',
        )
        self._motion.executar_objetivo(
            GRUPO_BRACO, restricoes_de_pre_pegada(retreat_pose),
            VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
        )
        self._transfer_state(
            'Preparando detect_apriltags após o depósito'
        )
        return (
            f'Objeto {tag_id} depositado: {destination}.',
            ManipulationResult.LOCATION_DESTINATION,
            release_pose,
        )

    def _execute_place_on_table(self, goal_handle: Any) -> PlaceOnTable.Result:
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int, Any]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            height_cm = float(goal_handle.request.ws_height_cm)
            if not math.isfinite(height_cm) or height_cm < 0.0:
                raise ConfigurationError('ws_height_cm deve ser finita e não negativa.')
            table_height_m = self._table_height_in_arm_frame(height_cm / 100.0)
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
            candidates = self._table_search_candidates(profile)
            observation = self._profiles.pickup_profile('tabletop').observation_state
            self._feedback(
                goal_handle, PlaceOnTable, ManipulationFeedback.OBSERVING,
                0.10, 'Analisando AprilTags e containers no mesmo frame de imagem',
            )
            self._arm_state(observation, 'Preparando câmera para depósito na mesa')
            duration = float(self.get_parameter('vision_analysis_duration_s').value)
            try:
                detections, containers = self._motion.analisar_cena(
                    duration, analisar_apriltags=True, analisar_containers=True,
                    altura_da_mesa_m=height_cm / 100.0)
            except OperacaoCancelada:
                raise
            except RuntimeError as error:
                raise PerceptionUnavailable(str(error)) from error
            tags = [item for item in detections if int(item.id) != tag_id]
            rejected_scene = (
                self._motion.ultimas_apriltags_rejeitadas
                + self._motion.ultimos_containers_rejeitados)
            uncertain = [
                item for item in rejected_scene
                if (not bool(item.pose_valid)
                    or item.header.frame_id != REFERENCIAL_BASE)
            ]
            if uncertain:
                raise PerceptionUnavailable(
                    'A cena contém detecções rejeitadas sem pose localizável; '
                    'o depósito cego foi bloqueado.')
            tags.extend([
                item for item in self._motion.ultimas_apriltags_rejeitadas
                if int(item.id) != tag_id and bool(item.pose_valid)
            ])
            containers.extend([
                item for item in self._motion.ultimos_containers_rejeitados
                if bool(item.pose_valid)
            ])
            cube_size = self._profiles.pickup_profile('tabletop').cube_size_m
            release_x_m, release_y_m = self._select_safe_table_position(
                candidates, tags, containers, profile, cube_size)
            self._feedback(
                goal_handle, PlaceOnTable, ManipulationFeedback.OBSERVING,
                0.30, f'Posição segura x={release_x_m:.3f}, y={release_y_m:.3f} m; '
                f'{len(tags)} cubo(s), {len(containers)} container(s)',
            )
            release_pose = criar_pose(
                float(release_x_m),
                float(release_y_m),
                table_height_m + float(tcp_offset_cm) / 100.0,
                float(release_yaw_deg),
            )
            return self._release_at_pose(
                goal_handle, PlaceOnTable, tag_id, release_pose, profile,
                f'mesa com altura de {height_cm:g} cm',
            )

        return self._run(
            PlaceOnTable, goal_handle, 'place_on_table', tag_id, operation,
        )

    def _execute_place_in_container(
        self, goal_handle: Any
    ) -> PlaceInContainer.Result:
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int, Any]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            color = int(goal_handle.request.container_color)
            colors = {
                PlaceInContainer.Goal.RED: 'vermelho',
                PlaceInContainer.Goal.BLUE: 'azul',
            }
            if color not in colors:
                raise ConfigurationError(f'Cor de contêiner inválida: {color}.')
            height_cm = float(goal_handle.request.ws_height_cm)
            if not math.isfinite(height_cm) or height_cm < 0.0:
                raise ConfigurationError('ws_height_cm deve ser finita e não negativa.')
            table_height_m = self._table_height_in_arm_frame(height_cm / 100.0)
            profile = self._placement_profile('container', 'Depósito em container')
            observation = self._profiles.pickup_profile('tabletop').observation_state
            self._feedback(
                goal_handle, PlaceInContainer, ManipulationFeedback.OBSERVING,
                0.10,
                f'Buscando contêiner {colors[color]} sobre WS de {height_cm:g} cm',
            )
            self._arm_state(observation, 'Preparando visão do container')
            duration = float(self.get_parameter('vision_analysis_duration_s').value)
            try:
                _, detected = self._motion.analisar_cena(
                    duration, analisar_apriltags=False, analisar_containers=True,
                    altura_da_mesa_m=height_cm / 100.0)
            except RuntimeError as error:
                raise PerceptionUnavailable(str(error)) from error
            now_ns = self.get_clock().now().nanoseconds
            selected = self._select_container_detection(
                detected, color, profile.minimum_detection_confidence,
                profile.maximum_detection_age_s, now_ns)
            target = criar_pose(
                float(selected.pose.position.x), float(selected.pose.position.y),
                self._container_tcp_z(
                    table_height_m, selected, profile.placement_offset_m),
                0.0,
            )
            constraints, path_constraints = restricoes_de_deposito_em_container(target)
            self._feedback(
                goal_handle, PlaceInContainer, ManipulationFeedback.APPROACHING,
                0.40, 'Planejando diretamente para o centro da abertura',
            )
            self._motion.planejar_validar_e_executar(
                GRUPO_BRACO, constraints, path_constraints,
                junta_validada='link4_to_link5',
                posicao_da_junta=math.pi/2.0,
                tolerancia_da_junta=math.pi/36.0,
                velocidade=VELOCIDADE_MAXIMA, aceleracao=ACELERACAO_MAXIMA,
            )
            self._feedback(
                goal_handle, PlaceInContainer, ManipulationFeedback.RELEASING,
                0.75, f'Abrindo a garra no container {colors[color]}',
            )
            try:
                self._gripper('open', 'Abrindo a garra no container')
            except Exception:
                self._mark_effect_unknown()
                raise
            self._record_effect(ManipulationResult.LOCATION_DESTINATION)
            self._transfer_state('Retornando diretamente para detect_apriltags')
            return (
                f'Objeto {tag_id} depositado no container {colors[color]}.',
                ManipulationResult.LOCATION_DESTINATION, target,
            )

        return self._run(
            PlaceInContainer, goal_handle, 'place_in_container', tag_id, operation,
        )

    def _execute_stack(self, goal_handle: Any) -> StackObject.Result:
        tag_id = int(goal_handle.request.object_tag_id)
        support_tag_id = int(goal_handle.request.support_tag_id)

        def operation() -> tuple[str, int, Any]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            if support_tag_id < 0 or support_tag_id == tag_id:
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
            duration = float(self.get_parameter('vision_analysis_duration_s').value)
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
                goal_handle, StackObject, tag_id, release_pose, profile,
                f'empilhamento sobre o objeto {support_tag_id}',
            )

        return self._run(StackObject, goal_handle, 'stack', tag_id, operation)

    def _execute_place_on_shelf(
        self, goal_handle: Any
    ) -> PlaceOnShelf.Result:
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
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
                self._mark_effect_unknown()
                raise
            self._record_effect(ManipulationResult.LOCATION_DESTINATION)
            self._safe(False)
            return (
                f'Objeto {tag_id} depositado na prateleira.',
                ManipulationResult.LOCATION_DESTINATION,
            )

        return self._run(
            PlaceOnShelf, goal_handle, 'place_on_shelf', tag_id, operation
        )

    def _execute_place_at_pose(self, goal_handle: Any) -> PlaceAtPose.Result:
        tag_id = int(goal_handle.request.object_tag_id)

        def operation() -> tuple[str, int, Any]:
            if tag_id < 0:
                raise ConfigurationError('object_tag_id não pode ser negativo.')
            release_pose = copy.deepcopy(goal_handle.request.release_pose)
            self._validate_target_pose(release_pose)
            profile = self._placement_profile('explicit_pose', 'Depósito em pose')
            return self._release_at_pose(
                goal_handle, PlaceAtPose, tag_id, release_pose, profile,
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
                self._safe(bool(goal_handle.request.gripper_loaded))
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
        self._motion.parar_monitoramento_dos_estados()
        for server in self._servers:
            server.destroy()
        return super().destroy_node()


def _spin_executor(executor: Any, node: Node) -> None:
    """Keep spinning when an entity is removed from a concurrent wait set."""
    while rclpy.ok():
        try:
            executor.spin_once()
        except InvalidHandle as error:
            if not rclpy.ok():
                break
            node.get_logger().debug(
                'Entidade ROS removida durante a atualização do executor: '
                f'{error}'
            )


def main(args=None) -> int:
    """Run the manipulation server in an executor that can service child actions."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    exit_code = 0
    try:
        node = ManipulationServer()
        executor.add_node(node)
        _spin_executor(executor, node)
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
