"""Single-process action server for all SO-101 cube manipulation skills."""

from __future__ import annotations

import threading
from typing import Any, Callable

import rclpy
from interfaces.action import PickCube, PlaceCube, RetrieveCube, StoreCube
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .configuracao import (
    ACELERACAO_MAXIMA,
    ACELERACAO_MAXIMA_DA_GARRA,
    ALTURA_DE_APROXIMACAO,
    ESTADOS_DOS_GRUPOS,
    GRUPO_BRACO,
    GRUPO_GARRA,
    TAMANHO_DO_CUBO,
    TEMPO_DE_ANALISE_DA_APRIL_TAG,
    TOLERANCIA_DA_JUNTA_DA_GARRA,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_MAXIMA,
    VELOCIDADE_MAXIMA_DA_GARRA,
)
from .movimento import ExecutorDoMoveIt, OperacaoCancelada
from .restricoes import (
    criar_pose,
    normalizar_angulo_de_pegada,
    restricoes_de_pegada,
    restricoes_de_pre_pegada,
)


SUCCESS, NOT_FOUND, MOTION_FAILED, CANCELED, BUSY, CONFIGURATION_ERROR = range(6)


class CargoIncerto(RuntimeError):
    """The physical command may have changed where the cube is."""


class ManipulationServer(Node):
    def __init__(self) -> None:
        super().__init__("manipulation_server")
        defaults = {
            "left_store_state": "deposit_cube_left",
            "left_retrieve_state": "pick_cube_left",
            # Empty defaults deliberately prevent unsafe guessed joint values.
            "right_store_state": "",
            "right_retrieve_state": "",
            "transport_empty_state": "transport_empty",
            "transport_loaded_state": "transport_loaded",
            "pick_attempts": 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._group = ReentrantCallbackGroup()
        self._busy = False
        self._cancel_requested = False
        self._cargo_loaded = False
        self._lock = threading.Lock()
        self.executor_moveit = ExecutorDoMoveIt(
            self,
            cancelamento_solicitado=lambda: self._cancel_requested,
            callback_group=self._group,
        )
        common = dict(
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._servers = [
            ActionServer(self, PickCube, "/manipulation/pick_cube",
                         execute_callback=self._execute_pick, **common),
            ActionServer(self, StoreCube, "/manipulation/store_cube",
                         execute_callback=self._execute_store, **common),
            ActionServer(self, RetrieveCube, "/manipulation/retrieve_cube",
                         execute_callback=self._execute_retrieve, **common),
            ActionServer(self, PlaceCube, "/manipulation/place_cube",
                         execute_callback=self._execute_place, **common),
        ]
        self.get_logger().info("Servidor de manipulação pronto; ações serializadas.")

    def _goal(self, _request: Any) -> GoalResponse:
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
            self._cancel_requested = False
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_requested = True
        self.executor_moveit.cancelar_objetivo_ativo()
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle: Any, action_type: Any,
                  phase: str, progress: float) -> None:
        feedback = action_type.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(phase)

    def _arm_state(self, state: str, description: str) -> None:
        if not state or state not in ESTADOS_DOS_GRUPOS.get(GRUPO_BRACO, {}):
            raise ValueError(f"Estado MoveIt do braço não configurado: '{state}'.")
        self.executor_moveit.mover_para_estado(
            GRUPO_BRACO, state, description,
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )

    def _gripper(self, state: str, description: str) -> None:
        self.executor_moveit.mover_para_estado(
            GRUPO_GARRA, state, description,
            tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
        )

    def _safe(self, loaded: bool) -> None:
        name = "transport_loaded_state" if loaded else "transport_empty_state"
        self._arm_state(str(self.get_parameter(name).value), "Recolhendo o braço para transporte")

    def _slot_state(self, slot: int, operation: str) -> str:
        if slot == StoreCube.Goal.LEFT:
            side = "left"
        elif slot == StoreCube.Goal.RIGHT:
            side = "right"
        else:
            raise ValueError(f"Compartimento inválido: {slot}.")
        return str(self.get_parameter(f"{side}_{operation}_state").value)

    def _result(self, action_type: Any, goal_handle: Any, *, code: int,
                known: bool, message: str) -> Any:
        result = action_type.Result()
        result.code = code
        result.cargo_state_known = known
        result.message = message
        if code == SUCCESS:
            goal_handle.succeed()
        elif code == CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _run(self, action_type: Any, goal_handle: Any,
             operation: Callable[[], tuple[bool, str]]) -> Any:
        known = True
        try:
            known, message = operation()
            return self._result(action_type, goal_handle, code=SUCCESS,
                                known=known, message=message)
        except OperacaoCancelada as error:
            self._cancel_requested = False
            try:
                self._safe(loaded=self._cargo_loaded)
            except Exception as safe_error:
                self.get_logger().error(f"Recolhimento após cancelamento falhou: {safe_error}")
            return self._result(action_type, goal_handle, code=CANCELED,
                                known=known, message=str(error))
        except CargoIncerto as error:
            return self._result(action_type, goal_handle, code=MOTION_FAILED,
                                known=False, message=str(error))
        except ValueError as error:
            return self._result(action_type, goal_handle, code=CONFIGURATION_ERROR,
                                known=known, message=str(error))
        except Exception as error:
            code = NOT_FOUND if action_type is PickCube and "não encontrada" in str(error) else MOTION_FAILED
            self.get_logger().error(str(error))
            return self._result(action_type, goal_handle, code=code,
                                known=known, message=str(error))
        finally:
            self._cancel_requested = False
            with self._lock:
                self._busy = False

    def _execute_pick(self, goal_handle: Any) -> PickCube.Result:
        self._cargo_loaded = False

        def operation() -> tuple[bool, str]:
            attempts = int(self.get_parameter("pick_attempts").value)
            last_error: Exception | None = None
            for attempt in range(1, attempts + 1):
                closed = False
                try:
                    self._feedback(goal_handle, PickCube, "Abrindo a garra", 0.05)
                    self._gripper("open", "Abrindo a garra para a coleta")
                    self._feedback(goal_handle, PickCube, "Localizando AprilTag", 0.15)
                    self._arm_state("detect_apriltags", "Posicionando a câmera")
                    x, y, z, yaw = self.executor_moveit.obter_pose_da_april_tag(
                        goal_handle.request.tag_id, TEMPO_DE_ANALISE_DA_APRIL_TAG
                    )
                    z -= TAMANHO_DO_CUBO
                    grasp_yaw = normalizar_angulo_de_pegada(yaw) + 90.0
                    pick_pose = criar_pose(x, y, z, grasp_yaw)
                    above_pose = criar_pose(x, y, z + ALTURA_DE_APROXIMACAO, grasp_yaw)
                    self._feedback(goal_handle, PickCube, "Aproximando do cubo", 0.40)
                    self.executor_moveit.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pre_pegada(above_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self.executor_moveit.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pegada(pick_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._feedback(goal_handle, PickCube, "Fechando a garra", 0.70)
                    self._gripper("grip", "Fechando a garra no cubo")
                    closed = True
                    self._cargo_loaded = True
                    self.executor_moveit.executar_objetivo(
                        GRUPO_BRACO, restricoes_de_pre_pegada(above_pose),
                        VELOCIDADE_MAXIMA, ACELERACAO_MAXIMA,
                    )
                    self._safe(loaded=True)
                    self._feedback(goal_handle, PickCube, "Coleta concluída", 1.0)
                    return True, f"Tag {goal_handle.request.tag_id} coletada."
                except OperacaoCancelada:
                    raise
                except Exception as error:
                    last_error = error
                    if closed:
                        raise CargoIncerto(
                            f"Falha após fechar a garra; estado da carga incerto: {error}"
                        ) from error
                    if attempt < attempts:
                        self.get_logger().warning(
                            f"Tentativa {attempt}/{attempts} falhou: {error}; repetindo."
                        )
            assert last_error is not None
            raise last_error
        return self._run(PickCube, goal_handle, operation)

    def _execute_store(self, goal_handle: Any) -> StoreCube.Result:
        self._cargo_loaded = True

        def operation() -> tuple[bool, str]:
            state = self._slot_state(goal_handle.request.slot, "store")
            self._feedback(goal_handle, StoreCube, "Movendo para o compartimento", 0.25)
            self._arm_state(state, "Levando o cubo ao compartimento")
            self._feedback(goal_handle, StoreCube, "Soltando no compartimento", 0.65)
            try:
                self._gripper("open", "Soltando o cubo no compartimento")
                self._cargo_loaded = False
            except Exception as error:
                raise CargoIncerto(f"Falha ao soltar no compartimento: {error}") from error
            self._safe(loaded=False)
            self._feedback(goal_handle, StoreCube, "Armazenamento concluído", 1.0)
            return True, f"Tag {goal_handle.request.tag_id} armazenada."
        return self._run(StoreCube, goal_handle, operation)

    def _execute_retrieve(self, goal_handle: Any) -> RetrieveCube.Result:
        self._cargo_loaded = False

        def operation() -> tuple[bool, str]:
            state = self._slot_state(goal_handle.request.slot, "retrieve")
            self._gripper("open", "Abrindo a garra para retirar o cubo")
            self._feedback(goal_handle, RetrieveCube, "Movendo ao compartimento", 0.30)
            self._arm_state(state, "Posicionando para retirar o cubo")
            self._feedback(goal_handle, RetrieveCube, "Fechando a garra", 0.65)
            try:
                self._gripper("grip", "Fechando a garra no cubo armazenado")
                self._cargo_loaded = True
            except Exception as error:
                raise CargoIncerto(f"Falha ao agarrar cubo armazenado: {error}") from error
            self._safe(loaded=True)
            self._feedback(goal_handle, RetrieveCube, "Retirada concluída", 1.0)
            return True, f"Tag {goal_handle.request.tag_id} retirada."
        return self._run(RetrieveCube, goal_handle, operation)

    def _execute_place(self, goal_handle: Any) -> PlaceCube.Result:
        self._cargo_loaded = True

        def operation() -> tuple[bool, str]:
            self._feedback(goal_handle, PlaceCube, "Movendo para o depósito", 0.30)
            self._arm_state(goal_handle.request.placement, "Posicionando para depositar o cubo")
            self._feedback(goal_handle, PlaceCube, "Soltando o cubo", 0.70)
            try:
                self._gripper("open", "Soltando o cubo no destino")
                self._cargo_loaded = False
            except Exception as error:
                raise CargoIncerto(f"Falha ao soltar no destino: {error}") from error
            self._safe(loaded=False)
            self._feedback(goal_handle, PlaceCube, "Depósito concluído", 1.0)
            return True, f"Tag {goal_handle.request.tag_id} depositada."
        return self._run(PlaceCube, goal_handle, operation)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulationServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.executor_moveit.destruir()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
