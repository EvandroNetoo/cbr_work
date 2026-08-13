"""Objetos do ambiente e contatos permitidos na Planning Scene do MoveIt."""

from __future__ import annotations

import copy
import math
import re
from collections.abc import Sequence

import rclpy
from cbr_interfaces.msg import AprilTagStampedDetection
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .configuracao import REFERENCIAL_BASE, TAMANHO_DO_CUBO


PREFIXO_DOS_CUBOS = "cubo_apriltag_"
ID_DO_CUBO_MANUAL = "cubo_configurado"
LINKS_DE_CONTATO_DA_GARRA = ("clamp_1", "clamp_2")


class CenaDePlanejamento:
    """Aplica mudanças síncronas à cena monitorada pelo ``move_group``."""

    def __init__(self, no: Node) -> None:
        self.no = no
        self.cliente_aplicar = no.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.cliente_obter = no.create_client(GetPlanningScene, "/get_planning_scene")
        self._objetos: dict[str, CollisionObject] = {}

    def aguardar_servicos(self, timeout: float = 15.0) -> None:
        if not self.cliente_aplicar.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Serviço /apply_planning_scene não encontrado.")
        if not self.cliente_obter.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Serviço /get_planning_scene não encontrado.")

    @staticmethod
    def id_do_cubo(deteccao: AprilTagStampedDetection) -> str:
        familia = re.sub(r"[^A-Za-z0-9_-]+", "_", deteccao.family).strip("_")
        if not familia:
            raise ValueError("A família da AprilTag não pode ser vazia.")
        if deteccao.id < 0:
            raise ValueError("O ID da AprilTag não pode ser negativo.")
        return f"{PREFIXO_DOS_CUBOS}{familia}_{deteccao.id}"

    @staticmethod
    def criar_cubo_da_deteccao(
        deteccao: AprilTagStampedDetection,
    ) -> CollisionObject:
        if deteccao.header.frame_id != REFERENCIAL_BASE:
            raise ValueError(
                f"AprilTag {deteccao.id} está em '{deteccao.header.frame_id}', "
                f"não em '{REFERENCIAL_BASE}'."
            )
        posicao = deteccao.pose.position
        valores = (posicao.x, posicao.y, posicao.z)
        if not all(math.isfinite(float(valor)) for valor in valores):
            raise ValueError(f"A posição da AprilTag {deteccao.id} não é finita.")

        yaw = CenaDePlanejamento._yaw(deteccao.pose)
        objeto = CenaDePlanejamento._criar_cubo(
            CenaDePlanejamento.id_do_cubo(deteccao),
            float(posicao.x),
            float(posicao.y),
            float(posicao.z) - TAMANHO_DO_CUBO / 2.0,
            yaw,
        )
        objeto.header.stamp = deteccao.header.stamp
        return objeto

    @staticmethod
    def criar_cubo_manual(
        x: float, y: float, z_da_pegada: float, yaw: float
    ) -> CollisionObject:
        """Cria o cubo quando XYZ descreve a pose do TCP na pegada."""
        return CenaDePlanejamento._criar_cubo(
            ID_DO_CUBO_MANUAL,
            x,
            y,
            z_da_pegada + TAMANHO_DO_CUBO / 2.0,
            math.radians(yaw),
        )

    @staticmethod
    def _criar_cubo(
        identificador: str, x: float, y: float, z: float, yaw: float
    ) -> CollisionObject:
        valores = (x, y, z, yaw, TAMANHO_DO_CUBO)
        if not all(math.isfinite(float(valor)) for valor in valores):
            raise ValueError("A pose e as dimensões do cubo devem ser finitas.")
        if TAMANHO_DO_CUBO <= 0.0:
            raise ValueError("O tamanho do cubo deve ser positivo.")

        objeto = CollisionObject()
        objeto.header.frame_id = REFERENCIAL_BASE
        objeto.id = identificador
        objeto.operation = CollisionObject.ADD
        objeto.pose.position.x = float(x)
        objeto.pose.position.y = float(y)
        objeto.pose.position.z = float(z)
        objeto.pose.orientation.z = math.sin(yaw / 2.0)
        objeto.pose.orientation.w = math.cos(yaw / 2.0)

        caixa = SolidPrimitive()
        caixa.type = SolidPrimitive.BOX
        caixa.dimensions = [TAMANHO_DO_CUBO] * 3
        objeto.primitives = [caixa]
        pose_local = Pose()
        pose_local.orientation.w = 1.0
        objeto.primitive_poses = [pose_local]
        return objeto

    @staticmethod
    def _yaw(pose: Pose) -> float:
        q = pose.orientation
        valores = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(valor) for valor in valores):
            raise ValueError("A orientação da AprilTag não é finita.")
        norma = math.sqrt(sum(valor * valor for valor in valores))
        if norma == 0.0:
            raise ValueError("A orientação da AprilTag possui quaternion nulo.")
        x, y, z, w = (valor / norma for valor in valores)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def sincronizar_cubos(
        self, deteccoes: Sequence[AprilTagStampedDetection]
    ) -> dict[int, str]:
        objetos = [self.criar_cubo_da_deteccao(item) for item in deteccoes]
        ids_por_tag = {item.id: objeto.id for item, objeto in zip(deteccoes, objetos)}
        self._sincronizar_objetos(objetos)
        return ids_por_tag

    def sincronizar_cubo_manual(
        self, x: float, y: float, z_da_pegada: float, yaw: float
    ) -> str:
        objeto = self.criar_cubo_manual(x, y, z_da_pegada, yaw)
        self._sincronizar_objetos([objeto])
        return objeto.id

    def verificar_sem_cubos_anexados(self) -> None:
        """Impede uma nova tarefa enquanto uma recuperação estiver pendente."""
        cena = self._obter_cena(
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        anexados = sorted(
            item.object.id
            for item in cena.robot_state.attached_collision_objects
            if self._id_gerenciado(item.object.id)
        )
        if anexados:
            raise RuntimeError(
                "Há cubo(s) ainda anexado(s) na Planning Scene: "
                f"{anexados}. Faça a recuperação operacional antes de iniciar "
                "outro pick and place."
            )

    def _sincronizar_objetos(self, objetos: Sequence[CollisionObject]) -> None:
        if len({objeto.id for objeto in objetos}) != len(objetos):
            raise ValueError("A fotografia contém IDs de cubo duplicados.")

        cena_atual = self._obter_cena(PlanningSceneComponents.WORLD_OBJECT_NAMES)
        ids_novos = {objeto.id for objeto in objetos}
        ids_antigos = {
            objeto.id
            for objeto in cena_atual.world.collision_objects
            if self._id_gerenciado(objeto.id)
        }

        cena = self._nova_cena_diff()
        for identificador in sorted(ids_antigos - ids_novos):
            remocao = CollisionObject()
            remocao.header.frame_id = REFERENCIAL_BASE
            remocao.id = identificador
            remocao.operation = CollisionObject.REMOVE
            cena.world.collision_objects.append(remocao)
        cena.world.collision_objects.extend(copy.deepcopy(list(objetos)))
        self._aplicar_cena(cena, "sincronizar os cubos detectados")
        self._objetos = {objeto.id: copy.deepcopy(objeto) for objeto in objetos}
        self.no.get_logger().info(
            f"Planning Scene atualizada com {len(objetos)} cubo(s)."
        )

    @staticmethod
    def _id_gerenciado(identificador: str) -> bool:
        return (
            identificador.startswith(PREFIXO_DOS_CUBOS)
            or identificador == ID_DO_CUBO_MANUAL
        )

    def permitir_contato_com_a_garra(self, id_do_objeto: str) -> AllowedCollisionMatrix:
        cena_atual = self._obter_cena(
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        original = copy.deepcopy(cena_atual.allowed_collision_matrix)
        alterada = copy.deepcopy(original)
        for nome in (id_do_objeto, *LINKS_DE_CONTATO_DA_GARRA):
            self._garantir_entrada(alterada, nome)
        for link in LINKS_DE_CONTATO_DA_GARRA:
            self._definir_par(alterada, id_do_objeto, link, True)

        cena = self._nova_cena_diff()
        cena.allowed_collision_matrix = alterada
        self._aplicar_cena(cena, f"permitir contato da garra com '{id_do_objeto}'")
        return original

    def restaurar_matriz_de_colisoes(self, matriz: AllowedCollisionMatrix) -> None:
        cena = self._nova_cena_diff()
        cena.allowed_collision_matrix = copy.deepcopy(matriz)
        self._aplicar_cena(cena, "restaurar a matriz de colisões")

    @staticmethod
    def _garantir_entrada(matriz: AllowedCollisionMatrix, nome: str) -> None:
        quantidade = len(matriz.entry_names)
        if len(matriz.entry_values) != quantidade or any(
            len(linha.enabled) != quantidade for linha in matriz.entry_values
        ):
            raise RuntimeError("A matriz de colisões recebida do MoveIt é inválida.")
        if nome in matriz.entry_names:
            return
        matriz.entry_names.append(nome)
        for linha in matriz.entry_values:
            linha.enabled.append(False)
        nova_linha = AllowedCollisionEntry()
        nova_linha.enabled = [False] * (quantidade + 1)
        matriz.entry_values.append(nova_linha)

    @staticmethod
    def _definir_par(
        matriz: AllowedCollisionMatrix, primeiro: str, segundo: str, valor: bool
    ) -> None:
        indice_a = matriz.entry_names.index(primeiro)
        indice_b = matriz.entry_names.index(segundo)
        matriz.entry_values[indice_a].enabled[indice_b] = valor
        matriz.entry_values[indice_b].enabled[indice_a] = valor

    def ignorar_cubo(self, identificador: str) -> None:
        """Remove apenas o cubo-alvo da checagem de colisão do MoveIt."""
        if identificador not in self._objetos:
            raise ValueError(
                f"Cubo '{identificador}' não pertence à fotografia atual."
            )
        remocao = CollisionObject()
        remocao.header.frame_id = REFERENCIAL_BASE
        remocao.id = identificador
        remocao.operation = CollisionObject.REMOVE
        cena = self._nova_cena_diff()
        cena.world.collision_objects = [remocao]
        self._aplicar_cena(cena, f"ignorar colisões do cubo '{identificador}'")
        del self._objetos[identificador]

    @staticmethod
    def _nova_cena_diff() -> PlanningScene:
        cena = PlanningScene()
        cena.is_diff = True
        cena.robot_state.is_diff = True
        return cena

    def _obter_cena(self, componentes: int) -> PlanningScene:
        requisicao = GetPlanningScene.Request()
        requisicao.components.components = componentes
        futuro = self.cliente_obter.call_async(requisicao)
        rclpy.spin_until_future_complete(self.no, futuro, timeout_sec=5.0)
        if not futuro.done() or futuro.exception() is not None:
            detalhe = futuro.exception() if futuro.done() else "tempo limite excedido"
            raise RuntimeError(
                f"Não foi possível consultar a Planning Scene: {detalhe}."
            )
        resposta = futuro.result()
        if resposta is None:
            raise RuntimeError("O MoveIt não devolveu a Planning Scene.")
        return resposta.scene

    def _aplicar_cena(self, cena: PlanningScene, operacao: str) -> None:
        requisicao = ApplyPlanningScene.Request()
        requisicao.scene = cena
        futuro = self.cliente_aplicar.call_async(requisicao)
        rclpy.spin_until_future_complete(self.no, futuro, timeout_sec=5.0)
        if not futuro.done() or futuro.exception() is not None:
            detalhe = futuro.exception() if futuro.done() else "tempo limite excedido"
            raise RuntimeError(f"Falha ao {operacao}: {detalhe}.")
        resposta = futuro.result()
        if resposta is None or not resposta.success:
            raise RuntimeError(f"O MoveIt recusou a operação de {operacao}.")
