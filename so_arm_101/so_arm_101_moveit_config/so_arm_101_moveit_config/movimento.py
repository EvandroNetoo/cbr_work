"""Execução de objetivos e movimentos através do MoveIt 2."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence

import rclpy
from action_msgs.msg import GoalStatus
from cbr_interfaces.action import AnalyzeAprilTags
from cbr_interfaces.msg import AprilTagStampedDetection
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from .configuracao import (
    ACELERACAO_MAXIMA,
    AMOSTRAS_ESTAVEIS_NECESSARIAS,
    ESTADOS_DOS_GRUPOS,
    GRUPO_GARRA,
    REFERENCIAL_BASE,
    TENTATIVAS_DE_PLANEJAMENTO,
    TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR,
    TEMPO_DE_PLANEJAMENTO,
    TEMPO_DE_PLANEJAMENTO_ARTICULAR,
    TEMPO_LIMITE_DE_ASSENTAMENTO,
    TOLERANCIA_DE_ASSENTAMENTO_DA_GARRA,
    TOLERANCIA_DE_ASSENTAMENTO_DO_BRACO,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_DE_ASSENTAMENTO_DA_GARRA,
    VELOCIDADE_DE_ASSENTAMENTO_DO_BRACO,
    VELOCIDADE_MAXIMA,
    PosicoesJuntas,
)
from .restricoes import ListaDeRestricoes, restricoes_de_posicao_inicial


class ExecutorDoMoveIt:
    """Encapsula a comunicação com o servidor de ação do MoveIt."""

    def __init__(self) -> None:
        self.no = rclpy.create_node("pegar_e_colocar")
        self.cliente_do_move_group = ActionClient(self.no, MoveGroup, "/move_action")
        self.cliente_da_april_tag = ActionClient(
            self.no, AnalyzeAprilTags, "/apriltags/analyze"
        )
        self.posicoes_juntas_atuais: dict[str, float] = {}
        self.velocidades_juntas_atuais: dict[str, float] = {}
        self.sequencia_dos_estados_das_juntas = 0
        self.inscricao_nos_estados_das_juntas = self.no.create_subscription(
            JointState, "/joint_states", self._receber_estados_das_juntas, 10
        )

    def _receber_estados_das_juntas(self, mensagem: JointState) -> None:
        self.posicoes_juntas_atuais.update(zip(mensagem.name, mensagem.position))
        self.velocidades_juntas_atuais = dict(zip(mensagem.name, mensagem.velocity))
        self.sequencia_dos_estados_das_juntas += 1

    def aguardar_o_servidor(self) -> None:
        self.no.get_logger().info("Aguardando o servidor de planejamento do MoveIt...")
        if not self.cliente_do_move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "Servidor /move_action não encontrado. Inicie o real_planning.launch.py na Banana Pi."
            )

    def obter_pose_da_april_tag(
        self, tag_id: int, duracao_da_analise: float
    ) -> tuple[float, float, float, float]:
        """Devolve posição e yaw da tag no referencial da base."""
        if tag_id < 0:
            raise ValueError("O ID da AprilTag não pode ser negativo.")
        if duracao_da_analise <= 0.0:
            raise ValueError("A duração da análise da AprilTag deve ser positiva.")

        self.no.get_logger().info(
            f"Aguardando /apriltags/analyze para localizar a AprilTag {tag_id}..."
        )
        if not self.cliente_da_april_tag.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "A ação /apriltags/analyze não está disponível. "
                "Inicie o detector AprilTag antes da sequência."
            )

        objetivo = AnalyzeAprilTags.Goal()
        nanossegundos_totais = round(duracao_da_analise * 1_000_000_000)
        segundos, nanossegundos = divmod(nanossegundos_totais, 1_000_000_000)
        objetivo.duration.sec = segundos
        objetivo.duration.nanosec = nanossegundos

        futuro_do_envio = self.cliente_da_april_tag.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self.no, futuro_do_envio, timeout_sec=5.0)
        if not futuro_do_envio.done():
            raise RuntimeError("O detector não respondeu ao pedido de análise.")
        manipulador_do_objetivo = futuro_do_envio.result()
        if manipulador_do_objetivo is None or not manipulador_do_objetivo.accepted:
            raise RuntimeError("O detector rejeitou o pedido de análise de AprilTags.")

        futuro_do_resultado = manipulador_do_objetivo.get_result_async()
        rclpy.spin_until_future_complete(
            self.no, futuro_do_resultado, timeout_sec=duracao_da_analise + 5.0
        )
        if not futuro_do_resultado.done():
            manipulador_do_objetivo.cancel_goal_async()
            raise RuntimeError("O detector excedeu o tempo limite da análise.")
        resultado_da_acao = futuro_do_resultado.result()
        if (
            resultado_da_acao is None
            or resultado_da_acao.status != GoalStatus.STATUS_SUCCEEDED
        ):
            estado = (
                resultado_da_acao.status
                if resultado_da_acao is not None
                else "sem resultado"
            )
            detalhe = (
                resultado_da_acao.result.message
                if resultado_da_acao is not None
                and resultado_da_acao.result is not None
                else "sem detalhes"
            )
            raise RuntimeError(
                f"A análise de AprilTags falhou (estado {estado}): {detalhe}"
            )

        deteccao = self._selecionar_april_tag(
            resultado_da_acao.result.best_detections_base, tag_id
        )
        if deteccao is None:
            ids_encontrados = sorted(
                {item.id for item in resultado_da_acao.result.best_detections_base}
            )
            raise RuntimeError(
                f"AprilTag {tag_id} não encontrada em base_link durante "
                f"{duracao_da_analise:.1f}s. IDs encontrados: {ids_encontrados}. "
                "Confirme a visibilidade da tag e o TF da câmera."
            )

        if deteccao.header.frame_id != REFERENCIAL_BASE:
            raise RuntimeError(
                f"A AprilTag {tag_id} foi retornada em "
                f"'{deteccao.header.frame_id}', não em '{REFERENCIAL_BASE}'."
            )

        posicao = deteccao.pose.position
        orientacao = deteccao.pose.orientation
        xyz = (float(posicao.x), float(posicao.y), float(posicao.z))
        yaw_em_graus = self._yaw_em_graus(
            float(orientacao.x),
            float(orientacao.y),
            float(orientacao.z),
            float(orientacao.w),
        )
        self.no.get_logger().info(
            f"AprilTag {tag_id} localizada em {REFERENCIAL_BASE}: "
            f"x={xyz[0]:.4f}, y={xyz[1]:.4f}, z={xyz[2]:.4f} m, "
            f"yaw={yaw_em_graus:.1f}°."
        )
        return (*xyz, yaw_em_graus)

    def obter_xyz_da_april_tag(
        self, tag_id: int, duracao_da_analise: float
    ) -> tuple[float, float, float]:
        """Mantém a interface anterior para consumidores que usam somente XYZ."""
        x, y, z, _ = self.obter_pose_da_april_tag(tag_id, duracao_da_analise)
        return x, y, z

    @staticmethod
    def _yaw_em_graus(x: float, y: float, z: float, w: float) -> float:
        """Extrai o yaw de um quaternion, normalizando pequenas imprecisões."""
        norma = math.sqrt(x * x + y * y + z * z + w * w)
        if norma == 0.0:
            raise ValueError("A orientação da AprilTag possui quaternion nulo.")
        x, y, z, w = x / norma, y / norma, z / norma, w / norma
        seno = 2.0 * (w * z + x * y)
        cosseno = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(seno, cosseno))

    @staticmethod
    def _selecionar_april_tag(
        deteccoes: Sequence[AprilTagStampedDetection], tag_id: int
    ) -> AprilTagStampedDetection | None:
        return next((item for item in deteccoes if item.id == tag_id), None)

    def mover_para_estado(
        self,
        grupo: str,
        nome_do_estado: str,
        descricao: str,
        tolerancia: float = TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
        velocidade: float = VELOCIDADE_MAXIMA,
        aceleracao: float = ACELERACAO_MAXIMA,
    ) -> None:
        """Move um grupo para um estado nomeado definido no SRDF."""
        try:
            posicoes_das_juntas = ESTADOS_DOS_GRUPOS[grupo][nome_do_estado]
        except KeyError as erro_do_estado:
            estados_disponiveis = sorted(ESTADOS_DOS_GRUPOS.get(grupo, {}))
            raise ValueError(
                f"Estado '{nome_do_estado}' não encontrado para o grupo '{grupo}'. "
                f"Estados disponíveis: {estados_disponiveis}"
            ) from erro_do_estado

        self.mover_para_posicoes_das_juntas(
            grupo,
            posicoes_das_juntas,
            descricao,
            tolerancia,
            velocidade,
            aceleracao,
        )

    def mover_para_posicoes_das_juntas(
        self,
        grupo: str,
        posicoes_das_juntas: PosicoesJuntas,
        descricao: str,
        tolerancia: float = TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
        velocidade: float = VELOCIDADE_MAXIMA,
        aceleracao: float = ACELERACAO_MAXIMA,
    ) -> None:
        """Planeja e executa um movimento para as posições das juntas dadas."""
        if not posicoes_das_juntas:
            raise ValueError("Informe ao menos uma posição de junta.")

        self.no.get_logger().info(descricao)
        restricoes = restricoes_de_posicao_inicial(posicoes_das_juntas, tolerancia)
        self.executar_objetivo(
            grupo,
            restricoes,
            velocidade,
            aceleracao,
            tentativas_de_planejamento=TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR,
            tempo_de_planejamento=TEMPO_DE_PLANEJAMENTO_ARTICULAR,
        )

    def executar_objetivo(
        self,
        grupo: str,
        restricoes: ListaDeRestricoes,
        velocidade: float = VELOCIDADE_MAXIMA,
        aceleracao: float = ACELERACAO_MAXIMA,
        tentativas_de_planejamento: int = TENTATIVAS_DE_PLANEJAMENTO,
        tempo_de_planejamento: float = TEMPO_DE_PLANEJAMENTO,
    ) -> None:
        inicio = time.monotonic()
        objetivo = MoveGroup.Goal()
        objetivo.request.group_name = grupo
        objetivo.request.num_planning_attempts = tentativas_de_planejamento
        objetivo.request.allowed_planning_time = tempo_de_planejamento
        objetivo.request.max_velocity_scaling_factor = velocidade
        objetivo.request.max_acceleration_scaling_factor = aceleracao
        objetivo.request.goal_constraints = restricoes
        objetivo.request.start_state.is_diff = True
        objetivo.planning_options.plan_only = False
        objetivo.planning_options.replan = False
        # Aplica o objetivo sobre a Planning Scene monitorada, preservando os
        # objetos do ambiente para a verificação de colisão de toda a trajetória.
        objetivo.planning_options.planning_scene_diff.is_diff = True
        objetivo.planning_options.planning_scene_diff.robot_state.is_diff = True

        futuro_do_envio = self.cliente_do_move_group.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self.no, futuro_do_envio)
        manipulador_do_objetivo = futuro_do_envio.result()
        if manipulador_do_objetivo is None or not manipulador_do_objetivo.accepted:
            raise RuntimeError(f"Objetivo rejeitado pelo MoveIt para o grupo '{grupo}'.")

        futuro_do_resultado = manipulador_do_objetivo.get_result_async()
        rclpy.spin_until_future_complete(self.no, futuro_do_resultado)
        resultado_da_acao = futuro_do_resultado.result()
        if resultado_da_acao is None or resultado_da_acao.status != GoalStatus.STATUS_SUCCEEDED:
            codigo_do_erro = (
                getattr(resultado_da_acao.result, "error_code", None)
                if resultado_da_acao
                else None
            )
            valor_do_erro = getattr(codigo_do_erro, "val", "desconhecido")
            self.no.get_logger().error(
                f"MoveIt não conseguiu executar o grupo '{grupo}'. "
                f"Código de erro: {valor_do_erro}"
            )
            raise RuntimeError(f"Movimento falhou no MoveIt (código {valor_do_erro}).")

        resultado = resultado_da_acao.result
        tempo_do_moveit = float(getattr(resultado, "planning_time", 0.0))
        tempo_total = time.monotonic() - inicio
        texto_tempo_de_planejamento = (
            f"{tempo_do_moveit:.3f}s"
            if tempo_do_moveit > 0.0
            else "não informado pelo move_group"
        )
        self.no.get_logger().info(
            f"MoveIt finalizou o grupo '{grupo}': "
            f"planejamento={texto_tempo_de_planejamento}, "
            f"planejamento+execução={tempo_total:.3f}s."
        )
        self._aguardar_assentamento(grupo, resultado.planned_trajectory)
        self.no.get_logger().info(f"Estado físico parado confirmado para '{grupo}'.")

    def _aguardar_assentamento(self, grupo: str, trajetoria_planejada: object) -> None:
        """Confirma por /joint_states que o robô físico parou de se mover."""
        trajetoria = getattr(trajetoria_planejada, "joint_trajectory", None)
        if trajetoria is None or not trajetoria.points:
            self.no.get_logger().info(
                f"Trajetória planejada para '{grupo}' não contém pontos; "
                "não será possível confirmar o assentamento físico."
            )
            return

        ultimo_ponto = trajetoria.points[-1]
        alvos = dict(zip(trajetoria.joint_names, ultimo_ponto.positions))
        if grupo == GRUPO_GARRA:
            tolerancia = TOLERANCIA_DE_ASSENTAMENTO_DA_GARRA
            velocidade_maxima = VELOCIDADE_DE_ASSENTAMENTO_DA_GARRA
        else:
            tolerancia = TOLERANCIA_DE_ASSENTAMENTO_DO_BRACO
            velocidade_maxima = VELOCIDADE_DE_ASSENTAMENTO_DO_BRACO

        prazo = time.monotonic() + TEMPO_LIMITE_DE_ASSENTAMENTO
        ultima_sequencia = self.sequencia_dos_estados_das_juntas
        amostras_estaveis = 0
        posicoes_anteriores: dict[str, float] | None = None
        maior_variacao = float("inf")
        maior_velocidade = float("inf")
        maior_erro_no_alvo = float("inf")

        while time.monotonic() < prazo:
            rclpy.spin_once(self.no, timeout_sec=0.05)
            if self.sequencia_dos_estados_das_juntas == ultima_sequencia:
                continue
            ultima_sequencia = self.sequencia_dos_estados_das_juntas

            # A junta esquerda da garra é passiva (mimic) e pode não aparecer
            # em /joint_states. Todas as juntas realmente comandadas são
            # obrigatórias para liberar o próximo movimento.
            juntas_exigidas = [
                nome
                for nome in alvos
                if not (grupo == GRUPO_GARRA and nome == "left_clamp")
            ]
            if not juntas_exigidas or any(
                nome not in self.posicoes_juntas_atuais for nome in juntas_exigidas
            ):
                continue

            posicoes_atuais = {
                nome: self.posicoes_juntas_atuais[nome] for nome in juntas_exigidas
            }
            maior_erro_no_alvo = max(
                abs(posicoes_atuais[nome] - alvos[nome]) for nome in juntas_exigidas
            )
            if posicoes_anteriores is None:
                posicoes_anteriores = posicoes_atuais
                continue

            maior_variacao = max(
                abs(posicoes_atuais[nome] - posicoes_anteriores[nome])
                for nome in juntas_exigidas
            )
            posicoes_estaveis = maior_variacao <= tolerancia

            velocidades_disponiveis = all(
                nome in self.velocidades_juntas_atuais for nome in juntas_exigidas
            )
            if velocidades_disponiveis:
                maior_velocidade = max(
                    abs(self.velocidades_juntas_atuais[nome])
                    for nome in juntas_exigidas
                )
                velocidades_baixas = maior_velocidade <= velocidade_maxima
            else:
                # Alguns publicadores não preenchem JointState.velocity. Nesse
                # caso, a variação entre amostras continua sendo a confirmação.
                maior_velocidade = 0.0
                velocidades_baixas = True

            amostras_estaveis = (
                amostras_estaveis + 1
                if posicoes_estaveis and velocidades_baixas
                else 0
            )
            if amostras_estaveis >= AMOSTRAS_ESTAVEIS_NECESSARIAS:
                if maior_erro_no_alvo > tolerancia:
                    self.no.get_logger().warning(
                        f"O grupo '{grupo}' parou com diferença máxima de "
                        f"{maior_erro_no_alvo:.4f} rad em relação à trajetória; "
                        "o controlador MoveIt declarou a execução concluída."
                    )
                return
            posicoes_anteriores = posicoes_atuais

        raise RuntimeError(
            f"O grupo '{grupo}' não confirmou parada em "
            f"{TEMPO_LIMITE_DE_ASSENTAMENTO:.1f}s "
            f"(variação={maior_variacao:.4f} rad, "
            f"velocidade={maior_velocidade:.4f} rad/s, "
            f"erro no alvo={maior_erro_no_alvo:.4f} rad); "
            "o próximo movimento não será enviado."
        )

    def destruir(self) -> None:
        self.no.destroy_node()
