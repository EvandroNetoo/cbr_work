"""Execução de objetivos e movimentos através do MoveIt 2."""

from __future__ import annotations

import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from .configuracao import (
    ACELERACAO_MAXIMA,
    ESTADOS_DOS_GRUPOS,
    TENTATIVAS_DE_PLANEJAMENTO,
    TEMPO_DE_PLANEJAMENTO,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_MAXIMA,
    PosicoesJuntas,
)
from .restricoes import ListaDeRestricoes, restricoes_de_posicao_inicial


class ExecutorDoMoveIt:
    """Encapsula a comunicação com o servidor de ação do MoveIt."""

    def __init__(self) -> None:
        self.no = rclpy.create_node("pegar_e_colocar")
        self.cliente_do_move_group = ActionClient(self.no, MoveGroup, "/move_action")
        self.posicoes_juntas_atuais: dict[str, float] = {}
        self.sequencia_dos_estados_das_juntas = 0
        self.inscricao_nos_estados_das_juntas = self.no.create_subscription(
            JointState, "/joint_states", self._receber_estados_das_juntas, 10
        )

    def _receber_estados_das_juntas(self, mensagem: JointState) -> None:
        self.posicoes_juntas_atuais.update(zip(mensagem.name, mensagem.position))
        self.sequencia_dos_estados_das_juntas += 1

    def aguardar_o_servidor(self) -> None:
        self.no.get_logger().info("Aguardando o servidor de planejamento do MoveIt...")
        if not self.cliente_do_move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "Servidor /move_action não encontrado. Inicie o real_planning.launch.py na Banana Pi."
            )

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
        self.executar_objetivo(grupo, restricoes, velocidade, aceleracao)
        time.sleep(1)

    def executar_objetivo(
        self,
        grupo: str,
        restricoes: ListaDeRestricoes,
        velocidade: float = VELOCIDADE_MAXIMA,
        aceleracao: float = ACELERACAO_MAXIMA,
    ) -> None:
        objetivo = MoveGroup.Goal()
        objetivo.request.group_name = grupo
        objetivo.request.num_planning_attempts = TENTATIVAS_DE_PLANEJAMENTO
        objetivo.request.allowed_planning_time = TEMPO_DE_PLANEJAMENTO
        objetivo.request.max_velocity_scaling_factor = velocidade
        objetivo.request.max_acceleration_scaling_factor = aceleracao
        objetivo.request.goal_constraints = restricoes
        objetivo.request.start_state.is_diff = True
        objetivo.planning_options.plan_only = False
        objetivo.planning_options.replan = True
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

    def destruir(self) -> None:
        self.no.destroy_node()
