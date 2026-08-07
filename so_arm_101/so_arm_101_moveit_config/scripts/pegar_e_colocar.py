#!/usr/bin/env python3
"""Sequência de pegar e colocar um objeto usando MoveIt 2."""

from __future__ import annotations

import sys
import time

import rclpy

from so_arm_101_moveit_config.configuracao import (
    ACELERACAO_MAXIMA,
    ACELERACAO_MAXIMA_DA_GARRA,
    ALTURA_DE_APROXIMACAO,
    ANGULO_DO_OBJETO_EM_GRAUS,
    GRUPO_BRACO,
    GRUPO_GARRA,
    OBJETO_X,
    OBJETO_Y,
    OBJETO_Z,
    TOLERANCIA_DA_JUNTA_DA_GARRA,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_MAXIMA,
    VELOCIDADE_MAXIMA_DA_GARRA,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt
from so_arm_101_moveit_config.restricoes import (
    criar_pose,
    restricoes_de_deposito_acima,
    restricoes_de_pegada,
    restricoes_de_pre_pegada,
)


class PegarEColocar:
    """Descreve somente a sequência operacional da tarefa."""

    def __init__(self, executor: ExecutorDoMoveIt) -> None:
        self.executor = executor

    def executar(self) -> None:
        self.executor.aguardar_o_servidor()


        pose_do_objeto = criar_pose(
            OBJETO_X,
            OBJETO_Y,
            OBJETO_Z,
            ANGULO_DO_OBJETO_EM_GRAUS,
        )
        pose_acima_do_objeto = criar_pose(
            OBJETO_X,
            OBJETO_Y,
            OBJETO_Z + ALTURA_DE_APROXIMACAO,
            ANGULO_DO_OBJETO_EM_GRAUS,
        )

        self.executor.mover_para_estado(
            GRUPO_BRACO,
            "home",
            "Indo para a pose home",
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_GARRA,
            "open",
            "Abrindo a garra",
            tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
        )

        self.executor.no.get_logger().info("Indo para cima do objeto")
        self.executor.executar_objetivo(
            GRUPO_BRACO,
            restricoes_de_pre_pegada(pose_acima_do_objeto),
            VELOCIDADE_MAXIMA,
            ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.executar_objetivo(
            GRUPO_BRACO,
            restricoes_de_pegada(pose_do_objeto),
            VELOCIDADE_MAXIMA,
            ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_GARRA,
            "grip",
            "Fechando a garra",
            tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
        )
        time.sleep(1.0)
        self.executor.executar_objetivo(
            GRUPO_BRACO,
            restricoes_de_pre_pegada(pose_acima_do_objeto),
            VELOCIDADE_MAXIMA,
            ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_BRACO,
            "home",
            "Indo para a pose home",
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_BRACO,
            "deposit_cube_left",
            "Indo para o depósito do cubo à esquerda",
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_GARRA,
            "open",
            "Abrindo a garra",
            tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
        )
        time.sleep(1.0)
        self.executor.mover_para_estado(
            GRUPO_BRACO,
            "home",
            "Indo para a pose home",
            tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            velocidade=VELOCIDADE_MAXIMA,
            aceleracao=ACELERACAO_MAXIMA,
        )
        self.executor.no.get_logger().info("Sequência concluída")


def main() -> int:
    rclpy.init(args=sys.argv)
    executor = ExecutorDoMoveIt()
    tarefa = PegarEColocar(executor)

    try:
        tarefa.executar()
    except Exception as erro:
        executor.no.get_logger().error(f"SEQUÊNCIA INTERROMPIDA: {erro}")
        codigo_de_retorno = 1
    else:
        codigo_de_retorno = 0
    finally:
        executor.destruir()
        rclpy.shutdown()

    return codigo_de_retorno


if __name__ == "__main__":
    sys.exit(main())
