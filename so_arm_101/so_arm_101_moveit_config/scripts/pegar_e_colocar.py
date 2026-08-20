#!/usr/bin/env python3
"""Sequência de pegar e colocar um objeto usando MoveIt 2."""

from __future__ import annotations

import sys

import rclpy

from so_arm_101_moveit_config.configuracao import (
    ACELERACAO_MAXIMA,
    ACELERACAO_MAXIMA_DA_GARRA,
    ALTURA_DE_APROXIMACAO,
    ANGULO_DO_OBJETO_EM_GRAUS,
    APRIL_TAG_ID,
    GRUPO_BRACO,
    GRUPO_GARRA,
    OBJETO_X,
    OBJETO_Y,
    OBJETO_Z,
    TAMANHO_DO_CUBO,
    TEMPO_DE_ANALISE_DA_APRIL_TAG,
    TOLERANCIA_DA_JUNTA_DA_GARRA,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    VELOCIDADE_MAXIMA,
    VELOCIDADE_MAXIMA_DA_GARRA,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt
from so_arm_101_moveit_config.restricoes import (
    criar_pose,
    normalizar_angulo_de_pegada,
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

        # self.executor.mover_para_estado(
        #     GRUPO_BRACO,
        #     "home",
        #     "Indo para a pose home",
        #     tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
        #     velocidade=VELOCIDADE_MAXIMA,
        #     aceleracao=ACELERACAO_MAXIMA,
        # )
        while True:
            self.executor.mover_para_estado(
                GRUPO_GARRA,
                "open",
                "Abrindo a garra",
                tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
                velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
                aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
            )

            objeto_x, objeto_y, objeto_z = OBJETO_X, OBJETO_Y, OBJETO_Z
            angulo_do_objeto = ANGULO_DO_OBJETO_EM_GRAUS
            if APRIL_TAG_ID is not None:
                self.executor.mover_para_estado(
                    GRUPO_BRACO,
                    "detect_apriltags",
                    f"Posicionando a câmera para procurar a AprilTag {APRIL_TAG_ID}",
                    tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
                    velocidade=VELOCIDADE_MAXIMA,
                    aceleracao=ACELERACAO_MAXIMA,
                )
                (
                    objeto_x,
                    objeto_y,
                    objeto_z,
                    angulo_do_objeto,
                ) = self.executor.obter_pose_da_april_tag(
                    APRIL_TAG_ID, TEMPO_DE_ANALISE_DA_APRIL_TAG
                )
                objeto_z -= TAMANHO_DO_CUBO
                self.executor.no.get_logger().info(
                    f"Compensando {TAMANHO_DO_CUBO:.3f} m no Z da AprilTag; "
                    f"Z da pegada: {objeto_z:.3f} m"
                )
                # self.executor.mover_para_estado(
                #     GRUPO_BRACO,
                #     "home",
                #     "Voltando para home após localizar a AprilTag",
                #     tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
                #     velocidade=VELOCIDADE_MAXIMA,
                #     aceleracao=ACELERACAO_MAXIMA,
                # )
            else:
                self.executor.no.get_logger().info(
                    "APRIL_TAG_ID=None; usando as coordenadas XYZ configuradas."
                )

            angulo_da_pegada = normalizar_angulo_de_pegada(angulo_do_objeto)
            self.executor.no.get_logger().info(
                f"Yaw do objeto: {angulo_do_objeto:.1f}°; "
                f"yaw equivalente escolhido para a garra: {angulo_da_pegada:.1f}°."
            )

            pose_do_objeto = criar_pose(
                objeto_x,
                objeto_y,
                objeto_z,
                angulo_da_pegada + 90,
            )
            pose_acima_do_objeto = criar_pose(
                objeto_x,
                objeto_y,
                objeto_z + ALTURA_DE_APROXIMACAO,
                angulo_da_pegada + 90,
            )

            self.executor.no.get_logger().info("Indo para cima do objeto")
            self.executor.executar_objetivo(
                GRUPO_BRACO,
                restricoes_de_pre_pegada(pose_acima_do_objeto),
                VELOCIDADE_MAXIMA,
                ACELERACAO_MAXIMA,
            )
            self.executor.executar_objetivo(
                GRUPO_BRACO,
                restricoes_de_pegada(pose_do_objeto),
                VELOCIDADE_MAXIMA,
                ACELERACAO_MAXIMA,
            )
            self.executor.mover_para_estado(
                GRUPO_GARRA,
                "grip",
                "Fechando a garra",
                tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
                velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
                aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
            )
            self.executor.executar_objetivo(
                GRUPO_BRACO,
                restricoes_de_pre_pegada(pose_acima_do_objeto),
                VELOCIDADE_MAXIMA,
                ACELERACAO_MAXIMA,
            )
            self.executor.mover_para_estado(
                GRUPO_BRACO,
                "home",
                "Indo para a pose home",
                tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
                velocidade=VELOCIDADE_MAXIMA,
                aceleracao=ACELERACAO_MAXIMA,
            )
            # self.executor.mover_para_estado(
            #     GRUPO_BRACO,
            #     "deposit_cube_left",
            #     "Indo para o depósito do cubo à esquerda",
            #     tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            #     velocidade=VELOCIDADE_MAXIMA,
            #     aceleracao=ACELERACAO_MAXIMA,
            # )
            # self.executor.mover_para_estado(
            #     GRUPO_GARRA,
            #     "open",
            #     "Abrindo a garra",
            #     tolerancia=TOLERANCIA_DA_JUNTA_DA_GARRA,
            #     velocidade=VELOCIDADE_MAXIMA_DA_GARRA,
            #     aceleracao=ACELERACAO_MAXIMA_DA_GARRA,
            # )
            # self.executor.mover_para_estado(
            #     GRUPO_BRACO,
            #     "home",
            #     "Indo para a pose home",
            #     tolerancia=TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
            #     velocidade=VELOCIDADE_MAXIMA,
            #     aceleracao=ACELERACAO_MAXIMA,
            # )
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
