"""Valida as diferenças de orientação entre aproximação e pegada."""

import math

from so_arm_101_moveit_config.configuracao import (
    TOLERANCIA_DE_ANGULO,
    TOLERANCIA_DE_INCLINACAO,
    TOLERANCIA_DE_INCLINACAO_DA_PRE_PEGADA,
)
from so_arm_101_moveit_config.restricoes import (
    criar_pose,
    restricoes_de_pegada,
    restricoes_de_pre_pegada,
)


def _orientacao_das(restricoes):
    return restricoes[0].orientation_constraints[0]


def test_pre_pegada_permite_mais_inclinacao_que_a_pegada():
    pose = criar_pose(0.0, -0.25, 0.09, 0.0)

    pre_pegada = _orientacao_das(restricoes_de_pre_pegada(pose))
    pegada = _orientacao_das(restricoes_de_pegada(pose))

    assert pre_pegada.absolute_x_axis_tolerance == (
        TOLERANCIA_DE_INCLINACAO_DA_PRE_PEGADA
    )
    assert pre_pegada.absolute_z_axis_tolerance == (
        TOLERANCIA_DE_INCLINACAO_DA_PRE_PEGADA
    )
    assert pre_pegada.absolute_x_axis_tolerance > pegada.absolute_x_axis_tolerance
    assert pre_pegada.absolute_z_axis_tolerance > pegada.absolute_z_axis_tolerance
    assert pegada.absolute_x_axis_tolerance == TOLERANCIA_DE_INCLINACAO
    assert pegada.absolute_z_axis_tolerance == TOLERANCIA_DE_INCLINACAO


def test_pre_pegada_limita_inclinacao_e_preserva_angulo_do_objeto():
    pose = criar_pose(0.0, -0.25, 0.09, 15.0)
    pre_pegada = _orientacao_das(restricoes_de_pre_pegada(pose))

    assert 0.0 < TOLERANCIA_DE_INCLINACAO_DA_PRE_PEGADA < math.pi / 2.0
    assert pre_pegada.absolute_y_axis_tolerance == TOLERANCIA_DE_ANGULO
