"""Criação das poses e restrições usadas pelo MoveIt."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

from .configuracao import (
    LINK_FIM_DA_GARRA,
    REFERENCIAL_BASE,
    PosicoesJuntas,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
    TOLERANCIA_DE_ANGULO,
    TOLERANCIA_DE_INCLINACAO,
    TOLERANCIA_DE_POSICAO,
)


ListaDeRestricoes = list[Constraints]


def criar_pose(x: float, y: float, z: float, angulo_em_graus: float) -> PoseStamped:
    """Cria uma pose com a garra apontada para baixo."""
    metade_do_angulo = math.radians(angulo_em_graus) / 2.0
    metade_da_raiz = math.sqrt(0.5)

    pose = PoseStamped()
    pose.header.frame_id = REFERENCIAL_BASE
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = metade_da_raiz * math.cos(metade_do_angulo)
    pose.pose.orientation.y = metade_da_raiz * math.sin(metade_do_angulo)
    pose.pose.orientation.z = metade_da_raiz * math.sin(metade_do_angulo)
    pose.pose.orientation.w = metade_da_raiz * math.cos(metade_do_angulo)
    return pose


def _criar_restricao_de_posicao(posicao: PoseStamped) -> PositionConstraint:
    restricao = PositionConstraint()
    restricao.header = posicao.header
    restricao.link_name = LINK_FIM_DA_GARRA

    primitiva = SolidPrimitive()
    primitiva.type = SolidPrimitive.SPHERE
    primitiva.dimensions = [TOLERANCIA_DE_POSICAO]
    restricao.constraint_region.primitives.append(primitiva)
    restricao.constraint_region.primitive_poses.append(posicao.pose)
    restricao.weight = 1.0
    return restricao


def _criar_restricao_de_orientacao(posicao: PoseStamped) -> OrientationConstraint:
    restricao = OrientationConstraint()
    restricao.header = posicao.header
    restricao.link_name = LINK_FIM_DA_GARRA
    restricao.orientation = posicao.pose.orientation
    restricao.absolute_x_axis_tolerance = TOLERANCIA_DE_INCLINACAO
    restricao.absolute_y_axis_tolerance = TOLERANCIA_DE_ANGULO
    restricao.absolute_z_axis_tolerance = TOLERANCIA_DE_INCLINACAO
    restricao.weight = 1.0
    return restricao


def restricoes_de_deposito_acima(posicao: PoseStamped) -> ListaDeRestricoes:
    """Restringe a posição e duas juntas durante a aproximação superior."""
    restricoes = Constraints()
    restricoes.position_constraints.append(_criar_restricao_de_posicao(posicao))

    for nome_da_junta, valor in (
        ("link3_to_link4", -0.4),
        ("link4_to_link5", 0.0),
    ):
        restricao_de_junta = JointConstraint()
        restricao_de_junta.joint_name = nome_da_junta
        restricao_de_junta.position = valor
        restricao_de_junta.tolerance_above = 0.2
        restricao_de_junta.tolerance_below = 0.2
        restricao_de_junta.weight = 1.0
        restricoes.joint_constraints.append(restricao_de_junta)

    return [restricoes]


def restricoes_de_pre_pegada(posicao: PoseStamped) -> ListaDeRestricoes:
    """Restringe posição e orientação antes de pegar o objeto."""
    restricoes = Constraints()
    restricoes.position_constraints.append(_criar_restricao_de_posicao(posicao))
    restricoes.orientation_constraints.append(_criar_restricao_de_orientacao(posicao))
    return [restricoes]


def restricoes_de_pegada(posicao: PoseStamped) -> ListaDeRestricoes:
    """Restringe posição e orientação durante a pegada."""
    return restricoes_de_pre_pegada(posicao)


def restricoes_de_posicao_inicial(
    posicoes_das_juntas: PosicoesJuntas,
    tolerancia: float = TOLERANCIA_DAS_JUNTAS_DE_ESTADOS,
) -> ListaDeRestricoes:
    """Cria restrições articulares para um conjunto de juntas."""
    restricoes = Constraints()
    for nome_da_junta, posicao in posicoes_das_juntas.items():
        restricao_de_junta = JointConstraint()
        restricao_de_junta.joint_name = nome_da_junta
        restricao_de_junta.position = posicao
        restricao_de_junta.tolerance_above = tolerancia
        restricao_de_junta.tolerance_below = tolerancia
        restricao_de_junta.weight = 1.0
        restricoes.joint_constraints.append(restricao_de_junta)
    return [restricoes]
