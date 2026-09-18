import math
from types import SimpleNamespace

import pytest

from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt, FalhaDoMoveIt


def trajectory(values, names=('link3_to_link4', 'link4_to_link5')):
    return SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=list(names),
        points=[SimpleNamespace(positions=[0.0, value]) for value in values],
    ))


def test_every_trajectory_point_must_keep_wrist_at_90_plus_or_minus_5_degrees():
    ExecutorDoMoveIt.validar_restricao_articular_da_trajetoria(
        trajectory([math.pi/2, math.pi/2 + math.pi/36]),
        'link4_to_link5', math.pi/2, math.pi/36)
    with pytest.raises(FalhaDoMoveIt, match='ponto 1'):
        ExecutorDoMoveIt.validar_restricao_articular_da_trajetoria(
            trajectory([math.pi/2, math.pi/2 + math.pi/35]),
            'link4_to_link5', math.pi/2, math.pi/36)


def test_trajectory_without_required_joint_is_rejected():
    with pytest.raises(FalhaDoMoveIt, match='junta obrigatória'):
        ExecutorDoMoveIt.validar_restricao_articular_da_trajetoria(
            trajectory([math.pi/2], names=('a', 'b')),
            'link4_to_link5', math.pi/2, math.pi/36)
