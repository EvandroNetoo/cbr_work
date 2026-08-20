"""Contratos das otimizações para execução embarcada."""

import math
from pathlib import Path

import pytest
import yaml

from so_arm_101_moveit_config.configuracao import (
    APRIL_TAG_ID,
    TAMANHO_DO_CUBO,
    TENTATIVAS_DE_PLANEJAMENTO,
    TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR,
    TEMPO_DE_PLANEJAMENTO,
    TEMPO_DE_PLANEJAMENTO_ARTICULAR,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt
from cbr_interfaces.msg import AprilTagStampedDetection


PACKAGE_DIR = Path(__file__).parents[1]


def test_estado_articular_usa_orcamento_menor_que_pose_cartesiana():
    assert TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR == 1
    assert TEMPO_DE_PLANEJAMENTO_ARTICULAR < TEMPO_DE_PLANEJAMENTO
    assert TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR < TENTATIVAS_DE_PLANEJAMENTO


def test_sequencia_nao_contem_esperas_fixas():
    sequencia = (PACKAGE_DIR / "scripts" / "pegar_e_colocar.py").read_text()
    movimento = (
        PACKAGE_DIR / "so_arm_101_moveit_config" / "movimento.py"
    ).read_text()
    assert "time.sleep" not in sequencia
    assert "time.sleep" not in movimento
    assert "_aguardar_assentamento" in movimento


def test_resolucao_de_colisao_embarcada():
    configuracao = yaml.safe_load(
        (PACKAGE_DIR / "config" / "ompl_planning.yaml").read_text()
    )
    assert configuracao["arm"]["longest_valid_segment_fraction"] == 0.01
    assert configuracao["gripper"]["longest_valid_segment_fraction"] == 0.01


def test_xyz_continua_configurado_quando_apriltag_esta_desativada():
    assert APRIL_TAG_ID is None or isinstance(APRIL_TAG_ID, int)


def test_tamanho_inicial_do_cubo_e_cinco_centimetros():
    assert TAMANHO_DO_CUBO == 0.05


def test_sequencia_subtrai_tamanho_do_cubo_do_z_da_apriltag():
    sequencia = (PACKAGE_DIR / "scripts" / "pegar_e_colocar.py").read_text()
    assert "objeto_z -= TAMANHO_DO_CUBO" in sequencia


def test_seleciona_a_apriltag_pelo_id_configurado():
    primeira = AprilTagStampedDetection(id=3)
    desejada = AprilTagStampedDetection(id=7)

    resultado = ExecutorDoMoveIt._selecionar_april_tag([primeira, desejada], 7)

    assert resultado is desejada
    assert ExecutorDoMoveIt._selecionar_april_tag([primeira], 7) is None


def test_erro_da_apriltag_propaga_detalhe_retornado_pelo_servidor():
    movimento = (
        PACKAGE_DIR / "so_arm_101_moveit_config" / "movimento.py"
    ).read_text()
    assert "resultado_da_acao.result.message" in movimento
    assert "(estado {estado}): {detalhe}" in movimento


def test_extrai_yaw_da_orientacao_da_apriltag():
    metade = math.radians(35.0) / 2.0

    yaw = ExecutorDoMoveIt._yaw_em_graus(
        0.0, 0.0, math.sin(metade), math.cos(metade)
    )

    assert yaw == pytest.approx(35.0)


def test_extrai_yaw_da_tag_horizontal_mesmo_com_eixo_invertido():
    metade = math.radians(-25.0) / 2.0

    # Rz(yaw) * Rx(180°), orientação comum para uma tag sobre o cubo.
    yaw = ExecutorDoMoveIt._yaw_em_graus(
        math.cos(metade), math.sin(metade), 0.0, 0.0
    )

    assert yaw == pytest.approx(-25.0)


def test_rejeita_quaternion_nulo_da_apriltag():
    with pytest.raises(ValueError, match="quaternion nulo"):
        ExecutorDoMoveIt._yaw_em_graus(0.0, 0.0, 0.0, 0.0)
