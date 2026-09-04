"""Contratos das otimizações para execução embarcada."""

import math
from pathlib import Path
import threading

import pytest
import yaml

from so_arm_101_moveit_config.configuracao import (
    ACELERACAO_MAXIMA,
    APRIL_TAG_ID,
    TAMANHO_DO_CUBO,
    TENTATIVAS_DE_PLANEJAMENTO,
    TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR,
    TEMPO_DE_PLANEJAMENTO,
    TEMPO_DE_PLANEJAMENTO_ARTICULAR,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt
from interfaces.msg import AprilTagStampedDetection
from sensor_msgs.msg import JointState


PACKAGE_DIR = Path(__file__).parents[1]


def test_estado_articular_usa_orcamento_menor_que_pose_cartesiana():
    assert TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR == 1
    assert TEMPO_DE_PLANEJAMENTO_ARTICULAR < TEMPO_DE_PLANEJAMENTO
    assert TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR < TENTATIVAS_DE_PLANEJAMENTO


def test_aceleracao_do_braco_usa_escala_maxima():
    assert ACELERACAO_MAXIMA == pytest.approx(1.0)


def test_estado_articular_ja_atingido_e_omitido():
    executor = ExecutorDoMoveIt.__new__(ExecutorDoMoveIt)
    executor._condicao_dos_estados = threading.Condition()
    executor.sequencia_dos_estados_das_juntas = 1
    executor.posicoes_juntas_atuais = {'joint': 1.0}
    executor.velocidades_juntas_atuais = {'joint': 0.0}

    assert executor._estado_articular_ja_atingido(
        'arm', {'joint': 1.005}, 0.01
    )
    assert not executor._estado_articular_ja_atingido(
        'arm', {'joint': 1.02}, 0.01
    )


def test_monitoramento_dos_estados_pode_dormir_entre_operacoes():
    class NoFalso:
        def __init__(self):
            self.callbacks = []
            self.inscricoes_destruidas = []

        def create_subscription(
            self, _tipo, _topico, callback, _profundidade, *, callback_group
        ):
            del callback_group
            inscricao = object()
            self.callbacks.append(callback)
            return inscricao

        def destroy_subscription(self, inscricao):
            self.inscricoes_destruidas.append(inscricao)
            return True

    executor = ExecutorDoMoveIt.__new__(ExecutorDoMoveIt)
    executor.no = NoFalso()
    executor._condicao_dos_estados = threading.Condition()
    executor._topico_dos_estados_das_juntas = '/joint_states'
    executor._grupo_de_callbacks = None
    executor._geracao_do_monitoramento = 0
    executor.inscricao_nos_estados_das_juntas = None
    executor.posicoes_juntas_atuais = {'antiga': 1.0}
    executor.velocidades_juntas_atuais = {'antiga': 0.0}
    executor.sequencia_dos_estados_das_juntas = 4

    executor.iniciar_monitoramento_dos_estados()
    primeira_inscricao = executor.inscricao_nos_estados_das_juntas
    primeiro_callback = executor.no.callbacks[-1]

    assert primeira_inscricao is not None
    assert executor.posicoes_juntas_atuais == {}
    assert executor.sequencia_dos_estados_das_juntas == 0
    executor.iniciar_monitoramento_dos_estados()
    assert len(executor.no.callbacks) == 1

    primeiro_callback(JointState(name=['joint'], position=[0.5], velocity=[0.0]))
    assert executor.posicoes_juntas_atuais == {'joint': 0.5}
    assert executor.sequencia_dos_estados_das_juntas == 1
    assert executor.aguardar_primeiro_estado(timeout_sec=0.0)

    executor.parar_monitoramento_dos_estados()
    assert executor.inscricao_nos_estados_das_juntas is None
    assert executor.no.inscricoes_destruidas == [primeira_inscricao]
    executor.parar_monitoramento_dos_estados()
    assert executor.no.inscricoes_destruidas == [primeira_inscricao]

    # Um callback já enfileirado da assinatura antiga deve ser ignorado.
    primeiro_callback(JointState(name=['joint'], position=[0.9], velocity=[0.0]))
    assert executor.posicoes_juntas_atuais == {'joint': 0.5}

    executor.iniciar_monitoramento_dos_estados()
    assert executor.posicoes_juntas_atuais == {}
    assert executor.sequencia_dos_estados_das_juntas == 0


def test_sequencia_nao_contem_esperas_fixas():
    sequencia = (PACKAGE_DIR / "scripts" / "pegar_e_colocar.py").read_text()
    movimento = (
        PACKAGE_DIR / "so_arm_101_moveit_config" / "movimento.py"
    ).read_text()
    assert "time.sleep" not in sequencia
    assert "sleep(" not in sequencia
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
