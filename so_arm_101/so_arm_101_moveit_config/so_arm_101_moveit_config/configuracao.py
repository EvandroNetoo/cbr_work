"""Configurações e estados nomeados do SO-ARM-101."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Final, TypeAlias
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory


PosicoesJuntas: TypeAlias = Mapping[str, float]
EstadosDeGrupo: TypeAlias = dict[str, dict[str, dict[str, float]]]

GRUPO_BRACO: Final[str] = "arm"
GRUPO_GARRA: Final[str] = "gripper"
# Referencial cartesiano canônico do manipulador. No robô composto, base_link
# pertence ao chassi e arm_base_link inclui o yaw físico do suporte; usar o
# frame do braço mantém poses e yaw idênticos ao perfil standalone.
REFERENCIAL_BASE: Final[str] = "arm_base_link"
LINK_FIM_DA_GARRA: Final[str] = "gripper_tcp"

OBJETO_X: Final[float] = -0.0
OBJETO_Y: Final[float] = -0.25
OBJETO_Z: Final[float] = 0.03
# Use None para manter as coordenadas OBJETO_X/Y/Z acima. Quando definido,
# o ID é procurado pelo detector AprilTag e sua posição em arm_base_link substitui
# as três coordenadas do objeto.
APRIL_TAG_ID: Final[int | None] = 1
TEMPO_DE_ANALISE_DA_APRIL_TAG: Final[float] = 2.0
# A AprilTag fica sobre o cubo, enquanto o TCP fica na ponta da garra. Portanto,
# a altura completa do cubo deve ser descontada do Z medido para obter o Z da
# pegada. Todas as medidas cartesianas deste arquivo estão em metros.
TAMANHO_DO_CUBO: Final[float] = 0.042
# Usado apenas quando APRIL_TAG_ID=None. Com detecção ativa, o yaw vem da
# pose da tag e é reduzido a uma orientação de pegada equivalente em ±45 graus.
ANGULO_DO_OBJETO_EM_GRAUS: Final[float] = 0.0
ALTURA_DE_APROXIMACAO: Final[float] = 0.08

TOLERANCIA_DE_POSICAO: Final[float] = 0.0025
TOLERANCIA_DE_INCLINACAO: Final[float] = 0.20
TOLERANCIA_DE_INCLINACAO_DA_PRE_PEGADA: Final[float] = math.radians(35.0)
TOLERANCIA_DE_ANGULO: Final[float] = math.radians(5.0)
TOLERANCIA_DA_JUNTA_DA_GARRA: Final[float] = 0.001
TOLERANCIA_DAS_JUNTAS_DE_ESTADOS: Final[float] = 0.01

# Poses cartesianas, com restrições de posição e orientação, precisam de um
# orçamento um pouco maior que estados articulares conhecidos.
TEMPO_DE_PLANEJAMENTO: Final[float] = 15.0
TENTATIVAS_DE_PLANEJAMENTO: Final[int] = 2

# Estados nomeados do SRDF já fornecem diretamente o alvo de cada junta. Uma
# única tentativa curta evita gastar CPU procurando várias soluções equivalentes.
TEMPO_DE_PLANEJAMENTO_ARTICULAR: Final[float] = 2.0
TENTATIVAS_DE_PLANEJAMENTO_ARTICULAR: Final[int] = 1

# Após o MoveIt informar o fim da execução, confirme que o estado físico ficou
# estável antes de enviar o próximo objetivo. Isso substitui sleeps fixos por
# uma barreira real sem exigir precisão maior que a aceita pelo controlador.
TEMPO_LIMITE_DE_ASSENTAMENTO: Final[float] = 3.0
AMOSTRAS_ESTAVEIS_NECESSARIAS: Final[int] = 2
TOLERANCIA_DE_ASSENTAMENTO_DO_BRACO: Final[float] = 0.01
TOLERANCIA_DE_ASSENTAMENTO_DA_GARRA: Final[float] = 0.001
VELOCIDADE_DE_ASSENTAMENTO_DO_BRACO: Final[float] = 0.05
VELOCIDADE_DE_ASSENTAMENTO_DA_GARRA: Final[float] = 0.01
VELOCIDADE_MAXIMA: Final[float] = 1.0
ACELERACAO_MAXIMA: Final[float] = 1.0
VELOCIDADE_MAXIMA_DA_GARRA: Final[float] = 1.0
ACELERACAO_MAXIMA_DA_GARRA: Final[float] = 1.0


def carregar_estados_do_srdf() -> EstadosDeGrupo:
    """Carrega os estados nomeados no formato grupo -> estado -> juntas."""
    diretorio_pacote = get_package_share_directory("so_arm_101_moveit_config")
    caminho_srdf = f"{diretorio_pacote}/config/so_arm_101.srdf"
    raiz = ElementTree.parse(caminho_srdf).getroot()

    estados: EstadosDeGrupo = {}
    for estado_grupo in raiz.findall("group_state"):
        nome_estado = estado_grupo.attrib.get("name")
        nome_grupo = estado_grupo.attrib.get("group")
        if not nome_estado or not nome_grupo:
            continue

        estados.setdefault(nome_grupo, {})[nome_estado] = {
            junta.attrib["name"]: float(junta.attrib["value"])
            for junta in estado_grupo.findall("joint")
        }

    return estados


ESTADOS_DOS_GRUPOS: Final[EstadosDeGrupo] = carregar_estados_do_srdf()
