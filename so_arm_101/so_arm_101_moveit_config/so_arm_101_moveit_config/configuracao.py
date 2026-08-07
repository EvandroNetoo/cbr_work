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
REFERENCIAL_BASE: Final[str] = "base_link"
LINK_FIM_DA_GARRA: Final[str] = "gripper_tcp"

OBJETO_X: Final[float] = -0.15
OBJETO_Y: Final[float] = -0.25
OBJETO_Z: Final[float] = 0.02
ANGULO_DO_OBJETO_EM_GRAUS: Final[float] = 0.0
ALTURA_DE_APROXIMACAO: Final[float] = 0.0

TOLERANCIA_DE_POSICAO: Final[float] = 0.01
TOLERANCIA_DE_INCLINACAO: Final[float] = 0.20
TOLERANCIA_DE_ANGULO: Final[float] = math.radians(5.0)
TOLERANCIA_DA_JUNTA_DA_GARRA: Final[float] = 0.001
TOLERANCIA_DAS_JUNTAS_DE_ESTADOS: Final[float] = 0.01

TEMPO_DE_PLANEJAMENTO: Final[float] = 15.0
TENTATIVAS_DE_PLANEJAMENTO: Final[int] = 10
VELOCIDADE_MAXIMA: Final[float] = 1.0
ACELERACAO_MAXIMA: Final[float] = 0.5
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
