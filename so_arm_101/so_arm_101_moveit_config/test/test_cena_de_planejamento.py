"""Contratos da representação dos cubos na Planning Scene."""

import copy
import math

import pytest
from cbr_interfaces.msg import AprilTagStampedDetection
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
)

from so_arm_101_moveit_config.cena_de_planejamento import (
    LINKS_DE_CONTATO_DA_GARRA,
    CenaDePlanejamento,
)
from so_arm_101_moveit_config.configuracao import TAMANHO_DO_CUBO


class _Logger:
    def info(self, _mensagem):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


def _deteccao(tag_id=1, yaw_graus=30.0):
    deteccao = AprilTagStampedDetection()
    deteccao.header.frame_id = "base_link"
    deteccao.family = "tag36h11"
    deteccao.id = tag_id
    deteccao.pose.position.x = 0.1
    deteccao.pose.position.y = -0.2
    deteccao.pose.position.z = 0.12
    metade = math.radians(yaw_graus) / 2.0
    deteccao.pose.orientation.x = math.cos(metade)
    deteccao.pose.orientation.y = math.sin(metade)
    deteccao.pose.orientation.w = 0.0
    return deteccao


def _matriz(nomes):
    matriz = AllowedCollisionMatrix()
    matriz.entry_names = list(nomes)
    for indice in range(len(nomes)):
        linha = AllowedCollisionEntry()
        linha.enabled = [indice == outro for outro in range(len(nomes))]
        matriz.entry_values.append(linha)
    matriz.default_entry_names = ["octomap"]
    matriz.default_entry_values = [False]
    return matriz


def _cena_sem_ros(cena_atual=None):
    cena = CenaDePlanejamento.__new__(CenaDePlanejamento)
    cena.no = _Node()
    cena._objetos = {}
    cena._obter_cena = lambda _componentes: copy.deepcopy(
        cena_atual if cena_atual is not None else PlanningScene()
    )
    cena.aplicadas = []
    cena._aplicar_cena = lambda mensagem, operacao: cena.aplicadas.append(
        (copy.deepcopy(mensagem), operacao)
    )
    return cena


def test_cubo_usa_id_estavel_dimensoes_centro_e_yaw():
    objeto = CenaDePlanejamento.criar_cubo_da_deteccao(_deteccao(tag_id=7))

    assert objeto.id == "cubo_apriltag_tag36h11_7"
    assert objeto.header.frame_id == "base_link"
    assert list(objeto.primitives[0].dimensions) == [TAMANHO_DO_CUBO] * 3
    assert objeto.pose.position.x == pytest.approx(0.1)
    assert objeto.pose.position.y == pytest.approx(-0.2)
    assert objeto.pose.position.z == pytest.approx(0.12 - TAMANHO_DO_CUBO / 2.0)
    yaw = 2.0 * math.atan2(objeto.pose.orientation.z, objeto.pose.orientation.w)
    assert math.degrees(yaw) == pytest.approx(30.0)


@pytest.mark.parametrize("campo", ["x", "y", "z"])
def test_cubo_rejeita_posicao_nao_finita(campo):
    deteccao = _deteccao()
    setattr(deteccao.pose.position, campo, float("nan"))

    with pytest.raises(ValueError, match="não é finita"):
        CenaDePlanejamento.criar_cubo_da_deteccao(deteccao)


def test_sincronizacao_remove_somente_cubos_gerenciados_ausentes():
    atual = PlanningScene()
    for identificador in (
        "cubo_apriltag_tag36h11_1",
        "cubo_apriltag_tag36h11_99",
        "mesa_externa",
    ):
        atual.world.collision_objects.append(CollisionObject(id=identificador))
    cena = _cena_sem_ros(atual)

    ids = cena.sincronizar_cubos([_deteccao(1), _deteccao(2)])

    enviados = cena.aplicadas[0][0].world.collision_objects
    removidos = [
        item.id for item in enviados if item.operation == CollisionObject.REMOVE
    ]
    adicionados = [
        item.id for item in enviados if item.operation == CollisionObject.ADD
    ]
    assert removidos == ["cubo_apriltag_tag36h11_99"]
    assert adicionados == [
        "cubo_apriltag_tag36h11_1",
        "cubo_apriltag_tag36h11_2",
    ]
    assert ids == {1: adicionados[0], 2: adicionados[1]}


def test_contato_preserva_matriz_e_libera_apenas_os_dois_clamps():
    atual = PlanningScene()
    atual.allowed_collision_matrix = _matriz(["base_link", "link1_1"])
    original = copy.deepcopy(atual.allowed_collision_matrix)
    cena = _cena_sem_ros(atual)
    alvo = "cubo_apriltag_tag36h11_1"

    fotografia = cena.permitir_contato_com_a_garra(alvo)

    assert fotografia == original
    alterada = cena.aplicadas[0][0].allowed_collision_matrix
    for link in LINKS_DE_CONTATO_DA_GARRA:
        i, j = alterada.entry_names.index(alvo), alterada.entry_names.index(link)
        assert alterada.entry_values[i].enabled[j]
        assert alterada.entry_values[j].enabled[i]
    indice_alvo = alterada.entry_names.index(alvo)
    indice_base = alterada.entry_names.index("base_link")
    assert not alterada.entry_values[indice_alvo].enabled[indice_base]
    assert alterada.default_entry_names == original.default_entry_names
    assert alterada.default_entry_values == original.default_entry_values


def test_restauracao_reaplica_exatamente_a_matriz_original():
    cena = _cena_sem_ros()
    original = _matriz(["base_link", "link1_1"])

    cena.restaurar_matriz_de_colisoes(original)

    assert cena.aplicadas[0][0].allowed_collision_matrix == original


def test_ignorar_cubo_remove_somente_o_alvo_da_cena():
    cena = _cena_sem_ros()
    objeto = CenaDePlanejamento.criar_cubo_da_deteccao(_deteccao())
    cena._objetos[objeto.id] = objeto

    cena.ignorar_cubo(objeto.id)

    removido = cena.aplicadas[0][0].world.collision_objects[0]
    assert removido.id == objeto.id
    assert removido.operation == CollisionObject.REMOVE
    assert objeto.id not in cena._objetos
