"""Ordem das transições entre movimentos e Planning Scene."""

import importlib.util
from pathlib import Path

import pytest
from cbr_interfaces.msg import AprilTagStampedDetection

from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt


CAMINHO_DO_SCRIPT = Path(__file__).parents[1] / "scripts" / "pegar_e_colocar.py"
ESPECIFICACAO = importlib.util.spec_from_file_location(
    "pegar_e_colocar_em_teste", CAMINHO_DO_SCRIPT
)
MODULO = importlib.util.module_from_spec(ESPECIFICACAO)
assert ESPECIFICACAO.loader is not None
ESPECIFICACAO.loader.exec_module(MODULO)
PegarEColocar = MODULO.PegarEColocar


class _Logger:
    def __init__(self, eventos):
        self.eventos = eventos

    def info(self, _mensagem):
        pass

    def error(self, mensagem):
        self.eventos.append(("erro", mensagem))


class _Node:
    def __init__(self, eventos):
        self.logger = _Logger(eventos)

    def get_logger(self):
        return self.logger


class _Cena:
    def __init__(self, eventos, falhar_em=None):
        self.eventos = eventos
        self.falhar_em = falhar_em

    def sincronizar_cubos(self, _deteccoes):
        self.eventos.append(("cena", "sincronizar"))
        if self.falhar_em == "sincronizar":
            raise RuntimeError("sincronização falhou")
        return {1: "cubo_alvo"}

    def verificar_sem_cubos_anexados(self):
        self.eventos.append(("cena", "verificar_anexados"))

    def permitir_contato_com_a_garra(self, _identificador):
        self.eventos.append(("cena", "permitir"))
        return "matriz_original"

    def restaurar_matriz_de_colisoes(self, _matriz):
        self.eventos.append(("cena", "restaurar"))
        if self.falhar_em == "restaurar":
            raise RuntimeError("restauração falhou")

    def ignorar_cubo(self, _identificador):
        self.eventos.append(("cena", "ignorar_alvo"))
        if self.falhar_em == "ignorar_alvo":
            raise RuntimeError("remoção do alvo falhou")


class _Executor:
    def __init__(self, falhar_em=None):
        self.eventos = []
        self.no = _Node(self.eventos)
        self.cena = _Cena(self.eventos, falhar_em)
        self.deteccao = AprilTagStampedDetection()
        self.deteccao.header.frame_id = "base_link"
        self.deteccao.family = "tag36h11"
        self.deteccao.id = 1
        self.deteccao.pose.orientation.w = 1.0

    def aguardar_o_servidor(self):
        self.eventos.append(("sistema", "aguardar"))

    def mover_para_estado(self, grupo, estado, _descricao, **_opcoes):
        self.eventos.append(("estado", grupo, estado))

    def obter_deteccoes_de_april_tags(self, _duracao):
        self.eventos.append(("deteccao", "analisar"))
        return [self.deteccao]

    def obter_pose_da_deteccao(self, _deteccao):
        return 0.1, -0.2, 0.1, 0.0

    _selecionar_april_tag = staticmethod(ExecutorDoMoveIt._selecionar_april_tag)

    def executar_objetivo(self, grupo, _restricoes, _velocidade, _aceleracao):
        self.eventos.append(("objetivo", grupo))


def _posicao(eventos, evento):
    return eventos.index(evento)


def test_ordem_da_fotografia_pegada_transporte_e_deposito():
    executor = _Executor()

    PegarEColocar(executor).executar()

    eventos = executor.eventos
    sincronizar = _posicao(eventos, ("cena", "sincronizar"))
    permitir = _posicao(eventos, ("cena", "permitir"))
    grip = _posicao(eventos, ("estado", "gripper", "grip"))
    ignorar = _posicao(eventos, ("cena", "ignorar_alvo"))
    restaurar = _posicao(eventos, ("cena", "restaurar"))
    depositar = _posicao(eventos, ("estado", "arm", "deposit_cube_left"))
    abrir = len(eventos) - 1 - eventos[::-1].index(("estado", "gripper", "open"))
    ultimo_home = len(eventos) - 1 - eventos[::-1].index(("estado", "arm", "home"))
    assert sincronizar < permitir < grip < ignorar < restaurar < depositar
    assert depositar < abrir < ultimo_home


def test_falha_ao_ignorar_alvo_restaura_matriz_e_impede_transporte():
    executor = _Executor(falhar_em="ignorar_alvo")

    with pytest.raises(RuntimeError, match="remoção do alvo falhou"):
        PegarEColocar(executor).executar()

    assert ("cena", "restaurar") in executor.eventos
    assert ("estado", "arm", "deposit_cube_left") not in executor.eventos


def test_falha_na_sincronizacao_impede_primeira_aproximacao():
    executor = _Executor(falhar_em="sincronizar")

    with pytest.raises(RuntimeError, match="sincronização falhou"):
        PegarEColocar(executor).executar()

    assert not any(evento[0] == "objetivo" for evento in executor.eventos)


def test_falha_na_restauracao_apos_ignorar_alvo_impede_transporte():
    executor = _Executor(falhar_em="restaurar")

    with pytest.raises(RuntimeError, match="restauração falhou"):
        PegarEColocar(executor).executar()

    assert ("cena", "ignorar_alvo") in executor.eventos
    assert ("estado", "arm", "deposit_cube_left") not in executor.eventos
    assert any(
        evento[0] == "erro" and "transporte não será iniciado" in evento[1]
        for evento in executor.eventos
    )
