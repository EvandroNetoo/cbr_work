"""Controle unificado dos motores do brick e da expansão por velocidade."""

from abc import ABC, abstractmethod
from collections.abc import Iterable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor, as_completed
from numbers import Real
import threading
from types import TracebackType

from .motores import Motores
from .placaControleMotor import PlacaControleMotor


class ErroControleMotores(RuntimeError):
    """Erro ao enviar velocidade para um ou mais grupos de motores."""

    def __init__(
        self,
        falhas: Mapping[str, Exception],
        falhas_freio: Mapping[str, Exception] | None = None,
    ) -> None:
        self.falhas = falhas
        self.falhas_freio = falhas_freio or {}

        detalhes = "; ".join(
            f"{grupo}: {erro}" for grupo, erro in falhas.items()
        )
        mensagem = f"Falha ao controlar motores ({detalhes})"

        if self.falhas_freio:
            detalhes_freio = "; ".join(
                f"{grupo}: {erro}"
                for grupo, erro in self.falhas_freio.items()
            )
            mensagem += f"; também houve falha ao frear ({detalhes_freio})"

        super().__init__(mensagem)


def _validar_nome(nome: str) -> str:
    if not isinstance(nome, str) or not nome.strip():
        raise ValueError("O nome de cada motor deve ser uma string não vazia")
    return nome


def _normalizar_velocidade(valor: Real) -> int:
    if isinstance(valor, bool) or not isinstance(valor, Real):
        raise TypeError("A velocidade deve ser um número entre -100 e 100")
    return max(-100, min(100, int(round(valor))))


class GrupoMotores(ABC):
    """Interface comum para um grupo de motores nomeados."""

    nomes: tuple[str, ...]
    nome_grupo: str

    def __init__(self, nomes: Iterable[str], nome_grupo: str) -> None:
        self.nomes = tuple(nomes)
        self.nome_grupo = nome_grupo

    @abstractmethod
    def aplicar(self, velocidades: Mapping[str, float]) -> None:
        """Aplica as velocidades dos motores pertencentes ao grupo."""

    @abstractmethod
    def definir_freio(self, travado: bool) -> None:
        """Define se o freio dos motores do grupo deve ficar travado."""

    @abstractmethod
    def angulo_motor(self, nome: str) -> int:
        """Retorna o ângulo relativo de um motor."""

    @abstractmethod
    def angulos_motores(self) -> dict[str, int]:
        """Retorna os ângulos relativos de todos os motores do grupo."""

    @abstractmethod
    def reseta_angulo_motor(self, nome: str) -> None:
        """Zera a referência relativa de um motor."""

    @abstractmethod
    def reseta_angulos_motores(self) -> None:
        """Zera as referências relativas de todos os motores do grupo."""


class GrupoMotoresBrick(GrupoMotores):
    """Adapta os dois motores do brick para o controle por velocidade."""

    def __init__(
        self,
        controlador: Motores,
        nomes: Sequence[str],
        invertidos: Sequence[bool] = (False, False),
    ) -> None:
        if len(nomes) != 2:
            raise ValueError("GrupoMotoresBrick requer exatamente dois nomes")
        if len(invertidos) != 2:
            raise ValueError("GrupoMotoresBrick requer duas configurações de inversão")

        self.controlador = controlador
        nomes_validados = tuple(_validar_nome(nome) for nome in nomes)
        if len(set(nomes_validados)) != 2:
            raise ValueError("Os nomes dos motores do brick devem ser diferentes")

        self._invertidos = tuple(bool(valor) for valor in invertidos)
        self._indice_por_nome = {
            nome: indice
            for indice, nome in enumerate(nomes_validados, start=1)
        }
        super().__init__(
            nomes_validados,
            f"motores do brick ({self.controlador.ser})",
        )

    def aplicar(self, velocidades: Mapping[str, float]) -> None:
        valores = [
            -velocidades[nome] if invertido else velocidades[nome]
            for nome, invertido in zip(self.nomes, self._invertidos)
        ]

        # O protocolo do brick envia os dois motores no mesmo pacote.
        self.controlador.velocidade_motores(valores[0], valores[1])

    def definir_freio(self, travado: bool) -> None:
        modo = Motores.HOLD if travado else Motores.BREAK
        self.controlador.set_modo_freio(modo)

        # set_modo_freio altera o pacote em memória; atualiza_motores envia
        # esse pacote sem modificar as velocidades configuradas.
        self.controlador.atualiza_motores()

    def _angulo_atual(self, nome: str) -> int:
        indice = self._indice_por_nome[nome]
        angulo = self.controlador.angulo_motor(indice)
        if self._invertidos[indice - 1]:
            return -angulo
        return angulo

    def angulo_motor(self, nome: str) -> int:
        # Atualiza somente o estado; não altera as velocidades configuradas.
        self.controlador.estado()
        return self._angulo_atual(nome)

    def angulos_motores(self) -> dict[str, int]:
        # Uma única consulta atualiza os encoders dos dois motores do brick.
        self.controlador.estado()
        return {nome: self._angulo_atual(nome) for nome in self.nomes}

    def reseta_angulo_motor(self, nome: str) -> None:
        self.controlador.reseta_angulo_motor(self._indice_por_nome[nome])

    def reseta_angulos_motores(self) -> None:
        for nome in self.nomes:
            self.reseta_angulo_motor(nome)


class GrupoMotoresExpansao(GrupoMotores):
    """Adapta motores individuais no barramento de expansão."""

    def __init__(
        self,
        motores: Mapping[str, PlacaControleMotor],
        invertidos: Mapping[str, bool] | None = None,
    ) -> None:
        if not motores:
            raise ValueError("GrupoMotoresExpansao requer ao menos um motor")

        self._motores: dict[str, PlacaControleMotor] = {}
        for nome, motor in motores.items():
            nome = _validar_nome(nome)
            if nome in self._motores:
                raise ValueError(f"Motor duplicado: {nome}")
            self._motores[nome] = motor

        invertidos = invertidos or {}
        desconhecidos = set(invertidos) - set(self._motores)
        if desconhecidos:
            raise ValueError(
                "Inversões configuradas para motores desconhecidos: "
                + ", ".join(sorted(desconhecidos))
            )

        self._invertidos = {
            nome: bool(invertidos.get(nome, False)) for nome in self._motores
        }
        super().__init__(self._motores, "motores de expansão")

    def aplicar(self, velocidades: Mapping[str, float]) -> None:
        # Os motores compartilham a mesma serial. A sequência garante que a
        # escrita, o eco e a resposta terminem antes do próximo comando.
        for nome, motor in self._motores.items():
            velocidade = velocidades[nome]
            if self._invertidos[nome]:
                velocidade = -velocidade

            resultado = motor.velocidade_motor(velocidade)
            if resultado is None:
                raise RuntimeError(f"motor '{nome}' não respondeu")

    def definir_freio(self, travado: bool) -> None:
        modo = (
            PlacaControleMotor.FREIO_TRAVADO
            if travado
            else PlacaControleMotor.FREIO_BREAK
        )
        for nome, motor in self._motores.items():
            if motor.set_freio(modo) is None:
                raise RuntimeError(f"motor '{nome}' não respondeu")

    def angulo_motor(self, nome: str) -> int:
        angulo = self._motores[nome].angulo_motor()
        if self._invertidos[nome]:
            return -angulo
        return angulo

    def angulos_motores(self) -> dict[str, int]:
        # As leituras permanecem sequenciais porque usam a mesma serial.
        return {nome: self.angulo_motor(nome) for nome in self.nomes}

    def reseta_angulo_motor(self, nome: str) -> None:
        self._motores[nome].reseta_angulo_motor()

    def reseta_angulos_motores(self) -> None:
        # Os resets permanecem sequenciais porque usam a mesma serial.
        for nome in self.nomes:
            self.reseta_angulo_motor(nome)


class ControleMotores:
    """Controla por velocidade motores nomeados em diferentes barramentos."""

    def __init__(
        self,
        grupos: Iterable[GrupoMotores],
        freio_travado: bool = False,
    ) -> None:
        self._grupos = tuple(grupos)
        if not self._grupos:
            raise ValueError("É necessário configurar ao menos um grupo de motores")

        nomes: list[str] = []
        self._grupo_por_nome: dict[str, GrupoMotores] = {}
        for grupo in self._grupos:
            if not isinstance(grupo, GrupoMotores):
                raise TypeError("Grupo de motores inválido")
            nomes.extend(grupo.nomes)
            for nome in grupo.nomes:
                self._grupo_por_nome[nome] = grupo

        duplicados = sorted({nome for nome in nomes if nomes.count(nome) > 1})
        if duplicados:
            raise ValueError("Nomes de motores duplicados: " + ", ".join(duplicados))

        self._velocidades = {nome: 0 for nome in nomes}
        self._lock = threading.RLock()
        self._fechado = False
        self._executor = ThreadPoolExecutor(
            max_workers=len(self._grupos),
            thread_name_prefix="controle-motores",
        )
        try:
            self.definir_freio(freio_travado)
        except Exception:
            self._fechado = True
            self._executor.shutdown(wait=True)
            raise

    @property
    def nomes(self) -> tuple[str, ...]:
        return tuple(self._velocidades)

    @property
    def velocidades_atuais(self) -> dict[str, int]:
        with self._lock:
            return dict(self._velocidades)

    def definir_velocidades(self, **velocidades: float) -> None:
        """Atualiza velocidades pelo nome; motores omitidos mantêm seu valor."""
        if not velocidades:
            return

        with self._lock:
            self._garantir_aberto()
            desconhecidos = set(velocidades) - set(self._velocidades)
            if desconhecidos:
                raise ValueError(
                    "Motores desconhecidos: " + ", ".join(sorted(desconhecidos))
                )

            novo_estado = dict(self._velocidades)
            for nome, valor in velocidades.items():
                novo_estado[nome] = _normalizar_velocidade(valor)

            grupos_afetados = [
                grupo
                for grupo in self._grupos
                if any(nome in velocidades for nome in grupo.nomes)
            ]
            falhas = self._executar(novo_estado, grupos_afetados)

            if falhas:
                estado_parado = {nome: 0 for nome in self._velocidades}
                self._velocidades = estado_parado
                falhas_freio = self._executar(estado_parado, self._grupos)
                raise ErroControleMotores(falhas, falhas_freio)

            self._velocidades = novo_estado

    def definir_velocidade(self, nome: str, velocidade: float) -> None:
        self.definir_velocidades(**{nome: velocidade})

    def definir_freio(self, travado: bool) -> None:
        """Trava ou libera o freio de todos os grupos de motores."""
        with self._lock:
            self._garantir_aberto()
            futuros = {
                self._executor.submit(grupo.definir_freio, travado): grupo
                for grupo in self._grupos
            }

            falhas: dict[str, Exception] = {}
            for futuro in as_completed(futuros):
                grupo = futuros[futuro]
                try:
                    futuro.result()
                except Exception as exc:
                    falhas[self._nome_grupo(grupo)] = exc

            if falhas:
                raise ErroControleMotores(falhas)

    def angulo_motor(self, nome: str) -> int:
        """Retorna os pulsos relativos do encoder de um motor nomeado."""
        with self._lock:
            self._garantir_aberto()
            grupo = self._obter_grupo(nome)
            try:
                return grupo.angulo_motor(nome)
            except Exception as exc:
                raise ErroControleMotores(
                    {self._nome_grupo(grupo): exc}
                ) from exc

    def angulos_motores(self) -> dict[str, int]:
        """Retorna os pulsos relativos dos encoders de todos os motores."""
        with self._lock:
            self._garantir_aberto()
            futuros = {
                self._executor.submit(grupo.angulos_motores): grupo
                for grupo in self._grupos
            }

            angulos: dict[str, int] = {}
            falhas: dict[str, Exception] = {}
            for futuro in as_completed(futuros):
                grupo = futuros[futuro]
                try:
                    angulos.update(futuro.result())
                except Exception as exc:
                    falhas[self._nome_grupo(grupo)] = exc

            if falhas:
                raise ErroControleMotores(falhas)
            return {nome: angulos[nome] for nome in self._velocidades}

    def reseta_angulo_motor(self, nome: str) -> None:
        """Zera a referência relativa do encoder de um motor nomeado."""
        with self._lock:
            self._garantir_aberto()
            grupo = self._obter_grupo(nome)
            try:
                grupo.reseta_angulo_motor(nome)
            except Exception as exc:
                raise ErroControleMotores(
                    {self._nome_grupo(grupo): exc}
                ) from exc

    def reseta_angulos_motores(self) -> None:
        """Zera simultaneamente as referências dos quatro encoders."""
        with self._lock:
            self._garantir_aberto()
            futuros = {
                self._executor.submit(grupo.reseta_angulos_motores): grupo
                for grupo in self._grupos
            }

            falhas: dict[str, Exception] = {}
            for futuro in as_completed(futuros):
                grupo = futuros[futuro]
                try:
                    futuro.result()
                except Exception as exc:
                    falhas[self._nome_grupo(grupo)] = exc

            if falhas:
                raise ErroControleMotores(falhas)

    def frear(self) -> None:
        with self._lock:
            self._garantir_aberto()
            estado_parado = {nome: 0 for nome in self._velocidades}
            falhas = self._executar(estado_parado, self._grupos)
            self._velocidades = estado_parado

            if falhas:
                # Repete a tentativa como frenagem de emergência.
                falhas_freio = self._executar(estado_parado, self._grupos)
                raise ErroControleMotores(falhas, falhas_freio)

    def fechar(self, parar_motores: bool = True) -> None:
        with self._lock:
            if self._fechado:
                return

            erro: ErroControleMotores | None = None
            try:
                if parar_motores:
                    self.frear()
            except ErroControleMotores as exc:
                erro = exc
            finally:
                self._fechado = True
                self._executor.shutdown(wait=True)

            if erro is not None:
                raise erro

    def _executar(
        self,
        estado: Mapping[str, int],
        grupos: Iterable[GrupoMotores],
    ) -> dict[str, Exception]:
        futuros = {
            self._executor.submit(
                grupo.aplicar,
                {nome: estado[nome] for nome in grupo.nomes},
            ): grupo
            for grupo in grupos
        }

        falhas: dict[str, Exception] = {}
        for futuro in as_completed(futuros):
            grupo = futuros[futuro]
            try:
                futuro.result()
            except Exception as exc:
                falhas[self._nome_grupo(grupo)] = exc
        return falhas

    def _obter_grupo(self, nome: str) -> GrupoMotores:
        try:
            return self._grupo_por_nome[nome]
        except KeyError:
            raise ValueError(f"Motor desconhecido: {nome}") from None

    @staticmethod
    def _nome_grupo(grupo: GrupoMotores) -> str:
        return grupo.nome_grupo

    def _garantir_aberto(self) -> None:
        if self._fechado:
            raise RuntimeError("O controle de motores já foi fechado")

    def __enter__(self) -> "ControleMotores":
        return self

    def __exit__(
        self,
        tipo: type[BaseException] | None,
        valor: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.fechar()

