"""Unified speed controller copied from MariolaZero's expansion example.

Source: ``MariolaZero/exemplos/10-expansaoMotor/controleMotores.py``.
Only the module name follows Python/ROS snake_case conventions.
"""

from concurrent.futures import as_completed, ThreadPoolExecutor
from numbers import Real
import threading


class ErroControleMotores(RuntimeError):
    """Erro ao enviar velocidade para um ou mais grupos de motores."""

    def __init__(self, falhas, falhas_freio=None):
        self.falhas = falhas
        self.falhas_freio = falhas_freio or {}

        detalhes = '; '.join(
            f'{grupo}: {erro}' for grupo, erro in falhas.items()
        )
        mensagem = f'Falha ao controlar motores ({detalhes})'

        if self.falhas_freio:
            detalhes_freio = '; '.join(
                f'{grupo}: {erro}'
                for grupo, erro in self.falhas_freio.items()
            )
            mensagem += f'; também houve falha ao frear ({detalhes_freio})'

        super().__init__(mensagem)


def _validar_nome(nome):
    if not isinstance(nome, str) or not nome.strip():
        raise ValueError('O nome de cada motor deve ser uma string não vazia')
    return nome


def _normalizar_velocidade(valor):
    if isinstance(valor, bool) or not isinstance(valor, Real):
        raise TypeError('A velocidade deve ser um número entre -100 e 100')
    return max(-100, min(100, int(round(valor))))


class GrupoMotoresBrick:
    """Adapta os dois motores do brick para o controle por velocidade."""

    def __init__(self, controlador, nomes, invertidos=(False, False)):
        if len(nomes) != 2:
            raise ValueError('GrupoMotoresBrick requer exatamente dois nomes')
        if len(invertidos) != 2:
            raise ValueError(
                'GrupoMotoresBrick requer duas configurações de inversão')

        self.controlador = controlador
        self.nomes = tuple(_validar_nome(nome) for nome in nomes)
        if len(set(self.nomes)) != 2:
            raise ValueError(
                'Os nomes dos motores do brick devem ser diferentes')

        self._invertidos = tuple(bool(valor) for valor in invertidos)
        self.nome_grupo = 'motores do brick (SERIAL0)'

    def aplicar(self, velocidades):
        valores = [
            -velocidades[nome] if invertido else velocidades[nome]
            for nome, invertido in zip(self.nomes, self._invertidos)
        ]

        # O protocolo do brick envia os dois motores no mesmo pacote.
        self.controlador.velocidade_motores(valores[0], valores[1])


class GrupoMotoresExpansao:
    """Adapta motores individuais no mesmo barramento de expansão."""

    def __init__(self, motores, invertidos=None):
        if not motores:
            raise ValueError('GrupoMotoresExpansao requer ao menos um motor')

        self._motores = {}
        for nome, motor in motores.items():
            nome = _validar_nome(nome)
            if nome in self._motores:
                raise ValueError(f'Motor duplicado: {nome}')
            self._motores[nome] = motor

        invertidos = invertidos or {}
        desconhecidos = set(invertidos) - set(self._motores)
        if desconhecidos:
            raise ValueError(
                'Inversões configuradas para motores desconhecidos: '
                + ', '.join(sorted(desconhecidos))
            )

        self._invertidos = {
            nome: bool(invertidos.get(nome, False)) for nome in self._motores
        }
        self.nomes = tuple(self._motores)
        self.nome_grupo = 'motores de expansão'

    def aplicar(self, velocidades):
        # Todos compartilham a mesma serial. A sequência é intencional para que
        # escrita, eco e resposta de um motor terminem antes do motor seguinte.
        for nome, motor in self._motores.items():
            velocidade = velocidades[nome]
            if self._invertidos[nome]:
                velocidade = -velocidade

            resultado = motor.velocidade_motor(velocidade)
            if resultado is None:
                raise RuntimeError(f"motor '{nome}' não respondeu")


class ControleMotores:
    """Controla por velocidade motores nomeados em diferentes barramentos."""

    def __init__(self, grupos):
        self._grupos = tuple(grupos)
        if not self._grupos:
            raise ValueError(
                'É necessário configurar ao menos um grupo de motores')

        nomes = []
        for grupo in self._grupos:
            if not hasattr(grupo, 'nomes') or not callable(
                getattr(grupo, 'aplicar', None)
            ):
                raise TypeError('Grupo de motores inválido')
            nomes.extend(grupo.nomes)

        duplicados = sorted({nome for nome in nomes if nomes.count(nome) > 1})
        if duplicados:
            raise ValueError(
                'Nomes de motores duplicados: ' + ', '.join(duplicados))

        self._velocidades = {nome: 0 for nome in nomes}
        self._lock = threading.RLock()
        self._fechado = False
        self._executor = ThreadPoolExecutor(
            max_workers=len(self._grupos),
            thread_name_prefix='controle-motores',
        )

    @property
    def nomes(self):
        return tuple(self._velocidades)

    @property
    def velocidades_atuais(self):
        with self._lock:
            return dict(self._velocidades)

    def definir_velocidades(self, **velocidades):
        """Atualiza velocidades pelo nome; motores omitidos mantêm seu valor."""
        if not velocidades:
            return

        with self._lock:
            self._garantir_aberto()
            desconhecidos = set(velocidades) - set(self._velocidades)
            if desconhecidos:
                raise ValueError(
                    'Motores desconhecidos: '
                    + ', '.join(sorted(desconhecidos))
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
                falhas_freio = self._executar(
                    estado_parado, self._grupos)
                raise ErroControleMotores(falhas, falhas_freio)

            self._velocidades = novo_estado

    def definir_velocidade(self, nome, velocidade):
        self.definir_velocidades(**{nome: velocidade})

    def frear(self):
        with self._lock:
            self._garantir_aberto()
            estado_parado = {nome: 0 for nome in self._velocidades}
            falhas = self._executar(estado_parado, self._grupos)
            self._velocidades = estado_parado
            if falhas:
                # Repete a tentativa como frenagem de emergência.
                falhas_freio = self._executar(
                    estado_parado, self._grupos)
                raise ErroControleMotores(falhas, falhas_freio)

    def fechar(self):
        with self._lock:
            if self._fechado:
                return

            erro = None
            try:
                self.frear()
            except ErroControleMotores as exc:
                erro = exc
            finally:
                self._fechado = True
                self._executor.shutdown(wait=True)

            if erro is not None:
                raise erro

    def _executar(self, estado, grupos):
        futuros = {
            self._executor.submit(
                grupo.aplicar,
                {nome: estado[nome] for nome in grupo.nomes},
            ): grupo
            for grupo in grupos
        }

        falhas = {}
        for futuro in as_completed(futuros):
            grupo = futuros[futuro]
            try:
                futuro.result()
            except Exception as exc:
                nome_grupo = getattr(
                    grupo, 'nome_grupo', grupo.__class__.__name__)
                falhas[nome_grupo] = exc
        return falhas

    def _garantir_aberto(self):
        if self._fechado:
            raise RuntimeError('O controle de motores já foi fechado')

    def __enter__(self):
        return self

    def __exit__(self, tipo, valor, traceback):
        self.fechar()
