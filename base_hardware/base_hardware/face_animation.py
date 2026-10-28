"""Rostos pixelados do OLED, com animação contínua sem bloquear o ROS."""

from dataclasses import dataclass, fields, replace
import math
import time

from PIL import Image, ImageChops, ImageDraw


# Traços assimétricos reproduzem as sobrancelhas da referência.
_ARCO_E = ((-11, 3), (-8, 0), (-4, -2), (3, -2))
_ARCO_D = ((-3, -2), (4, -2), (8, 0), (11, 3))
_BRAVO_E = ((-11, -3), (-5, -1), (1, 2), (8, 5))
_BRAVO_D = ((-8, 5), (-1, 2), (5, -1), (11, -3))
_RETO = ((-8, 0), (-3, 0), (3, 0), (8, 0))
_PREOCUPADO_E = ((-10, 4), (-5, 2), (0, -1), (3, -5))
_PREOCUPADO_D = ((-3, -5), (0, -1), (5, 2), (10, 4))


@dataclass(frozen=True)
class _Rosto:
    olhos: tuple = (1.0, 1.0)
    arcos: tuple = (0.0, 0.0)
    apertos: tuple = (0.0, 0.0)
    palpebras: tuple = (0.0, 0.0)
    olhar: tuple = (0.0, 0.0)
    desnivel: tuple = (0.0, 0.0)
    sobrancelhas: tuple = (_ARCO_E, _ARCO_D)
    boca: str = 'sorriso'
    largura: float = 10.0
    altura: float = 5.0
    lingua: float = 0.0
    suor: float = 0.0
    suor_x: float = 113.0
    brilhos: float = 0.0
    tracos: float = 0.0  # Negativo: lado esquerdo; positivo: lado direito.
    parede: float = 0.0
    mesa: float = 0.0
    deslocamento: float = 0.0
    vertical: float = 0.0
    cena: str = ''
    queda: float = 0.0
    fase_slots: float = 0.0
    fogos: float = 0.0
    garra_y: float = -18.0
    garra_abertura: float = 1.0


@dataclass(frozen=True)
class _TemposGarra:
    aproximacao: float
    fechamento: float
    movimento: float
    abertura: float
    recuo: float
    queda_livre: float = 0.0
    pausa: float = 0.3

    @property
    def inicio_movimento(self):
        return self.aproximacao + self.fechamento

    @property
    def fim_movimento(self):
        return self.inicio_movimento + self.movimento

    @property
    def fim_cubo(self):
        return self.fim_movimento + (self.abertura + self.queda_livre if self.queda_livre else 0)

    @property
    def fim_recuo(self):
        return self.fim_movimento + self.abertura + self.recuo

    @property
    def ciclo(self):
        return max(self.fim_cubo, self.fim_recuo) + self.pausa


def _mistura(a, b, peso):
    """Interpola medidas e traços; troca o tipo de boca no meio da transição."""
    if isinstance(a, tuple):
        return tuple(_mistura(x, y, peso) for x, y in zip(a, b))
    if isinstance(a, (float, int)):
        return a + (b - a) * peso
    return a if peso < 0.5 else b


def _interpolar(a, b, peso):
    return _Rosto(**{f.name: _mistura(getattr(a, f.name), getattr(b, f.name), peso)
                     for f in fields(_Rosto)})


class ExpressoesFaciais:
    _POSES = {
        'navigate': _Rosto(sobrancelhas=(_BRAVO_E, _BRAVO_D), boca='cheio', altura=6),
        'finish': _Rosto(arcos=(9, 9), boca='cheio', largura=15, altura=13,
                          vertical=-2, fogos=1),
        'follow_wall': _Rosto(sobrancelhas=(_PREOCUPADO_E, _ARCO_D),
                              boca='torto', parede=1, deslocamento=-4),
        'analyze_scene': _Rosto(olhos=(1, 0.65), palpebras=(0, 0.28), olhar=(0, 6),
                                sobrancelhas=(_ARCO_E, _RETO), boca='triste',
                                largura=7, altura=4),
        'pick': _Rosto(olhar=(0, 7), sobrancelhas=(_BRAVO_E, _BRAVO_D), boca='linha',
                       largura=7, altura=0, lingua=1, cena='pick', deslocamento=-10),
        'store': _Rosto(boca='oval', largura=4, altura=6,
                         cena='store', deslocamento=-10),
        'retrieve': _Rosto(boca='oval', largura=4, altura=6,
                            cena='retrieve', deslocamento=-10),
        'place_on_table': _Rosto(boca='vazado', largura=12, altura=10,
                                  cena='place_on_table', deslocamento=-10),
        'place_in_container': _Rosto(boca='triste', largura=4, altura=3,
                                     cena='container', deslocamento=-10),
        'place_on_shelf': _Rosto(olhar=(0, -7),
                                  sobrancelhas=(_PREOCUPADO_E, _PREOCUPADO_D),
                                  largura=7, altura=4, suor=1),
        'place_on_precision_table': _Rosto(apertos=(1, 1),
                                          sobrancelhas=(_RETO, _RETO),
                                          boca='linha', largura=7, altura=0, lingua=1, suor=1),
        'stack': _Rosto(olhar=(0, -5), desnivel=(-2, 3),
                        sobrancelhas=(_PREOCUPADO_E, _PREOCUPADO_D),
                        boca='ondulada', largura=12, altura=3, suor=1, suor_x=96,
                        cena='stack', deslocamento=-10),
        'organize_precision': _Rosto(palpebras=(0.46, 0.46), olhar=(0, 6),
                                     sobrancelhas=(_BRAVO_E, _BRAVO_D), boca='triste',
                                     largura=9, altura=4, vertical=-10,
                                     suor=1, cena='slots'),
        'piscar': _Rosto(olhos=(0.08, 0.08)),
        'piscadinha': _Rosto(olhos=(0.08, 1), sobrancelhas=(_ARCO_E, _PREOCUPADO_D)),
        'comemorar': _Rosto(arcos=(9, 9), boca='cheio', largura=15, altura=13,
                             vertical=-2, fogos=1),
    }
    # Poses da referência usadas nas sequências, sem setas.
    _QUADROS = {
        'navigate': (
            _Rosto(olhar=(-9, 0), desnivel=(-2, 2),
                    sobrancelhas=(_ARCO_E, _PREOCUPADO_D),
                    boca='anel', largura=5, altura=5, tracos=-1),
            _Rosto(olhar=(9, 0), sobrancelhas=(_PREOCUPADO_E, _ARCO_D),
                    boca='vazado', largura=12, altura=10, tracos=1),
            _POSES['navigate'],
        ),
        'follow_wall': (
            replace(_POSES['follow_wall'], olhar=(7, 6)),
            replace(_POSES['follow_wall'], olhar=(5, -6), desnivel=(3, -4),
                    boca='zigue', largura=4, altura=4, suor=1, suor_x=10),
        ),
        'place_on_table': (
            replace(_POSES['place_on_table'], arcos=(8, 8), brilhos=0.55),
        ),
    }
    _ALIASES = {
        'neutro': 'navigate', 'feliz': 'finish', 'triste': 'place_in_container',
        'bravo': 'pick', 'surpreso': 'retrieve', 'dormindo': 'piscar',
        'piscando': 'piscadinha', 'correndo': 'navigate',
    }
    EXPRESSOES = tuple(_POSES) + tuple(_ALIASES)
    _CICLO = 14.0
    # Referência original do stack: 34 pixels em 1,5 s, mantida nos novos trajetos.
    _VELOCIDADE_CUBO = 34 / 1.5
    _VELOCIDADE_GARRA = 2 * _VELOCIDADE_CUBO
    _GARRA_FORA = -18.0
    _CENAS_GARRA = ('pick', 'place_on_table', 'container', 'stack')
    _SOLTURA_CONTAINER = 34.0  # Cubo e pontas da garra ficam acima da borda em y=44.
    # Segundos por quadro, duração da transição e ordem (-1 = padrão).
    _SEQUENCIAS = {
        'navigate': (0.70, 0.25, (2, 0, 2, 1)),
        'follow_wall': (0.80, 0.25, (0, 1)),
    }

    def __init__(self, tela, fps=20):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('FPS deve ser um número positivo e finito.')
        self.tela = tela
        self.fps = fps
        self._inicio = time.monotonic()
        self._ultimo_quadro = None
        self._expressao = 'navigate'
        self._origem = self._POSES['navigate']
        self._inicio_transicao = self._inicio
        self._inicio_expressao = self._inicio
        self._tempo_transicao = 0.4

    @staticmethod
    def _janela(tempo, centro):
        """0,4 s na pose, com entrada/saída suave de 0,35 s cada."""
        distancia = abs(tempo - centro)
        if distancia <= 0.2:
            return 1.0
        if distancia >= 0.55:
            return 0.0
        u = (0.55 - distancia) / 0.35
        return u * u * (3 - 2 * u)

    def _alvo_animado(self, tempo):
        base = self._POSES[self._expressao]
        if self._expressao in self._SEQUENCIAS:
            intervalo, transicao, ordem = self._SEQUENCIAS[self._expressao]
            posicao = tempo / intervalo
            indice = math.floor(posicao) % len(ordem)
            fase = (posicao % 1) * intervalo
            quadros = self._QUADROS[self._expressao]
            atual = base if ordem[indice] == -1 else quadros[ordem[indice]]
            proximo_indice = ordem[(indice + 1) % len(ordem)]
            proximo = base if proximo_indice == -1 else quadros[proximo_indice]
            # Sustenta cada pose brevemente e já passa à próxima, sem pausa extra.
            peso = max(0.0, (fase - intervalo + transicao) / transicao)
            peso = peso * peso * (3 - 2 * peso)
            return _interpolar(atual, proximo, peso)

        if base.cena in self._CENAS_GARRA:
            return self._alvo_manipulacao(base, tempo)
        if self._expressao in ('analyze_scene', 'place_on_shelf', 'organize_precision'):
            periodo = 2.4 if self._expressao == 'place_on_shelf' else 1.8
            x = 7 * math.sin(2 * math.pi * tempo / periodo)
            ciclo_slots = 12 * (0.12 + self._duracao_cubo('slots'))
            return replace(base, olhar=(x, base.olhar[1]), fase_slots=tempo % ciclo_slots)
        if base.cena in ('retrieve', 'store'):
            duracao = self._duracao_cubo(base.cena)
            progresso = (tempo % duracao) / duracao
            _, y = self._posicao_transferencia(base.cena, progresso)
            olhar_y = max(-7, min(7, (y - 31) * 0.4))
            return replace(base, queda=progresso, olhar=(8, olhar_y))
        fase = tempo % self._CICLO
        # Os demais rostos mantêm o olhar frontal e a intenção vertical original.
        lado = self._janela(fase, 5.2) - self._janela(fase, 10.2)
        base = replace(base, olhar=(6 * lado, base.olhar[1]))
        if self._expressao == 'piscar':
            abertura = 1 - 0.92 * self._pulso(tempo, 2.2, 1.1)
            base = replace(base, olhos=(abertura, abertura))
        elif self._expressao == 'piscadinha':
            base = replace(base, olhos=(1 - 0.92 * self._pulso(tempo, 2.4, 1.1), 1))
        return base

    @staticmethod
    def _suave(progresso):
        progresso = min(1.0, max(0.0, progresso))
        return progresso * progresso * (3 - 2 * progresso)

    @classmethod
    def _posicao_cubo(cls, cena, progresso):
        posicao = (cls._posicao_transferencia if cena in ('retrieve', 'store')
                   else cls._posicao_queda)
        return posicao(cena, progresso)

    @classmethod
    def _ponto_soltura(cls, cena):
        return cls._SOLTURA_CONTAINER if cena == 'container' else cls._posicao_cubo(cena, 1)[1]

    @classmethod
    def _tempos_garra(cls, cena):
        coleta = cena == 'pick'
        inicio_y = cls._posicao_cubo(cena, 0)[1]
        destino_y = cls._posicao_cubo(cena, 1)[1]
        soltura_y = cls._ponto_soltura(cena)
        aproximacao = max(0, inicio_y - 11 - cls._GARRA_FORA) / cls._VELOCIDADE_GARRA
        recuo = 0 if coleta else max(0, soltura_y - 11 - cls._GARRA_FORA) / cls._VELOCIDADE_GARRA
        return _TemposGarra(
            aproximacao=aproximacao,
            fechamento=0.2 if aproximacao > 0 else 0,
            movimento=abs(soltura_y - inicio_y) / cls._VELOCIDADE_CUBO,
            abertura=0 if coleta else 0.25,
            recuo=recuo,
            queda_livre=abs(destino_y - soltura_y) / cls._VELOCIDADE_CUBO,
            pausa=1.0 if cena == 'place_on_table' else 0.3,
        )

    def _alvo_manipulacao(self, base, tempo):
        tempos = self._tempos_garra(base.cena)
        fase = tempo % tempos.ciclo
        inicio_y = self._posicao_cubo(base.cena, 0)[1]
        destino_y = self._posicao_cubo(base.cena, 1)[1]
        soltura_y = self._ponto_soltura(base.cena)
        if fase < tempos.aproximacao:
            peso = self._suave(fase / tempos.aproximacao)
            cubo_y, abertura = inicio_y, 1.0
            garra_y = self._GARRA_FORA + (inicio_y - 11 - self._GARRA_FORA) * peso
        elif fase < tempos.inicio_movimento:
            cubo_y, garra_y = inicio_y, inicio_y - 11
            abertura = 1 - self._suave((fase - tempos.aproximacao) / tempos.fechamento)
        elif fase < tempos.fim_movimento:
            progresso = (fase - tempos.inicio_movimento) / tempos.movimento
            progresso = self._suave(progresso) if base.cena == 'pick' else progresso ** 2
            cubo_y = inicio_y + (soltura_y - inicio_y) * progresso
            garra_y, abertura = cubo_y - 11, 0.0
        elif fase < tempos.fim_movimento + tempos.abertura:
            cubo_y, garra_y = soltura_y, soltura_y - 11
            abertura = self._suave((fase - tempos.fim_movimento) / tempos.abertura)
        else:
            depois_soltura = fase - tempos.fim_movimento - tempos.abertura
            queda = min(1.0, depois_soltura / tempos.queda_livre) ** 2 if tempos.queda_livre else 1
            cubo_y = soltura_y + (destino_y - soltura_y) * queda
            peso = self._suave(depois_soltura / tempos.recuo) if tempos.recuo else 1
            garra_y = soltura_y - 11 + (self._GARRA_FORA - soltura_y + 11) * peso
            abertura = 1.0
        progresso = min(1.0, max(0.0, (cubo_y - inicio_y) / (destino_y - inicio_y)))
        # No recipiente, segue o cubo durante o transporte e a queda livre.
        alvo_y = cubo_y if base.cena == 'container' else garra_y + 7
        olhar_y = max(-7, min(7, (alvo_y - 31) * 0.4))
        rosto = replace(base, queda=progresso, olhar=(8, olhar_y),
                        garra_y=garra_y, garra_abertura=abertura)
        if base.cena == 'place_on_table':
            # Mantém as pupilas visíveis acompanhando a garra; comemora quando ela sai.
            peso = self._suave((fase - tempos.fim_recuo) / 0.25)
            feliz = replace(self._QUADROS['place_on_table'][0], queda=progresso,
                            olhar=rosto.olhar, garra_y=garra_y, garra_abertura=abertura)
            rosto = _interpolar(rosto, feliz, peso)
        return rosto

    def _pose(self, agora):
        progresso = min(1.0, max(0.0, (agora - self._inicio_transicao) / self._tempo_transicao))
        peso = progresso * progresso * (3 - 2 * progresso)
        alvo = self._alvo_animado(max(0, agora - self._inicio_expressao))
        return _interpolar(self._origem, alvo, peso)

    def _seleciona(self, expressao, agora):
        expressao = self._ALIASES.get(expressao, expressao)
        if expressao not in self._POSES:
            raise ValueError('Expressão inválida: ' + str(expressao))
        if expressao != self._expressao:
            self._origem = self._pose(agora)
            self._expressao = expressao
            self._inicio_transicao = agora
            self._inicio_expressao = agora

    @staticmethod
    def _pulso(tempo, periodo, duracao):
        """Fechamento e abertura suaves em uma pequena parte de cada ciclo."""
        fase = tempo % periodo
        if fase >= duracao:
            return 0.0
        return math.sin(math.pi * fase / duracao) ** 2

    def atualizar(self, expressao=None):
        """Atualiza sem aguardar; None mantém a expressão atual. Retorna se desenhou."""
        agora = time.monotonic()
        self._seleciona(self._expressao if expressao is None else expressao, agora)
        if self._ultimo_quadro is not None and agora - self._ultimo_quadro < 1 / self.fps:
            return False
        self._renderiza(agora)
        self._ultimo_quadro = agora
        return True

    def desenha(self, expressao='neutro'):
        """Seleciona e atualiza um quadro; chame atualizar() para movimento contínuo."""
        return self.atualizar(expressao)

    def _renderiza(self, agora):
        from luma.core.render import canvas

        with canvas(self.tela.display) as draw:
            self._desenha(draw, self._pose(agora), agora - self._inicio)

    @staticmethod
    def _traco(draw, pontos, largura=3, cor='white'):
        pontos = [(round(x), round(y)) for x, y in pontos]
        draw.line(pontos, fill=cor, width=largura, joint='curve')

    def _olho(self, draw, x, y, abertura, arco, aperto, palpebra, olhar, piscar, cilios=False):
        largura, altura = 30, 30
        h = max(2, round(altura * abertura * (1 - piscar)))
        if abs(aperto) > 0.5:
            sinal = 1 if aperto > 0 else -1
            # Mesmo os olhos apertados se fecham e reabrem na piscada.
            espaco = 8 * (1 - 0.8 * piscar)
            self._traco(draw, [(x - sinal * 13, y - espaco), (x + sinal * 10, y),
                               (x - sinal * 13, y + espaco)], 4)
            return
        if arco > 1 and piscar < 0.8:
            self._traco(draw, [(x + u, y - arco * (1 - (u / 14) ** 2))
                               for u in range(-14, 15)], 5)
            return
        if h <= 4:
            if cilios and piscar < 0.8:
                # Três traços curvos da piscadinha na nova referência.
                self._traco(draw, [(x - 10, y - 8), (x - 4, y - 11),
                                   (x + 3, y - 10), (x + 10, y - 6)], 3)
                self._traco(draw, [(x - 13, y), (x - 5, y - 3),
                                   (x + 3, y - 2), (x + 12, y + 2)], 4)
                self._traco(draw, [(x - 9, y + 7), (x - 3, y + 4), (x + 7, y + 5)], 3)
            else:
                self._traco(draw, [(x - 14, y), (x + 14, y)], 4)
            return

        # Desenho local evita que a pupila ou o brilho escapem da esclera.
        olho = Image.new('1', (largura + 1, h + 1))
        tinta = ImageDraw.Draw(olho)
        tinta.ellipse((0, 0, largura, h), fill=1)
        mascara = olho.copy()
        px, py = 15 + round(olhar[0]), h // 2 + round(olhar[1])
        tinta.ellipse((px - 6, py - 8, px + 6, py + 8), fill=0)
        # Pequeno reflexo branco no canto superior da pupila.
        tinta.rectangle((px + 3, py - 6, px + 5, py - 4), fill=1)
        corte = round(h * palpebra)
        if corte:
            tinta.rectangle((0, 0, largura, corte), fill=0)
            tinta.rectangle((1, corte + 1, largura - 1, corte + 3), fill=1)
        # Recorta o reflexo e a pálpebra pelos limites ovais do olho.
        olho = ImageChops.logical_and(olho, mascara)
        draw.bitmap((round(x - 15), round(y - h // 2)), olho, fill='white')

    def _boca(self, draw, rosto, x, y, tempo):
        w, h = rosto.largura, rosto.altura
        if rosto.boca in ('oval', 'anel'):
            respirar = 1 + 0.10 * math.sin(tempo * 3)
            draw.ellipse((round(x - w), round(y - h * respirar),
                          round(x + w), round(y + h * respirar)), fill='white')
            if rosto.boca == 'anel':
                draw.ellipse((round(x - w + 2), round(y - h + 2),
                              round(x + w - 2), round(y + h - 2)), fill='black')
        elif rosto.boca in ('cheio', 'vazado'):
            # Borda superior levemente curva e base arredondada, em vez de uma barra reta.
            superior = [(x + u, y + 1.5 * (1 - (u / w) ** 2))
                        for u in range(-round(w), round(w) + 1)]
            inferior = [(x + w * math.cos(a * math.pi / 20),
                         y + h * math.sin(a * math.pi / 20)) for a in range(21)]
            draw.polygon([(round(a), round(b)) for a, b in superior + inferior], fill='white')
            if rosto.boca == 'vazado':
                self._traco(draw, [(x + u, y + 3 + (h - 6) * (1 - (u / (w - 4)) ** 2))
                                   for u in range(-round(w - 4), round(w - 4) + 1)], 3, 'black')
        elif rosto.boca == 'zigue':
            self._traco(draw, [(x - 2, y - 4), (x - 4, y), (x + 2, y),
                               (x, y + 4), (x + 3, y + 4)], 3)
        else:
            pontos = []
            for u in range(-round(w), round(w) + 1):
                curva = h * (1 - (u / w) ** 2)
                if rosto.boca == 'triste':
                    curva = -curva
                elif rosto.boca == 'ondulada':
                    curva = h * math.sin(u * 0.48 + 0.4 * math.sin(tempo * 2))
                elif rosto.boca == 'torto':
                    curva -= 0.25 * u
                pontos.append((x + u, y + curva))
            self._traco(draw, pontos, 3)
        if rosto.lingua > 0.5:
            tx = x + w - 1 if rosto.boca == 'linha' else x - 5
            self._desenha_lingua(draw, tx, y + 1)

    def _desenha_lingua(self, draw, x, y):
        # Desenho original, com a ranhura e sem deformação própria.
        self._traco(draw, [(x - 3, y - 2), (x + 1, y + 4)], 3)
        pontos = [(x, y), (x + 7, y), (x + 7, y + 5),
                  (x + 4, y + 7), (x + 1, y + 6)]
        draw.polygon([(round(a), round(b)) for a, b in pontos], fill='white')
        self._traco(draw, [(x + 3, y), (x + 4, y + 3)], 1, 'black')

    def _desenha(self, draw, rosto, tempo):
        # Balanço pequeno e contínuo; não desloca lateralmente o olhar em repouso.
        balanco = 0.85 * math.sin(tempo * 2.4)
        respirar = 0.55 * math.sin(tempo * 1.6)
        piscar = self._pulso(tempo + 1.1, 4.2, 0.30)
        for i, centro in enumerate((39, 89)):
            x = centro + rosto.deslocamento
            y = 31 + rosto.vertical + balanco + rosto.desnivel[i]
            aperto = rosto.apertos[i] * (1 if i == 0 else -1)
            self._olho(draw, x, y, rosto.olhos[i], rosto.arcos[i], aperto,
                       rosto.palpebras[i], rosto.olhar, piscar,
                       cilios=rosto.olhos[1 - i] > 0.5)
            topo_sobrancelha = min(b for _, b in rosto.sobrancelhas[i])
            sobrancelha_y = max(2 - topo_sobrancelha, 9 + rosto.vertical) + balanco + respirar
            pontos = [(x + a, sobrancelha_y + b + rosto.desnivel[i] * 0.4)
                      for a, b in rosto.sobrancelhas[i]]
            self._traco(draw, pontos, 3)

        mx, my = 64 + rosto.deslocamento, 51 + rosto.vertical + balanco
        self._boca(draw, rosto, mx, my, tempo)
        if rosto.suor > 0.5:
            x = round(rosto.suor_x)
            y = max(6, round(16 + rosto.vertical + 2 * math.sin(tempo * 2.7)))
            draw.polygon([(x, y - 5), (x - 3, y + 3), (x + 3, y + 3)], fill='white')
            draw.ellipse((x - 3, y, x + 3, y + 6), fill='white')
        if rosto.brilhos > 0.3:
            sobre_mesa = rosto.mesa > 0.5 or rosto.cena == 'place_on_table'
            locais = ((12, 25), (113, 30)) if sobre_mesa else ((12, 27), (18, 44), (114, 39))
            for i, (x, y) in enumerate(locais):
                if rosto.brilhos < 0.8 and i == 0:
                    continue
                r = 2 + round(1 + math.sin(tempo * 4 + i * 2))
                self._traco(draw, [(x - r, y), (x + r, y)], 2)
                self._traco(draw, [(x, y - r), (x, y + r)], 2)
        if abs(rosto.tracos) > 0.5:
            x = 10 if rosto.tracos < 0 else (113 if rosto.mesa else 119)
            y = 39 if rosto.mesa else 31
            for dy in (-8, 0, 8):
                self._traco(draw, [(x - 2, y + dy), (x + 3, y + dy * 1.3)], 2)
        if rosto.parede > 0.5:
            draw.rectangle((121, 3, 125, 61), fill='white')
        if rosto.mesa > 0.5:
            draw.rectangle((84, 61, 125, 63), fill='white')
            draw.rectangle((98, 49, 110, 59), fill='white')

        if rosto.cena in ('container', 'stack', 'place_on_table', 'pick'):
            self._desenha_entrega(draw, rosto)
        elif rosto.cena in ('retrieve', 'store'):
            self._desenha_transferencia(draw, rosto)
        elif rosto.cena == 'slots':
            self._desenha_slots(draw, rosto.fase_slots)
        if rosto.cena in self._CENAS_GARRA:
            self._desenha_garra(draw, rosto)
        if rosto.fogos > 0.5:
            self._desenha_fogos(draw, tempo)

    def _desenha_garra(self, draw, rosto):
        x, y = 111, rosto.garra_y
        if y + 14 < 0:
            return  # A garra inteira, incluindo as pontas, já saiu pelo alto.
        abertura = rosto.garra_abertura
        if y > 3:
            self._traco(draw, [(x, 0), (x, y - 3)], 2)
        draw.rectangle((x - 6, round(y - 2), x + 6, round(y + 1)), fill='white')
        for lado in (-1, 1):
            dedos_x = x + lado * (8 + 3 * abertura)
            ponta_x = x + lado * (5 + 3 * abertura)
            self._traco(draw, [(x + lado * 6, y), (dedos_x, y + 4),
                               (dedos_x, y + 14), (ponta_x, y + 14)], 2)

    @staticmethod
    def _posicao_queda(cena, progresso):
        # O mesmo centro guia o desenho do cubo e o acompanhamento pelas pupilas.
        if cena == 'place_on_table':
            return 111, -7 + 61 * progresso
        if cena == 'pick':
            return 111, 54 - 61 * progresso
        destino = 52 if cena == 'container' else 42
        return 111, -7 + (destino + 7) * progresso

    def _desenha_entrega(self, draw, rosto):
        draw.rectangle((84, 61, 125, 63), fill='white')
        if rosto.cena == 'stack':
            draw.rectangle((106, 49, 116, 59), fill='white')
        x, y = self._posicao_queda(rosto.cena, rosto.queda)
        draw.rectangle((x - 5, round(y - 5), x + 5, round(y + 5)), fill='white')
        if rosto.cena == 'container':
            # Recipiente aberto: paredes, fundo e duas bordas na abertura.
            self._traco(draw, [(99, 44), (101, 59), (121, 59), (123, 44)], 2)
            self._traco(draw, [(97, 44), (103, 44)], 2)
            self._traco(draw, [(119, 44), (125, 44)], 2)

    @staticmethod
    def _posicao_transferencia(cena, progresso):
        # Os extremos deixam o cubo inteiro fora dos 64 pixels de altura.
        y = -7 + 78 * progresso
        return 111, 64 - y if cena == 'retrieve' else y

    def _desenha_transferencia(self, draw, rosto):
        x, y = self._posicao_transferencia(rosto.cena, rosto.queda)
        draw.rectangle((x - 5, round(y - 5), x + 5, round(y + 5)), fill='white')

    @classmethod
    def _duracao_cubo(cls, cena):
        if cena == 'slots':
            distancia = 16
        else:
            posicao = (cls._posicao_transferencia if cena in ('retrieve', 'store')
                       else cls._posicao_queda)
            distancia = abs(posicao(cena, 1)[1] - posicao(cena, 0)[1])
        return distancia / cls._VELOCIDADE_CUBO

    @classmethod
    def _cubos_slots(cls, tempo):
        """Seis cubos e uma vaga: a vaga percorre os sete slots e retorna."""
        duracao = cls._duracao_cubo('slots')
        intervalo = 0.12 + duracao
        passo = int((tempo % (12 * intervalo)) / intervalo)
        fase = ((tempo / intervalo) % 1) * intervalo
        vazio = 6
        posicoes = list(range(6))
        for indice in range(passo):
            origem = vazio - 1 if indice < 6 else vazio + 1
            cubo = posicoes.index(origem)
            posicoes[cubo], vazio = vazio, origem
        origem = vazio - 1 if passo < 6 else vazio + 1
        movendo = posicoes.index(origem)
        peso = min(1.0, max(0.0, (fase - 0.12) / duracao))
        peso = peso * peso * (3 - 2 * peso)
        cubos = []
        for indice, slot in enumerate(posicoes):
            destino = slot + (vazio - slot) * peso if indice == movendo else slot
            subir = 3 * math.sin(math.pi * peso) if indice == movendo else 0
            cubos.append((14 + 16 * destino, 55 - subir))
        return tuple(cubos)

    def _desenha_slots(self, draw, tempo):
        for slot in range(7):
            x = 14 + 16 * slot
            self._traco(draw, [(x - 6, 62), (x + 6, 62)], 1)
        for x, y in self._cubos_slots(tempo):
            draw.rectangle((round(x - 4), round(y - 4),
                            round(x + 4), round(y + 4)), fill='white')

    def _desenha_fogos(self, draw, tempo):
        # Foguete subindo, explosão radial e partículas caindo, em ciclos defasados.
        for indice, (x, y) in enumerate(((11, 15), (116, 19), (14, 49), (114, 49))):
            fase = (tempo + indice * 0.43) % 1.8
            if fase < 0.35:
                topo = 62 + (y - 62) * fase / 0.35
                self._traco(draw, [(x, topo), (x, topo + 3)], 1)
            elif fase < 1.35:
                idade = (fase - 0.35) / 1.0
                raio = 2 + 9 * idade
                for particula in range(8):
                    angulo = particula * math.pi / 4 + indice * 0.2
                    dx, dy = math.cos(angulo), math.sin(angulo)
                    queda = 4 * idade ** 2
                    ponta = (x + raio * dx, y + raio * dy + queda)
                    if idade < 0.6:
                        self._traco(draw, [(x + raio * 0.6 * dx,
                                           y + raio * 0.6 * dy + queda), ponta], 1)
                    elif (particula + int(idade * 12)) % 3:
                        draw.point((round(ponta[0]), round(ponta[1])), fill='white')

    def animar(self, expressao='neutro', duracao=None):
        """Anima por duracao segundos; None mantém até Ctrl+C. Bloqueia o chamador."""
        if duracao is not None and (not math.isfinite(duracao) or duracao < 0):
            raise ValueError('Duração deve ser finita e não negativa.')
        self._seleciona(expressao, time.monotonic())
        fim = None if duracao is None else time.monotonic() + duracao
        while fim is None or time.monotonic() < fim:
            self.atualizar()
            restante = 1 / self.fps
            if self._ultimo_quadro is not None:
                restante -= time.monotonic() - self._ultimo_quadro
            if fim is not None:
                restante = min(restante, fim - time.monotonic())
            if restante > 0:
                time.sleep(restante)
