"""Animações do OLED embarcadas na base.

Adaptado de MariolaZero/libs/main.py para instalar e distribuir com o pacote ROS,
sem depender de caminhos da árvore de desenvolvimento.
"""

import math
import time

from PIL import Image, ImageDraw


class ExpressoesFaciais:
    # Olhos: abertura E/D, arco feliz, aperto, pálpebras E/D; olhar X/Y;
    # sobrancelhas E/D; boca: largura, sorriso, abertura, língua, suor, brilho.
    _POSES = {
        'navigate': (1, 1, 0, 0, 0, 0, 3, 0, -3, 3, 10, 5, 5, 0, 0, 0),
        'finish': (0.22, 0.22, 9, 0, 0, 0, 0, 0, -3, 3, 16, 9, 8, 0, 0, 0),
        'follow_wall': (1, 1, 0, 0, 0, 0, -4, 0, -3, -2, 10, 4, 0, 0, 0, 0),
        'analyze_scene': (1, 0.65, 0, 0, 0, 0.4, -4, 0, -3, 0, 8, -4, 0, 0, 0, 0),
        'pick': (1, 1, 0, 0, 0, 0, 4, 1, 5, -5, 8, 0, 0, 1, 0, 0),
        'store': (0.22, 0.22, 8, 0, 0, 0, 0, 0, -2, 2, 10, 5, 0, 0, 0, 0),
        'retrieve': (1, 1, 0, 0, 0, 0, 0, 0, -3, 3, 4, 0, 9, 0, 0, 0),
        'place_on_table': (1, 1, 0, 0, 0, 0, 0, 0, -3, 3, 12, 7, 6, 0, 0, 0),
        'place_in_container': (1, 1, 0, 0, 0, 0, 0, 6, -3, 3, 4, -3, 0, 0, 0, 0),
        'place_on_shelf': (1, 1, 0, 0, 0, 0, 0, -6, -5, 5, 8, 4, 0, 0, 1, 0),
        'place_on_precision_table': (0.7, 0.7, 0, 1, 0, 0, 0, 0, 0, 0, 8, 0, 0, 1, 0, 0),
        'stack': (0.9, 1, 0, 0, 0, 0, -3, -3, -5, 4, 12, 0, 0, 0, 1, 0),
        'organize_precision': (1, 1, 0, 0, 0.55, 0.55, 2, 1, 0, 0, 10, 5, 0, 0, 0, 0),
        'piscar': (0.12, 0.12, 0, 0, 0, 0, 0, 0, -3, 3, 10, 5, 0, 0, 0, 0),
        'piscadinha': (0.12, 1, 0, 0, 0, 0, -3, 0, -3, 3, 10, 5, 0, 0, 0, 0),
        'comemorar': (0.22, 0.22, 9, 0, 0, 0, 0, 0, -3, 3, 16, 9, 9, 0, 0, 1),
    }
    # Nomes antigos continuam válidos para configurações e chamadas existentes.
    _ALIASES = {
        'neutro': 'navigate', 'feliz': 'finish', 'triste': 'place_in_container',
        'bravo': 'pick', 'surpreso': 'retrieve', 'dormindo': 'piscar',
        'piscando': 'piscadinha', 'correndo': 'navigate',
    }
    EXPRESSOES = tuple(_POSES) + tuple(_ALIASES)

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
        self._tempo_transicao = 0.4

    def _pose(self, agora):
        progresso = min(1.0, max(0.0, (agora - self._inicio_transicao) / self._tempo_transicao))
        peso = progresso * progresso * (3 - 2 * progresso)
        return tuple(a + (b - a) * peso
                     for a, b in zip(self._origem, self._POSES[self._expressao]))

    def _seleciona(self, expressao, agora):
        expressao = self._ALIASES.get(expressao, expressao)
        if expressao not in self._POSES:
            raise ValueError('Expressão inválida: ' + str(expressao))
        if expressao != self._expressao:
            self._origem = self._pose(agora)
            self._expressao = expressao
            self._inicio_transicao = agora

    @staticmethod
    def _pulso(tempo, periodo, duracao):
        """Fechamento e abertura suaves em uma pequena parte de cada ciclo."""
        fase = tempo % periodo
        if fase >= duracao:
            return 0.0
        return math.sin(math.pi * fase / duracao) ** 2

    def atualizar(self, expressao=None):
        """Atualiza um quadro quando necessário, sem aguardar; retorna se desenhou.

        Chame continuamente no laço principal. None mantém a expressão atual.
        A frequência máxima de escrita no OLED é controlada por fps.
        """
        agora = time.monotonic()
        self._seleciona(self._expressao if expressao is None else expressao, agora)
        if self._ultimo_quadro is not None and agora - self._ultimo_quadro < 1 / self.fps:
            return False
        self._renderiza(agora)
        self._ultimo_quadro = agora
        return True

    def desenha(self, expressao='neutro'):
        """Compatibilidade: seleciona a expressão e atualiza um quadro.

        Para movimento contínuo, use animar() ou chame atualizar() no seu laço.
        """
        return self.atualizar(expressao)

    def _renderiza(self, agora):
        from luma.core.render import canvas

        t = agora - self._inicio
        (esquerdo, direito, arco, aperto, palpebra_e, palpebra_d, olhar_x, olhar_y,
         sobrancelha_e, sobrancelha_d, largura, sorriso, abertura,
         lingua, suor, brilho) = self._pose(agora)
        # Todos os rostos respiram; as pupilas procuram e piscam sem novas mensagens.
        dx = 1.6 * math.sin(t * 1.7) + 0.7 * math.sin(t * 0.73)
        dy = 0.8 * math.sin(t * 1.1)
        balanco = 0.9 * math.sin(t * 2.4)
        piscar = self._pulso(t + 1.1, 4.2, 0.30)
        if self._expressao == 'piscar':
            # Alterna olhos abertos e fechados, sem ficar preso no quadro fechado.
            esquerdo = direito = 1 - 0.88 * self._pulso(t, 2.2, 1.1)
        elif self._expressao == 'piscadinha':
            esquerdo = 1 - 0.88 * self._pulso(t, 1.8, 1.0)

        with canvas(self.tela.display) as draw:
            for indice, (x, abertura_olho, sobrancelha, palpebra) in enumerate((
                (39, esquerdo, sobrancelha_e, palpebra_e),
                (89, direito, sobrancelha_d, palpebra_d),
            )):
                y = round(28 + balanco + (math.sin(t * 2) if indice else 0))
                h = max(2, round(27 * abertura_olho * (1 - piscar)))
                if aperto > 0.5:
                    sinal = 1 if indice == 0 else -1
                    draw.line([(x - sinal * 13, y - 7), (x + sinal * 10, y),
                               (x - sinal * 13, y + 7)], fill='white', width=3)
                elif arco > 1 and piscar < 0.8:
                    pontos = [(x + u, round(y - arco * (1 - (u / 14) ** 2)))
                              for u in range(-14, 15)]
                    draw.line(pontos, fill='white', width=4)
                elif h <= 4:
                    draw.line((x - 14, y, x + 14, y), fill='white', width=3)
                else:
                    caixa = (x - 15, y - h // 2, x + 15, y + h // 2)
                    draw.ellipse(caixa, fill='white')
                    px = round(x + olhar_x + dx)
                    py = round(y + olhar_y + dy)
                    # A máscara recorta pupilas e pálpebras dentro do olho.
                    olho = Image.new('1', (31, h + 1))
                    desenho = ImageDraw.Draw(olho)
                    desenho.ellipse((0, 0, 30, h), fill=1)
                    desenho.ellipse((px - x + 10, py - y + h // 2 - 7,
                                     px - x + 20, py - y + h // 2 + 7), fill=0)
                    corte = round(h * palpebra)
                    if corte > 0:
                        desenho.rectangle((0, 0, 30, corte), fill=0)
                        desenho.line((0, corte + 1, 30, corte + 1), fill=1, width=2)
                    draw.rectangle(caixa, fill='black')
                    draw.bitmap((x - 15, y - h // 2), olho, fill='white')
                # Sobrancelhas separadas, como no modelo pixelado.
                by = round(9 + balanco + 0.6 * math.sin(t * 1.9 + indice))
                draw.line([(x - 9, round(by - sobrancelha / 2)),
                           (x, by - (2 if abs(sobrancelha) > 1 else 0)),
                           (x + 8, round(by + sobrancelha / 2))],
                          fill='white', width=3)

            mx, my = 64, round(49 + balanco)
            if self._expressao == 'stack':
                pontos = [(mx + u, round(my + 2 * math.sin(u * 0.5 + t * 3)))
                          for u in range(-12, 13)]
                draw.line(pontos, fill='white', width=3)
            elif abertura > 2:
                if sorriso > 2:
                    draw.polygon([(mx - round(largura), my - 2),
                                  (mx + round(largura), my - 2)] + [
                        (round(mx + largura * math.cos(a * math.pi / 16)),
                         round(my + abertura * math.sin(a * math.pi / 16)))
                        for a in range(17)], fill='white')
                else:
                    h = abertura * (1 + 0.12 * math.sin(t * 3)) / 2
                    draw.ellipse((mx - largura, round(my - h),
                                  mx + largura, round(my + h)), fill='white')
            else:
                pontos = [(mx + u, round(my + sorriso * (1 - (u / largura) ** 2)))
                          for u in range(-round(largura), round(largura) + 1)]
                draw.line(pontos, fill='white', width=3)
            if lingua > 0.5:
                x = mx + round(largura) + 1
                draw.line([(x - 2, my - 1), (x + 1, my + 5),
                           (x + 5, my + 6), (x + 6, my + 2),
                           (x + 4, my - 2)], fill='white', width=3)
            if suor > 0.5:
                y = round(16 + 3 * math.sin(t * 2.7))
                draw.polygon([(115, y - 4), (112, y + 3), (118, y + 3)], fill='white')
                draw.ellipse((112, y, 118, y + 6), fill='white')
            if brilho > 0.5:
                for indice, (x, y) in enumerate(((12, 26), (18, 43), (115, 39))):
                    raio = 2 + round(1 + math.sin(t * 4 + indice * 2))
                    draw.line((x - raio, y, x + raio, y), fill='white', width=2)
                    draw.line((x, y - raio, x, y + raio), fill='white', width=2)

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

