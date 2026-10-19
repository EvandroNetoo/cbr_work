"""Animações do OLED embarcadas na base.

Adaptado de MariolaZero/libs/main.py para instalar e distribuir com o pacote ROS,
sem depender de caminhos da árvore de desenvolvimento.
"""

import math
import time

class ExpressoesFaciais:
    # Altura/curvatura dos olhos, sorriso, abertura da boca, sobrancelha, piscadela, vento.
    _POSES = {
        'neutro': (20, 0, 0, 0, 0, 0, 0),
        'feliz': (4, 9, 10, 0, 0, 0, 0),
        'triste': (15, 0, -8, 0, -7, 0, 0),
        'bravo': (15, 0, -5, 0, 9, 0, 0),
        'surpreso': (26, 0, 0, 16, 0, 0, 0),
        'dormindo': (2, 0, 0, 5, 0, 0, 0),
        'piscando': (20, 0, 9, 0, 0, 1, 0),
        'correndo': (11, 0, -1, 0, 7, 0, 1),
    }
    EXPRESSOES = tuple(_POSES)

    def __init__(self, tela, fps=20):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('FPS deve ser um número positivo e finito.')
        self.tela = tela
        self.fps = fps
        self._inicio = time.monotonic()
        self._ultimo_quadro = None
        self._expressao = 'neutro'
        self._origem = self._POSES['neutro']
        self._inicio_transicao = self._inicio
        self._tempo_transicao = 0.4

    def _pose(self, agora):
        progresso = min(1.0, max(0.0, (agora - self._inicio_transicao) / self._tempo_transicao))
        peso = progresso * progresso * (3 - 2 * progresso)
        return tuple(a + (b - a) * peso
                     for a, b in zip(self._origem, self._POSES[self._expressao]))

    def _seleciona(self, expressao, agora):
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

        tempo = agora - self._inicio
        altura, arco, sorriso, abertura, sobrancelha, piscadela, vento = self._pose(agora)
        sono = max(0.0, min(1.0, (5 - altura) / 3)) if arco < 1 else 0.0
        # Olhar e balanço continuam mesmo quando a expressão não muda.
        olhar = 3 * math.sin(tempo * 1.3) * (1 - sono)
        balanco = math.sin(tempo * 2.4) * (1 + sono)
        rajada = (1 + math.sin(tempo * 5.5)) / 2
        olhar *= 1 - vento
        balanco = balanco * (1 - vento) + vento * 0.8 * math.sin(tempo * 14)
        altura *= 1 - vento * 0.2 * rajada
        piscar = self._pulso(tempo + 1.1, 4.2, 0.30) * (1 - sono)
        piscar_direito = max(piscar, piscadela * self._pulso(tempo, 1.6, 0.65))

        with canvas(self.tela.display) as draw:
            if vento > 0.05:
                self._desenha_velocidade(draw, tempo, vento)
            for indice, centro in enumerate((38, 90)):
                x = 64 + (centro - 64) * (1 + 0.06 * vento * rajada) + olhar
                y = 27 + balanco
                fechamento = piscar if indice == 0 else piscar_direito
                h = max(2, altura * (1 - fechamento))
                curva = arco * (1 - fechamento)
                if curva < 0.5 and vento > 0.05:
                    # Olhos angulares e estreitos, encarando o vento de frente.
                    inclinacao = vento * min(5, h * 0.65) * (1 if indice == 0 else -1)
                    draw.polygon([
                        (round(x - 13), round(y - h / 2 - inclinacao / 2)),
                        (round(x + 13), round(y - h / 2 + inclinacao / 2)),
                        (round(x + 11), round(y + h / 2)),
                        (round(x - 11), round(y + h / 2)),
                    ], fill='white')
                elif curva < 0.5:
                    draw.rounded_rectangle(
                        (round(x - 13), round(y - h / 2), round(x + 13), round(y + h / 2)),
                        radius=max(1, round(min(7, h / 2))), fill='white',
                    )
                else:
                    superior, inferior = [], []
                    for passo in range(27):
                        u = (passo - 13) / 13
                        centro_y = y - curva * (1 - u * u)
                        superior.append((round(x + passo - 13), round(centro_y - h / 2)))
                        inferior.append((round(x + passo - 13), round(centro_y + h / 2)))
                    draw.polygon(superior + inferior[::-1], fill='white')
                if abs(sobrancelha) > 0.5:
                    inclinacao = sobrancelha * (1 if indice == 0 else -1)
                    # Recorta a parte superior dos olhos para reforçar a emoção.
                    draw.polygon([
                        (round(x - 16), 0), (round(x + 16), 0),
                        (round(x + 16), round(y - 15 + inclinacao / 2)),
                        (round(x - 16), round(y - 15 - inclinacao / 2)),
                    ], fill='black')
                    draw.line((round(x - 15), round(y - 15 - inclinacao / 2),
                               round(x + 15), round(y - 15 + inclinacao / 2)),
                              fill='white', width=2)

            boca_y = 48 + balanco - 2 * vento
            boca_x = 64
            if abertura > 1:
                respirar = 1 + 0.16 * math.sin(tempo * 3)
                metade_h = max(1, abertura * respirar / 2)
                metade_w = 3 + abertura / 3
                draw.ellipse((round(boca_x - metade_w), round(boca_y - metade_h),
                              round(boca_x + metade_w), round(boca_y + metade_h)),
                             outline='white', width=2)
            else:
                # Curva contínua permite transformar sorriso em tristeza.
                amplitude = sorriso * (1 + 0.12 * math.sin(tempo * 3))
                largura = 1 + 0.2 * vento * rajada
                pontos = [(round(boca_x + dx * largura),
                           round(boca_y + amplitude * (1 - (dx / 19) ** 2)))
                          for dx in range(-19, 20)]
                draw.line(pontos, fill='white', width=3)
            if vento > 0.05:
                # Mandíbula larga e quadrada reforça o rosto firme contra o vento.
                queixo = [(29, 43), (35, 52), (48, 60),
                          (80, 60), (93, 52), (99, 43)]
                pontos = [(round(64 + (x - 64) * vento),
                           round(y + balanco)) for x, y in queixo]
                draw.line(pontos, fill='white', width=2)

    @staticmethod
    def _desenha_velocidade(draw, tempo, intensidade):
        """Ar frontal se divide e escoa em curvas contínuas pelos dois lados."""
        trajetorias = (
            ((64, 2), (34, -2), (-16, 15)),
            ((64, 8), (28, 2), (-16, 23)),
            ((23, 34), (13, 37), (-16, 47)),
            ((28, 49), (17, 55), (-16, 59)),
        )
        for indice, (a, b, c) in enumerate(trajetorias):
            for onda in range(2):
                fase = (tempo * (1.6 + indice * 0.12) + onda * 0.5 + indice * 0.19) % 1
                comprimento = (0.20 + 0.08 * math.sin(tempo * 5) ** 2) * intensidade
                for lado in (-1, 1):
                    pontos = []
                    for passo in range(10):
                        t = fase + comprimento * passo / 9
                        if t > 1:
                            break
                        x = (1 - t) ** 2 * a[0] + 2 * (1 - t) * t * b[0] + t * t * c[0]
                        y = (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * b[1] + t * t * c[1]
                        pontos.append((round(64 + lado * (64 - x)), round(y)))
                    if len(pontos) > 1:
                        draw.line(pontos, fill='white', width=1)

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

