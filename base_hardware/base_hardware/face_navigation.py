"""Placa periódica de destino, independente de ROS e do OLED físico."""

import unicodedata

from PIL import Image, ImageDraw


# Fonte 5×7 explícita para manter o mesmo traço pixelado em qualquer Pillow.
_GLYPHS = {
    'A': (14, 17, 17, 31, 17, 17, 17), 'B': (30, 17, 17, 30, 17, 17, 30),
    'C': (14, 17, 16, 16, 16, 17, 14), 'D': (30, 17, 17, 17, 17, 17, 30),
    'E': (31, 16, 16, 30, 16, 16, 31), 'F': (31, 16, 16, 30, 16, 16, 16),
    'G': (14, 17, 16, 23, 17, 17, 15), 'H': (17, 17, 17, 31, 17, 17, 17),
    'I': (14, 4, 4, 4, 4, 4, 14), 'J': (7, 2, 2, 2, 18, 18, 12),
    'K': (17, 18, 20, 24, 20, 18, 17), 'L': (16, 16, 16, 16, 16, 16, 31),
    'M': (17, 27, 21, 21, 17, 17, 17), 'N': (17, 25, 21, 19, 17, 17, 17),
    'O': (14, 17, 17, 17, 17, 17, 14), 'P': (30, 17, 17, 30, 16, 16, 16),
    'Q': (14, 17, 17, 17, 21, 18, 13), 'R': (30, 17, 17, 30, 20, 18, 17),
    'S': (15, 16, 16, 14, 1, 1, 30), 'T': (31, 4, 4, 4, 4, 4, 4),
    'U': (17, 17, 17, 17, 17, 17, 14), 'V': (17, 17, 17, 17, 17, 10, 4),
    'W': (17, 17, 17, 21, 21, 21, 10), 'X': (17, 17, 10, 4, 10, 17, 17),
    'Y': (17, 17, 10, 4, 4, 4, 4), 'Z': (31, 1, 2, 4, 8, 16, 31),
    '0': (14, 17, 19, 21, 25, 17, 14), '1': (4, 12, 4, 4, 4, 4, 14),
    '2': (14, 17, 1, 2, 4, 8, 31), '3': (30, 1, 1, 14, 1, 1, 30),
    '4': (2, 6, 10, 18, 31, 2, 2), '5': (31, 16, 16, 30, 1, 1, 30),
    '6': (14, 16, 16, 30, 17, 17, 14), '7': (31, 1, 2, 4, 8, 8, 8),
    '8': (14, 17, 17, 14, 17, 17, 14), '9': (14, 17, 17, 15, 1, 1, 14),
    '_': (0, 0, 0, 0, 0, 0, 31), '-': (0, 0, 0, 31, 0, 0, 0),
    '?': (14, 17, 1, 2, 4, 0, 4), ' ': (0, 0, 0, 0, 0, 0, 0),
    'i': (4, 0, 12, 4, 4, 4, 14), 'n': (0, 0, 30, 17, 17, 17, 17),
    'd': (1, 1, 15, 17, 17, 17, 15), 'o': (0, 0, 14, 17, 17, 17, 14),
}


def _texto(draw, xy, texto, cor=1):
    x, y = xy
    for caractere in texto:
        for linha, bits in enumerate(_GLYPHS.get(caractere, _GLYPHS['?'])):
            for coluna in range(5):
                if bits & (1 << (4 - coluna)):
                    draw.point((x + coluna, y + linha), fill=cor)
        x += 6


class DestinoNavegacao:
    TEMPO_DESTINO = 2.0
    TEMPO_ROSTO = 4.0

    def __init__(self):
        self.destino = None
        self.mudou_em = 0.0

    def definir(self, destino, agora):
        if destino is not None:
            texto = ' '.join(str(destino).split())[:64]
            texto = unicodedata.normalize('NFKD', texto).encode('ascii', 'ignore').decode().upper()
            destino = ''.join(c if c in _GLYPHS else '?' for c in texto) or '--'
        if destino != self.destino:
            self.destino = destino
            self.mudou_em = agora

    def compor(self, rosto, agora):
        if self.destino is None:
            return rosto
        idade = max(0, agora - self.mudou_em)
        if idade % (self.TEMPO_DESTINO + self.TEMPO_ROSTO) >= self.TEMPO_DESTINO:
            return rosto  # Rosto original, sem redução nem legenda sobreposta.
        quadro = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(quadro)
        _texto(draw, (52, 4), 'indo')
        largura = len(self.destino) * 6 - 1
        escala = max(3, min(6, 116 // largura))
        texto = Image.new('1', (largura, 7))
        _texto(ImageDraw.Draw(texto), (0, 0), self.destino)
        texto = texto.resize((largura * escala, 7 * escala), Image.Resampling.NEAREST)
        visivel = min(116, texto.width)
        excesso = texto.width - visivel
        deslocamento = 0
        if excesso:
            # Mantém letras grandes; identificadores longos rolam apenas na placa.
            viagem = excesso / 20
            fase = idade % (2 * viagem + 2)
            if 1 < fase < 1 + viagem:
                deslocamento = round((fase - 1) * 20)
            elif 1 + viagem <= fase < 2 + viagem:
                deslocamento = excesso
            elif fase >= 2 + viagem:
                deslocamento = round(excesso - (fase - 2 - viagem) * 20)
        x, y = (128 - visivel) // 2, 17 + (42 - texto.height) // 2
        quadro.paste(texto.crop((deslocamento, 0, deslocamento + visivel, texto.height)), (x, y))
        # Atividade, sem representar distância ou porcentagem de chegada.
        for indice in range(3):
            xx = 56 + indice * 7
            if indice == int(agora * 4) % 3:
                draw.rectangle((xx, 61, xx + 2, 63), fill=1)
            else:
                draw.point((xx + 1, 62), fill=1)
        return quadro
