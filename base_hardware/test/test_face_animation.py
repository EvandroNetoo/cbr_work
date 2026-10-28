"""Render the installed animation on Pillow without luma or physical I2C."""

from contextlib import contextmanager
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

from PIL import Image, ImageDraw


def test_every_expression_keeps_moving_after_its_transition(monkeypatch):
    @contextmanager
    def canvas(display):
        display.image = Image.new('1', (128, 64))
        yield ImageDraw.Draw(display.image)

    for name in ('luma', 'luma.core', 'luma.core.render'):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules['luma.core.render'].canvas = canvas
    source = Path(__file__).parents[1] / 'base_hardware' / 'face_animation.py'
    spec = importlib.util.spec_from_file_location('tested_face_animation', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    now = [0.0]
    module.time = SimpleNamespace(monotonic=lambda: now[0])
    display = SimpleNamespace(image=None)
    face = module.ExpressoesFaciais(SimpleNamespace(display=display))
    for expression in face.EXPRESSOES:
        now[0] += 0.051
        assert face.atualizar(expression)
        assert not face.atualizar(), 'O OLED deve respeitar o limite de FPS.'
        images = set()
        eyes = set()
        for frame in range(120):
            now[0] += 0.051
            assert face.atualizar()
            assert display.image.getbbox() is not None
            if frame > 10:  # A transição de 0,4 s já terminou.
                images.add(display.image.tobytes())
                eyes.add(display.image.crop((20, 12, 108, 43)).tobytes())
        assert len(images) > 5, f'{expression} ficou estático.'

        assert len(eyes) > 5, f'{expression} ficou com os olhos estáticos.'

    # A piscada precisa fechar e reabrir, mesmo sem troca de expressão.
    face._seleciona('navigate', now[0])
    face._inicio_transicao = -1
    face._inicio = 0
    counts = []
    for instant in (2.95, 3.25, 3.60):
        face._renderiza(instant)
        counts.append(sum(display.image.crop((20, 15, 108, 43)).getdata()))
    assert counts[1] < min(counts[0], counts[2]) / 2

    # Os 16 desenhos da referência precisam continuar distintos no OLED.
    portraits = set()
    for expression in face._POSES:
        face._expressao = expression
        face._inicio_transicao = -1
        face._renderiza(0.45)
        portraits.add(display.image.tobytes())
    assert len(portraits) == 16
