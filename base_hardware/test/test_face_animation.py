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
        for frame in range(60):
            now[0] += 0.051
            assert face.atualizar()
            assert display.image.getbbox() is not None
            if frame > 10:  # A transição de 0,4 s já terminou.
                images.add(display.image.tobytes())
        assert len(images) > 5, f'{expression} ficou estático.'
