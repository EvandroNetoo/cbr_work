#!/usr/bin/env python3
"""Exporta as mesmas rotinas do OLED para PNG/GIF sem ROS, luma ou I2C."""

from pathlib import Path
import sys
from types import SimpleNamespace

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from base_hardware.face_animation import ExpressoesFaciais  # noqa: E402


OUTPUT = Path(__file__).resolve().parents[1] / 'docs'
SCALE = 3
CELL = (128 * SCALE + 16, 64 * SCALE + 36)
try:
    FONT = ImageFont.truetype('DejaVuSansMono.ttf', 13)
except OSError:
    FONT = ImageFont.load_default()


def tile(sheet, face, pose, time, column, row, label):
    image = Image.new('1', (128, 64))
    face._desenha(ImageDraw.Draw(image), pose, time)
    x, y = column * CELL[0], row * CELL[1]
    sheet.paste(image.convert('RGB').resize((128 * SCALE, 64 * SCALE),
                                           Image.Resampling.NEAREST), (x + 8, y + 8))
    draw = ImageDraw.Draw(sheet)
    draw.rectangle((x + 6, y + 6, x + 128 * SCALE + 9, y + 64 * SCALE + 9), outline='#666666')
    draw.text((x + 8, y + 64 * SCALE + 16), label, fill='white', font=FONT)


def scene_times(face, name):
    scene = face._POSES[name].cena
    if scene in ('retrieve', 'store'):
        duration = face._duracao_cubo(scene)
        return tuple(duration * fraction for fraction in (0.06, 0.31, 0.58, 0.88))
    if scene not in face._CENAS_GARRA:
        return (0.2, 0.7, 1.2, 1.7)
    times = face._tempos_garra(scene)
    start, end, duration = times.inicio_movimento, times.fim_movimento, times.movimento
    if name == 'place_on_table':
        return (0, start + duration * 0.53, end, times.fim_recuo + 0.3)
    if name == 'pick':
        return (0, start + duration * 0.4, end, times.ciclo - 0.1)
    if scene == 'container':
        return (0, duration * 0.55, end + times.abertura, times.fim_cubo)
    return tuple(start + duration * fraction for fraction in (0, 1 / 3, 2 / 3, 1))


def export():
    OUTPUT.mkdir(exist_ok=True)
    face = ExpressoesFaciais(SimpleNamespace())
    frames = []
    for frame in range(140):
        time = frame / 10
        sheet = Image.new('RGB', (CELL[0] * 4, CELL[1] * 4))
        for i, name in enumerate(face._POSES):
            face._expressao = name
            tile(sheet, face, face._alvo_animado(time), time, i % 4, i // 4, name)
        frames.append(sheet.convert('P', palette=Image.Palette.ADAPTIVE, colors=8))
    frames[15].save(OUTPUT / 'faces_preview.png')
    frames[0].save(OUTPUT / 'faces_preview.gif', save_all=True, append_images=frames[1:],
                   duration=100, loop=0)

    sheet = Image.new('RGB', (CELL[0] * 4, CELL[1] * 3))
    for row, (name, (_, _, order)) in enumerate(face._SEQUENCIAS.items()):
        for col, index in enumerate(order):
            pose = face._POSES[name] if index == -1 else face._QUADROS[name][index]
            label = 'padrão' if index == -1 or (name == 'navigate' and index == 2) else str(index + 1)
            tile(sheet, face, pose, 1.5, col, row, f'{name} / {label}')
    face._expressao = 'place_on_table'
    for col, time in enumerate(scene_times(face, 'place_on_table')):
        tile(sheet, face, face._alvo_animado(time), time, col, 2, f'place_on_table / {time:.1f} s')
    sheet.save(OUTPUT / 'faces_movement.png')
    sheet = Image.new('RGB', (CELL[0] * 4, CELL[1] * 4))
    for row, name in enumerate(('place_in_container', 'stack', 'organize_precision', 'finish')):
        face._expressao = name
        for col, time in enumerate(scene_times(face, name)):
            tile(sheet, face, face._alvo_animado(time), time, col, row, f'{name} / {time:.1f} s')
    sheet.save(OUTPUT / 'faces_scenes.png')
    sheet = Image.new('RGB', (CELL[0] * 4, CELL[1] * 5))
    for row, name in enumerate(('retrieve', 'store', 'pick', 'place_on_precision_table',
                                'organize_precision')):
        face._expressao = name
        for col, time in enumerate(scene_times(face, name)):
            tile(sheet, face, face._alvo_animado(time), time, col, row, f'{name} / {time:.2f} s')
    sheet.save(OUTPUT / 'faces_transfer.png')
    sheet = Image.new('RGB', (CELL[0] * 4, CELL[1] * 2))
    for row, name in enumerate(('place_on_table', 'pick')):
        face._expressao = name
        for col, time in enumerate(scene_times(face, name)):
            tile(sheet, face, face._alvo_animado(time), time, col, row, f'{name} / {time:.1f} s')
    sheet.save(OUTPUT / 'faces_table_pick.png')
    sheet = Image.new('RGB', (CELL[0] * 5, CELL[1] * 4))
    for row, name in enumerate(('pick', 'place_on_table', 'place_in_container', 'stack')):
        face._expressao = name
        times = face._tempos_garra(face._POSES[name].cena)
        holding = (times.inicio_movimento if times.aproximacao
                   else times.inicio_movimento + times.movimento * 0.4)
        late = (times.fim_movimento - times.movimento * 0.15 if not times.recuo
                else times.fim_movimento + times.abertura + times.recuo * 0.5)
        instants = (0, holding, times.inicio_movimento + times.movimento * 0.65,
                    late, times.ciclo - 0.05)
        for col, time in enumerate(instants):
            tile(sheet, face, face._alvo_animado(time), time, col, row, f'{name} / {time:.2f} s')
    sheet.save(OUTPUT / 'faces_gripper.png')
    print(f'Prévias salvas em {OUTPUT}')


if __name__ == '__main__':
    export()
