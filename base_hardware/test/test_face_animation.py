"""Render the installed animation on Pillow without luma or physical I2C."""

from contextlib import contextmanager
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from PIL import Image, ImageDraw


@pytest.fixture
def face_environment(monkeypatch):
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
    return face, display, now


def test_every_expression_keeps_moving_after_its_transition(face_environment):
    face, display, now = face_environment
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

    # Finish reutiliza comemorar; os demais desenhos continuam distintos.
    portraits = set()
    for expression in face._POSES:
        face._expressao = expression
        face._inicio_transicao = -1
        face._renderiza(0.45)
        portraits.add(display.image.tobytes())
    assert len(portraits) == 15
    assert face._POSES['finish'] == face._POSES['comemorar']


def test_other_faces_keep_occasional_side_glances(face_environment):
    face, _, _ = face_environment
    for expression in ('piscar', 'piscadinha', 'finish', 'comemorar'):
        face._seleciona(expression, 0)
        poses = [face._alvo_animado(frame / 20) for frame in range(280)]
        forward = sum(abs(pose.olhar[0]) < 0.1 for pose in poses)
        assert forward / len(poses) >= 0.75, expression
        assert any(pose.olhar[0] > 4 for pose in poses), expression


@pytest.mark.parametrize('name, instants', [
    ('navigate', (0.1, 0.8, 2.2)),
    ('follow_wall', (0.1, 0.9)),
    ('place_on_table', (0.1, 0.95, 1.8)),
])
def test_fast_sequences_render_distinct_poses_without_navigation_arrows(face_environment, name, instants):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    pictures = []
    for instant in instants:
        picture = Image.new('1', (128, 64))
        face._desenha(ImageDraw.Draw(picture), face._alvo_animado(instant), 1.5)
        pictures.append(picture.tobytes())
        if name == 'navigate':
            assert picture.crop((56, 0, 73, 8)).getbbox() is None
    assert len(set(pictures)) == len(instants), name


def test_navigation_looks_both_ways_and_returns_forward_within_three_seconds(face_environment):
    face, _, _ = face_environment
    assert face._alvo_animado(0.1).olhar[0] == 0
    assert face._alvo_animado(0.8).olhar[0] == -9
    assert face._alvo_animado(1.5).olhar[0] == 0
    assert face._alvo_animado(2.2).olhar[0] == 9
    assert face._alvo_animado(2.9).olhar[0] == 0


def test_wall_alternates_only_first_two_poses_without_neutral_or_wink(face_environment):
    face, _, _ = face_environment
    face._seleciona('follow_wall', 0)
    first, second = face._QUADROS['follow_wall']
    assert first.olhar == (7, 6)  # Lateral para baixo.
    assert second.olhar == (5, -6)  # Lateral para cima.
    for instant, expected in ((0.1, first), (0.9, second), (1.7, first), (2.5, second)):
        assert face._alvo_animado(instant) == expected
    for frame in range(64):
        pose = face._alvo_animado(frame / 20)
        assert pose != face._POSES['follow_wall']
        assert pose.olhos == (1, 1)
        assert 5 <= pose.olhar[0] <= 7
    # A transição mistura os dois quadros sem passar pelo padrão.
    middle = face._alvo_animado(0.675)
    assert 0 < middle.suor < 1


def test_table_tracks_the_descent_then_celebrates_after_landing(face_environment):
    face, _, _ = face_environment
    face._seleciona('place_on_table', 0)
    times = face._tempos_garra('place_on_table')
    duration = times.movimento
    first = Image.new('1', (128, 64))
    face._desenha_entrega(ImageDraw.Draw(first), face._alvo_animado(0))
    assert first.crop((106, 0, 117, 61)).getbbox() is None
    tops, gaze = [], []
    for instant in (0.2 * duration, 0.53 * duration, 0.87 * duration, duration):
        pose = face._alvo_animado(instant)
        image = Image.new('1', (128, 64))
        face._desenha_entrega(ImageDraw.Draw(image), pose)
        tops.append(image.crop((106, 0, 117, 61)).getbbox()[1])
        gaze.append(pose.olhar[1])
        assert pose.olhar[0] == 8
        assert pose.arcos == (0, 0), 'A comemoração só deve começar após a chegada.'
        assert pose.brilhos == 0
        assert image.getpixel((90, 62)), 'A mesa deve continuar visível.'
    assert all(a < b for a, b in zip(tops, tops[1:]))
    assert all(a <= b for a, b in zip(gaze, gaze[1:]))
    assert gaze[-1] - gaze[0] > 10
    for instant in (times.fim_recuo + 0.3, times.fim_recuo + 0.5, times.fim_recuo + 0.9):
        happy = face._alvo_animado(instant)
        assert happy.arcos == (8, 8)
        assert happy.boca == 'vazado'
        assert happy.brilhos == 0.55
        assert happy.queda == 1
        image = Image.new('1', (128, 64))
        face._desenha_entrega(ImageDraw.Draw(image), happy)
        assert image.crop((106, 0, 117, 61)).getbbox() == (0, 49, 11, 60)
    assert face._alvo_animado(0) == face._alvo_animado(face._tempos_garra('place_on_table').ciclo)
    before = face._pose(duration + 0.6)
    face._seleciona('place_on_table', duration + 0.6)
    assert face._pose(duration + 0.6) == before


def test_repeated_mission_state_keeps_animation_progress_and_switches_start_forward(face_environment):
    face, _, _ = face_environment
    face._seleciona('follow_wall', 1)
    before = face._pose(6.2)
    assert before.olhar[0] > 0
    face._seleciona('follow_wall', 6.2)
    assert face._pose(6.2) == before
    face._seleciona('place_on_shelf', 6.2)
    assert face._pose(6.2) == before, 'A troca deve partir do quadro atual.'
    after = face._pose(6.7)
    assert after.olhar[1] == -7
    assert after.olhar[0] > 0


@pytest.mark.parametrize('name, vertical, period', [
    ('analyze_scene', 6, 1.8),
    ('place_on_shelf', -7, 2.4),
    ('organize_precision', 6, 1.8),
])
def test_frequent_scanning_preserves_vertical_intent(face_environment, name, vertical, period):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    right = face._alvo_animado(period / 4)
    left = face._alvo_animado(3 * period / 4)
    assert right.olhar[0] == pytest.approx(7)
    assert left.olhar[0] == pytest.approx(-7)
    for frame in range(100):
        assert face._alvo_animado(frame / 20).olhar[1] == vertical


@pytest.mark.parametrize('name', ['pick', 'bravo'])
def test_pick_lifts_the_cube_from_the_table_and_tracks_it_until_exit(face_environment, name):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    times = face._tempos_garra('pick')
    duration, start = times.movimento, times.inicio_movimento
    tops, gaze = [], []
    for instant in (start, start + 0.27 * duration, start + 0.53 * duration, start + 0.8 * duration):
        pose = face._alvo_animado(instant)
        image = Image.new('1', (128, 64))
        face._desenha_entrega(ImageDraw.Draw(image), pose)
        tops.append(image.crop((106, 0, 117, 61)).getbbox()[1])
        gaze.append(pose.olhar[1])
        assert pose.olhar[0] == 8
        assert pose.lingua == 1
        assert image.getpixel((90, 62))
    assert tops[0] == 49, 'O cubo deve começar sobre a mesa.'
    assert all(a > b for a, b in zip(tops, tops[1:]))
    assert all(a >= b for a, b in zip(gaze, gaze[1:]))
    assert gaze[0] - gaze[-1] > 10
    image = Image.new('1', (128, 64))
    face._desenha_entrega(ImageDraw.Draw(image), face._alvo_animado(times.fim_movimento + 0.1))
    assert image.crop((106, 0, 117, 61)).getbbox() is None
    assert image.getpixel((90, 62)), 'A mesa permanece após a saída do cubo.'
    assert face._alvo_animado(0) == face._alvo_animado(times.ciclo)


@pytest.mark.parametrize('name, direction', [('retrieve', -1), ('store', 1)])
def test_transfer_cube_crosses_the_right_side_and_pupils_follow(face_environment, name, direction):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    duration, start = face._duracao_cubo(name), 0
    heights, gaze = [], []
    for instant in (start + part * duration for part in (0.1, 0.3, 0.5, 0.7, 0.9)):
        pose = face._alvo_animado(instant)
        image = Image.new('1', (128, 64))
        face._desenha_transferencia(ImageDraw.Draw(image), pose)
        left, top, right, bottom = image.getbbox()
        assert left >= 105 and right <= 118, 'O cubo deve ficar à direita do rosto.'
        heights.append((top + bottom) / 2)
        gaze.append(pose.olhar[1])
        assert pose.olhar[0] > 0
    assert all(direction * (b - a) > 0 for a, b in zip(heights, heights[1:]))
    assert all(direction * (b - a) >= 0 for a, b in zip(gaze, gaze[1:]))
    assert direction * (gaze[-1] - gaze[0]) > 10
    for instant in (0, duration - 0.01):
        image = Image.new('1', (128, 64))
        face._desenha_transferencia(ImageDraw.Draw(image), face._alvo_animado(instant))
        assert image.getbbox() is None, 'O cubo deve começar e terminar fora da tela.'
    assert face._alvo_animado(0) == face._alvo_animado(duration)


def test_store_and_retrieve_share_the_same_face(face_environment):
    face, _, _ = face_environment
    images = []
    for name in ('retrieve', 'store'):
        image = Image.new('1', (128, 64))
        face._desenha(ImageDraw.Draw(image), face._POSES[name], 1.5)
        images.append(image.tobytes())
    assert images[0] == images[1]


@pytest.mark.parametrize('name', ['pick', 'place_on_precision_table'])
def test_original_tongue_keeps_its_shape(face_environment, name):
    face, _, _ = face_environment
    images = set()
    for frame in range(24):
        image = Image.new('1', (128, 64))
        # A língua original mantém a forma quando a boca fica no mesmo lugar.
        face._boca(ImageDraw.Draw(image), face._POSES[name], 64, 51, frame / 20)
        assert image.getpixel((64, 51)), 'A boca deve continuar no mesmo lugar.'
        assert image.crop((0, 63, 128, 64)).getbbox() is None
        images.add(image.tobytes())
    assert len(images) == 1


def test_angrier_organizer_keeps_the_same_eyes(face_environment):
    from dataclasses import replace

    face, _, _ = face_environment
    new = face._POSES['organize_precision']
    old = replace(new, sobrancelhas=face._POSES['place_on_precision_table'].sobrancelhas,
                  boca='sorriso', largura=10, altura=5)
    images = []
    for pose in (old, new):
        image = Image.new('1', (128, 64))
        face._desenha(ImageDraw.Draw(image), pose, 0.3)
        images.append(image)
    for eye in ((24, 17, 55, 37), (74, 17, 105, 37)):
        assert images[0].crop(eye).tobytes() == images[1].crop(eye).tobytes()
    for changed in ((24, 0, 105, 14), (48, 37, 80, 48)):
        assert images[0].crop(changed).tobytes() != images[1].crop(changed).tobytes()


@pytest.mark.parametrize('name', ['place_in_container', 'stack'])
def test_carried_cube_enters_from_above_and_lands(face_environment, name):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    times = face._tempos_garra(face._POSES[name].cena)
    assert times.aproximacao == times.fechamento == 0
    first = face._alvo_animado(0)
    image = Image.new('1', (128, 64))
    face._desenha_entrega(ImageDraw.Draw(image), first)
    bottom = 49 if name == 'stack' else 59
    assert image.crop((106, 0, 117, bottom)).getbbox() is None
    assert first.garra_abertura == 0
    assert first.garra_y + 14 < 0
    tops, gaze = [], []
    for fraction in (0.4, 0.7, 1):
        pose = face._alvo_animado(times.movimento * fraction)
        image = Image.new('1', (128, 64))
        face._desenha_entrega(ImageDraw.Draw(image), pose)
        tops.append(image.crop((106, 0, 117, bottom)).getbbox()[1])
        gaze.append(pose.olhar[1])
        cube_y = face._posicao_cubo(pose.cena, pose.queda)[1]
        assert pose.garra_y == pytest.approx(cube_y - 11)
        assert pose.olhar[0] > 0
        assert image.getpixel((90, 62))
    assert tops[0] < tops[1] < tops[2]
    assert all(a <= b for a, b in zip(gaze, gaze[1:]))
    assert gaze[-1] > gaze[0]
    landed = face._alvo_animado(times.fim_cubo)
    image = Image.new('1', (128, 64))
    face._desenha_entrega(ImageDraw.Draw(image), landed)
    assert landed.queda == pytest.approx(1)
    if name == 'stack':
        assert image.getpixel((111, 53))
        assert not image.getpixel((111, 48))
    else:
        assert image.getpixel((101, 57))
        assert image.getpixel((111, 52))
    assert face._alvo_animado(times.ciclo).queda == pytest.approx(0)


def _components(image):
    pixels = {(x, y) for y in range(image.height) for x in range(image.width)
              if image.getpixel((x, y))}
    count = 0
    while pixels:
        pending = [pixels.pop()]
        count += 1
        while pending:
            x, y = pending.pop()
            for adjacent in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if adjacent in pixels:
                    pixels.remove(adjacent)
                    pending.append(adjacent)
    return count


def test_organization_renders_seven_slots_and_six_moving_cubes(face_environment):
    face, _, _ = face_environment
    face._seleciona('organize_precision', 0)
    assert face._alvo_animado(0).vertical < 0
    assert face._alvo_animado(0).suor == 1
    positions = set()
    cycle = 12 * (0.12 + face._duracao_cubo('slots'))
    for frame in range(int(cycle * 20) + 1):
        pose = face._alvo_animado(frame / 20)
        image = Image.new('1', (128, 64))
        face._desenha_slots(ImageDraw.Draw(image), pose.fase_slots)
        assert _components(image.crop((0, 62, 128, 63))) == 7
        assert _components(image.crop((0, 47, 128, 61))) == 6
        positions.add(image.tobytes())
    assert len(positions) > 20
    for first, repeated in zip(face._cubos_slots(0.1), face._cubos_slots(cycle + 0.1)):
        assert first == pytest.approx(repeated)


def test_finish_and_celebration_use_animated_fireworks_and_precision_sweats(face_environment):
    face, _, _ = face_environment
    assert face._POSES['place_on_precision_table'].suor == face._POSES['stack'].suor == 1
    for name in ('finish', 'comemorar'):
        assert face._POSES[name].fogos == 1
        assert face._POSES[name].brilhos == 0
    pictures = set()
    for frame in range(36):
        image = Image.new('1', (128, 64))
        face._desenha_fogos(ImageDraw.Draw(image), frame / 20)
        assert image.getbbox() is not None
        pictures.add(image.tobytes())
    assert len(pictures) > 20


@pytest.mark.parametrize('name', ['pick', 'place_on_table', 'place_in_container', 'stack'])
def test_gripper_enters_from_above_holds_the_cube_and_exits(face_environment, name):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    scene = face._POSES[name].cena
    times = face._tempos_garra(scene)
    for instant in (0, times.ciclo - 0.05):
        image = Image.new('1', (128, 64))
        face._desenha_garra(ImageDraw.Draw(image), face._alvo_animado(instant))
        assert image.getbbox() is None, 'A garra deve estar inteiramente fora da tela.'
    if times.aproximacao:
        approaching = face._alvo_animado(times.aproximacao / 2)
        arrived = face._alvo_animado(times.aproximacao)
        assert -18 < approaching.garra_y < arrived.garra_y
        assert approaching.garra_abertura == 1
        assert approaching.queda == arrived.queda == 0, 'O cubo espera a garra chegar.'
    for fraction in (0.25, 0.5, 0.75):
        pose = face._alvo_animado(times.inicio_movimento + times.movimento * fraction)
        _, cube_y = face._posicao_cubo(scene, pose.queda)
        assert pose.garra_abertura == 0, 'A garra segura o cubo durante o deslocamento.'
        assert pose.garra_y == pytest.approx(cube_y - 11)
        image = Image.new('1', (128, 64))
        face._desenha_garra(ImageDraw.Draw(image), pose)
        if cube_y + 3 < 0:
            assert image.getbbox() is None  # Garra e cubo ainda estão acima da tela.
            continue
        left, top, right, _ = image.getbbox()
        assert top == 0, 'O braço deve continuar ligado ao topo da tela.'
        assert 99 <= left < right <= 124
    if name != 'pick':
        opening = face._alvo_animado(times.fim_movimento + times.abertura / 2)
        released = face._alvo_animado(times.fim_movimento + times.abertura)
        returning = face._alvo_animado(times.fim_movimento + times.abertura + times.recuo / 2)
        assert 0 < opening.garra_abertura < 1
        assert released.garra_abertura == returning.garra_abertura == 1
        assert returning.garra_y < released.garra_y
        if name == 'place_in_container':
            assert released.queda < returning.queda <= 1
        else:
            assert released.queda == returning.queda == 1


@pytest.mark.parametrize('name', ['organize_precision', 'store', 'retrieve'])
def test_scenes_without_gripper_do_not_draw_one(face_environment, monkeypatch, name):
    face, _, _ = face_environment
    face._seleciona(name, 0)

    def unexpected_gripper(*args):
        pytest.fail(f'{name} deve continuar sem garra.')

    monkeypatch.setattr(face, '_desenha_garra', unexpected_gripper)
    for instant in (0, 0.8, 2, 5):
        image = Image.new('1', (128, 64))
        face._desenha(ImageDraw.Draw(image), face._alvo_animado(instant), instant)
        if name == 'organize_precision':
            assert _components(image.crop((0, 62, 128, 63))) == 7
            assert _components(image.crop((0, 47, 128, 61))) == 6
        else:
            assert image.getbbox() is not None


def test_container_releases_above_the_rim_while_gripper_retracts(face_environment):
    face, _, _ = face_environment
    face._seleciona('place_in_container', 0)
    times = face._tempos_garra('container')
    release = times.fim_movimento + times.abertura
    before = face._alvo_animado(times.fim_movimento)
    assert face._posicao_cubo('container', before.queda)[1] + 5 < 44
    later = []
    for fraction in (0, 0.5, 1):
        pose = face._alvo_animado(release + times.queda_livre * fraction)
        later.append(pose)
        assert pose.garra_abertura == 1
    assert later[0].queda < later[1].queda < later[2].queda
    assert later[0].garra_y > later[1].garra_y > later[2].garra_y
    assert later[0].olhar[1] < later[1].olhar[1] <= later[2].olhar[1]
    assert later[-1].queda == pytest.approx(1)
    for frame in range(int(times.ciclo * 20) + 1):
        pose = face._alvo_animado(frame / 20)
        image = Image.new('1', (128, 64))
        face._desenha_garra(ImageDraw.Draw(image), pose)
        if image.getbbox():
            assert image.getbbox()[3] < 44, 'A garra nunca deve entrar no recipiente.'


@pytest.mark.parametrize('name', ['pick', 'place_on_table', 'place_in_container', 'stack'])
def test_pupils_follow_the_scene_target_through_release(face_environment, name):
    face, _, _ = face_environment
    face._seleciona(name, 0)
    times = face._tempos_garra(face._POSES[name].cena)
    for frame in range(int(times.ciclo * 20)):
        pose = face._alvo_animado(frame / 20)
        target_y = (face._posicao_cubo(pose.cena, pose.queda)[1]
                    if name == 'place_in_container' else pose.garra_y + 7)
        expected_y = max(-7, min(7, (target_y - 31) * 0.4))
        assert pose.olhar == pytest.approx((8, expected_y))
    if name == 'pick':
        approaching = [face._alvo_animado(times.aproximacao * fraction) for fraction in (0.25, 0.75)]
        assert approaching[0].queda == approaching[1].queda == 0
        assert approaching[0].olhar[1] < approaching[1].olhar[1]
    else:
        start = times.fim_movimento + times.abertura
        returning = [face._alvo_animado(start + times.recuo * fraction) for fraction in (0.1, 0.8)]
        if name == 'place_in_container':
            assert returning[0].olhar[1] < returning[1].olhar[1]
        else:
            assert returning[0].olhar[1] > returning[1].olhar[1]
        if name == 'place_on_table':
            assert all(pose.arcos == (0, 0) for pose in returning)
