import math

import pytest

from base_hardware.controleMotores import (
    GrupoMotoresBrick,
    GrupoMotoresExpansao,
)
from base_hardware.mariola_adapter import (
    BRICK_TICKS_PER_REVOLUTION,
    EXPANSION_TICKS_PER_REVOLUTION,
    MIN_EFFECTIVE_WHEEL_COMMAND,
    MariolaConfig,
    MariolaBase,
    WHEEL_NAMES,
    radians_per_second_to_command,
    ticks_to_radians,
    validate_complete_command,
)


class FakeControle:
    def __init__(self):
        self.nomes = WHEEL_NAMES
        self.commands = []
        self.ticks = {
            'front_left_wheel_joint': 3288,
            'front_right_wheel_joint': 3288,
            'rear_left_wheel_joint': 1644,
            'rear_right_wheel_joint': 1644,
        }
        self.reset_calls = 0
        self.brake_calls = 0
        self.close_calls = []

    def reseta_angulos_motores(self):
        self.reset_calls += 1

    def definir_velocidades(self, **commands):
        self.commands.append(commands)

    def angulos_motores(self):
        return dict(self.ticks)

    def frear(self):
        self.brake_calls += 1

    def fechar(self, parar_motores=True):
        self.close_calls.append(parar_motores)


class FakeBrick:
    BREAK = 0
    HOLD = 1
    ser = 'fake'

    def __init__(self):
        self.commands = []
        self.mode = None
        self.angles = {1: 1644, 2: 1644}

    def velocidade_motores(self, left, right):
        self.commands.append((left, right))

    def set_modo_freio(self, mode):
        self.mode = mode

    def atualiza_motores(self):
        pass

    def estado(self):
        pass

    def angulo_motor(self, index):
        return self.angles[index]

    def reseta_angulo_motor(self, index):
        self.angles[index] = 0


class FakeExpansion:
    FREIO_BREAK = 0
    FREIO_TRAVADO = 1

    def __init__(self):
        self.commands = []
        self.angle = 3288
        self.brake = None

    def velocidade_motor(self, value):
        self.commands.append(value)
        return True

    def set_freio(self, value):
        self.brake = value
        return True

    def angulo_motor(self):
        return self.angle

    def reseta_angulo_motor(self):
        self.angle = 0


def test_one_revolution_uses_backend_specific_resolution():
    assert ticks_to_radians(1644, BRICK_TICKS_PER_REVOLUTION) == pytest.approx(2 * math.pi)
    assert ticks_to_radians(3288, EXPANSION_TICKS_PER_REVOLUTION) == pytest.approx(2 * math.pi)


def test_linear_velocity_conversion_and_bounds():
    assert MIN_EFFECTIVE_WHEEL_COMMAND == 2
    assert radians_per_second_to_command(0.0) == 0
    assert radians_per_second_to_command(0.001) == 2
    assert radians_per_second_to_command(-0.001) == -2
    assert radians_per_second_to_command(7.0) == 100
    assert radians_per_second_to_command(3.5) == 50
    assert radians_per_second_to_command(-7.0) == -100
    with pytest.raises(ValueError):
        radians_per_second_to_command(7.01)
    with pytest.raises(ValueError):
        radians_per_second_to_command(float('nan'))
    with pytest.raises(ValueError):
        radians_per_second_to_command(1.0, min_effective_command=101)


def test_minimum_effective_command_is_configurable():
    controle = FakeControle()
    base = MariolaBase(
        config=MariolaConfig(min_effective_wheel_command=3),
        controle=controle,
    )
    base.write({name: 0.001 for name in WHEEL_NAMES})
    assert controle.commands[-1] == {name: 3 for name in WHEEL_NAMES}


@pytest.mark.parametrize('minimum', [-1, 101, 2.5, True])
def test_config_rejects_invalid_minimum_effective_command(minimum):
    with pytest.raises(ValueError):
        MariolaConfig(min_effective_wheel_command=minimum)


def test_command_must_have_all_wheels():
    with pytest.raises(ValueError):
        validate_complete_command({'front_left_wheel_joint': 0.0})


def test_adapter_uses_controle_motores_for_commands_and_encoders():
    controle = FakeControle()
    base = MariolaBase(controle=controle)
    assert controle.reset_calls == 1

    base.write({name: 3.5 for name in WHEEL_NAMES})
    assert controle.commands[-1] == {name: 50 for name in WHEEL_NAMES}

    states = base.read(now=1.0)
    for name in WHEEL_NAMES:
        assert states[name].position == pytest.approx(2 * math.pi)

    base.stop()
    assert controle.brake_calls == 1
    base.close(stop=False)
    assert controle.close_calls == [False]


def test_original_groups_apply_physical_inversions():
    brick = FakeBrick()
    rear = GrupoMotoresBrick(
        brick,
        ('rear_left_wheel_joint', 'rear_right_wheel_joint'),
        (False, True),
    )
    rear.aplicar({
        'rear_left_wheel_joint': 50,
        'rear_right_wheel_joint': 50,
    })
    assert brick.commands[-1] == (50, -50)

    front_left = FakeExpansion()
    front_right = FakeExpansion()
    front = GrupoMotoresExpansao(
        {
            'front_left_wheel_joint': front_left,
            'front_right_wheel_joint': front_right,
        },
        {
            'front_left_wheel_joint': True,
            'front_right_wheel_joint': False,
        },
    )
    front.aplicar({
        'front_left_wheel_joint': 50,
        'front_right_wheel_joint': 50,
    })
    assert front_left.commands[-1] == -50
    assert front_right.commands[-1] == 50
