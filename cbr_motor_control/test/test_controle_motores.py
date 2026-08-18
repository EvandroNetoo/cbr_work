from cbr_motor_control.controle_motores import (
    ControleMotores,
    ErroControleMotores,
    GrupoMotoresBrick,
    GrupoMotoresExpansao,
)
import pytest


class FakeBrick:
    def __init__(self):
        self.calls = []

    def velocidade_motores(self, left, right):
        self.calls.append((left, right))


class FakeExpansionMotor:
    def __init__(self, response=True):
        self.calls = []
        self.response = response

    def velocidade_motor(self, speed):
        self.calls.append(speed)
        return {'ok': True} if self.response else None


def make_controller(right_motor_response=True):
    brick = FakeBrick()
    front_left = FakeExpansionMotor()
    front_right = FakeExpansionMotor(right_motor_response)
    controller = ControleMotores([
        GrupoMotoresBrick(
            brick,
            ('traseiro_esquerdo', 'traseiro_direito'),
            (False, True),
        ),
        GrupoMotoresExpansao(
            {
                'dianteiro_esquerdo': front_left,
                'dianteiro_direito': front_right,
            },
            {
                'dianteiro_esquerdo': True,
                'dianteiro_direito': False,
            },
        ),
    ])
    return controller, brick, front_left, front_right


def test_original_mounting_inversions_are_applied():
    controller, brick, front_left, front_right = make_controller()
    controller.definir_velocidades(
        dianteiro_esquerdo=10,
        dianteiro_direito=20,
        traseiro_esquerdo=30,
        traseiro_direito=40,
    )
    assert brick.calls == [(30, -40)]
    assert front_left.calls == [-10]
    assert front_right.calls == [20]
    controller.fechar()


def test_missing_expansion_response_stops_every_group():
    controller, brick, front_left, front_right = make_controller(False)
    with pytest.raises(ErroControleMotores):
        controller.definir_velocidades(
            dianteiro_esquerdo=10,
            dianteiro_direito=20,
            traseiro_esquerdo=30,
            traseiro_direito=40,
        )
    assert controller.velocidades_atuais == dict.fromkeys(
        controller.nomes, 0)
    assert brick.calls[-1] == (0, 0)
    assert front_left.calls[-1] == 0
    with pytest.raises(ErroControleMotores):
        controller.fechar()
