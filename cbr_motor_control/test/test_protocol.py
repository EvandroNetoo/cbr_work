from cbr_motor_control.motores import Motores
from cbr_motor_control.placa_controle_motor import PlacaControleMotor


class FakeSerial:
    pass


def test_command_packet_keeps_original_six_byte_crc_protocol():
    motor = PlacaControleMotor(FakeSerial(), id_equipamento=7)
    packet = motor._montar_pacote(motor.MODO_PID, -42)
    assert packet == bytes([7, 0, 214, 0, 0, 209])
    assert PlacaControleMotor._calcular_crc(packet[:5]) == packet[5]


def test_expansion_speed_command_uses_pid_mode():
    motor = PlacaControleMotor(FakeSerial(), id_equipamento=7)
    calls = []
    motor.envia_comando = lambda mode, value, turns=0: calls.append(
        (mode, value, turns))

    motor.velocidade_motor(-42)

    assert calls == [(motor.MODO_PID, -42, 0)]


def test_brick_speed_command_uses_speed_packet():
    motors = Motores.__new__(Motores)
    motors.lista_motores = [0] * 10
    motors.motor_invertido = [False] * 4
    motors.atualiza_instantaneo = False

    motors.velocidade_motores(42, -42)

    assert motors.lista_motores[:3] == [motors.ENVIA_MOTORES, 42, 214]


def test_valid_response_is_decoded():
    motor = PlacaControleMotor(FakeSerial(), id_equipamento=7)
    data = bytearray([7, 1, 2, 0, 0, 0, 123])
    data.append(PlacaControleMotor._calcular_crc(data))
    assert motor._processar_resposta(bytes(data)) == {
        'id': 7,
        'modo': 1,
        'estado': 2,
        'pulsos': 123,
    }


def test_response_mode_is_handled_like_the_original_driver():
    motor = PlacaControleMotor(FakeSerial(), id_equipamento=7)
    # A reset reply may already report the board's resulting PID mode (0).
    data = bytearray([7, 0, 0, 0, 0, 0, 0])
    data.append(PlacaControleMotor._calcular_crc(data))
    assert motor._processar_resposta(bytes(data))['modo'] == 0
