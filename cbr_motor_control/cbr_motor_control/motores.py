"""Brick motor protocol copied from MariolaZero's ``motores.py``.

This package keeps the protocol paths used by ``main.py``: initialization,
speed control, brake mode and calibration query. The serial dependency comes
from the workspace virtual environment.
"""

import struct

from .portas import Portas


class Motores:
    DEBUG = False
    BREAK = 0
    HOLD = 1
    ENVIA_SERVOS = 0xFD
    ENVIA_MOTORES = 0xFC
    ENVIA_MOTORES_4X4 = 0xF8
    OBTEM_CALIBRACAO_MOTORES = 0xF4

    def __init__(self, atualiza_instantaneo=False):
        # Protocol packets must always keep their ten-byte length.
        self.lista_servos = [
            self.ENVIA_SERVOS, 200, 200, 200, 200, 200, 200, 0, 0, 0]
        self.lista_motores = [
            self.ENVIA_MOTORES, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.angulo_motor1 = 0
        self.angulo_motor2 = 0
        self.angulo_absoluto_motor1 = 0
        self.angulo_absoluto_motor2 = 0
        self.angulo_delta_motor1 = 0
        self.angulo_delta_motor2 = 0
        self.estado_motores = 0
        self.modo_freio = self.BREAK
        self.motor_invertido = [False, False, False, False]
        self.atualiza_instantaneo = atualiza_instantaneo
        self._fechado = False

        self.ser = Portas().abre_porta_serial(
            Portas._SERIAL0, 250000)
        if self.ser is None:
            raise RuntimeError('Erro ao abrir a porta serial do brick')
        self.atualiza_motores()
        self.atualiza_servos()
        self.reseta_angulo_motor(1)
        self.reseta_angulo_motor(2)

    def atualiza_servos(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(bytes(self.lista_servos))
        retorno_serial = self.ser.read(1)
        if len(retorno_serial) == 1:
            if retorno_serial[0] == self.ENVIA_SERVOS:
                return True
        raise RuntimeError('Erro ao ler o estado dos servos')

    def atualiza_motores(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(bytes(self.lista_motores))
        self.angulo_motor1 = 0
        self.angulo_motor2 = 0
        self.lista_motores[5:9] = [0, 0, 0, 0]
        retorno_serial = self.ser.read(10)
        if len(retorno_serial) == 10:
            if retorno_serial[0] == self.ENVIA_MOTORES:
                self.angulo_absoluto_motor1 = struct.unpack(
                    '>i', bytes(retorno_serial[1:5]))[0]
                self.angulo_absoluto_motor2 = struct.unpack(
                    '>i', bytes(retorno_serial[5:9]))[0]
                self.estado_motores = retorno_serial[9]
                return True
        raise RuntimeError('Erro ao ler o estado dos motores')

    def velocidade_motores(self, velocidade1, velocidade2):
        print(f'Velocidade dos motores: {velocidade1}, {velocidade2}')
        self.lista_motores[0] = self.ENVIA_MOTORES
        velocidade1 = max(-120, min(120, int(velocidade1)))
        velocidade2 = max(-120, min(120, int(velocidade2)))
        if self.motor_invertido[0]:
            velocidade1 = -velocidade1
        if self.motor_invertido[1]:
            velocidade2 = -velocidade2
        self.lista_motores[1] = struct.pack('b', velocidade1)[0]
        self.lista_motores[2] = struct.pack('b', velocidade2)[0]
        if self.atualiza_instantaneo:
            self.atualiza_motores()

    def set_modo_freio(self, modo):
        self.modo_freio = self.BREAK if modo == self.BREAK else self.HOLD
        self.lista_motores[9] = self.modo_freio

    def obtem_calibracao_motores(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.lista_motores[0] = self.OBTEM_CALIBRACAO_MOTORES
        self.ser.write(bytes(self.lista_motores))
        retorno_serial = self.ser.read(10)
        if len(retorno_serial) == 10:
            if retorno_serial[0] == self.OBTEM_CALIBRACAO_MOTORES:
                motor1 = struct.unpack('>i', bytes(retorno_serial[1:5]))[0]
                motor2 = struct.unpack('>i', bytes(retorno_serial[5:9]))[0]
                return motor1, motor2
        raise RuntimeError('Erro ao ler a calibração dos motores')

    def reseta_angulo_motor(self, motor):
        if motor == 1:
            self.angulo_delta_motor1 = self.angulo_absoluto_motor1
        elif motor == 2:
            self.angulo_delta_motor2 = self.angulo_absoluto_motor2
        else:
            return
        if self.atualiza_instantaneo:
            self.atualiza_motores()

    def fechar(self):
        if self._fechado:
            return
        try:
            self.velocidade_motores(0, 0)
        finally:
            self._fechado = True
            self.ser.close()

    def __del__(self):
        try:
            self.fechar()
        except Exception:
            # Destructors must not hide the original shutdown error.
            pass
