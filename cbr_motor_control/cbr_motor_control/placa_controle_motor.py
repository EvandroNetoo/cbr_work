"""Expansion motor-board protocol copied from MariolaZero.

Source: ``MariolaZero/exemplos/10-expansaoMotor/placaControleMotor.py``.
The implementation below contains every protocol operation used by the
original ``main.py`` and keeps its packet, echo, retry and CRC behavior.
"""

import struct
import time


class PlacaControleMotor:
    TAMANHO_PACOTE_CMD = 6
    TAMANHO_PACOTE_RESP = 8
    DEBUG = False

    MODO_PID = 0
    MODO_SET_FREIO = 2
    MODO_RESET = 3
    MODO_SET_KP = 5
    MODO_SET_KI = 6
    MODO_SET_KD = 7
    MODO_SET_CALIBRACAO = 8
    MODO_SET_KP_FREIO = 11
    MODO_SET_KD_FREIO = 12
    MODO_SET_DELTA_FREIO = 13

    FREIO_BREAK = 0
    FREIO_TRAVADO = 1

    def __init__(self, objeto_porta_serial, id_equipamento):
        self.id_equipamento = id_equipamento & 0xFF
        self.motor_invertido = False
        self.angulo_delta = 0
        self.ser = objeto_porta_serial
        self.ignorar_retorno = False
        if self.ser is None:
            raise RuntimeError(
                'Erro ao abrir a porta serial da PlacaControleMotor')

    @staticmethod
    def _calcular_crc(buf):
        crc = 0
        for byte in buf:
            crc ^= byte
        return crc & 0xFF

    def _montar_pacote(self, modo, valor, giro=0):
        modo = max(0, min(255, modo))
        valor_byte = valor & 0xFF
        giro = max(0, min(65535, giro))
        pacote = [
            self.id_equipamento,
            modo & 0xFF,
            valor_byte,
            (giro >> 8) & 0xFF,
            giro & 0xFF,
        ]
        pacote.append(self._calcular_crc(pacote))
        return bytes(pacote)

    def _processar_resposta(self, dados):
        if len(dados) != self.TAMANHO_PACOTE_RESP:
            return None
        crc_calculado = self._calcular_crc(dados[:7])
        if crc_calculado != dados[7]:
            return None
        return {
            'id': dados[0],
            'modo': dados[1],
            'estado': dados[2],
            'pulsos': struct.unpack('>i', bytes(dados[3:7]))[0],
        }

    def envia_comando(self, modo, valor, giro=0, tentativas=3):
        pacote = self._montar_pacote(modo, valor, giro)
        for tentativa in range(tentativas):
            self.ser.reset_input_buffer()
            self.ser.write(pacote)
            self.ser.flush()

            eco = self.ser.read(self.TAMANHO_PACOTE_CMD)
            resposta = self.ser.read(self.TAMANHO_PACOTE_RESP)
            if self.DEBUG:
                print(f'TX:  {[f"0x{b:02x}" for b in pacote]}')
                print(f'ECO: {[f"0x{b:02x}" for b in eco]}')
                print(f'RX:  {[f"0x{b:02x}" for b in resposta]}')

            if self.ignorar_retorno:
                return None
            resultado = self._processar_resposta(resposta)
            if resultado is not None:
                return resultado
            if self.DEBUG:
                print(f'Tentativa {tentativa + 1}/{tentativas} falhou')
            time.sleep(0.002)
        return None

    def mover_pid(self, velocidade, giro=0):
        velocidade = max(-100, min(100, velocidade))
        return self.envia_comando(self.MODO_PID, velocidade, giro)

    def velocidade_motor(self, velocidade):
        velocidade = max(-100, min(100, velocidade))
        if self.motor_invertido:
            velocidade = -velocidade
        return self.mover_pid(velocidade)

    def reset(self):
        resultado = self.envia_comando(self.MODO_RESET, 0)
        if resultado is not None:
            self.motor_invertido = False
            self.angulo_delta = 0
        return resultado

    def set_freio(self, modo_freio):
        return self.envia_comando(self.MODO_SET_FREIO, modo_freio)

    def calibracao_manual(
        self, giro_max_horario, giro_max_antihorario, tentativas=3
    ):
        giro_max_horario = max(
            -32768, min(32767, int(giro_max_horario)))
        giro_max_antihorario = max(
            -32768, min(32767, int(giro_max_antihorario)))

        resultado = self._enviar_calibracao(
            0, giro_max_horario, tentativas)
        if resultado is None:
            return None
        resultado = self._enviar_calibracao(
            1, giro_max_antihorario, tentativas)
        if resultado is None:
            return None
        return resultado

    def _enviar_calibracao(self, direcao, valor, tentativas=3):
        valor_u16 = valor & 0xFFFF
        pacote = [
            self.id_equipamento,
            self.MODO_SET_CALIBRACAO,
            direcao & 0xFF,
            (valor_u16 >> 8) & 0xFF,
            valor_u16 & 0xFF,
        ]
        pacote.append(self._calcular_crc(pacote))
        dados = bytes(pacote)

        for _ in range(tentativas):
            self.ser.reset_input_buffer()
            self.ser.write(dados)
            self.ser.flush()
            self.ser.read(self.TAMANHO_PACOTE_CMD)
            resposta = self.ser.read(self.TAMANHO_PACOTE_RESP)
            if len(resposta) == self.TAMANHO_PACOTE_RESP:
                crc = self._calcular_crc(resposta[:7])
                if crc == resposta[7]:
                    return {
                        'giro_max_horario': struct.unpack(
                            '>h', bytes(resposta[3:5]))[0],
                        'giro_max_antihorario': struct.unpack(
                            '>h', bytes(resposta[5:7]))[0],
                    }
            time.sleep(0.005)
        return None

    def set_kp(self, valor):
        return self._enviar_constante_pid(self.MODO_SET_KP, valor)

    def set_ki(self, valor):
        return self._enviar_constante_pid(self.MODO_SET_KI, valor)

    def set_kd(self, valor):
        return self._enviar_constante_pid(self.MODO_SET_KD, valor)

    def set_kp_freio(self, valor):
        return self._enviar_constante_pid(self.MODO_SET_KP_FREIO, valor)

    def set_kd_freio(self, valor):
        return self._enviar_constante_pid(self.MODO_SET_KD_FREIO, valor)

    def set_delta_freio(self, valor):
        valor_raw = max(1, min(65535, int(valor)))
        pacote = [
            self.id_equipamento,
            self.MODO_SET_DELTA_FREIO & 0xFF,
            (valor_raw >> 8) & 0xFF,
            valor_raw & 0xFF,
            0,
        ]
        pacote.append(self._calcular_crc(pacote))
        dados = bytes(pacote)
        return self._enviar_pacote_configuracao(dados, 3)

    def _enviar_constante_pid(self, modo, valor, tentativas=3):
        valor_raw = max(0, min(65535, int(round(valor * 100))))
        pacote = [
            self.id_equipamento,
            modo & 0xFF,
            (valor_raw >> 8) & 0xFF,
            valor_raw & 0xFF,
            0,
        ]
        pacote.append(self._calcular_crc(pacote))
        return self._enviar_pacote_configuracao(bytes(pacote), tentativas)

    def _enviar_pacote_configuracao(self, dados, tentativas):
        for _ in range(tentativas):
            self.ser.reset_input_buffer()
            self.ser.write(dados)
            self.ser.flush()
            self.ser.read(self.TAMANHO_PACOTE_CMD)
            resposta = self.ser.read(self.TAMANHO_PACOTE_RESP)
            resultado = self._processar_resposta(resposta)
            if resultado is not None:
                return resultado
            time.sleep(0.005)
        return None

    def pid_motor(self, kp, ki, kd):
        r1 = self.set_kp(kp)
        r2 = self.set_ki(ki)
        r3 = self.set_kd(kd)
        return r1 is not None and r2 is not None and r3 is not None
