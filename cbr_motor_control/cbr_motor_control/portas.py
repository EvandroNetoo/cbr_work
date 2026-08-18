"""Serial port mapping copied from MariolaZero's expansion example."""

import subprocess

import serial


class Portas:
    _SERIAL0 = 0
    SERIAL1 = 1
    SERIAL2 = 2
    SERIAL3 = 3
    SERIAL4 = 4
    SERIAL5 = 5
    SERIAL6 = 6
    I2C1 = 0
    I2C2 = 1
    I2C3 = 7
    I2C4 = 2
    I2C5 = 3
    I2C6 = 6
    I2C7 = 5
    I2C8 = 4

    _USB_PATHS = {
        _SERIAL0: 'usb-0:1.1',
        SERIAL1: 'usb-0:1.4',
        SERIAL2: 'usb-0:1.3',
        SERIAL3: 'usb-0:1.2',
    }
    _UART_PATHS = {
        SERIAL4: '/dev/ttyS4',
        SERIAL5: '/dev/ttyS2',
        SERIAL6: '/dev/ttyS5',
    }

    def porta_serial_real(self, porta):
        """Resolve the Mariola logical connector to a Linux device."""
        if porta in self._USB_PATHS:
            try:
                output = subprocess.check_output(
                    ['ls', '-l', '/dev/serial/by-path/'], text=True)
            except Exception as exc:
                raise RuntimeError(
                    f'Erro ao descobrir a porta serial: {exc}') from exc

            marker = self._USB_PATHS[porta]
            for line in output.splitlines():
                if marker in line:
                    target = line.split('->')[-1].strip()
                    return '/dev/' + target.split('/')[-1]
            raise RuntimeError(
                f'Nenhuma porta correspondente a {marker} foi encontrada')

        if porta in self._UART_PATHS:
            return self._UART_PATHS[porta]
        raise ValueError('Porta serial inválida.')

    def abre_porta_serial(self, porta, baud_rate=250000, timeout=0.010):
        porta_real = self.porta_serial_real(porta)
        try:
            ser = serial.Serial(porta_real, baud_rate, timeout=timeout)
            print(
                f'Comunicação estabelecida com sucesso na porta {porta_real}.')
            return ser
        except serial.SerialException as exc:
            print(f'Erro ao tentar se comunicar com a porta {porta_real}: {exc}')
            return None
