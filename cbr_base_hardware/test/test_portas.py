from cbr_base_hardware.portas import Portas


PORTS = """\
lrwxrwxrwx 1 root root 13 platform-5200000.usb-usb-0:1.1.1:1.0-port0 -> ../../ttyUSB4
lrwxrwxrwx 1 root root 13 platform-5310000.usb-usb-0:1.1:1.0-port0 -> ../../ttyUSB0
lrwxrwxrwx 1 root root 13 platform-5310000.usb-usb-0:1.2:1.0-port0 -> ../../ttyUSB1
lrwxrwxrwx 1 root root 13 platform-5310000.usb-usb-0:1.3:1.0-port0 -> ../../ttyUSB2
lrwxrwxrwx 1 root root 13 platform-5310000.usb-usb-0:1.4:1.0-port0 -> ../../ttyUSB3
"""


def test_serial0_does_not_match_arm_on_nested_usb_path(monkeypatch):
    monkeypatch.setattr(
        'cbr_base_hardware.portas.subprocess.check_output',
        lambda *args, **kwargs: PORTS,
    )

    portas = Portas()
    assert portas.porta_serial_real(Portas._SERIAL0) == '/dev/ttyUSB0'
    assert portas.porta_serial_real(Portas.SERIAL3) == '/dev/ttyUSB1'
