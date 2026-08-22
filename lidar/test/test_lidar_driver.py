import math

from lidar.lidar_driver import LidarConfig, LidarDriver, resolve_serial_port


class FakeSerial:
    in_waiting = 0

    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FakeRelay:
    def __init__(self):
        self.on = False
        self.closed = False

    def turn_on(self):
        self.on = True

    def turn_off(self):
        self.on = False

    def close(self):
        self.turn_off()
        self.closed = True


def make_driver():
    serial = FakeSerial()
    relay = FakeRelay()
    driver = LidarDriver(
        LidarConfig(angle_start_deg=307, angle_end_deg=67),
        serial_connection=serial,
        relay=relay,
        start_thread=False,
    )
    driver._rpm = 274.0
    return driver, serial, relay


def test_sparse_sector_keeps_fixed_angular_positions():
    driver, serial, relay = make_driver()
    now_ns = 1_000_000_000
    angles = [306] + list(range(307, 360)) + list(range(0, 69))
    for angle in angles:
        now_ns += 608_000
        if angle in (320, 10):
            continue
        driver._process_sample(angle, 1.0, now_ns)

    scan = driver.take_scan()
    assert scan is not None
    assert len(scan.ranges_m) == 121
    assert scan.ranges_m[0] == 1.0
    assert scan.ranges_m[-1] == 1.0
    assert math.isnan(scan.ranges_m[13])  # 320 graus
    assert math.isnan(scan.ranges_m[63])  # 10 graus
    assert scan.rpm == 274.0
    assert driver.take_scan() is None

    driver.close()
    assert serial.closed
    assert relay.closed
    assert not relay.on


def test_serial1_resolves_usb_physical_port_1_4(tmp_path):
    expected = tmp_path / 'platform-x-usb-0:1.4:1.0-port0'
    expected.touch()
    assert resolve_serial_port(1, tmp_path) == str(expected)


def test_real_configuration_has_121_samples():
    config = LidarConfig()
    config.validate()
    assert config.serial_port == 1
    assert config.serial_read_chunk_size == 64
    assert config.relay_pin == 266
    assert config.sample_count == 121


def test_read_loop_processes_serial_in_configured_blocks():
    driver, _, _ = make_driver()
    received = []

    class ChunkSerial:
        def __init__(self):
            self.requested_sizes = []

        def read(self, size):
            self.requested_sizes.append(size)
            return bytes((0xFA, 0xA0, 0x01))

        def close(self):
            pass

    serial = ChunkSerial()
    driver._serial = serial

    def process(value):
        received.append(value)
        if len(received) == 3:
            driver._stop_event.set()

    driver._process_byte = process
    driver._read_loop()

    assert serial.requested_sizes == [64]
    assert received == [0xFA, 0xA0, 0x01]
    driver.close()
