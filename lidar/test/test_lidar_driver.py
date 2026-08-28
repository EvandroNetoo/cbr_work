import math

import lidar.lidar_driver as lidar_driver_module
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


class CountingLock:
    def __init__(self):
        self.enter_count = 0

    def __enter__(self):
        self.enter_count += 1
        return self

    def __exit__(self, *_args):
        return False


def make_packet(index=0xA0, rpm=274.0, distance_mm=1000):
    packet = bytearray(22)
    packet[0] = 0xFA
    packet[1] = index
    speed = round(rpm * 64.0)
    packet[2] = speed & 0xFF
    packet[3] = (speed >> 8) & 0xFF
    for offset in (4, 8, 12, 16):
        packet[offset] = distance_mm & 0xFF
        packet[offset + 1] = (distance_mm >> 8) & 0x3F

    checksum = 0
    for word_index in range(10):
        word = packet[word_index * 2] | (packet[word_index * 2 + 1] << 8)
        checksum = ((checksum << 1) + word) & 0xFFFFFFFF
    checksum = ((checksum & 0x7FFF) + (checksum >> 15)) & 0x7FFF
    packet[20] = checksum & 0xFF
    packet[21] = (checksum >> 8) & 0xFF
    return bytes(packet)


def make_driver():
    serial = FakeSerial()
    relay = FakeRelay()
    driver = LidarDriver(
        LidarConfig(
            angle_start_deg=307,
            angle_end_deg=217,
            valid_intervals_deg=(307, 67, 167, 196),
        ),
        serial_connection=serial,
        relay=relay,
        start_thread=False,
    )
    driver._rpm = 274.0
    return driver, serial, relay


def test_two_valid_intervals_keep_fixed_angular_positions():
    driver, serial, relay = make_driver()
    lock = CountingLock()
    driver._scan_lock = lock
    now_ns = 1_000_000_000
    angles = [306] + list(range(307, 360)) + list(range(0, 219))
    for angle in angles:
        now_ns += 608_000
        if angle in (320, 10):
            continue
        driver._process_sample(angle, 1.0, now_ns)

    assert lock.enter_count == 1
    scan = driver.take_scan()
    assert lock.enter_count == 2
    assert scan is not None
    assert len(scan.ranges_m) == 271
    assert scan.ranges_m[0] == 1.0
    assert math.isnan(scan.ranges_m[-1])  # 217 graus: parte bloqueada
    assert math.isnan(scan.ranges_m[13])  # 320 graus
    assert math.isnan(scan.ranges_m[63])  # 10 graus
    assert math.isnan(scan.ranges_m[121])  # 68 graus: parte interna
    assert math.isnan(scan.ranges_m[219])  # 166 graus: parte interna
    assert scan.ranges_m[220] == 1.0  # 167 graus: setor traseiro
    assert scan.ranges_m[249] == 1.0  # 196 graus: fim do setor traseiro
    assert math.isnan(scan.ranges_m[250])  # 197 graus: parte bloqueada
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


def test_real_configuration_has_two_valid_intervals():
    config = LidarConfig()
    config.validate()
    assert config.serial_port == 1
    assert config.serial_read_chunk_size == 128
    assert config.relay_pin == 266
    assert config.sample_count == 271
    assert config.valid_intervals_deg == (307, 67, 167, 196)
    assert config.is_angle_valid(350)
    assert config.is_angle_valid(20)
    assert config.is_angle_valid(180)
    assert not config.is_angle_valid(200)
    assert not config.is_angle_valid(100)


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

    def process(data):
        received.append(data)
        driver._stop_event.set()

    driver._process_data = process
    driver._read_loop()

    assert serial.requested_sizes == [128]
    assert received == [bytes((0xFA, 0xA0, 0x01))]
    driver.close()


def test_block_parser_handles_fragmentation_and_resynchronizes_after_noise():
    driver, _, _ = make_driver()
    driver._rpm = 0.0
    packet = make_packet()

    driver._process_data(b'ruido\xfa\x00' + packet[:7])
    assert driver._rpm == 0.0
    assert driver._last_packet_monotonic == 0.0

    driver._process_data(packet[7:] + make_packet(index=0xA1))

    assert driver._rpm == 274.0
    assert driver._last_packet_monotonic > 0.0
    assert driver._input_buffer == bytearray()
    driver.close()


def test_packet_uses_one_monotonic_timestamp_for_four_samples(monkeypatch):
    driver, _, _ = make_driver()
    timestamps = []

    def monotonic_ns():
        timestamps.append(2_000_000_000)
        return timestamps[-1]

    monkeypatch.setattr(lidar_driver_module.time, 'monotonic_ns', monotonic_ns)
    driver._process_data(make_packet())

    assert timestamps == [2_000_000_000]
    driver.close()
