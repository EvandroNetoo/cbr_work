import math
import struct

from cbr_imu.imu_driver import GyroBiasCalibrator, ImuDriver
import pytest


VALUES = (1.0, 0.01, -0.02, 0.03, 0.001, -0.002, 0.003, 0.1, -0.2, 9.81)


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.reset_count = 0
        self.closed = False
        self.is_open = True

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data):
        self.writes.append(data)

    def read(self, size):
        del size
        return self.chunks.pop(0) if self.chunks else b''

    def close(self):
        self.closed = True
        self.is_open = False


def test_reads_fragmented_little_endian_packet_and_closes():
    packet = struct.pack('<10f', *VALUES)
    serial = FakeSerial([packet[:7], packet[7:31], packet[31:]])
    driver = ImuDriver(serial_instance=serial)
    sample = driver.read_sample()
    assert serial.writes == [b'\x03']
    assert sample.orientation_wxyz == pytest.approx(VALUES[:4])
    assert sample.angular_velocity_xyz == pytest.approx(VALUES[4:7])
    assert sample.linear_acceleration_xyz == pytest.approx(VALUES[7:])
    driver.close()
    assert serial.closed


@pytest.mark.parametrize('values', [
    (math.nan,) + VALUES[1:],
    (0.1, 0.0, 0.0, 0.0) + VALUES[4:],
])
def test_rejects_non_finite_or_invalid_quaternion(values):
    driver = ImuDriver(serial_instance=FakeSerial([struct.pack('<10f', *values)]))
    with pytest.raises(RuntimeError):
        driver.read_sample()


def test_rejects_short_packet():
    driver = ImuDriver(serial_instance=FakeSerial([b'123']))
    with pytest.raises(RuntimeError, match='Pacote incompleto'):
        driver.read_sample()


def test_calibrator_resets_on_motion_and_removes_xyz_bias():
    calibrator = GyroBiasCalibrator(sample_count=3, stationary_max_rad_s=0.05)
    assert not calibrator.add((0.01, -0.02, 0.003))
    assert not calibrator.add((0.1, 0.0, 0.0))
    assert calibrator.collected_samples == 0
    assert not calibrator.add((0.01, -0.02, 0.003))
    assert not calibrator.add((0.02, -0.01, 0.006))
    assert calibrator.add((0.0, -0.03, 0.0))
    assert calibrator.bias == pytest.approx((0.01, -0.02, 0.003))
    assert calibrator.correct((0.015, -0.01, 0.001)) == pytest.approx(
        (0.005, 0.01, -0.002))
