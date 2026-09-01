from collections import deque

import pytest

import vl53_distance.sensor_pair as sensor_pair_module
from vl53_distance.sensor_pair import SensorPairConfig, VL53SensorPair


class FakeBus:
    def __init__(self):
        self.events = []
        self.closed = False

    def write_byte(self, address, value):
        self.events.append(('mux', address, value))

    def close(self):
        self.closed = True


class FakeSensor:
    values = {}
    events = []

    def __init__(self, channel, **kwargs):
        self.channel = channel
        self.kwargs = kwargs
        self.values_for_channel = deque(self.values[channel])
        self.events.append(('init', channel))

    def solicita_leitura(self):
        self.events.append(('request', self.channel))

    def leitura_mm(self):
        self.events.append(('read', self.channel))
        return self.values_for_channel.popleft()

    def close(self):
        self.events.append(('close', self.channel))


@pytest.fixture(autouse=True)
def reset_fake_sensor():
    FakeSensor.values = {0: [100, 300, 200], 1: [200, 400, 300]}
    FakeSensor.events = []


def test_pair_triggers_both_sensors_before_reading_and_applies_median_offsets():
    bus = FakeBus()
    pair = VL53SensorPair(
        SensorPairConfig(), bus=bus, sensor_factory=FakeSensor)
    FakeSensor.events.clear()

    pair.read()
    pair.read()
    sample = pair.read()

    assert FakeSensor.events[:4] == [
        ('request', 0), ('request', 1), ('read', 0), ('read', 1)]
    assert sample.raw_right_mm == 200
    assert sample.raw_left_mm == 300
    assert sample.right_mm == 152
    assert sample.left_mm == 194
    assert sample.average_mm == pytest.approx(173.0)


def test_invalid_reading_is_rejected_before_entering_filter():
    FakeSensor.values = {0: [2500], 1: [300]}
    pair = VL53SensorPair(
        SensorPairConfig(), bus=FakeBus(), sensor_factory=FakeSensor)
    with pytest.raises(ValueError, match='fora da faixa'):
        pair.read()


def test_pair_owns_and_closes_bus_created_internally(monkeypatch):
    bus = FakeBus()
    monkeypatch.setattr(sensor_pair_module, 'SMBus', lambda _index: bus)
    pair = VL53SensorPair(SensorPairConfig(), sensor_factory=FakeSensor)
    pair.close()
    assert ('close', 0) in FakeSensor.events
    assert ('close', 1) in FakeSensor.events
    assert ('mux', 0x70, 0x00) in bus.events
    assert bus.closed


def test_sensor_channels_must_be_distinct_and_filter_window_odd():
    with pytest.raises(ValueError, match='canais diferentes'):
        SensorPairConfig(left_channel=0, right_channel=0)
    with pytest.raises(ValueError, match='ímpar'):
        SensorPairConfig(median_window=2)
