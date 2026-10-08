import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from interfaces.action import PlaceOnPrecisionTable
from manipulation.base_wiggle import BaseWiggleProfile, circular_target, run_base_wiggle
from manipulation.errors import ConfigurationError
from manipulation.node import ManipulationServer
from manipulation.profiles import load_profiles
from so_arm_101_moveit_config.movimento import OperacaoCancelada

PACKAGE = Path(__file__).parents[1]


class TimedCommands:
    def __init__(self):
        self.time = 0.0
        self.commands = []
        self.velocity = (0.0, 0.0, 0.0)
        self.x = self.y = 0.0

    def publish(self, vx, vy, wz):
        self.commands.append((vx, vy, wz))
        self.velocity = (vx, vy, wz)

    def wait(self, dt):
        self.x += self.velocity[0] * dt
        self.y += self.velocity[1] * dt
        self.time += dt

    def run(self, profile, check_active=lambda: None):
        run_base_wiggle(profile, publish=self.publish, clock=lambda: self.time,
                        wait=self.wait, check_active=check_active)


@pytest.mark.parametrize('speed', [0.02, 0.001])
def test_timed_circle_runs_without_odometry_and_preserves_closed_commands_when_capped(speed):
    base = TimedCommands()
    profile = BaseWiggleProfile(enabled=True, radius_m=0.005, period_s=1.5, max_speed_m_s=speed)
    base.run(profile)
    assert base.time == pytest.approx(profile.period_s * profile.cycles + profile.settle_s)
    assert any(math.hypot(vx, vy) > 0 for vx, vy, wz in base.commands)
    assert all(math.hypot(vx, vy) <= speed + 1e-10 and wz == 0 for vx, vy, wz in base.commands)
    assert math.hypot(base.x, base.y) < 0.00001
    assert base.commands[-1] == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('when', [0.3, 4.1])
def test_cancel_during_motion_or_final_pause_stops_base(when):
    base = TimedCommands()
    def check():
        if base.time >= when:
            raise OperacaoCancelada('cancelado')
    with pytest.raises(OperacaoCancelada):
        base.run(BaseWiggleProfile(enabled=True), check)
    assert base.commands[-1] == (0.0, 0.0, 0.0)


def test_publisher_failure_still_attempts_zero_velocity():
    base = TimedCommands()
    def publish(vx, vy, wz):
        base.publish(vx, vy, wz)
        if vx or vy:
            raise RuntimeError('publication failed')
    with pytest.raises(RuntimeError):
        run_base_wiggle(BaseWiggleProfile(), publish=publish, clock=lambda: base.time,
                        wait=base.wait, check_active=lambda: None)
    assert base.commands[-1] == (0.0, 0.0, 0.0)


def test_radius_envelope_starts_and_ends_at_zero():
    profile = BaseWiggleProfile()
    assert circular_target(0, profile) == (0.0, 0.0, 0.0, 0.0)
    assert circular_target(profile.cycles * profile.period_s, profile) == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize('key,value', [('enabled', 'true'), ('cycles', True), ('cycles', 0),
    ('cycles', 1.5), ('period_s', 0), ('radius_m', float('nan')), ('max_speed_m_s', -1),
    ('rate_hz', 0), ('settle_s', -1)])
def test_invalid_parameter_types_and_nonpositive_timings_are_rejected(tmp_path, key, value):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['precision_table']['base_wiggle'][key] = value
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ConfigurationError):
        load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')


def test_removed_feedback_settings_are_ignored_in_old_yaml(tmp_path):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['precision_table']['base_wiggle'].update(
        odom_timeout_s=None, position_tolerance_m=None, return_timeout_s=None)
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))
    profile = load_profiles(path, PACKAGE / 'config/cargo_slots.yaml').placements['precision_table']
    assert not hasattr(profile.base_wiggle, 'odom_timeout_s')


@pytest.mark.parametrize('failure', [False, True])
def test_precision_release_only_opens_after_successful_wiggle(failure):
    from test_semantic_placement import _cartesian_profile, _pose
    server = ManipulationServer.__new__(ManipulationServer)
    profile = replace(_cartesian_profile(), name='precision_table', approach_height_m=0,
                      retreat_height_m=0, base_wiggle=BaseWiggleProfile(enabled=True))
    events = []
    server._motion = SimpleNamespace(executar_objetivo=lambda *_: events.append('release_pose'))
    server._open_for_placement = lambda *_: events.append('open')
    server._return_after_placement = lambda: events.append('observation')

    def wiggle(*_):
        events.append('wiggle')
        if failure:
            raise RuntimeError('base failed')

    server._wiggle_before_precision_release = wiggle
    if failure:
        with pytest.raises(RuntimeError):
            server._release_at_pose(None, PlaceOnPrecisionTable, _pose(), profile, 'PP')
        assert events == ['release_pose', 'wiggle']
    else:
        server._release_at_pose(None, PlaceOnPrecisionTable, _pose(), profile, 'PP')
        assert events == ['release_pose', 'wiggle', 'open', 'observation']



@pytest.mark.parametrize('enabled', [False, True])
def test_only_enabled_precision_release_runs_wiggle(enabled):
    from interfaces.action import StackObject
    from test_semantic_placement import _cartesian_profile, _pose
    server = ManipulationServer.__new__(ManipulationServer)
    profile = replace(_cartesian_profile(), approach_height_m=0, retreat_height_m=0,
                      base_wiggle=BaseWiggleProfile(enabled=enabled))
    events = []
    server._motion = SimpleNamespace(executar_objetivo=lambda *_: None)
    server._wiggle_before_precision_release = lambda *_: events.append('wiggle')
    server._open_for_placement = lambda *_: events.append('open')
    server._return_after_placement = lambda: None
    server._release_at_pose(None, StackObject, _pose(), profile, 'stack')
    assert events == ['open']
    events.clear()
    server._release_at_pose(None, PlaceOnPrecisionTable, _pose(), profile, 'PP')
    assert events == (['wiggle', 'open'] if enabled else ['open'])


@pytest.mark.parametrize('radius,speed', [(0.0, 0.02), (0.005, 0.0)])
def test_zero_radius_or_speed_sends_zero_without_aborting(radius, speed):
    base = TimedCommands()
    base.run(BaseWiggleProfile(radius_m=radius, max_speed_m_s=speed))
    assert all(command == (0.0, 0.0, 0.0) for command in base.commands)
