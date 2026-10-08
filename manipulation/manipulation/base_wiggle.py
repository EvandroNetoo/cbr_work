"""Timed circular base commands while the gripper stays closed.

The commanded radius grows and shrinks smoothly. No odometry feedback or
physical movement/return confirmation is required.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class BaseWiggleProfile:
    enabled: bool = False
    radius_m: float = 0.002
    cycles: int = 2
    period_s: float = 2.0
    max_speed_m_s: float = 0.01
    settle_s: float = 0.3
    rate_hz: float = 30.0
    sequence: tuple[BaseWiggleProfile, ...] = ()


def circular_target(elapsed: float, profile: BaseWiggleProfile):
    duration = profile.cycles * profile.period_s
    if elapsed >= duration:
        return 0.0, 0.0, 0.0, 0.0
    phase = 2.0 * math.pi * elapsed / profile.period_s
    envelope = math.sin(math.pi * elapsed / duration) ** 2
    envelope_rate = math.pi / duration * math.sin(2.0 * math.pi * elapsed / duration)
    omega = 2.0 * math.pi / profile.period_s
    radius = profile.radius_m * envelope
    rate = profile.radius_m * envelope_rate
    c, s = math.cos(phase), math.sin(phase)
    return radius * c, radius * s, rate * c - radius * omega * s, rate * s + radius * omega * c


def run_base_wiggle(
    profile: BaseWiggleProfile, *, publish: Callable[[float, float, float], None],
    clock: Callable[[], float], wait: Callable[[float], None],
    check_active: Callable[[], None],
) -> None:
    """Execute timed circular velocity commands without odometry checks."""
    duration = profile.cycles * profile.period_s
    start = clock()
    # Uniform scaling preserves the closed commanded trajectory when capped.
    peak_speed = 2.0 * math.pi * profile.radius_m / profile.period_s
    scale = min(1.0, profile.max_speed_m_s / peak_speed) if peak_speed else 1.0
    try:
        while True:
            check_active()
            elapsed = clock() - start
            if elapsed >= duration:
                break
            _, _, vx, vy = circular_target(elapsed, profile)
            publish(vx * scale, vy * scale, 0.0)
            wait(min(1.0 / profile.rate_hz, duration - elapsed))
        publish(0.0, 0.0, 0.0)
        finish = clock() + profile.settle_s
        while clock() < finish:
            check_active()
            wait(min(1.0 / profile.rate_hz, finish - clock()))
    finally:
        publish(0.0, 0.0, 0.0)


def run_base_wiggle_sequence(
    profile: BaseWiggleProfile, *, publish: Callable[[float, float, float], None],
    clock: Callable[[], float], wait: Callable[[float], None],
    check_active: Callable[[], None],
    on_stage: Callable[[int, int, BaseWiggleProfile], None] | None = None,
) -> None:
    """Run each configured circle and its pause before allowing gripper release."""
    stages = profile.sequence or (profile,)
    try:
        for index, stage in enumerate(stages, start=1):
            check_active()
            if on_stage is not None:
                on_stage(index, len(stages), stage)
            run_base_wiggle(stage, publish=publish, clock=clock, wait=wait,
                            check_active=check_active)
    finally:
        publish(0.0, 0.0, 0.0)
