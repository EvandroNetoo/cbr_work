"""Kinematics for an X-configured four-wheel mecanum base."""

from math import isfinite


WHEEL_NAMES = (
    'dianteiro_esquerdo',
    'dianteiro_direito',
    'traseiro_esquerdo',
    'traseiro_direito',
)


def twist_to_motor_speeds(
    linear_x,
    linear_y,
    angular_z,
    *,
    max_linear_speed,
    max_angular_speed,
    max_motor_speed,
    deadband=0.0,
):
    """Convert a ROS REP-103 body twist to logical motor speeds.

    Positive x is forward, positive y is left and positive z is
    counter-clockwise. The result is normalized before ``max_motor_speed`` is
    applied, so combined commands never exceed the configured speed limit.
    """
    values = (
        linear_x,
        linear_y,
        angular_z,
        max_linear_speed,
        max_angular_speed,
        max_motor_speed,
        deadband,
    )
    if not all(isfinite(float(value)) for value in values):
        raise ValueError('Velocidades e limites devem ser números finitos')
    if max_linear_speed <= 0.0:
        raise ValueError('max_linear_speed deve ser maior que zero')
    if max_angular_speed <= 0.0:
        raise ValueError('max_angular_speed deve ser maior que zero')
    if not 0.0 < max_motor_speed <= 100.0:
        raise ValueError('max_motor_speed deve estar no intervalo (0, 100]')
    if not 0.0 <= deadband < 1.0:
        raise ValueError('deadband deve estar no intervalo [0, 1)')

    x = float(linear_x) / float(max_linear_speed)
    y = float(linear_y) / float(max_linear_speed)
    rotation = float(angular_z) / float(max_angular_speed)

    raw = {
        'dianteiro_esquerdo': x - y - rotation,
        'dianteiro_direito': x + y + rotation,
        'traseiro_esquerdo': x + y - rotation,
        'traseiro_direito': x - y + rotation,
    }
    largest = max(1.0, *(abs(value) for value in raw.values()))

    result = {}
    for name, value in raw.items():
        normalized = value / largest
        if abs(normalized) < deadband:
            normalized = 0.0
        result[name] = int(round(normalized * float(max_motor_speed)))
    return result


def sequence_to_motor_speeds(values, max_motor_speed=100.0):
    """Map [FL, FR, RL, RR] into named, clamped integer speeds."""
    if len(values) != len(WHEEL_NAMES):
        raise ValueError('motor_speeds deve conter exatamente quatro valores')
    if not all(isfinite(float(value)) for value in values):
        raise ValueError('As velocidades devem ser números finitos')
    if (
        not isfinite(float(max_motor_speed))
        or not 0.0 < max_motor_speed <= 100.0
    ):
        raise ValueError('max_motor_speed deve estar no intervalo (0, 100]')
    return {
        name: int(round(max(
            -max_motor_speed,
            min(max_motor_speed, float(value)),
        )))
        for name, value in zip(WHEEL_NAMES, values)
    }
