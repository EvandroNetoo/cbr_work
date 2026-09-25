"""Small calibrated camera-frame geometry helpers."""

from __future__ import annotations

import math
import numpy as np


def rotation_from_quaternion(quaternion) -> np.ndarray:
    """Return a normalized 3x3 rotation matrix."""
    values = np.array([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
    ], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid camera transform rotation')
    x, y, z, w = values / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
