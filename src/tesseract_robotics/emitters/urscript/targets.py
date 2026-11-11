"""Pure URScript literal formatters — no state, no I/O, property-testable.

URScript joint targets are radians; Cartesian poses are `p[x, y, z, rx, ry, rz]`
with translation in **metres** (UR's native unit — no scaling) and orientation as
a **rotation vector** (axis-angle, `axis * angle`, radians).

The rotation vector is the matrix logarithm of the rotation: `angle` from the
trace, axis from the off-diagonal antisymmetric part. The angle≈π case is the
numerically delicate one (the antisymmetric part vanishes there); the robust
recipe used here recovers the axis from the symmetric part via the largest
diagonal element — Shepperd's matrix→quaternion branch, which selects the same
`(R + I)` diagonal component a direct `(R + I)/2` extraction would, but stays
exact for all angles. See `rotation_vector` for the per-branch derivation.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from tesseract_robotics.planning.transforms import Pose

from ..core.errors import ExternalAxisError
from ..core.precision import M_DECIMALS, RAD_DECIMALS, fixed
from .syntax import BRAND

#: Robot axes in a URScript joint vector.
_ROBOT_AXES = 6


def rotation_vector(pose: Pose) -> tuple[float, float, float]:
    """Return the URScript rotation vector `(rx, ry, rz)` (axis-angle, radians).

    The unit quaternion comes from Shepperd's largest-diagonal branch, which is
    exact at every angle, including angle = pi where the antisymmetric part of
    the matrix vanishes. Canonicalized to `w >= 0`, its vector part `v` gives
    the axis and `angle = 2 atan2(|v|, w)`; the rotation vector is
    `v / |v| * angle`, and exactly zero for the identity (`|v| = 0`). Near the
    identity `angle / |v|` tends to 2, so no small-angle cut-off is needed.
    """
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    w, vx, vy, vz = _matrix_to_quaternion(r)
    vec = np.array([vx, vy, vz], dtype=np.float64)
    vec_norm = float(np.linalg.norm(vec))
    if vec_norm == 0.0:
        return (0.0, 0.0, 0.0)
    # Canonicalize to w >= 0 so the recovered angle lands in [0, π].
    if w < 0.0:
        vec = -vec
        w = -w
    theta = 2.0 * math.atan2(vec_norm, w)
    axis = vec / vec_norm
    return (float(axis[0] * theta), float(axis[1] * theta), float(axis[2] * theta))


def _matrix_to_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    """Robust rotation-matrix → unit quaternion `(w, x, y, z)` (Shepperd).

    Branches on the largest of `trace` / the three diagonal elements so the
    divisor `s` is never near zero — numerically stable across the whole
    rotation group, the angle≈π case included.
    """
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0  # s = 4w
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0  # s = 4x
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0  # s = 4y
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0  # s = 4z
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return (w, x, y, z)


def joints_literal(joints_rad: Sequence[float]) -> str:
    """Six-axis joint vector `[j1, ..., j6]` (radians).

    Raises:
        ExternalAxisError: not exactly six values (external axes are not emitted).
    """
    if len(joints_rad) != _ROBOT_AXES:
        raise ExternalAxisError(len(joints_rad), _ROBOT_AXES, BRAND)
    return "[" + ", ".join(fixed(float(j), RAD_DECIMALS) for j in joints_rad) + "]"


def pose_literal(pose: Pose) -> str:
    """Cartesian pose `p[x, y, z, rx, ry, rz]` (metres, rotation vector in radians).

    Translation is emitted in metres (UR's native unit, never scaled);
    orientation is the rotation vector from `rotation_vector`.
    """
    xyz = [fixed(float(v), M_DECIMALS) for v in pose.translation]
    rot = [fixed(v, RAD_DECIMALS) for v in rotation_vector(pose)]
    return "p[" + ", ".join(xyz + rot) + "]"
