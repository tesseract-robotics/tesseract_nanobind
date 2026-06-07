"""Pure URScript literal formatters — no state, no I/O, property-testable.

URScript joint targets are radians; Cartesian poses are ``p[x, y, z, rx, ry, rz]``
with translation in **metres** (UR's native unit — no scaling) and orientation as
a **rotation vector** (axis-angle, ``axis * angle``, radians).

The rotation vector is the matrix logarithm of the rotation: ``angle`` from the
trace, axis from the off-diagonal antisymmetric part. The angle≈π case is the
numerically delicate one (the antisymmetric part vanishes there); the robust
recipe used here recovers the axis from the symmetric part via the largest
diagonal element — Shepperd's matrix→quaternion branch, which selects the same
``(R + I)`` diagonal component a direct ``(R + I)/2`` extraction would, but stays
exact for all angles. See ``rotation_vector`` for the per-branch derivation.
"""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.planning import Pose

#: Joint, pose, and rotation-vector components print with 6 decimals.
_JOINT_DECIMALS = 6
_POSE_DECIMALS = 6
#: Below this rotation angle (radians) the rotation vector is the zero vector.
#: ~1e-7 rad ≈ 6e-6 deg — far tighter than any controller resolution.
_ANGLE_FLOOR = 1e-7
#: |quaternion vector part| below this ⇒ identity rotation (pure numeric guard).
_QUAT_VEC_FLOOR = 1e-12


def _fmt(value: float, decimals: int) -> str:
    """Fixed-decimal format with negative-zero normalized to ``0`` (``round`` to
    the display precision then ``+ 0.0`` collapses ``-0.0`` and tiny negatives
    like ``-1e-16`` to a clean ``0.000000``). Mirrors KRL's ``_fmt``."""
    return f"{round(value, decimals) + 0.0:.{decimals}f}"


def rotation_vector(pose: Pose) -> tuple[float, float, float]:
    """Return the URScript rotation vector ``(rx, ry, rz)`` (axis-angle, radians).

    ``angle = arccos(clamp((trace − 1) / 2))``. For ``angle ≈ 0`` the result is
    ``(0, 0, 0)``. Otherwise the unit axis is recovered through the matrix→
    quaternion conversion (Shepperd's largest-diagonal branch): this is exact at
    every angle, including the ``angle ≈ π`` singularity where the antisymmetric
    ``(R − Rᵀ)`` part vanishes and a direct ``1/(2 sin angle)`` extraction blows
    up. The vector part of the (canonicalized, ``w ≥ 0``) quaternion gives the
    axis; ``angle = 2·atan2(|vec|, w)`` and the result is ``axis · angle``.
    """
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    if angle < _ANGLE_FLOOR:
        return (0.0, 0.0, 0.0)

    w, vx, vy, vz = _matrix_to_quaternion(r)
    vec = np.array([vx, vy, vz], dtype=np.float64)
    vec_norm = float(np.linalg.norm(vec))
    if vec_norm < _QUAT_VEC_FLOOR:
        return (0.0, 0.0, 0.0)
    # Canonicalize to w >= 0 so the recovered angle lands in [0, π].
    if w < 0.0:
        vec = -vec
        w = -w
    theta = 2.0 * math.atan2(vec_norm, w)
    axis = vec / vec_norm
    return (float(axis[0] * theta), float(axis[1] * theta), float(axis[2] * theta))


def _matrix_to_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    """Robust rotation-matrix → unit quaternion ``(w, x, y, z)`` (Shepperd).

    Branches on the largest of ``trace`` / the three diagonal elements so the
    divisor ``s`` is never near zero — numerically stable across the whole
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


def joints_literal(joints_rad: tuple[float, ...]) -> str:
    """Format a 6-axis joint target ``[j1, …, j6]`` (radians, 6 decimals)."""
    if len(joints_rad) < 6:
        raise ValueError(f"URScript joint target needs >=6 joints; got {len(joints_rad)}")
    parts = [_fmt(float(j), _JOINT_DECIMALS) for j in joints_rad[:6]]
    return "[" + ", ".join(parts) + "]"


def pose_literal(pose: Pose) -> str:
    """Format a Cartesian pose ``p[x, y, z, rx, ry, rz]`` (metres + rotvec rad).

    Translation is emitted as-is in metres (UR's native unit — never scaled);
    orientation is the rotation vector from :func:`rotation_vector`.
    """
    t = pose.translation
    rx, ry, rz = rotation_vector(pose)
    n = _POSE_DECIMALS
    return (
        "p["
        f"{_fmt(float(t[0]), n)}, {_fmt(float(t[1]), n)}, {_fmt(float(t[2]), n)}, "
        f"{_fmt(rx, n)}, {_fmt(ry, n)}, {_fmt(rz, n)}"
        "]"
    )
