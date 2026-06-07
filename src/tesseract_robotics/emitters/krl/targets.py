"""Pure KRL literal formatters — no state, no I/O, property-testable.

KUKA ABC = intrinsic ZYX Tait–Bryan (``R = Rz(A)·Ry(B)·Rx(C)``), computed
directly from the rotation matrix with the canonical atan2 extraction (not
Eigen ``eulerAngles``, whose range differs) and a gimbal-lock branch at |B|≈90°.
"""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.planning import Pose

from ..core.units import Metres, Radians, to_degrees, to_millimetres

_AXIS_DECIMALS = 5
_FRAME_DECIMALS = 3
#: |cos(B)| floor for the ZYX gimbal singularity (|B| ≈ 90°). Pure numeric guard.
_GIMBAL_FLOOR = 1e-9
#: deg distance to ±180 within which we normalize -180 → +180 for stable goldens.
_PI_DEG_SNAP = 1e-6


def abc_from_pose(pose: Pose) -> tuple[float, float, float]:
    """Return KUKA ``(A, B, C)`` in degrees for ``pose`` (intrinsic ZYX)."""
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    sy = math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
    if sy > _GIMBAL_FLOOR:
        a = math.atan2(r[1, 0], r[0, 0])
        b = math.atan2(-r[2, 0], sy)
        c = math.atan2(r[2, 1], r[2, 2])
    else:  # gimbal lock: B = ±90°, fold into A, set C = 0
        a = math.atan2(-r[1, 2], r[1, 1])
        b = math.atan2(-r[2, 0], sy)
        c = 0.0
    return (
        _snap_pi(to_degrees(Radians(a))),
        _snap_pi(to_degrees(Radians(b))),
        _snap_pi(to_degrees(Radians(c))),
    )


def _snap_pi(deg: float) -> float:
    """Normalize -180.0 → +180.0 (atan2 sign at ±π) for deterministic output."""
    return 180.0 if abs(deg + 180.0) < _PI_DEG_SNAP else deg


def _fmt(value: float, decimals: int) -> str:
    """Fixed-decimal format with negative-zero normalized to ``0`` (``round``
    to the display precision then ``+ 0.0`` collapses ``-0.0`` and tiny
    negatives like ``-1e-16`` to a clean ``0.000``)."""
    return f"{round(value, decimals) + 0.0:.{decimals}f}"


def krl_axis_literal(joints_rad: tuple[float, ...]) -> str:
    """Format a 6-axis joint target ``{A1 …,A6 …}`` (degrees, 5 decimals)."""
    if len(joints_rad) < 6:
        raise ValueError(f"KRL axis target needs >=6 joints; got {len(joints_rad)}")
    parts = [
        f"A{i + 1} {_fmt(to_degrees(Radians(j)), _AXIS_DECIMALS)}"
        for i, j in enumerate(joints_rad[:6])
    ]
    return "{" + ",".join(parts) + "}"


def krl_frame_literal(pose: Pose) -> str:
    """Format a Cartesian frame ``{X …,A …,C …}`` (mm + deg, 3 decimals)."""
    t = pose.translation
    x = to_millimetres(Metres(float(t[0])))
    y = to_millimetres(Metres(float(t[1])))
    z = to_millimetres(Metres(float(t[2])))
    a, b, c = abc_from_pose(pose)
    n = _FRAME_DECIMALS
    return (
        "{"
        f"X {_fmt(x, n)},Y {_fmt(y, n)},Z {_fmt(z, n)},"
        f"A {_fmt(a, n)},B {_fmt(b, n)},C {_fmt(c, n)}"
        "}"
    )
