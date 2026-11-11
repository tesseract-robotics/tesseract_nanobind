"""Pure KRL literal formatters: no state, no I/O, property-testable.

KUKA ABC is intrinsic ZYX Tait-Bryan, `R = Rz(A) Ry(B) Rx(C)`, computed directly
from the rotation matrix with the canonical atan2 extraction (not Eigen
`eulerAngles`, whose range differs).

Gimbal lock (B = ±90 deg): only `A ∓ C` is determined. Multiplying out
`Rz(A) Ry(±90) Rx(C)` gives `r01 = sin(C - A)`, `r11 = cos(C - A)` at B = +90 and
`r01 = -sin(A + C)`, `r11 = cos(A + C)` at B = -90. Fixing `C = 0`, both reduce
to `A = atan2(-r01, r11)`: one formula, exact on both branches.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Final

import numpy as np

from tesseract_robotics.planning.transforms import Pose

from ..core.errors import ExternalAxisError
from ..core.precision import DEG_DECIMALS, MM_DECIMALS, fixed
from ..core.units import Metres, Radians, to_degrees, to_millimetres
from .syntax import BRAND

#: Robot axes in a KRL axis target (A1..A6).
_ROBOT_AXES: Final[int] = 6
#: `cos(B)` floor below which the ZYX extraction switches to the gimbal branch.
#: `cos(B)` is computed as `hypot(r00, r10)`; below sqrt(machine epsilon) ≈ 1.5e-8
#: the two atan2 arguments of the regular branch carry fewer than half their
#: significant digits, so the regular branch's A and C are no longer meaningful.
_GIMBAL_FLOOR: Final[float] = float(np.sqrt(np.finfo(np.float64).eps))
#: Degree distance to -180 within which -180 is printed as +180 (same angle).
_PI_DEG_SNAP: Final[float] = 0.5 * 10.0**-DEG_DECIMALS


def abc_from_pose(pose: Pose) -> tuple[float, float, float]:
    """Return KUKA `(A, B, C)` in degrees for `pose` (intrinsic ZYX)."""
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    cos_b = math.hypot(r[0, 0], r[1, 0])
    b = math.atan2(-r[2, 0], cos_b)
    if cos_b > _GIMBAL_FLOOR:
        a = math.atan2(r[1, 0], r[0, 0])
        c = math.atan2(r[2, 1], r[2, 2])
    else:
        a = math.atan2(-r[0, 1], r[1, 1])
        c = 0.0
    return (
        _snap_pi(to_degrees(Radians(a))),
        _snap_pi(to_degrees(Radians(b))),
        _snap_pi(to_degrees(Radians(c))),
    )


def _snap_pi(deg: float) -> float:
    """Print -180 as +180 (the same angle; atan2 returns either sign at ±pi)."""
    return 180.0 if abs(deg + 180.0) < _PI_DEG_SNAP else deg


def krl_axis_literal(joints_rad: Sequence[float]) -> str:
    """Six-axis target `{A1 ...,A6 ...}` in degrees.

    Raises:
        ExternalAxisError: not exactly six values (external axes are not emitted).
    """
    if len(joints_rad) != _ROBOT_AXES:
        raise ExternalAxisError(len(joints_rad), _ROBOT_AXES, BRAND)
    parts = [
        f"A{i + 1} {fixed(to_degrees(Radians(j)), DEG_DECIMALS)}" for i, j in enumerate(joints_rad)
    ]
    return "{" + ",".join(parts) + "}"


def krl_frame_literal(pose: Pose) -> str:
    """Cartesian frame `{X ...,Y ...,Z ...,A ...,B ...,C ...}` (mm, deg)."""
    t = pose.translation
    x, y, z = (fixed(to_millimetres(Metres(float(v))), MM_DECIMALS) for v in t)
    a, b, c = (fixed(v, DEG_DECIMALS) for v in abc_from_pose(pose))
    return "{" + f"X {x},Y {y},Z {z},A {a},B {b},C {c}" + "}"
