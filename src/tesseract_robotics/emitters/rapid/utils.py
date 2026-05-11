"""RAPID code generation low-level helpers — RAPID-native primitive formatters,
the `RapidType` storage-class enum, and the `MOTION_TYPES` axis-monitoring enum.

The RAPID-domain bundles (`RapidProfile`, `RapidTarget`) and the DSL classes
live in `rapid_writer.py`. This module is pure helpers — no tesseract imports
beyond the formatter conventions documented below.

RAPID conventions used throughout:
    Position:    millimetres, 4-decimal precision (`:.4f`).
    Quaternion:  scalar-first `[w, x, y, z]`, 8-decimal precision (`:.8f`).
    Joint angle: degrees, 2-decimal precision (`:.2f`) inside a jointtarget.

Tesseract conventions assumed (per project CLAUDE.md):
    Pose.position is in metres.
    Pose.quaternion is scalar-last `[qx, qy, qz, qw]`.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from enum import Enum
from math import pi
from typing import Any

logger = logging.getLogger(__name__)


class RapidType(str, Enum):
    """RAPID variable-declaration storage classes.

    `CONST` — value fixed at declaration; cannot reassign at runtime.
    `VAR`   — runtime-mutable; reset to declaration value at each new program run.
    `PERS`  — persistent across program runs and controller power cycles
              (the typical choice for `tooldata` / `wobjdata` / `speeddata`).
    """

    CONST = "CONST"
    VAR = "VAR"
    PERS = "PERS"

    def __str__(self) -> str:
        # Override so f"{RapidType.CONST}" emits "CONST" in Python 3.9–3.12.
        # Python 3.11+ changed (str, Enum) format behavior; explicit __str__
        # gives consistent rendering across versions.
        return self.value


def get_quat(q: Sequence[float] | Any) -> Sequence[float]:
    """Coerce a quaternion-like value into a sequence ordered `[w, x, y, z]`.

    Accepts list/tuple of 4 floats (assumed already in W,X,Y,Z order) or any
    object exposing a `wxyz` attribute.
    """
    if isinstance(q, (list, tuple)):
        return list(q)
    if hasattr(q, "wxyz"):
        return q.wxyz
    raise TypeError(f"cannot convert {type(q).__name__} to a RAPID quaternion list")


def get_rapid_bool(val: Any) -> str:
    """Python truthy → RAPID bool literal (`TRUE` / `FALSE`)."""
    return "TRUE" if val else "FALSE"


def radians_to_quadrant(angle: float) -> int:
    """Convert a joint angle in radians to a RAPID configuration quadrant index.

    RAPID's robtarget config field stores joint quadrants per axis: angle in
    [0, π/2) → 0, [π/2, π) → 1, etc. Negative angles use the inverse
    convention (-1, -2, ...).
    """
    if angle >= 0.0:
        return int((angle * 2.0) / pi)
    return int(((angle * 2.0) / pi) - 1.0)


class MOTION_TYPES:
    """Integer enum-style constants accepted by `Conf` to select Cartesian vs joint
    configuration monitoring."""

    linear: int = 1
    joint: int = 2
    circle: int = 3
    absolute_joints: int = 4


def format_ext_axis(external_axis: Sequence[float] | None = None) -> str:
    """Format the external-axis field of a RAPID robtarget/jointtarget.

    RAPID expects exactly six external-axis slots; unused slots get `9E+09`
    (ABB's sentinel for "no value").
    """
    if not external_axis:
        return "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]"
    values = [str(v) for v in external_axis]
    pad = 6 - len(values)
    values.extend(["9E+09"] * pad)
    return "[" + ",".join(values) + "]"


# `linear_motion` / `circular_motion` / `joint_motion` removed — replaced by
# `_move_line` / `_movec_line` private helpers in `rapid_writer.py` that take
# a `RapidProfile` instead of four separate strings.
