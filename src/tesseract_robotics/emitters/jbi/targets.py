"""Pure JBI position-table formatters — no state, no I/O, property-testable.

A Motoman ``.jbi`` ``//POS`` section in ``///POSTYPE PULSE`` mode lists each
position variable as a comma-separated tuple. v1 emits joint angles in
**degrees** straight into the ``C#####=`` table: the project accepts joint-space
targets directly and true encoder-pulse conversion needs per-robot gear ratios
we do not have, so degrees are written and this limitation is documented here
and in :mod:`tesseract_robotics.emitters.jbi.backend`.
"""

from __future__ import annotations

import numpy as np

from ..core.units import Radians, to_degrees

#: Joint angles in the PULSE position table are printed with 3 decimals.
_PULSE_DECIMALS = 3
#: Minimum robot axes a Motoman position tuple carries.
_MIN_AXES = 6


def _fmt(value: float, decimals: int) -> str:
    """Fixed-decimal format with negative-zero normalized to ``0``.

    ``round`` to the display precision then ``+ 0.0`` collapses ``-0.0`` and
    tiny negatives like ``-1e-16`` to a clean ``0.000`` (never emit ``-0.000``).
    """
    return f"{round(value, decimals) + 0.0:.{decimals}f}"


def pulse_pos(joints_rad: tuple[float, ...]) -> str:
    """Format a 6-axis joint target as a PULSE position tuple ``j1,…,j6``.

    Args:
        joints_rad: Joint angles in radians; the first 6 are the robot axes.

    Returns:
        The comma-separated joint angles in degrees, 3 decimals (the value that
        follows ``C#####=`` in the ``//POS`` table).

    Raises:
        ValueError: fewer than 6 joint values were supplied.
    """
    if len(joints_rad) < _MIN_AXES:
        raise ValueError(f"JBI PULSE position needs >={_MIN_AXES} joints; got {len(joints_rad)}")
    degrees = (to_degrees(Radians(float(j))) for j in joints_rad[:_MIN_AXES])
    return ",".join(_fmt(d, _PULSE_DECIMALS) for d in degrees)


def _to_tuple(values) -> tuple[float, ...]:
    """Ravel an array-like of joint values to a float tuple (radians)."""
    return tuple(float(v) for v in np.asarray(values, dtype=np.float64).ravel())
