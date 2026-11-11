"""Pure JBI position-table formatter: no state, no I/O, property-testable.

A `//POS` section in `///POSTYPE PULSE` mode lists each position variable as a
comma-separated tuple of integer encoder pulses, one per axis.
"""

from __future__ import annotations

from collections.abc import Sequence

from ..core.errors import ExternalAxisError
from ..core.units import Radians
from .syntax import BRAND, ROBOT_AXES, JbiPulseCalibration


def pulse_pos(joints_rad: Sequence[float], calibration: JbiPulseCalibration) -> str:
    """Six joint angles (radians) as a PULSE row `p1,...,p6` (integer pulses).

    Raises:
        ExternalAxisError: not exactly six values (external axes are not emitted).
    """
    if len(joints_rad) != ROBOT_AXES:
        raise ExternalAxisError(len(joints_rad), ROBOT_AXES, BRAND)
    pulses = calibration.pulses([Radians(float(j)) for j in joints_rad])
    return ",".join(str(p) for p in pulses)
