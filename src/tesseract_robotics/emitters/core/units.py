"""Typed physical quantities and the single SI→brand conversion chokepoint.

The emitter IR is strictly SI: metres, radians, seconds. A bare ``float`` must
never carry an implied unit across a brand boundary — each quantity is a
``NewType`` so a millimetre value cannot be silently passed where metres are
expected. Brand backends are the only conversion consumers, exclusively through
the named functions here. Zero runtime cost; the checker treats ``Millimetres``
as distinct from ``Metres`` while both stay plain floats (so f-strings work).
"""

from __future__ import annotations

from math import pi
from typing import Final, NewType

Metres = NewType("Metres", float)
Millimetres = NewType("Millimetres", float)
Radians = NewType("Radians", float)
Degrees = NewType("Degrees", float)
Seconds = NewType("Seconds", float)
MmPerSec = NewType("MmPerSec", float)
MPerSec = NewType("MPerSec", float)

#: Millimetres per metre. The one magic number, named once.
MM_PER_M: Final[float] = 1000.0
_DEG_PER_RAD: Final[float] = 180.0


def to_millimetres(v: Metres) -> Millimetres:
    """Metres → millimetres (the unit most controller position fields want)."""
    return Millimetres(v * MM_PER_M)


def to_degrees(v: Radians) -> Degrees:
    """Radians → degrees (joint/orientation fields on most controllers)."""
    return Degrees(v * _DEG_PER_RAD / pi)


def to_m_per_s(v: MmPerSec) -> MPerSec:
    """Millimetres/second → metres/second (e.g. KRL ``$VEL.CP``)."""
    return MPerSec(v / MM_PER_M)
