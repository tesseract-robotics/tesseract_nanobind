"""Typed physical quantities and the single SI-to-brand conversion chokepoint.

The emitter IR is strictly SI: metres, radians, seconds. A bare `float` must
never carry an implied unit across a brand boundary: each quantity is a
`NewType`, so a millimetre value cannot be passed where metres are expected
without the checker flagging it. Brand backends convert exclusively through the
named functions here. Zero runtime cost; the checker treats `Millimetres` as
distinct from `Metres` while both stay plain floats (so f-strings work).
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
RadPerSec = NewType("RadPerSec", float)
DegPerSec = NewType("DegPerSec", float)

#: Millimetres per metre (SI definition).
MM_PER_M: Final[float] = 1000.0
#: Degrees per half turn; with `pi` gives the exact radian-to-degree factor.
_DEG_PER_HALF_TURN: Final[float] = 180.0


def to_millimetres(v: Metres) -> Millimetres:
    """Metres to millimetres (the unit most controller position fields want)."""
    return Millimetres(v * MM_PER_M)


def to_degrees(v: Radians) -> Degrees:
    """Radians to degrees (joint/orientation fields on most controllers)."""
    return Degrees(v * _DEG_PER_HALF_TURN / pi)


def to_mm_per_s(v: MPerSec) -> MmPerSec:
    """Metres/second to millimetres/second (RAPID `speeddata` linear fields)."""
    return MmPerSec(v * MM_PER_M)


def to_deg_per_s(v: RadPerSec) -> DegPerSec:
    """Radians/second to degrees/second (RAPID `speeddata` reorientation fields)."""
    return DegPerSec(v * _DEG_PER_HALF_TURN / pi)


def to_m_per_s(v: MmPerSec) -> MPerSec:
    """Millimetres/second to metres/second (KRL `$VEL.CP`)."""
    return MPerSec(v / MM_PER_M)
