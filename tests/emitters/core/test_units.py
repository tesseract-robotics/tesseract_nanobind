"""Unit-conversion chokepoint — the only place SI crosses into brand units."""

from __future__ import annotations

import math

from tesseract_robotics.emitters.core.units import (
    MM_PER_M,
    Metres,
    MmPerSec,
    Radians,
    to_degrees,
    to_m_per_s,
    to_millimetres,
)


def test_metres_to_millimetres_exact() -> None:
    assert to_millimetres(Metres(1.0)) == 1000.0
    assert to_millimetres(Metres(0.6)) == 600.0


def test_radians_to_degrees_exact() -> None:
    assert to_degrees(Radians(math.pi)) == 180.0
    assert to_degrees(Radians(0.0)) == 0.0


def test_mm_per_s_to_m_per_s_matches_krl_vel_cp() -> None:
    assert to_m_per_s(MmPerSec(100.0)) == 0.1


def test_mm_per_m_constant() -> None:
    assert MM_PER_M == 1000.0
