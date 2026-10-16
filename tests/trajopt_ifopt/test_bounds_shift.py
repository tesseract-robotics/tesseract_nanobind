"""Bounds += / -= shift both limits and reclassify the bounds type (#286).

C++ declares both operators returning void; the binding returns self, so `b += 1` keeps `b`.
"""

from __future__ import annotations

import math

from tesseract_robotics import trajopt_ifopt as ti


def test_iadd_shifts_both_limits():
    b = ti.Bounds(0.0, 1.0)
    original = b
    b += 1
    assert b is original
    assert isinstance(b, ti.Bounds)
    assert (b.getLower(), b.getUpper()) == (1.0, 2.0)
    assert b.getType() == ti.BoundsType.RANGE_BOUND


def test_isub_shifts_both_limits():
    b = ti.Bounds(0.0, 1.0)
    original = b
    b -= 1
    assert b is original
    assert (b.getLower(), b.getUpper()) == (-1.0, 0.0)
    assert b.getType() == ti.BoundsType.RANGE_BOUND


def test_shift_reclassifies_a_one_sided_bound():
    """A lower bound of 0 shifted stays a lower bound; the infinite side stays infinite."""
    b = ti.Bounds(0.0, math.inf)
    b -= 2.5
    assert b.getLower() == -2.5
    assert b.getUpper() == math.inf
    assert b.getType() == ti.BoundsType.LOWER_BOUND


def test_shift_keeps_an_equality():
    b = ti.Bounds(1.0, 1.0)
    b += 0.5
    assert (b.getLower(), b.getUpper()) == (1.5, 1.5)
    assert b.getType() == ti.BoundsType.EQUALITY
