"""getClosestValidPoint, calcBoundsErrors, calcBoundsViolations (#294).

calcBoundsErrors / calcBoundsViolations fill a C++ out-param sized like the input; in Python
they return it. Hand-computed for bounds [0, 1] and inputs [-1, 0.5, 2]: the error is the signed
distance to the violated bound (negative below, positive above), the violation its magnitude.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

INPUT = np.array([-1.0, 0.5, 2.0])


@pytest.fixture
def bounds() -> list[ti.Bounds]:
    return [ti.Bounds(0.0, 1.0) for _ in INPUT]


def test_closest_valid_point(bounds):
    np.testing.assert_array_equal(ti.getClosestValidPoint(INPUT, bounds), [0.0, 0.5, 1.0])


def test_bounds_errors(bounds):
    np.testing.assert_array_equal(ti.calcBoundsErrors(INPUT, bounds), [-1.0, 0.0, 1.0])


def test_bounds_violations(bounds):
    np.testing.assert_array_equal(ti.calcBoundsViolations(INPUT, bounds), [1.0, 0.0, 1.0])


@pytest.mark.parametrize(
    "helper", [ti.getClosestValidPoint, ti.calcBoundsErrors, ti.calcBoundsViolations]
)
def test_size_mismatch_raises(helper, bounds):
    with pytest.raises(ti.BoundsSizeError):
        helper(INPUT, bounds[:2])
