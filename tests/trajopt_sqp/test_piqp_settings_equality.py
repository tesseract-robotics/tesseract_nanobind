"""PIQPSettings compares by value with the upstream operator== and is unhashable (#393).

`tesseract::motion_planners::operator==` (piqp/settings_utils.h) compares the floating-point
fields relatively at float epsilon (no absolute floor) and the rest exactly.
"""

from __future__ import annotations

import subprocess
import sys

import numpy as np
import pytest

from tesseract_robotics import trajopt_sqp as tsqp

# Upstream's relative tolerance: float epsilon (settings_utils.h).
FLOAT_EPS = float(np.finfo(np.float32).eps)
# Relative perturbations well inside and well outside that tolerance.
WITHIN_TOLERANCE = FLOAT_EPS / 10
OUTSIDE_TOLERANCE = FLOAT_EPS * 10


def test_equal_defaults():
    a, b = tsqp.PIQPSettings(), tsqp.PIQPSettings()
    assert a == b
    assert not (a != b)
    assert tsqp.PIQPSolver.defaultSettings() == tsqp.PIQPSolver.defaultSettings()


def test_unequal_exact_field():
    a, b = tsqp.PIQPSettings(), tsqp.PIQPSettings()
    b.max_iter = a.max_iter + 1
    assert a != b
    assert not (a == b)


def test_unequal_enum_field():
    a, b = tsqp.PIQPSettings(), tsqp.PIQPSettings()
    a.kkt_solver = tsqp.PIQPKKTSolver.sparse_ldlt
    b.kkt_solver = tsqp.PIQPKKTSolver.sparse_ldlt_cond
    assert a != b


def test_tolerance_field_within_relative_tolerance():
    a, b = tsqp.PIQPSettings(), tsqp.PIQPSettings()
    b.eps_abs = a.eps_abs * (1 + WITHIN_TOLERANCE)
    assert b.eps_abs != a.eps_abs
    assert a == b


def test_tolerance_field_outside_relative_tolerance():
    a, b = tsqp.PIQPSettings(), tsqp.PIQPSettings()
    b.eps_abs = a.eps_abs * (1 + OUTSIDE_TOLERANCE)
    assert a != b


def test_other_type_is_not_equal():
    assert tsqp.PIQPSettings() != object()


def test_unhashable():
    assert tsqp.PIQPSettings.__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(tsqp.PIQPSettings())


def test_equality_needs_no_planner_import():
    """The equality is trajopt_sqp's own, not attached by a module imported later."""
    code = (
        "import sys\n"
        "from tesseract_robotics import trajopt_sqp as t\n"
        "assert 'tesseract_robotics.tesseract_motion_planners' not in sys.modules\n"
        "assert t.PIQPSettings() == t.PIQPSettings()\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
