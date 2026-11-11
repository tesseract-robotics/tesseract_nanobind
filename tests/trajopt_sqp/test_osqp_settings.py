"""The OSQP settings of OSQPEigenSolver and TrajOptIfoptOSQPSolverProfile: one binding, read back.

Both classes bind the same set*/get* pairs (osqp_settings_bindings.h). The SQP solver used to
lack setAdaptiveRhoInterval, and neither had getters.
"""

import pytest

from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.motion_planners_trajopt_ifopt import TrajOptIfoptOSQPSolverProfile

SETTINGS = [
    ("Polish", False),
    ("WarmStart", False),
    ("AdaptiveRho", False),
    ("AdaptiveRhoInterval", 25),
    ("MaxIteration", 123),
    ("AbsoluteTolerance", 2.5e-5),
    ("RelativeTolerance", 3.5e-7),
    ("Verbosity", True),
]


@pytest.mark.parametrize("owner", [tsqp.OSQPEigenSolver, TrajOptIfoptOSQPSolverProfile])
@pytest.mark.parametrize(("name", "value"), SETTINGS, ids=[name for name, _ in SETTINGS])
def test_setting_round_trips(owner, name, value):
    target = owner()
    getattr(target, f"set{name}")(value)
    assert getattr(target, f"get{name}")() == value


def test_both_classes_expose_the_same_settings():
    names = {f"{prefix}{name}" for name, _ in SETTINGS for prefix in ("set", "get")}
    assert names <= set(dir(tsqp.OSQPEigenSolver))
    assert names <= set(dir(TrajOptIfoptOSQPSolverProfile))
