"""The PIQP solver profiles' YAML constructor and equality, and `checkPIQPSettings` (#373).

The two-argument C++ constructor takes `(YAML::Node, ProfilePluginFactory)`; Python reaches it
as `from_yaml(text)`, like the other solver profiles. `checkPIQPSettings` raises for settings
PIQP's sparse backend rejects (piqp/settings_utils.h).
"""

import pytest

from tesseract_robotics.tesseract_motion_planners import checkPIQPSettings
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptPIQPSolverProfile
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    TrajOptIfoptPIQPSolverProfile,
)
from tesseract_robotics.trajopt_sqp import PIQPKKTSolver, PIQPSettings, PIQPSolver

# iterations: a value off every default, so equality and from_yaml are seen to read it
MAX_ITER = 321

PROFILES = {
    "trajopt": (TrajOptPIQPSolverProfile, "settings"),
    "trajopt_ifopt": (TrajOptIfoptPIQPSolverProfile, "qp_settings"),
}


@pytest.fixture(params=list(PROFILES), ids=list(PROFILES))
def profile(request):
    return PROFILES[request.param]


def test_equal_for_equal_arguments(profile):
    cls, field = profile
    assert cls() == cls()
    assert not (cls() != cls())
    changed = cls()
    getattr(changed, field).max_iter = MAX_ITER
    assert changed != cls()
    assert not (changed == cls())


def test_from_yaml_reads_settings(profile):
    cls, field = profile
    # Both profiles read their PIQP settings from the `settings:` key
    loaded = cls.from_yaml(f"settings:\n  max_iter: {MAX_ITER}\n")
    assert getattr(loaded, field).max_iter == MAX_ITER
    expected = cls()
    getattr(expected, field).max_iter = MAX_ITER
    assert loaded == expected
    assert cls.from_yaml("{}") == cls()


def test_from_yaml_malformed_raises(profile):
    cls, _ = profile
    with pytest.raises(ValueError):
        cls.from_yaml("settings: [unclosed")


def test_from_yaml_rejects_dense_kkt_solver(profile):
    cls, _ = profile
    with pytest.raises(RuntimeError, match="kkt_solver"):
        cls.from_yaml("settings:\n  kkt_solver: dense_cholesky\n")


def test_check_piqp_settings():
    checkPIQPSettings(PIQPSolver.defaultSettings(), "profile")
    # PIQP's own default KKT solver is the dense one, which the sparse backend rejects
    dense = PIQPSettings()
    assert dense.kkt_solver == PIQPKKTSolver.dense_cholesky
    with pytest.raises(RuntimeError, match="my_profile: kkt_solver 'dense_cholesky'"):
        checkPIQPSettings(dense, "my_profile")
    bad = PIQPSolver.defaultSettings()
    bad.eps_abs = -1.0
    with pytest.raises(RuntimeError, match="verify_settings"):
        checkPIQPSettings(bad, "my_profile")
