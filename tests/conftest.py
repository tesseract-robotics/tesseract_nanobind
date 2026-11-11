"""Shared test fixtures.

OMPL RNG seeding: example and planning tests run OMPL (RRTConnect et al.),
which seeds its samplers from entropy per process. Pinning the seed per test
makes a run reproducible: the binding and the planner share one
RNGSeedGenerator per process (a shared libompl on Linux and macOS; on Windows
the wheel keeps OMPL's RNG state in the shared `ompl_runtime.dll`, see
docs/developer/windows-wheels.md). It is not what makes the tests pass: a test
has to pass for every seed. Sweeping seeds found two stages that did not
(#103), TOTG time parameterization after TrajOptIfopt and a TrajOpt collision
cost with no margin; both were replaced, and the seeds that exposed them replay
in tests/examples/test_examples.py.
"""

import pytest

from tesseract_robotics.motion_planners_ompl import RNG_setSeed
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning.toolpaths import zigzag_square

# Arbitrary nonzero value; any fixed seed works (zero is rejected by OMPL once
# generation has started). Changing it just selects a different fixed path.
OMPL_RNG_SEED = 25


@pytest.fixture(autouse=True)
def _seed_ompl_rng():
    """Re-seed OMPL's global RNG before every test.

    Per-test rather than per-session so determinism survives pytest-xdist
    work-stealing: a test's samplers draw from a freshly seeded generator
    regardless of which tests ran before it on the same worker. OMPL logs
    a "generation already started" error on re-seeds; harmless (samplers
    are created per solve) and invisible under `pytest -q` unless a test
    fails.
    """
    RNG_setSeed(OMPL_RNG_SEED)


@pytest.fixture
def zigzag_square_poses() -> list[Pose]:
    """The standard zigzag square toolpath (`planning.toolpaths.zigzag_square`, #84)."""
    return zigzag_square()
