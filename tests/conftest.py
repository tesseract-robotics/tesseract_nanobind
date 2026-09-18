"""Shared test fixtures.

OMPL RNG seeding: example and planning tests run OMPL (RRTConnect et al.),
which seeds its samplers from entropy per process. Pinning the seed per test
makes a run reproducible on Linux and macOS (shared libompl: one
RNGSeedGenerator per process). It is not what makes the tests pass: on Windows
the conda-forge ompl is a STATIC lib, so the planner DLL has its own
unreachable seed generator, this fixture is inert there, and every run plans
a fresh path. A test therefore has to pass for every seed. Sweeping seeds
found two stages that did not (#103), TOTG time parameterization after
TrajOptIfopt and a TrajOpt collision cost with no margin; both were replaced,
and the seeds that exposed them replay in tests/examples/test_examples.py.
"""

import pytest

from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed

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
