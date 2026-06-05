"""Shared test fixtures.

OMPL RNG seeding: example and planning tests run OMPL (RRTConnect et al.),
which seeds its samplers from entropy per process. Unseeded, every CI run
plans a different path, and downstream TOTG time parameterization has a hard
failure region in trajectory-shape space ("Negative path velocity",
MoveIt-inherited, moveit#1665) — measured ~1-3% stochastic failure rate per
run. Pinning the seed per test makes the whole pipeline deterministic.
See https://github.com/tesseract-robotics/tesseract_nanobind/issues/103.
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
