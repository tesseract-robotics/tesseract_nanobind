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

import os
from pathlib import Path

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


TWC_SUPPORT_ENV = "TWC_SUPPORT_PATH"
# Set to 1 to run without the TWC workcell: the tests that need it skip instead of failing.
SKIP_TWC_ENV = "TESSERACT_TESTS_WITHOUT_TWC"
TWC_SUBMODULE = Path(__file__).resolve().parents[1] / "ws/src/tesseract_ros_workcell/twc_support"


@pytest.fixture(scope="session")
def twc_support() -> Path:
    """The `twc_support` directory of the TWC workcell submodule.

    A missing workcell fails the test, naming the fix, so coverage is not lost without notice.
    CI sets `TWC_SUPPORT_PATH`; a path there that does not exist fails too. A developer without
    the submodule opts out explicitly with `TESSERACT_TESTS_WITHOUT_TWC=1`.
    """
    if env_path := os.environ.get(TWC_SUPPORT_ENV):
        path = Path(env_path)
    else:
        path = TWC_SUBMODULE
    if path.is_dir():
        return path
    if os.environ.get(SKIP_TWC_ENV) == "1":
        pytest.skip(f"{SKIP_TWC_ENV}=1 and no twc_support at {path}")
    pytest.fail(
        f"no twc_support at {path}: run 'git submodule update --init ws/src/tesseract_ros_workcell', "
        f"set {TWC_SUPPORT_ENV}, or set {SKIP_TWC_ENV}=1 to run without it",
        pytrace=False,
    )
