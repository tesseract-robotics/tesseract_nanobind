"""`pick_env`: what the part does during a pick, which the post-attach environment can't show."""

import importlib

import numpy as np
import pytest

from tesseract_robotics.viewer import Trajectory
from tesseract_robotics.viewer.poses import link_poses

# Part position between start and end of the pick [m]; float64 round-off over the FK chain.
FIXED_PART_ATOL_M = 1e-9
# A carried part moves far more than this between the pick's two ends [m].
CARRIED_PART_MIN_TRAVEL_M = 0.05


def _module(name):
    # Through importlib: the package attribute of this name is the example's main function.
    return importlib.import_module(f"tesseract_robotics.examples.{name}")


@pytest.mark.parametrize(
    ("example", "part"), [("pick_and_place_example", "box"), ("car_seat_example", "seat_1")]
)
def test_the_part_stays_put_in_pick_env_and_travels_in_the_attached_env(example, part):
    results = _module(example).run()
    pick = Trajectory.from_composite(results["pick_result"].raw_results)
    q_start, q_end = pick.sample(0.0), pick.sample(pick.duration_s)

    def part_xyz(env, q):
        return link_poses(env, pick.joint_names, q)[part][0]

    np.testing.assert_allclose(
        part_xyz(results["pick_env"], q_start),
        part_xyz(results["pick_env"], q_end),
        atol=FIXED_PART_ATOL_M,
    )
    attached = results["robot"].env
    travel_m = np.linalg.norm(part_xyz(attached, q_end) - part_xyz(attached, q_start))
    assert travel_m > CARRIED_PART_MIN_TRAVEL_M


def test_twc_support_is_found_in_the_repo_submodule(monkeypatch, tmp_path):
    twc = _module("twc_workcell_positioner_viewer")
    support = tmp_path / "ws/src/tesseract_ros_workcell/twc_support"
    support.mkdir(parents=True)
    source = tmp_path / "src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py"
    monkeypatch.setattr(twc, "__file__", str(source))
    monkeypatch.delenv("TWC_SUPPORT_PATH", raising=False)

    assert twc._find_twc_support() == support
