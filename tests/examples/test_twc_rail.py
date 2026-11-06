"""The TWC rail cell draws its rail, and the carriage stays on it (#380)."""

import importlib
import os
from pathlib import Path

import numpy as np
import pytest

SUBMODULE = Path(__file__).resolve().parents[2] / "ws/src/tesseract_ros_workcell/twc_support"
# CI sets the variable, so a broken path there fails rather than skips.
pytestmark = pytest.mark.skipif(
    "TWC_SUPPORT_PATH" not in os.environ and not SUBMODULE.exists(),
    reason="needs the TWC workcell: git submodule update --init ws/src/tesseract_ros_workcell",
)

RAIL_JOINT = "rail_joint_1"
# Along the rail's own axis, its local y. A carriage may touch the rail's end.
AXIS = 1
# Position agreement [m]: the meshes store float32 metres, ~1e-6 at 12 m, with headroom.
ON_RAIL_ATOL_M = 1e-4


def _twc():
    # Imported late: the module looks for the workcell when it loads.
    return importlib.import_module("tesseract_robotics.examples.twc_workcell_positioner_viewer")


@pytest.fixture(scope="module")
def env():
    return _twc().run("rail")["env"]


def _axis_range(env, link: str) -> tuple[float, float]:
    """A link's extent along its own y, over the meshes of its visuals."""
    points = [
        np.asarray(mesh.getVertices())[:, AXIS]
        for visual in env.getLink(link).visual
        for mesh in visual.geometry.getMeshes()
    ]
    assert points, f"{link} has no mesh"
    joined = np.concatenate(points)
    return float(joined.min()), float(joined.max())


def _joint_origin_y(env) -> float:
    return float(env.getJoint(RAIL_JOINT).parent_to_joint_origin_transform.translation[AXIS])


def test_the_rail_base_has_a_visual(env):
    assert len(env.getLink("rail_base_link").visual) > 0


def test_the_carriage_starts_on_the_rail(env):
    rail_lo, _ = _axis_range(env, "rail_base_link")
    carriage_lo, _ = _axis_range(env, "rail_link_1")
    assert _joint_origin_y(env) + carriage_lo >= rail_lo - ON_RAIL_ATOL_M


def test_the_animated_range_stays_on_the_rail(env):
    names, rows = _twc().animation_rows("rail")
    positions = rows[:, names.index(RAIL_JOINT)]
    _, rail_hi = _axis_range(env, "rail_base_link")
    _, carriage_hi = _axis_range(env, "rail_link_1")
    assert _joint_origin_y(env) + positions.max() + carriage_hi <= rail_hi + ON_RAIL_ATOL_M


def test_the_joint_limit_stays_on_the_rail(env):
    upper = env.getJoint(RAIL_JOINT).limits.upper
    _, rail_hi = _axis_range(env, "rail_base_link")
    _, carriage_hi = _axis_range(env, "rail_link_1")
    assert _joint_origin_y(env) + upper + carriage_hi <= rail_hi + ON_RAIL_ATOL_M


def test_the_positioner_cell_has_no_rail_to_restore():
    assert "rail_base_link" not in [
        str(link) for link in _twc().run("positioner")["env"].getLinkIds()
    ]
