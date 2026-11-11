"""Sample-grid utilities of tesseract_kinematics/utils.h (#373), on the ABB IRB2400.

`gatherJointLimits` returns the (N, 2) limits, `buildSampleGrid` one `LinSpaced` vector per
joint with `ceil((hi - lo) / resolution) + 1` samples, and `computeChainReachUpperBound` an
upper bound on the base-to-tip distance over every joint configuration.
"""

import itertools
import math

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_support_path
from tesseract_robotics import kinematics as tk
from tesseract_robotics.environment import Environment

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

# rad / m: a coarse resolution, a third of each joint's range, so the grid stays small
SAMPLES_PER_RANGE = 3


@pytest.fixture(scope="module")
def irb2400():
    env = Environment()
    support = get_tesseract_support_path()
    assert env.init(
        support / "urdf" / "abb_irb2400.urdf",
        support / "urdf" / "abb_irb2400.srdf",
        TesseractSupportResourceLocator(),
    )
    return env, env.getJointGroup("manipulator")


def test_gather_joint_limits(irb2400):
    env, group = irb2400
    limits = tk.gatherJointLimits(env.getSceneGraph(), group.getJointIds())
    assert limits.shape == (group.numJoints(), 2)
    np.testing.assert_array_equal(limits, group.getLimits().joint_limits)


def test_gather_joint_limits_unknown_joint_raises(irb2400):
    env, _ = irb2400
    with pytest.raises(RuntimeError):
        tk.gatherJointLimits(env.getSceneGraph(), ["no_such_joint"])


def test_positioner_sample_resolution_fields():
    res = tk.PositionerSampleResolution()
    assert res.name == ""
    assert res.value == 0.0
    assert res.min is None and res.max is None
    res = tk.PositionerSampleResolution("joint_1", 0.1, -1.0, 1.0)
    assert (res.name, res.value, res.min, res.max) == ("joint_1", 0.1, -1.0, 1.0)
    res.min = None
    assert res.min is None


def test_build_sample_grid(irb2400):
    env, group = irb2400
    limits = tk.gatherJointLimits(env.getSceneGraph(), group.getJointIds())
    resolutions = [
        tk.PositionerSampleResolution(jid.name(), float(hi - lo) / SAMPLES_PER_RANGE)
        for jid, lo, hi in zip(group.getJointIds(), limits[:, 0], limits[:, 1])
    ]
    resolution = np.array([r.value for r in resolutions])
    grid = tk.buildSampleGrid(limits, resolution)
    assert len(grid) == group.numJoints()
    for samples, lo, hi, step in zip(grid, limits[:, 0], limits[:, 1], resolution):
        assert len(samples) == math.ceil((hi - lo) / step) + 1
        assert samples[0] == pytest.approx(lo)
        assert samples[-1] == pytest.approx(hi)
        assert np.all((samples >= lo) & (samples <= hi))


def test_build_sample_grid_rejects_bad_resolution():
    with pytest.raises(RuntimeError):
        tk.buildSampleGrid(np.array([[0.0, 1.0]]), np.array([0.0]))


def test_chain_reach_upper_bound(irb2400):
    env, group = irb2400
    graph = env.getSceneGraph()
    bound = tk.computeChainReachUpperBound(graph, "base_link", "tool0")
    limits = tk.gatherJointLimits(graph, group.getJointIds())
    grid = tk.buildSampleGrid(limits, (limits[:, 1] - limits[:, 0]) / SAMPLES_PER_RANGE)
    reach = max(
        np.linalg.norm(group.calcFwdKin(np.array(q))["tool0"].translation)
        for q in itertools.product(*grid)
    )
    assert bound >= reach
    assert tk.computeChainReachUpperBound(graph, "tool0", "tool0") == 0.0
