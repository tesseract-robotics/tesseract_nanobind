"""toTrajArray / toJointTrajectory turn solved variables into a trajectory (#296)."""

from __future__ import annotations

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import JointTrajectory

JOINT_NAMES = ["j0", "j1", "j2"]
STATES = [np.array([0.0, 0.1, 0.2]), np.array([0.3, 0.4, 0.5]), np.array([0.6, 0.7, 0.8])]


def _vars():
    nodes = ti.createNodesVariables(
        "trajectory", JOINT_NAMES, STATES, ti.toBounds(-np.ones(3), np.ones(3))
    )
    return [node.getVar("joints") for node in nodes.getNodes()]


def test_traj_array_stacks_the_var_values():
    vars_ = _vars()
    array = ti.toTrajArray(vars_)
    np.testing.assert_array_equal(array, np.vstack([v.value() for v in vars_]))
    assert array.shape == (len(STATES), len(JOINT_NAMES))


def test_joint_trajectory_has_one_state_per_node():
    trajectory = ti.toJointTrajectory(_vars())
    assert isinstance(trajectory, JointTrajectory)
    assert len(trajectory) == len(STATES)
    for state, expected in zip(trajectory, STATES):
        assert list(state.joint_names) == JOINT_NAMES
        np.testing.assert_array_equal(state.position, expected)
