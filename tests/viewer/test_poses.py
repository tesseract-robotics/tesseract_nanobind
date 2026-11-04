"""link_poses: per-link world poses from tesseract scene state."""

from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer.errors import UnknownJointError
from tesseract_robotics.viewer.poses import isometry_to_pose, link_poses

# Pose agreement between two exact evaluations of the same kinematic chain [m]; float64 round-off.
POSE_ATOL_M = 1e-12

JOINT_NAMES = [f"joint_{i + 1}" for i in range(6)]
Q = np.array([0.3, -0.2, 0.1, 0.4, -0.5, 0.6])


@pytest.fixture(scope="module")
def env():
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


def test_link_poses_match_fwd_kin(env):
    xyz, _wxyz = link_poses(env, JOINT_NAMES, Q)["tool0"]
    expected = env.getKinematicGroup("manipulator").calcFwdKin(Q)["tool0"]
    np.testing.assert_allclose(xyz, expected.translation, atol=POSE_ATOL_M)


def test_link_poses_rotation_matches_fwd_kin(env):
    _xyz, wxyz = link_poses(env, JOINT_NAMES, Q)["tool0"]
    expected = env.getKinematicGroup("manipulator").calcFwdKin(Q)["tool0"]
    w, x, y, z = wxyz
    # Rotation matrix of the unit quaternion (w, x, y, z).
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    np.testing.assert_allclose(rotation, expected.rotation, atol=POSE_ATOL_M)


def test_link_poses_does_not_mutate_env(env):
    before = np.array(env.getCurrentJointValues())
    link_poses(env, JOINT_NAMES, Q)
    np.testing.assert_array_equal(env.getCurrentJointValues(), before)


def test_subset_joint_names(env):
    """Unnamed joints keep the env's current values."""
    poses = link_poses(env, ["joint_1"], [Q[0]])
    full = link_poses(env, JOINT_NAMES, [Q[0], *env.getCurrentJointValues()[1:]])
    np.testing.assert_allclose(poses["tool0"][0], full["tool0"][0], atol=POSE_ATOL_M)


def test_every_link_posed(env):
    assert set(link_poses(env, JOINT_NAMES, Q)) == set(env.getLinkNames())


def test_unknown_joint_raises(env):
    with pytest.raises(UnknownJointError, match="joint_7"):
        link_poses(env, ["joint_7"], [0.0])


def test_wxyz_identity():
    xyz, wxyz = isometry_to_pose(Isometry3d.Identity())
    np.testing.assert_array_equal(xyz, [0.0, 0.0, 0.0])
    np.testing.assert_array_equal(wxyz, [1.0, 0.0, 0.0, 0.0])
