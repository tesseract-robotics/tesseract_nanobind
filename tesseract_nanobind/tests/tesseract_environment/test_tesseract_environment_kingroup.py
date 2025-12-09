"""Tests for Environment kinematic group functionality."""
import numpy as np
import pytest

from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs, getRedundantSolutions


def test_get_environment(abb_env):
    """Test ABB environment creation."""
    env, manip_info, joint_names = abb_env
    assert env is not None
    assert len(joint_names) == 6


def test_kinematic_group(abb_env):
    """Test kinematic group forward and inverse kinematics."""
    env, manip_info, joint_names = abb_env

    kin_group = env.getKinematicGroup(manip_info.manipulator)

    joint_vals = np.ones((6,), dtype=np.float64) * 0.1
    pose_map = kin_group.calcFwdKin(joint_vals)
    pose = pose_map[manip_info.tcp_frame]

    ik = KinGroupIKInput()
    ik.pose = pose
    ik.tip_link_name = "tool0"
    ik.working_frame = "base_link"
    iks = KinGroupIKInputs()
    iks.append(ik)

    invkin1 = kin_group.calcInvKin(iks, joint_vals * 0.1)
    invkin = invkin1[0]

    np.testing.assert_allclose(invkin.flatten(), joint_vals)


def test_kinematic_info(abb_env):
    """Test getKinematicsInformation method."""
    env, manip_info, joint_names = abb_env

    if not hasattr(env, 'getKinematicsInformation'):
        pytest.skip("getKinematicsInformation not yet bound")

    kin_info = env.getKinematicsInformation()

    assert list(kin_info.joint_groups) == []
    assert list(kin_info.link_groups) == []


def test_tesseract_redundant_solutions_tesseract_function(abb_env):
    """Test getRedundantSolutions function."""
    env, manip_info, joint_names = abb_env

    kin_group = env.getKinematicGroup(manip_info.manipulator)
    limits = kin_group.getLimits()
    redundancy_indices = list(kin_group.getRedundancyCapableJointIndices())

    sol = np.ones(6) * np.deg2rad(5)
    redun_sol = getRedundantSolutions(sol, limits.joint_limits, redundancy_indices)

    assert len(redun_sol) == 2

    assert np.allclose(
        redun_sol[0].flatten(),
        np.array([0.08726646, 0.08726646, 0.08726646, 0.08726646, 0.08726646, -6.19591884])
    ) or np.allclose(
        redun_sol[0].flatten(),
        np.array([0.08726646, 0.08726646, 0.08726646, 0.08726646, 0.08726646, 6.19591884])
    )
