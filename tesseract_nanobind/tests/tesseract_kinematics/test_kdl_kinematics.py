"""Tests for KDL kinematics solver."""
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics import tesseract_kinematics
from tesseract_robotics import tesseract_urdf
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_state_solver
from tesseract_robotics.tesseract_common import _FilesystemPath

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


def run_inv_kin_test(inv_kin, fwd_kin):
    """Run inverse kinematics test with given solvers."""
    pose = np.eye(4)
    pose[2, 3] = 1.306

    seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
    tip_pose = tesseract_common.TransformMap()
    tip_pose["tool0"] = tesseract_common.Isometry3d(pose)
    solutions = inv_kin.calcInvKin(tip_pose, seed)
    assert len(solutions) > 0

    result = fwd_kin.calcFwdKin(solutions[0])
    nptest.assert_almost_equal(pose, result["tool0"].matrix(), decimal=3)


def test_kdl_kin_chain_lma_inverse_kinematic(ctx):
    """Test KDL LMA inverse kinematics solver."""
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]

    # Create plugin factory
    kin_config = _FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820_plugins.yaml"))
    p_locator = ctx.keep(TesseractSupportResourceLocator())
    plugin_factory = tesseract_kinematics.KinematicsPluginFactory(kin_config, p_locator)

    # Parse scene graph
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    sg_locator = ctx.keep(TesseractSupportResourceLocator())
    scene_graph = tesseract_urdf.parseURDFFile(path, sg_locator)

    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state1 = solver.getState(np.zeros((7,)))
    scene_state2 = solver.getState(np.zeros((7,)))

    inv_kin = plugin_factory.createInvKin("manipulator", "KDLInvKinChainLMA", scene_graph, scene_state1)
    fwd_kin = plugin_factory.createFwdKin("manipulator", "KDLFwdKinChain", scene_graph, scene_state2)

    assert inv_kin
    assert fwd_kin

    run_inv_kin_test(inv_kin, fwd_kin)


def test_jacobian(ctx):
    """Test Jacobian computation."""
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]

    # Create plugin factory
    kin_config = _FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820_plugins.yaml"))
    p_locator = ctx.keep(TesseractSupportResourceLocator())
    plugin_factory = tesseract_kinematics.KinematicsPluginFactory(kin_config, p_locator)

    # Parse scene graph
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    sg_locator = ctx.keep(TesseractSupportResourceLocator())
    scene_graph = tesseract_urdf.parseURDFFile(path, sg_locator)

    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state = solver.getState(np.zeros((7,)))

    fwd_kin = plugin_factory.createFwdKin("manipulator", "KDLFwdKinChain", scene_graph, scene_state)

    jvals = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
    jacobian = fwd_kin.calcJacobian(jvals, "tool0")

    assert jacobian.shape == (6, 7)
