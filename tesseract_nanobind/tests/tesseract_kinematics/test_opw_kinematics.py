"""Tests for OPW kinematics solver."""
import os
from pathlib import Path
import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import tesseract_kinematics
from tesseract_robotics import tesseract_urdf
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_state_solver
from tesseract_robotics.tesseract_common import FilesystemPath

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


def run_inv_kin_test(inv_kin, fwd_kin):
    """Run inverse kinematics test with given solvers."""
    pose = np.eye(4)
    pose[2, 3] = 1.306

    seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398])
    tip_pose = tesseract_common.TransformMap()
    tip_pose["tool0"] = tesseract_common.Isometry3d(pose)
    solutions = inv_kin.calcInvKin(tip_pose, seed)
    assert len(solutions) > 0

    result = fwd_kin.calcFwdKin(solutions[0])
    nptest.assert_almost_equal(pose, result["tool0"].matrix(), decimal=3)


@pytest.mark.skip(reason="OPW plugin loading fails under pytest due to fork/threading issues")
def test_opw_inverse_kinematic(ctx):
    """Test OPW inverse kinematics solver."""
    tesseract_support = Path(os.environ["TESSERACT_SUPPORT_DIR"])

    # Create plugin factory - keep locator alive
    # Must use FilesystemPath for config (str is treated as YAML content by nanobind overload)
    kin_config = FilesystemPath(str(tesseract_support / "urdf/abb_irb2400_plugins.yaml"))
    p_locator = ctx.keep(TesseractSupportResourceLocator())
    plugin_factory = ctx.keep(tesseract_kinematics.KinematicsPluginFactory(kin_config, p_locator))

    # Parse scene graph - keep locator and scene_graph alive
    path = str(tesseract_support / "urdf/abb_irb2400.urdf")
    sg_locator = ctx.keep(TesseractSupportResourceLocator())
    scene_graph = ctx.keep(tesseract_urdf.parseURDFFile(path, sg_locator))

    solver = ctx.keep(tesseract_state_solver.KDLStateSolver(scene_graph))
    scene_state1 = solver.getState(np.zeros((6,)))
    scene_state2 = solver.getState(np.zeros((6,)))

    inv_kin = ctx.keep(plugin_factory.createInvKin("manipulator", "OPWInvKin", scene_graph, scene_state1))
    fwd_kin = ctx.keep(plugin_factory.createFwdKin("manipulator", "KDLFwdKinChain", scene_graph, scene_state2))

    assert inv_kin
    assert fwd_kin

    run_inv_kin_test(inv_kin, fwd_kin)
