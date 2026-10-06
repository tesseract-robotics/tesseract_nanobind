import os
from pathlib import Path

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import (
    tesseract_common,
    tesseract_environment,
    tesseract_kinematics,
    tesseract_state_solver,
    tesseract_urdf,
)

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


def get_scene_graph():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator = TesseractSupportResourceLocator()
    return tesseract_urdf.parseURDFFile(path, locator)


def get_plugin_factory():
    support_dir = os.environ["TESSERACT_SUPPORT_DIR"]
    kin_config = Path(support_dir) / "urdf" / "lbr_iiwa_14_r820_plugins.yaml"
    locator = TesseractSupportResourceLocator()
    return tesseract_kinematics.KinematicsPluginFactory(kin_config, locator), locator


def run_inv_kin_test(inv_kin, fwd_kin):
    pose = np.eye(4)
    pose[2, 3] = 1.306

    seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
    tip_pose = {}
    tip_pose["tool0"] = tesseract_common.Isometry3d(pose)
    solutions = inv_kin.calcInvKin(tip_pose, seed)
    assert len(solutions) > 0

    result = fwd_kin.calcFwdKin(solutions[0])

    nptest.assert_almost_equal(pose, result["tool0"].matrix, decimal=3)


def test_kdl_kin_chain_lma_inverse_kinematic():
    plugin_factory, p_locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state1 = solver.getState(np.zeros((7,)))
    scene_state2 = solver.getState(np.zeros((7,)))
    inv_kin = plugin_factory.createInvKin(
        "manipulator", "KDLInvKinChainLMA", scene_graph, scene_state1
    )
    fwd_kin = plugin_factory.createFwdKin(
        "manipulator", "KDLFwdKinChain", scene_graph, scene_state2
    )

    assert inv_kin
    assert fwd_kin

    run_inv_kin_test(inv_kin, fwd_kin)

    del inv_kin
    del fwd_kin


def test_jacobian():
    plugin_factory, p_locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state = solver.getState(np.zeros((7,)))
    fwd_kin = plugin_factory.createFwdKin("manipulator", "KDLFwdKinChain", scene_graph, scene_state)
    scene_graph = get_scene_graph()

    jvals = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])

    link_name = "tool0"
    jacobian = fwd_kin.calcJacobian(jvals, link_name)

    assert jacobian.shape == (6, 7)

    del fwd_kin


# gh-184: the in-place frame math on real jacobians.
# [m/rad, rad/rad] float64 roundoff budget for O(1) jacobian entries (~1e3 ulp at 1.0).
JACOBIAN_ATOL = 1e-12
_IIWA_Q = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])


def _iiwa_fwd_kin():
    plugin_factory, locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    scene_state = tesseract_state_solver.KDLStateSolver(scene_graph).getState(np.zeros((7,)))
    fwd_kin = plugin_factory.createFwdKin("manipulator", "KDLFwdKinChain", scene_graph, scene_state)
    return fwd_kin, plugin_factory, locator


def test_jacobian_change_base_in_place():
    fwd_kin, _factory, _locator = _iiwa_fwd_kin()
    J = fwd_kin.calcJacobian(_IIWA_Q, "tool0")
    before = J.copy()
    T = tesseract_common.Isometry3d(
        tesseract_common.AngleAxisd(0.7, np.array([0.0, 0.0, 1.0]))
    ) * tesseract_common.Translation3d(1.0, 2.0, 3.0)
    R = T.rotation

    assert tesseract_common.jacobianChangeBase(J, T) is None
    nptest.assert_allclose(
        J, np.vstack([R @ before[:3], R @ before[3:]]), rtol=0, atol=JACOBIAN_ATOL
    )


def test_jacobian_change_rejects_c_order():
    """A C-order (6, n > 1) array would need a converted copy, so the in-place write would be lost."""
    fwd_kin, _factory, _locator = _iiwa_fwd_kin()
    J = np.ascontiguousarray(fwd_kin.calcJacobian(_IIWA_Q, "tool0"))
    before = J.copy()
    with pytest.raises(TypeError):
        tesseract_common.jacobianChangeBase(J, tesseract_common.Isometry3d.Identity())
    with pytest.raises(TypeError):
        tesseract_common.jacobianChangeRefPoint(J, np.zeros(3))
    nptest.assert_array_equal(J, before)


def test_jacobian_change_ref_point_matches_calc_jacobian_with_point():
    """Oracle: upstream's own calcJacobian(q, link, link_point) (joint_group.cpp, 0.35.0).

    `link_point` is in the link frame; upstream re-bases with the link's world rotation.
    """
    env = tesseract_environment.Environment()
    locator = TesseractSupportResourceLocator()
    support = Path(os.environ["TESSERACT_SUPPORT_DIR"])
    assert env.init(
        support / "urdf" / "abb_irb2400.urdf", support / "urdf" / "abb_irb2400.srdf", locator
    )
    jg = env.getJointGroup("manipulator")
    q = np.array([0.1, 0.2, -0.3, 0.4, -0.5, 0.6])
    link = "tool0"
    p_link = np.array([0.05, -0.02, 0.1])

    expected = jg.calcJacobianWithPoint(q, link, p_link)
    J = jg.calcJacobian(q, link)
    p_base = jg.calcFwdKin(q)[link].rotation @ p_link
    tesseract_common.jacobianChangeRefPoint(J, p_base)

    nptest.assert_allclose(J, expected, rtol=0, atol=JACOBIAN_ATOL)


@pytest.mark.parametrize("form", ["path", "content"])
def test_kinematics_factory_from_path_or_content(form):
    """gh-165: a `pathlib.Path` is the config file; a `str` is YAML content, never a path."""
    config = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf" / "lbr_iiwa_14_r820_plugins.yaml"
    arg = config if form == "path" else config.read_text()
    factory = tesseract_kinematics.KinematicsPluginFactory(arg, TesseractSupportResourceLocator())
    assert factory.getDefaultFwdKinPlugin("manipulator") == "KDLFwdKinChain"
    assert factory.getDefaultInvKinPlugin("manipulator") == "KDLInvKinChainLMA"
