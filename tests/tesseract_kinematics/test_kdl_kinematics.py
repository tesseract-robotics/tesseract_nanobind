import os
import subprocess
import sys
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


# gh-212: every JointGroup.calcJacobian overload under its native name, calcInvKin(list),
# and the KinematicGroup constructor.
_IIWA_JOINTS = [f"joint_a{i}" for i in range(1, 8)]
# A group of the last five joints: link_2 is then a static link with a non-identity pose.
_PARTIAL_JOINTS = _IIWA_JOINTS[2:]
_IIWA_STATE_Q = np.array([0.3, -0.4, 0.5, -0.6, 0.7, -0.8, 0.9])
_LINK_POINT = np.array([0.05, -0.02, 0.1])


def _joint_group(joint_names):
    scene_graph = get_scene_graph()
    scene_state = tesseract_state_solver.KDLStateSolver(scene_graph).getState(_IIWA_STATE_Q)
    jg = tesseract_kinematics.JointGroup("group", joint_names, scene_graph, scene_state)
    return jg, scene_state


def test_calc_jacobian_link_point_overload_is_the_alias():
    jg, _ = _joint_group(_IIWA_JOINTS)
    nptest.assert_array_equal(
        jg.calcJacobian(_IIWA_Q, "tool0", _LINK_POINT),
        jg.calcJacobianWithPoint(_IIWA_Q, "tool0", _LINK_POINT),
    )


def test_calc_jacobian_in_group_base_is_the_plain_jacobian():
    """joint_group.cpp:184-185 @ 0.35.0: base_link_name == group base is calcJacobian(q, link)."""
    jg, _ = _joint_group(_IIWA_JOINTS)
    nptest.assert_array_equal(
        jg.calcJacobian(_IIWA_Q, jg.getBaseLinkName(), "tool0"), jg.calcJacobian(_IIWA_Q, "tool0")
    )


def test_calc_jacobian_static_base_matches_jacobian_change_base():
    """Oracle: joint_group.cpp:211 @ 0.35.0, re-based on the static link's scene-state pose."""
    jg, scene_state = _joint_group(_PARTIAL_JOINTS)
    base = "link_2"
    assert base in jg.getStaticLinkNames()
    q = _IIWA_Q[2:]
    expected = jg.calcJacobian(q, "tool0")
    tesseract_common.jacobianChangeBase(expected, scene_state.link_transforms[base].inverse())
    nptest.assert_allclose(jg.calcJacobian(q, base, "tool0"), expected, rtol=0, atol=JACOBIAN_ATOL)


def test_calc_jacobian_static_base_with_link_point():
    """Oracle: joint_group.cpp:254-256 @ 0.35.0 (link_point re-expressed in the base frame)."""
    jg, scene_state = _joint_group(_PARTIAL_JOINTS)
    base = "link_2"
    q = _IIWA_Q[2:]
    expected = jg.calcJacobian(q, base, "tool0")
    base_to_link = scene_state.link_transforms[base].inverse() * jg.calcFwdKin(q)["tool0"]
    tesseract_common.jacobianChangeRefPoint(expected, base_to_link.linear @ _LINK_POINT)
    nptest.assert_allclose(
        jg.calcJacobian(q, base, "tool0", _LINK_POINT), expected, rtol=0, atol=JACOBIAN_ATOL
    )


def test_calc_jacobian_active_link_relative_to_itself_is_zero():
    """joint_group.cpp:194-208 @ 0.35.0: an active base subtracts its own Jacobian."""
    jg, _ = _joint_group(_IIWA_JOINTS)
    assert jg.isActiveLinkName("link_4")
    nptest.assert_allclose(
        jg.calcJacobian(_IIWA_Q, "link_4", "link_4"), np.zeros((6, 7)), rtol=0, atol=JACOBIAN_ATOL
    )


@pytest.mark.parametrize(
    "call",
    [
        lambda jg, q: jg.calcJacobian(q, "no_such_link"),
        lambda jg, q: jg.calcJacobian(q, "no_such_link", _LINK_POINT),
        lambda jg, q: jg.calcJacobianWithPoint(q, "no_such_link", _LINK_POINT),
        lambda jg, q: jg.calcJacobian(q, "no_such_link", "tool0"),
        lambda jg, q: jg.calcJacobian(q, "base", "no_such_link"),
        lambda jg, q: jg.calcJacobian(q, "no_such_link", "tool0", _LINK_POINT),
        lambda jg, q: jg.calcJacobian(q, "base", "no_such_link", _LINK_POINT),
    ],
    ids=[
        "link",
        "link_point",
        "with_point_alias",
        "base_unknown",
        "base_link_unknown",
        "base_unknown_point",
        "base_link_unknown_point",
    ],
)
def test_calc_jacobian_unknown_link_raises_key_error(call):
    jg, _ = _joint_group(_IIWA_JOINTS)
    with pytest.raises(KeyError, match="no link 'no_such_link' in joint group 'group'"):
        call(jg, _IIWA_Q)


def _iiwa_env():
    support = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf"
    env = tesseract_environment.Environment()
    assert env.init(
        support / "lbr_iiwa_14_r820.urdf",
        support / "lbr_iiwa_14_r820.srdf",
        TesseractSupportResourceLocator(),
    )
    return env


def _ik_input(kin_group):
    pose = kin_group.calcFwdKin(_IIWA_Q)["tool0"]
    return tesseract_kinematics.KinGroupIKInput(pose, kin_group.getBaseLinkName(), "tool0")


def _assert_same_solutions(actual, expected):
    assert len(expected) > 0
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        nptest.assert_array_equal(a, e)


def test_calc_inv_kin_accepts_a_list():
    kg = _iiwa_env().getKinematicGroup("manipulator")
    ik_input = _ik_input(kg)
    seed = np.zeros(7)
    opaque = tesseract_kinematics.KinGroupIKInputs()
    opaque.append(ik_input)

    expected = kg.calcInvKin(opaque, seed)
    _assert_same_solutions(kg.calcInvKin([ik_input], seed), expected)
    _assert_same_solutions(kg.calcInvKinMultiple([ik_input], seed), expected)


def _kinematic_group_from_factory(joint_names=_IIWA_JOINTS):
    factory, _locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    scene_state = tesseract_state_solver.KDLStateSolver(scene_graph).getState(np.zeros(7))
    inv_kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", scene_graph, scene_state)
    kg = tesseract_kinematics.KinematicGroup(
        "manipulator", joint_names, inv_kin, scene_graph, scene_state
    )
    return kg, inv_kin


def test_kinematic_group_constructor_solves_like_the_environment_group():
    kg, inv_kin = _kinematic_group_from_factory()
    env_kg = _iiwa_env().getKinematicGroup("manipulator")
    ik_input = _ik_input(env_kg)
    seed = np.zeros(7)
    _assert_same_solutions(kg.calcInvKin(ik_input, seed), env_kg.calcInvKin(ik_input, seed))
    # The constructor took a clone: the caller's solver still works.
    assert len(inv_kin.calcInvKin({"tool0": ik_input.pose}, seed)) > 0


def test_kinematic_group_constructor_wrong_joint_names_raises():
    """kinematic_group.cpp:115-116 @ 0.35.0: joint_names of the wrong size."""
    with pytest.raises(RuntimeError, match="joint_names is not the correct size"):
        _kinematic_group_from_factory(_IIWA_JOINTS[:6])


_GROUP_OUTLIVES_SOLVER_SCRIPT = """\
import gc
import os
from pathlib import Path

import numpy as np

import tesseract_robotics  # noqa: F401 - sets TESSERACT_SUPPORT_DIR
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_kinematics import (
    KinematicGroup, KinematicsPluginFactory, KinGroupIKInput,
)
from tesseract_robotics.tesseract_state_solver import KDLStateSolver
from tesseract_robotics.tesseract_urdf import parseURDFFile

support = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf"
locator = GeneralResourceLocator()
scene_graph = parseURDFFile(str(support / "lbr_iiwa_14_r820.urdf"), locator)
scene_state = KDLStateSolver(scene_graph).getState(np.zeros(7))
factory = KinematicsPluginFactory(support / "lbr_iiwa_14_r820_plugins.yaml", locator)
inv_kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", scene_graph, scene_state)
joints = [f"joint_a{i}" for i in range(1, 8)]
kg = KinematicGroup("manipulator", joints, inv_kin, scene_graph, scene_state)
del inv_kin, factory
gc.collect()
q = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
pose = kg.calcFwdKin(q)["tool0"]
solutions = kg.calcInvKin(KinGroupIKInput(pose, kg.getBaseLinkName(), "tool0"), np.zeros(7))
print("OK:", len(solutions) > 0)
"""


def test_kinematic_group_outlives_its_solver_and_factory():
    """gh-72 rule: the group's cloned solver runs code from the factory's plugin library.

    Before keep_alive<1, 4> was added to the constructor, this already passed on macOS
    (2026-10-09, `.scratch/phaseD-b-212-lifetime-red.log`), as the gh-170 by-name case did.
    The tie is kept anyway, as the gh-72 rule requires, since unmapping on dlclose is
    platform-dependent.
    """
    proc = subprocess.run(
        [sys.executable, "-c", _GROUP_OUTLIVES_SOLVER_SCRIPT],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode} (SIGSEGV is -11): {proc.stderr[-800:]}"
    assert "OK: True" in proc.stdout
