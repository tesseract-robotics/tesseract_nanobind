"""`CartAxisKinematics` and `calcJacobianBlock` on the CartAxis constraints (#373).

`CartAxisKinematics` maps joint values to a source link axis expressed in a target link's
coordinates, v = R_target^T R_source a, with its 3 x n Jacobian. The constraints'
`calcJacobianBlock(joint_vals)` returns the sparse Jacobian at the given joint values, which
equals `getJacobian()` at the linked variable's own values (as the #289 ones do).
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

from .scenes import Iiwa

TOOL_Z = np.array([0.0, 0.0, 1.0])
BASE_X = np.array([1.0, 0.0, 0.0])
# rad: a bent arm, so the tool axis is not along a base axis and no Jacobian column vanishes
CONFIG = np.array([0.3, 0.6, -0.2, -1.2, 0.4, 0.8, 0.1])
# rad: central-difference step of the numerical Jacobian oracle
FD_STEP = 1e-6
# 1/rad: central differences are O(h^2) ~ 1e-12 in truncation; rounding eps / h ~ 2e-10. 1e-7
# leaves three orders of headroom.
FD_ATOL = 1e-7
# unitless: v is a product of rotation matrices and a unit vector; a few ulp of 1
AXIS_ATOL = 1e-12
# rad: the cone's half angle, wide enough that CONFIG is not on its boundary
HALF_ANGLE = 0.5


@pytest.fixture
def kin(iiwa: Iiwa) -> ti.CartAxisKinematics:
    return ti.CartAxisKinematics(iiwa.manip, "tool0", TOOL_Z, "base_link")


def test_calc_axis_matches_forward_kinematics(iiwa: Iiwa, kin: ti.CartAxisKinematics):
    poses = iiwa.manip.calcFwdKin(CONFIG)
    expected = poses["base_link"].rotation.T @ poses["tool0"].rotation @ TOOL_Z
    np.testing.assert_allclose(kin.calcAxis(CONFIG), expected, atol=AXIS_ATOL)
    assert kin.numJoints() == len(iiwa.joint_names)
    np.testing.assert_array_equal(kin.getSourceAxis(), TOOL_Z)


def test_calc_axis_and_jacobian(kin: ti.CartAxisKinematics):
    axis, jacobian = kin.calcAxisAndJacobian(CONFIG)
    np.testing.assert_allclose(axis, kin.calcAxis(CONFIG), atol=AXIS_ATOL)
    assert jacobian.shape == (3, len(CONFIG))
    numerical = np.empty_like(jacobian)
    for j in range(len(CONFIG)):
        step = np.zeros(len(CONFIG))
        step[j] = FD_STEP
        numerical[:, j] = (kin.calcAxis(CONFIG + step) - kin.calcAxis(CONFIG - step)) / (
            2 * FD_STEP
        )
    np.testing.assert_allclose(jacobian, numerical, atol=FD_ATOL)


def test_static_helpers():
    np.testing.assert_allclose(ti.CartAxisKinematics.toUnitAxis([0.0, 0.0, 2.0], "axis"), TOOL_Z)
    with pytest.raises(ti.DegenerateAxisError, match="axis must be finite and non-zero"):
        ti.CartAxisKinematics.toUnitAxis(np.zeros(3), "axis")
    assert issubclass(ti.DegenerateAxisError, ValueError)
    direction = ti.CartAxisKinematics.mostReachableDirection(np.array([[1.0, 0.0], [0.0, 0.0]]))
    assert direction.shape == (2,)
    assert np.linalg.norm(direction) == pytest.approx(1.0)
    assert ti.CartAxisKinematics.SIN_ANGLE_FLOOR > 0.0


def test_unknown_link_raises(iiwa: Iiwa):
    with pytest.raises(RuntimeError):
        ti.CartAxisKinematics(iiwa.manip, "no_such_link", TOOL_Z, "base_link")


@pytest.mark.parametrize("kind", ["align", "cone"])
def test_calc_jacobian_block_equals_get_jacobian(iiwa: Iiwa, kind: str):
    nodes = iiwa.nodes(np.zeros(len(iiwa.joint_names)), CONFIG)
    var = nodes.getNodes()[1].getVar("joints")
    if kind == "align":
        constraint = ti.CartAxisAlignConstraint(
            var, iiwa.manip, "tool0", TOOL_Z, "base_link", BASE_X
        )
    else:
        constraint = ti.CartAxisConeConstraint(
            var, iiwa.manip, "tool0", TOOL_Z, "base_link", BASE_X, HALF_ANGLE
        )
    with pytest.raises(ti.ConstraintNotLinkedError):
        constraint.calcJacobianBlock(var.value())
    constraint.linkWithVariables(nodes)
    block = constraint.calcJacobianBlock(var.value())
    jacobian = constraint.getJacobian()
    assert block.shape == jacobian.shape == (constraint.getRows(), nodes.getRows())
    np.testing.assert_allclose(block.toarray(), jacobian.toarray())
    assert np.any(block.toarray() != 0.0)
    with pytest.raises(ti.JointValuesSizeError, match=f"{len(iiwa.joint_names)} joints"):
        constraint.calcJacobianBlock(np.zeros(3))
