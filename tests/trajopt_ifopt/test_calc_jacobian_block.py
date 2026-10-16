"""calcJacobianBlock(joint_vals) evaluates a constraint's Jacobian at given joint values (#289).

The C++ out-param becomes the return value: a sparse matrix sized like getJacobian(), which
needs the linked variables for its column count. At the linked variable's own values it equals
getJacobian(). InverseKinematicsConstraint declares the method but trajopt 0.35.0 never defines
it, so it is not bound.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import Isometry3d

from .scenes import Iiwa, TwoBoxes

# rows of a twist error: 3 linear + 3 angular
TWIST_SIZE = 6
# m: CartLineConstraint's segment along x
LINE_LENGTH = 0.1
# rad: a joint configuration with every joint off zero, so no Jacobian column vanishes
CONFIG = np.array([0.1, 0.4, -0.2, -1.2, 0.3, 0.6, -0.1])


@pytest.fixture
def linked(iiwa: Iiwa):
    nodes = iiwa.nodes(np.zeros(len(iiwa.joint_names)), CONFIG)
    return nodes, nodes.getNodes()[1].getVar("joints")


def _cart_line_info(iiwa: Iiwa, indices) -> ti.CartLineInfo:
    end = np.eye(4)
    end[0, 3] = LINE_LENGTH
    info = ti.CartLineInfo()
    info.manip = iiwa.manip
    info.source_frame = "tool0"
    info.target_frame = "base_link"
    info.source_frame_offset = Isometry3d.Identity()
    info.target_frame_offset1 = Isometry3d.Identity()
    info.target_frame_offset2 = Isometry3d(end)
    info.indices = np.asarray(indices)
    return info


def _assert_block_matches(
    constraint: ti.CartPosConstraint
    | ti.CartLineConstraint
    | ti.DiscreteCollisionNumericalConstraint,
    var: ti.Var,
):
    block = constraint.calcJacobianBlock(var.value())
    jacobian = constraint.getJacobian()
    assert block.shape == jacobian.shape
    np.testing.assert_allclose(block.toarray(), jacobian.toarray())
    assert np.any(block.toarray() != 0.0)


def test_cart_pos(iiwa: Iiwa, linked):
    nodes, var = linked
    constraint = ti.CartPosConstraint(
        var,
        iiwa.manip,
        "tool0",
        "base_link",
        Isometry3d.Identity(),
        iiwa.manip.calcFwdKin(np.zeros(len(iiwa.joint_names)))["tool0"],
    )
    constraint.linkWithVariables(nodes)
    _assert_block_matches(constraint, var)


def test_cart_line(iiwa: Iiwa, linked):
    nodes, var = linked
    constraint = ti.CartLineConstraint(
        _cart_line_info(iiwa, np.arange(TWIST_SIZE)), var, np.ones(TWIST_SIZE)
    )
    constraint.linkWithVariables(nodes)
    _assert_block_matches(constraint, var)


def test_cart_line_rejects_fewer_than_six_indices(iiwa: Iiwa, linked):
    nodes, var = linked
    constraint = ti.CartLineConstraint(_cart_line_info(iiwa, [0, 1, 2]), var, np.ones(3))
    constraint.linkWithVariables(nodes)
    with pytest.raises(ti.CartLineIndicesUnsupportedError):
        constraint.getJacobian()
    with pytest.raises(ti.CartLineIndicesUnsupportedError):
        constraint.calcJacobianBlock(var.value())


def test_discrete_collision_numerical(two_boxes: TwoBoxes):
    constraint = ti.DiscreteCollisionNumericalConstraint(
        two_boxes.discrete_evaluator(), two_boxes.var
    )
    constraint.linkWithVariables(two_boxes.nodes)
    constraint.update()
    _assert_block_matches(constraint, two_boxes.var)


def test_unlinked_raises(iiwa: Iiwa, linked):
    _, var = linked
    constraint = ti.CartLineConstraint(
        _cart_line_info(iiwa, np.arange(TWIST_SIZE)), var, np.ones(TWIST_SIZE)
    )
    with pytest.raises(ti.ConstraintNotLinkedError):
        constraint.calcJacobianBlock(var.value())


def test_wrong_joint_count_raises(iiwa: Iiwa, linked):
    nodes, var = linked
    constraint = ti.CartLineConstraint(
        _cart_line_info(iiwa, np.arange(TWIST_SIZE)), var, np.ones(TWIST_SIZE)
    )
    constraint.linkWithVariables(nodes)
    with pytest.raises(ti.JointValuesSizeError, match="7"):
        constraint.calcJacobianBlock(np.zeros(3))
