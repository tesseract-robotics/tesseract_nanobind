"""setBounds on every constraint that declares it, and the CartLine / IK accessors (#288)."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import Isometry3d

from .scenes import OVERLAP_X, Iiwa, TwoBoxes, two_boxes_at

# m: CartLineConstraint's segment along x; any nonzero length (equal endpoints are invalid)
LINE_LENGTH = 0.1
# rows of a twist error: 3 linear + 3 angular
TWIST_SIZE = 6
# m: second slider position of the continuous pair
SECOND_X = 0.15
# rad: joint values of the IK targets; inside the IIWA limits, away from singularities
IK_CONFIG_A = np.array([0.1, 0.4, 0.0, -1.2, 0.0, 0.6, 0.0])
IK_CONFIG_B = np.array([-0.3, 0.6, 0.2, -1.0, 0.1, 0.8, 0.2])


def _limits(bounds):
    return [(b.getLower(), b.getUpper()) for b in bounds]


def _shifted(n: int) -> list[ti.Bounds]:
    return [ti.Bounds(-1.0 - i, 0.5 + i) for i in range(n)]


def _line_end() -> Isometry3d:
    end = np.eye(4)
    end[0, 3] = LINE_LENGTH
    return Isometry3d(end)


def _cart_line(iiwa: Iiwa) -> ti.CartLineConstraint:
    var = iiwa.nodes(np.zeros(len(iiwa.joint_names))).getNodes()[0].getVar("joints")
    info = ti.CartLineInfo()
    info.manip = iiwa.manip
    info.source_frame = "tool0"
    info.target_frame = "base_link"
    info.source_frame_offset = Isometry3d.Identity()
    info.target_frame_offset1 = Isometry3d.Identity()
    info.target_frame_offset2 = _line_end()
    info.indices = list(range(TWIST_SIZE))
    return ti.CartLineConstraint(info, var, np.ones(TWIST_SIZE))


def _ik(iiwa: Iiwa, target: Isometry3d):
    n = len(iiwa.joint_names)
    nodes = iiwa.nodes(np.zeros(n), np.zeros(n))
    constraint_var, seed_var = (node.getVar("joints") for node in nodes.getNodes())
    info = ti.InverseKinematicsInfo()
    info.manip = iiwa.manip
    info.working_frame = "base_link"
    info.tcp_frame = "tool0"
    info.tcp_offset = Isometry3d.Identity()
    return ti.InverseKinematicsConstraint(target, info, constraint_var, seed_var), nodes


def _fk(iiwa: Iiwa, q: np.ndarray) -> Isometry3d:
    return iiwa.manip.calcFwdKin(q)["tool0"]


def test_cart_line_set_bounds(iiwa: Iiwa):
    constraint = _cart_line(iiwa)
    bounds = _shifted(TWIST_SIZE)
    constraint.setBounds(bounds)
    assert _limits(constraint.getBounds()) == _limits(bounds)


def test_discrete_collision_set_bounds(two_boxes: TwoBoxes):
    constraint = ti.DiscreteCollisionConstraint(two_boxes.discrete_evaluator(), two_boxes.var)
    constraint.setBounds(_shifted(1))
    assert _limits(constraint.getBounds()) == _limits(_shifted(1))


def test_discrete_collision_numerical_set_bounds(two_boxes: TwoBoxes):
    constraint = ti.DiscreteCollisionNumericalConstraint(
        two_boxes.discrete_evaluator(), two_boxes.var
    )
    constraint.setBounds(_shifted(1))
    assert _limits(constraint.getBounds()) == _limits(_shifted(1))


def test_continuous_collision_set_bounds():
    boxes = two_boxes_at(OVERLAP_X, SECOND_X)
    var0, var1 = (node.getVar("joints") for node in boxes.nodes.getNodes())
    constraint = ti.ContinuousCollisionConstraint(boxes.lvs_discrete_evaluator(), var0, var1)
    constraint.setBounds(_shifted(1))
    assert _limits(constraint.getBounds()) == _limits(_shifted(1))


def test_inverse_kinematics_set_bounds(iiwa: Iiwa):
    constraint, _ = _ik(iiwa, _fk(iiwa, IK_CONFIG_A))
    bounds = _shifted(len(iiwa.joint_names))
    constraint.setBounds(bounds)
    assert _limits(constraint.getBounds()) == _limits(bounds)


def test_set_bounds_rejects_a_wrong_count(two_boxes: TwoBoxes):
    constraint = ti.DiscreteCollisionConstraint(two_boxes.discrete_evaluator(), two_boxes.var)
    with pytest.raises(ti.BoundsSizeError, match="1 row"):
        constraint.setBounds(_shifted(2))


def test_inverse_kinematics_set_target_pose_changes_the_values(iiwa: Iiwa):
    constraint, nodes = _ik(iiwa, _fk(iiwa, IK_CONFIG_A))
    constraint.linkWithVariables(nodes)
    before = np.asarray(constraint.getValues()).copy()
    constraint.setTargetPose(_fk(iiwa, IK_CONFIG_B))
    after = np.asarray(constraint.getValues())
    assert not np.allclose(before, after)


def test_inverse_kinematics_get_kinematic_info(iiwa: Iiwa):
    constraint, _ = _ik(iiwa, _fk(iiwa, IK_CONFIG_A))
    info = constraint.getKinematicInfo()
    assert info.working_frame == "base_link"
    assert info.tcp_frame == "tool0"
    assert list(info.manip.getJointNames()) == iiwa.joint_names


def test_cart_line_get_line_and_info(iiwa: Iiwa):
    constraint = _cart_line(iiwa)
    start, end = constraint.getLine()
    np.testing.assert_allclose(start.matrix, np.eye(4))
    np.testing.assert_allclose(end.matrix, _line_end().matrix)
    info = constraint.getInfo()
    assert (info.source_frame, info.target_frame) == ("tool0", "base_link")
    np.testing.assert_array_equal(info.indices, np.arange(TWIST_SIZE))
