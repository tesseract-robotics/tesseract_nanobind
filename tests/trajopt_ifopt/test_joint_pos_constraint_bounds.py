"""JointPosConstraint's per-joint bounds overload.

The target constructor pins every joint to one value. The bounds constructor gives each
joint its own band: an equality, a one-sided limit, or a range. By default a range becomes
two one-sided rows (RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES), because the QP problems
accept only equality and one-sided rows; KEEP_AS_IS leaves it as one row.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

N_DOF = 3
INF = np.inf


def joint_var():
    nodes = ti.createNodesVariables(
        "trajectory",
        [f"j{k}" for k in range(N_DOF)],
        [np.array([0.1, -0.2, 0.3])],
        ti.toBounds(np.tile([-10.0, 10.0], (N_DOF, 1))),
    )
    return nodes, nodes.getNodes()[0].getVar("joints")


def limits(constraint) -> list[tuple[float, float]]:
    return [(b.getLower(), b.getUpper()) for b in constraint.getBounds()]


class TestJointPosConstraintBounds:
    def test_equality_and_one_sided_bounds_keep_one_row_each(self):
        _, var = joint_var()
        bounds = [ti.Bounds(0.5, 0.5), ti.Bounds(-INF, 1.0), ti.Bounds(-1.0, INF)]
        constraint = ti.JointPosConstraint(bounds, var, np.array([1.0, 2.0, 3.0]), "band")
        assert constraint.getRows() == N_DOF
        assert limits(constraint) == [(0.5, 0.5), (-INF, 1.0), (-1.0, INF)]
        np.testing.assert_array_equal(constraint.getCoefficients(), [1.0, 2.0, 3.0])

    def test_a_range_becomes_two_one_sided_rows(self):
        nodes, var = joint_var()
        bounds = [ti.Bounds(-0.1, 0.2), ti.Bounds(0.0, 0.0), ti.Bounds(-0.3, 0.3)]
        constraint = ti.JointPosConstraint(bounds, var, np.array([1.0, 2.0, 3.0]), "band")
        constraint.linkWithVariables(nodes)
        assert limits(constraint) == [
            (-0.1, INF),
            (-INF, 0.2),
            (0.0, 0.0),
            (-0.3, INF),
            (-INF, 0.3),
        ]
        np.testing.assert_array_equal(constraint.getCoefficients(), [1.0, 1.0, 2.0, 3.0, 3.0])
        np.testing.assert_array_equal(constraint.getValues(), [0.1, 0.1, -0.2, 0.3, 0.3])
        jacobian = constraint.getJacobian().toarray()
        np.testing.assert_array_equal(jacobian.argmax(axis=1), [0, 0, 1, 2, 2])

    def test_keep_as_is_leaves_a_range_as_one_row(self):
        _, var = joint_var()
        bounds = [ti.Bounds(-0.1, 0.2), ti.Bounds(0.0, 0.0), ti.Bounds(-0.3, 0.3)]
        constraint = ti.JointPosConstraint(
            bounds, var, np.ones(N_DOF), "band", ti.RangeBoundHandling.KEEP_AS_IS
        )
        assert limits(constraint) == [(-0.1, 0.2), (0.0, 0.0), (-0.3, 0.3)]

    def test_one_coefficient_weights_every_row(self):
        """trajopt 0.35.0's range split read past a length-1 coeffs; the binding broadcasts."""
        _, var = joint_var()
        constraint = ti.JointPosConstraint([ti.Bounds(-0.1, 0.2)] * N_DOF, var, np.array([4.0]))
        np.testing.assert_array_equal(constraint.getCoefficients(), np.full(2 * N_DOF, 4.0))

    def test_a_non_positive_coefficient_raises(self):
        _, var = joint_var()
        with pytest.raises(RuntimeError, match="greater than zero"):
            ti.JointPosConstraint([ti.Bounds(0.0, 0.0)] * N_DOF, var, np.array([0.0]))
