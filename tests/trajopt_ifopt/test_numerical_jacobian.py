"""Numerical Jacobian helpers for checking an analytic getJacobian (#295)."""

from __future__ import annotations

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti

# forward-difference step of the numerical gradient
EPSILON = 1e-6
# absolute agreement of a forward difference with an affine constraint's exact Jacobian; the
# rounding error of a step of EPSILON on values of order 1 is about 1e-16 / EPSILON = 1e-10
FD_ATOL = 1e-6
# the deliberately wrong Jacobian's scale; far outside FD_ATOL
WRONG_SCALE = 3.0


def _nodes(n_dof: int, steps: int) -> ti.NodesVariables:
    names = [f"j{i}" for i in range(n_dof)]
    states = [np.linspace(0.1, 0.5, n_dof) * (k + 1) for k in range(steps)]
    return ti.createNodesVariables(
        "trajectory", names, states, ti.toBounds(-np.ones(n_dof) * 5, np.ones(n_dof) * 5)
    )


def test_joint_velocity_matches_its_jacobian():
    nodes = _nodes(2, 3)
    vars_ = [node.getVar("joints") for node in nodes.getNodes()]
    constraint = ti.JointVelConstraint(np.zeros(2), vars_, np.ones(2))
    constraint.linkWithVariables(nodes)

    numerical = ti.calcNumericalConstraintGradient(nodes, constraint, EPSILON)
    assert scipy.sparse.issparse(numerical)
    np.testing.assert_allclose(
        numerical.toarray(), constraint.getJacobian().toarray(), atol=FD_ATOL
    )


class _ScaledIdentity(ti.ConstraintSet):
    """values = x; getJacobian returns `scale` * I, correct only for scale == 1."""

    def __init__(self, n: int, scale: float):
        super().__init__("scaled_identity", n)
        self._n, self._scale = n, scale

    def getValues(self):
        return np.asarray(self.getVariables().getValues())

    def getBounds(self):
        return [ti.Bounds(0.0, 0.0)] * self._n

    def getJacobian(self):
        return scipy.sparse.identity(self._n, format="csr") * self._scale

    def update(self):
        return self._n

    def getCoefficients(self):
        return np.ones(self._n)


def test_a_wrong_python_jacobian_does_not_match():
    nodes = _nodes(2, 1)
    wrong = _ScaledIdentity(2, WRONG_SCALE)
    wrong.linkWithVariables(nodes)
    numerical = ti.calcNumericalConstraintGradient(nodes, wrong)
    assert not np.allclose(numerical.toarray(), wrong.getJacobian().toarray(), atol=FD_ATOL)

    right = _ScaledIdentity(2, 1.0)
    right.linkWithVariables(nodes)
    numerical = ti.calcNumericalConstraintGradient(nodes, right)
    np.testing.assert_allclose(numerical.toarray(), right.getJacobian().toarray(), atol=FD_ATOL)


def test_forward_num_jac_of_a_linear_map():
    a = np.array([[1.0, 2.0, 0.0], [0.0, -1.0, 4.0]])
    jacobian = ti.calcForwardNumJac(lambda x: a @ x, np.array([0.3, -0.2, 0.7]), EPSILON)
    assert scipy.sparse.issparse(jacobian)
    np.testing.assert_allclose(jacobian.toarray(), a, atol=FD_ATOL)


def test_forward_num_jac_rejects_a_changing_output_length():
    calls = 0

    def ragged(x):
        nonlocal calls
        calls += 1
        return np.ones(3) if calls == 1 else np.ones(1)

    with pytest.raises(ti.NumericalJacobianSizeError, match="3"):
        ti.calcForwardNumJac(ragged, np.zeros(2), EPSILON)
    assert issubclass(ti.NumericalJacobianSizeError, ValueError)
