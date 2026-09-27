"""Tests for ConstraintSet Python trampoline bindings (tesseract 0.34).

Verifies that Python subclasses of ConstraintSet work correctly:
- Constructor with (name, n_constraints)
- Virtual method dispatch: getValues, getBounds, getJacobian, update, getCoefficients
- linkWithVariables / getVariables integration with NodesVariables
- Sparse Jacobian round-trip (scipy.sparse ↔ Eigen::SparseMatrix)
- addConstraintSet / addCostSet on TrajOptQPProblem
"""

from __future__ import annotations

import numpy as np
import scipy.sparse

from tesseract_robotics.trajopt_ifopt import (
    Bounds,
    ConstraintSet,
    createNodesVariables,
    toBounds,
)

# ---------------------------------------------------------------------------
# Minimal Python subclasses for testing the trampoline
# ---------------------------------------------------------------------------


class SimpleConstraint(ConstraintSet):
    """2 constraints on n_vars variables: x[0] == 1.0, x[1] == 2.0."""

    def __init__(self, name: str = "simple"):
        super().__init__(name, 2)
        self._targets = np.array([1.0, 2.0])

    def getValues(self) -> np.ndarray:
        variables = self.getVariables()
        if variables is None:
            return np.zeros(2)
        x = np.array(variables.getValues())
        return x[:2] - self._targets

    def getBounds(self) -> list:
        return [Bounds(0.0, 0.0), Bounds(0.0, 0.0)]

    def getJacobian(self):
        variables = self.getVariables()
        n_vars = len(variables.getValues()) if variables is not None else 2
        return scipy.sparse.csr_matrix(([1.0, 1.0], ([0, 1], [0, 1])), shape=(2, n_vars))

    def update(self) -> int:
        return self.getRows()

    def getCoefficients(self) -> np.ndarray:
        return np.ones(2)


class ScalarCost(ConstraintSet):
    """Single-row cost: 0.5 * ||x||^2."""

    def __init__(self, n_vars: int, name: str = "quadratic"):
        super().__init__(name, 1)
        self._n_vars = n_vars

    def getValues(self) -> np.ndarray:
        variables = self.getVariables()
        if variables is None:
            return np.zeros(1)
        x = np.array(variables.getValues())
        return np.array([0.5 * np.dot(x, x)])

    def getBounds(self) -> list:
        return [Bounds(0.0, 0.0)]

    def getJacobian(self):
        variables = self.getVariables()
        if variables is None:
            return scipy.sparse.csr_matrix((1, self._n_vars))
        x = np.array(variables.getValues())
        return scipy.sparse.csr_matrix(x.reshape(1, -1))

    def update(self) -> int:
        return 1

    def getCoefficients(self) -> np.ndarray:
        return np.ones(1)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------


def _make_nodes_variables(n_steps: int, n_dof: int, values: np.ndarray | None = None):
    """Create NodesVariables with given shape, optionally initialized."""
    joint_names = [f"j{i}" for i in range(n_dof)]
    if values is None:
        values = np.zeros((n_steps, n_dof))
    initial_values = [values[i].astype(np.float64) for i in range(n_steps)]
    bounds = toBounds(np.column_stack([np.full(n_dof, -10.0), np.full(n_dof, 10.0)]))
    return createNodesVariables("traj", joint_names, initial_values, bounds)


# ---------------------------------------------------------------------------
# Tests: Basic trampoline functionality
# ---------------------------------------------------------------------------


class TestConstraintSetTrampoline:
    """Core trampoline: construction, virtual dispatch, properties."""

    def test_construction(self):
        """Python subclass can be constructed with (name, n_constraints)."""
        c = SimpleConstraint("test_name")
        assert c.getName() == "test_name"
        assert c.getRows() == 2

    def test_getValues_without_variables(self):
        """getValues returns zeros when no variables linked."""
        c = SimpleConstraint()
        vals = c.getValues()
        np.testing.assert_array_equal(vals, [0.0, 0.0])

    def test_getBounds(self):
        """getBounds returns the correct bounds list."""
        c = SimpleConstraint()
        bounds = c.getBounds()
        assert len(bounds) == 2
        assert bounds[0].getLower() == 0.0
        assert bounds[0].getUpper() == 0.0

    def test_update_returns_rows(self):
        """update() returns the row count."""
        c = SimpleConstraint()
        assert c.update() == 2

    def test_getCoefficients(self):
        """getCoefficients returns ones vector."""
        c = SimpleConstraint()
        coeffs = c.getCoefficients()
        np.testing.assert_array_equal(coeffs, [1.0, 1.0])


class TestConstraintSetWithVariables:
    """Integration: link variables -> compute values/Jacobian."""

    def test_link_and_get_variables(self):
        """linkWithVariables + getVariables round-trip."""
        nodes = _make_nodes_variables(1, 3)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        v = c.getVariables()
        assert v is not None
        np.testing.assert_array_equal(v.getValues(), np.zeros(3))

    def test_getValues_with_variables(self):
        """getValues computes errors against linked variable values."""
        vals = np.array([[1.0, 2.0, 3.0]])
        nodes = _make_nodes_variables(1, 3, vals)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        np.testing.assert_allclose(c.getValues(), [0.0, 0.0], atol=1e-12)

    def test_getValues_nonzero_error(self):
        """getValues returns correct errors for non-target values."""
        vals = np.array([[3.0, 5.0, 0.0]])
        nodes = _make_nodes_variables(1, 3, vals)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        np.testing.assert_allclose(c.getValues(), [2.0, 3.0], atol=1e-12)

    def test_getJacobian_sparse_type(self):
        """getJacobian returns scipy sparse matrix."""
        nodes = _make_nodes_variables(1, 3)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        jac = c.getJacobian()
        assert scipy.sparse.issparse(jac)
        assert jac.shape == (2, 3)

    def test_getJacobian_values(self):
        """getJacobian has correct sparsity pattern and values."""
        nodes = _make_nodes_variables(1, 3)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        jac = c.getJacobian().toarray()
        expected = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ]
        )
        np.testing.assert_array_equal(jac, expected)

    def test_scalar_cost_gradient(self):
        """ScalarCost returns gradient = x for quadratic cost."""
        vals = np.array([[1.0, 2.0, 3.0]])
        nodes = _make_nodes_variables(1, 3, vals)
        c = ScalarCost(3)
        c.linkWithVariables(nodes)
        np.testing.assert_allclose(c.getValues(), [7.0], atol=1e-12)
        jac = c.getJacobian().toarray()
        np.testing.assert_allclose(jac, [[1.0, 2.0, 3.0]], atol=1e-12)

    def test_variable_mutation_propagates(self):
        """setVariables on NodesVariables propagates to constraint getValues."""
        nodes = _make_nodes_variables(1, 3)
        c = SimpleConstraint()
        c.linkWithVariables(nodes)
        np.testing.assert_allclose(c.getValues(), [-1.0, -2.0], atol=1e-12)

        nodes.setVariables(np.array([1.0, 2.0, 0.0]))
        np.testing.assert_allclose(c.getValues(), [0.0, 0.0], atol=1e-12)


class TestConstraintSetInTrajOptQPProblem:
    """Python ConstraintSet as a constraint and as a cost in TrajOptQPProblem.

    TrajOptQPProblem counts and names constraint and cost sets, not rows.
    """

    def test_add_constraint(self):
        from tesseract_robotics.trajopt_sqp import TrajOptQPProblem

        problem = TrajOptQPProblem(_make_nodes_variables(3, 2))
        problem.addConstraintSet(SimpleConstraint("test_constraint"))
        problem.setup()
        assert problem.getNLPConstraintNames() == ["test_constraint"]

    def test_add_squared_cost(self):
        """A squared cost needs equality bounds; ScalarCost has them."""
        from tesseract_robotics.trajopt_sqp import CostPenaltyType, TrajOptQPProblem

        problem = TrajOptQPProblem(_make_nodes_variables(3, 2))
        problem.addCostSet(ScalarCost(6, "test_cost"), CostPenaltyType.SQUARED)
        problem.setup()
        assert problem.getNLPCostNames() == ["test_cost"]

    def test_constraint_and_cost_together(self):
        from tesseract_robotics.trajopt_sqp import CostPenaltyType, TrajOptQPProblem

        problem = TrajOptQPProblem(_make_nodes_variables(3, 2))
        problem.addConstraintSet(SimpleConstraint("c1"))
        problem.addCostSet(ScalarCost(6, "cost1"), CostPenaltyType.SQUARED)
        problem.setup()
        assert problem.getNumNLPConstraints() == 1
        assert problem.getNumNLPCosts() == 1
