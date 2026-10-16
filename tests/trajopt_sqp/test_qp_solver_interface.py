"""QPSolver's update interface and setWarmStart(QPProblem), driven by hand (#283).

The 2-variable QP: minimise x'Hx + q'x subject to l <= A x <= u (updateHessianMatrix doubles H
for OSQP's 1/2 x'Px). With H = I, q = (-2, -4), A = I the unconstrained minimiser is (1, 2);
the bound x0 <= 0.5 makes it (0.5, 2).
"""

from __future__ import annotations

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

GRADIENT = np.array([-2.0, -4.0])
LOWER = np.array([-10.0, -10.0])
UPPER = np.array([0.5, 10.0])
EXPECTED = np.array([0.5, 2.0])
# OSQP's default absolute tolerance is 1e-4; polishing lands well inside it
SOLUTION_ATOL = 1e-4


def _drive(solver: tsqp.QPSolver) -> np.ndarray:
    assert solver.init(2, 2)
    assert solver.updateHessianMatrix(scipy.sparse.identity(2, format="csr"))
    assert solver.updateGradient(GRADIENT)
    assert solver.updateLinearConstraintsMatrix(scipy.sparse.identity(2, format="csr"))
    assert solver.updateBounds(LOWER, UPPER)
    assert solver.solve()
    return np.asarray(solver.getSolution())


def test_osqp_solves_the_closed_form_qp():
    np.testing.assert_allclose(_drive(tsqp.OSQPEigenSolver()), EXPECTED, atol=SOLUTION_ATOL)


def test_the_interface_lives_on_the_base():
    solver = tsqp.OSQPEigenSolver()
    np.testing.assert_allclose(_drive_through_base(solver), EXPECTED, atol=SOLUTION_ATOL)


def _drive_through_base(solver: tsqp.OSQPEigenSolver) -> np.ndarray:
    base = tsqp.QPSolver
    assert base.init(solver, 2, 2)
    assert base.updateHessianMatrix(solver, scipy.sparse.identity(2, format="csr"))
    assert base.updateGradient(solver, GRADIENT)
    assert base.updateLinearConstraintsMatrix(solver, scipy.sparse.identity(2, format="csr"))
    # updateLowerBound / updateUpperBound need a set-up OSQP problem: before the first solve
    # only updateBounds stores the bounds (osqp_eigen_solver.cpp, trajopt 0.35.0)
    assert base.updateBounds(solver, LOWER, LOWER + 1.0)
    assert base.solve(solver)
    assert base.updateLowerBound(solver, LOWER)
    assert base.updateUpperBound(solver, UPPER)
    assert base.solve(solver)
    return np.asarray(base.getSolution(solver))


def test_update_size_mismatch_raises():
    solver = tsqp.OSQPEigenSolver()
    solver.init(2, 2)
    with pytest.raises(tsqp.QPSizeError):
        solver.updateGradient(np.zeros(3))
    with pytest.raises(tsqp.QPSizeError):
        solver.updateBounds(np.zeros(1), np.zeros(2))
    with pytest.raises(tsqp.QPSizeError):
        solver.updateHessianMatrix(scipy.sparse.identity(3, format="csr"))


def _convexified_problem() -> tsqp.TrajOptQPProblem:
    nodes = ti.createNodesVariables(
        "trajectory",
        ["j0"],
        [np.array([0.0]), np.array([0.3])],
        ti.toBounds(-np.ones(1), np.ones(1)),
    )
    vars_ = [node.getVar("joints") for node in nodes.getNodes()]
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addConstraintSet(ti.JointPosConstraint(np.array([0.0]), vars_[0], np.ones(1), "Start"))
    problem.addCostSet(
        ti.JointVelConstraint(np.zeros(1), vars_, np.ones(1), "Velocity"),
        tsqp.CostPenaltyType.SQUARED,
    )
    problem.setup()
    problem.convexify()
    return problem


def test_set_warm_start_from_a_problem():
    problem = _convexified_problem()
    solver = tsqp.OSQPEigenSolver()
    solver.init(problem.getNumQPVars(), problem.getNumQPConstraints())
    assert solver.setWarmStart(problem) is True
    assert tsqp.QPSolver.setWarmStart(solver, problem) is True


def test_set_warm_start_needs_the_problems_size():
    problem = _convexified_problem()
    solver = tsqp.OSQPEigenSolver()
    solver.init(problem.getNumQPVars() - 1, problem.getNumQPConstraints())
    with pytest.raises(tsqp.QPSizeError):
        solver.setWarmStart(problem)


def test_set_warm_start_bool_still_sets_the_option():
    solver = tsqp.OSQPEigenSolver()
    assert solver.setWarmStart(False) is None
    # warm starting off: setWarmStart(problem) is a no-op that reports success
    problem = _convexified_problem()
    solver.init(problem.getNumQPVars(), problem.getNumQPConstraints())
    assert solver.setWarmStart(problem) is True
