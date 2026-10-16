"""QPProblem exposes the convex model of the last convexify() and a size-checked setVariables (#282)."""

from __future__ import annotations

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

N_DOF = 2
STEPS = 3
START = np.array([0.1, -0.2])
GOAL = np.array([0.6, 0.4])


@pytest.fixture
def problem() -> tsqp.TrajOptQPProblem:
    """Pinned start and goal as constraints, joint velocity as a squared cost."""
    states = [START + (GOAL - START) * k / (STEPS - 1) for k in range(STEPS)]
    nodes = ti.createNodesVariables(
        "trajectory",
        [f"j{i}" for i in range(N_DOF)],
        states,
        ti.toBounds(-np.ones(N_DOF), np.ones(N_DOF)),
    )
    vars_ = [node.getVar("joints") for node in nodes.getNodes()]
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addConstraintSet(ti.JointPosConstraint(START, vars_[0], np.ones(N_DOF), "Start"))
    problem.addConstraintSet(ti.JointPosConstraint(GOAL, vars_[-1], np.ones(N_DOF), "Goal"))
    problem.addCostSet(
        ti.JointVelConstraint(np.zeros(N_DOF), vars_, np.ones(N_DOF), "Velocity"),
        tsqp.CostPenaltyType.SQUARED,
    )
    problem.setup()
    problem.convexify()
    return problem


def test_model_shapes(problem: tsqp.TrajOptQPProblem):
    n_vars, n_cnts = problem.getNumQPVars(), problem.getNumQPConstraints()
    hessian = problem.getHessian()
    constraints = problem.getConstraintMatrix()
    assert scipy.sparse.issparse(hessian)
    assert scipy.sparse.issparse(constraints)
    assert hessian.shape == (n_vars, n_vars)
    assert constraints.shape == (n_cnts, n_vars)
    assert np.asarray(problem.getGradient()).shape == (n_vars,)
    assert np.asarray(problem.getBoundsLower()).shape == (n_cnts,)
    assert np.asarray(problem.getBoundsUpper()).shape == (n_cnts,)
    assert hessian.nnz > 0


def test_constraint_merit_coeff_round_trips(problem: tsqp.TrajOptQPProblem):
    # one merit coefficient per constraint set: Start and Goal
    assert len(problem.getConstraintMeritCoeff()) == 2
    coeff = np.array([3.0, 7.0])
    problem.setConstraintMeritCoeff(coeff)
    np.testing.assert_array_equal(problem.getConstraintMeritCoeff(), coeff)


def test_set_variables_round_trips(problem: tsqp.TrajOptQPProblem):
    x = np.linspace(-0.5, 0.5, problem.getNumNLPVars())
    problem.setVariables(x)
    np.testing.assert_array_equal(problem.getVariableValues(), x)


def test_set_variables_wrong_length_raises(problem: tsqp.TrajOptQPProblem):
    with pytest.raises(tsqp.VariablesSizeError, match=str(problem.getNumNLPVars())):
        problem.setVariables(np.zeros(problem.getNumNLPVars() - 1))
    with pytest.raises(ValueError):
        problem.setVariables(np.zeros(problem.getNumNLPVars() + 1))
