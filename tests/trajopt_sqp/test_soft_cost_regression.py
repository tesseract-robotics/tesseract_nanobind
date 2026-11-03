"""Penalty cost models evaluate the NLP variables, not the QP slacks."""

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as sqp

COEFFICIENT = 10.0
INITIAL_POSITION = 1.0
TRIAL_POSITION = 0.9
EXCESS_SLACK = 0.1


@pytest.fixture(params=[sqp.CostPenaltyType.HINGE, sqp.CostPenaltyType.ABSOLUTE])
def soft_problem(request):
    """One linear violation with a weighted hinge or absolute soft cost."""
    node = ti.Node("soft_cost")
    node.addVar("position", ["q"], np.array([INITIAL_POSITION]), [ti.Bounds(-5.0, 5.0)])
    variables = ti.NodesVariables("soft_cost", [node])
    position = variables.getNodes()[0].getVar("position")
    bound = ti.Bounds(-np.inf, 0.0) if request.param == sqp.CostPenaltyType.HINGE else ti.Bounds()
    cost = ti.JointPosConstraint([bound], position, np.array([COEFFICIENT]), "soft")
    problem = sqp.TrajOptQPProblem(variables)
    problem.addCostSet(cost, request.param)
    problem.setup()
    problem.convexify()
    return problem, request.param


def test_convex_soft_cost_does_not_depend_on_qp_slacks(soft_problem):
    problem, penalty = soft_problem
    if penalty == sqp.CostPenaltyType.HINGE:
        points = ([TRIAL_POSITION, TRIAL_POSITION], [TRIAL_POSITION, TRIAL_POSITION + EXCESS_SLACK])
    else:
        points = (
            [TRIAL_POSITION, 0.0, TRIAL_POSITION],
            [TRIAL_POSITION, EXCESS_SLACK, TRIAL_POSITION + EXCESS_SLACK],
        )
    for point in points:
        assert problem.evaluateConvexCosts(np.array(point))[0] == pytest.approx(
            COEFFICIENT * TRIAL_POSITION
        )
