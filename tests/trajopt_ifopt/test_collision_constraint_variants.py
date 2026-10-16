"""The planner's dynamic-row collision constraints and the continuous numerical one (#291).

The TrajOpt-Ifopt planner builds its collision terms from DiscreteCollisionConstraintD and
ContinuousCollisionConstraintD; ContinuousCollisionNumericalConstraint is the continuous sibling
of DiscreteCollisionNumericalConstraint. Bound, Python can build the planner's problem by hand.
"""

from __future__ import annotations

from typing import TypeVar

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

from .scenes import BOX_SIZE, OVERLAP_X, TwoBoxes, two_boxes_at

# m: second slider position of the continuous pair, still overlapping the fixed box
SECOND_X = 0.15


ConstraintT = TypeVar("ConstraintT", bound=ti.ConstraintSet)


def _linked(constraint: ConstraintT, nodes: ti.NodesVariables) -> ConstraintT:
    constraint.linkWithVariables(nodes)
    constraint.update()
    return constraint


def test_discrete_d_reports_the_overlap(two_boxes: TwoBoxes):
    constraint = _linked(
        ti.DiscreteCollisionConstraintD(two_boxes.discrete_evaluator(), two_boxes.var),
        two_boxes.nodes,
    )
    assert constraint.getRows() == 1
    values = np.asarray(constraint.getValues())
    assert values.shape == (1,)
    assert values[0] > 0.0
    assert constraint.getCollisionEvaluator() is not None


def test_discrete_d_set_bounds_round_trips(two_boxes: TwoBoxes):
    constraint = _linked(
        ti.DiscreteCollisionConstraintD(two_boxes.discrete_evaluator(), two_boxes.var),
        two_boxes.nodes,
    )
    constraint.setBounds([ti.Bounds(-1.0, 0.5)])
    (bound,) = constraint.getBounds()
    assert (bound.getLower(), bound.getUpper()) == (-1.0, 0.5)


def test_discrete_d_solves_collision_free(two_boxes: TwoBoxes):
    problem = tsqp.TrajOptQPProblem(two_boxes.nodes)
    problem.addConstraintSet(
        ti.DiscreteCollisionConstraintD(two_boxes.discrete_evaluator(), two_boxes.var)
    )
    stay = ti.JointPosConstraint(np.array([OVERLAP_X]), two_boxes.var, np.ones(1), "Stay")
    problem.addCostSet(stay, tsqp.CostPenaltyType.SQUARED)
    problem.setup()

    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.solve(problem)

    assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
    x = solver.getResults().best_var_vals[0]  # the slider; slack variables follow
    # the boxes touch at |x| == BOX_SIZE; the solve keeps the margin on top of that
    assert abs(x) >= BOX_SIZE


def test_continuous_d_reports_the_overlap():
    boxes = two_boxes_at(OVERLAP_X, SECOND_X)
    var0, var1 = (node.getVar("joints") for node in boxes.nodes.getNodes())
    constraint = _linked(
        ti.ContinuousCollisionConstraintD(
            boxes.lvs_discrete_evaluator(), position_var0=var0, position_var1=var1
        ),
        boxes.nodes,
    )
    assert constraint.getRows() >= 1
    assert np.all(np.asarray(constraint.getValues()) > 0.0)
    assert constraint.getCollisionEvaluator() is not None


def test_continuous_numerical_matches_the_analytic_values():
    boxes = two_boxes_at(OVERLAP_X, SECOND_X)
    var0, var1 = (node.getVar("joints") for node in boxes.nodes.getNodes())
    analytic = _linked(
        ti.ContinuousCollisionConstraint(boxes.lvs_discrete_evaluator(), var0, var1),
        boxes.nodes,
    )
    numerical = _linked(
        ti.ContinuousCollisionNumericalConstraint(
            boxes.lvs_discrete_evaluator(), position_var0=var0, position_var1=var1
        ),
        boxes.nodes,
    )
    np.testing.assert_allclose(numerical.getValues(), analytic.getValues())
    assert np.asarray(numerical.getValues())[0] > 0.0
    assert numerical.getCollisionEvaluator() is not None


def test_star_import_exports_every_bound_name():
    """No __all__: the package re-exports what the extension binds."""
    import tesseract_robotics.trajopt_ifopt as package
    from tesseract_robotics.trajopt_ifopt import _trajopt_ifopt

    assert not hasattr(package, "__all__")
    public = {name for name in dir(_trajopt_ifopt) if not name.startswith("_")}
    assert public <= set(dir(package))
