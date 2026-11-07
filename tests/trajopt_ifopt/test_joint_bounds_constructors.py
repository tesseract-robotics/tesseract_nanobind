"""The per-joint bounds constructors of JointVel/Accel/JerkConstraint (#290, trajopt#606).

The targets constructor pins every finite difference to one value. The bounds constructor gives
each joint a band per step: an equality, a one-sided limit, or a range. By default a range becomes
a lower-bound row followed by an upper-bound row (RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES),
because TrajOptQPProblem accepts only equality and one-sided rows; KEEP_AS_IS leaves it as one
row, which TrajOptQPProblem refuses. A symmetric band ``-v <= dq <= v`` is how a velocity,
acceleration or jerk limit is written.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

INF = np.inf
# joints of the bounds fixture: one per kind of bound (equality, one-sided, range)
N_DOF = 3
# states of the bounds fixture: enough for a jerk (third) difference to have two row blocks
N_STATES = 5
# rad / step^k: the per-step band of the solve, on every joint
BAND = 1.0
# rad: where the squared cost pulls the last state, well past what the band allows
PULL = 10.0
# joints of the solve: one pulled up, one pulled down, so both sides of the band bind
SOLVE_DOF = 2
# rad: generous joint limits, so only the band binds
JOINT_LIMIT = 100.0

# (class, difference order); a difference of order k has N_STATES - k row blocks
CLASSES = [
    (ti.JointVelConstraint, 1),
    (ti.JointAccelConstraint, 2),
    (ti.JointJerkConstraint, 3),
]
# Last state reachable from rest with every k-th difference in [-BAND, BAND] and the first k
# states pinned at 0, over N_STATES states: velocity 4 steps of 1; acceleration 1 + 2 + 3;
# jerk q3 <= 1, q4 <= 3 q3 + 1.
REACH = {1: 4.0 * BAND, 2: 6.0 * BAND, 3: 4.0 * BAND}

BOUNDS = [ti.Bounds(0.5, 0.5), ti.Bounds(-INF, 1.0), ti.Bounds(-0.3, 0.3)]
SPLIT = [(0.5, 0.5), (-INF, 1.0), (-0.3, INF), (-INF, 0.3)]


def _nodes(n_dof: int, n_states: int) -> tuple[ti.NodesVariables, list[ti.Var]]:
    nodes = ti.createNodesVariables(
        "trajectory",
        [f"j{k}" for k in range(n_dof)],
        [np.zeros(n_dof) for _ in range(n_states)],
        ti.toBounds(np.tile([-JOINT_LIMIT, JOINT_LIMIT], (n_dof, 1))),
    )
    return nodes, [node.getVar("joints") for node in nodes.getNodes()]


def _limits(constraint: ti.ConstraintSet) -> list[tuple[float, float]]:
    return [(b.getLower(), b.getUpper()) for b in constraint.getBounds()]


@pytest.mark.parametrize(("cls", "order"), CLASSES)
def test_joint_vel_bounds_ctor_get_bounds(cls, order):
    """KEEP_AS_IS repeats the bounds per row block; the default splits the range in two."""
    _, vars_list = _nodes(N_DOF, N_STATES)
    blocks = N_STATES - order

    kept = cls(BOUNDS, vars_list, np.ones(N_DOF), "band", ti.RangeBoundHandling.KEEP_AS_IS)
    assert _limits(kept) == [(b.getLower(), b.getUpper()) for b in BOUNDS] * blocks

    split = cls(BOUNDS, vars_list, np.ones(N_DOF), "band")
    assert split.getRows() == len(SPLIT) * blocks
    assert _limits(split) == SPLIT * blocks


@pytest.mark.parametrize(("cls", "order"), CLASSES)
def test_bounds_ctor_rejects_a_null_var(cls, order):
    _, vars_list = _nodes(N_DOF, N_STATES)
    with pytest.raises(TypeError, match=r"position_vars\[4\] is None"):
        cls(BOUNDS, [*vars_list[:-1], None], np.ones(N_DOF), "band")


def _band_problem(cls, order: int, handling: ti.RangeBoundHandling | None):
    nodes, vars_list = _nodes(SOLVE_DOF, N_STATES)
    problem = tsqp.TrajOptQPProblem(nodes)
    for k in range(order):
        problem.addConstraintSet(
            ti.JointPosConstraint(np.zeros(SOLVE_DOF), vars_list[k], np.ones(SOLVE_DOF), f"rest{k}")
        )
    band = [ti.Bounds(-BAND, BAND)] * SOLVE_DOF
    args = (band, vars_list, np.ones(SOLVE_DOF), "Band")
    constraint = cls(*args) if handling is None else cls(*args, handling)
    return nodes, vars_list, problem, constraint


@pytest.mark.parametrize(("cls", "order"), CLASSES)
def test_velocity_band_solves_in_qp_problem(cls, order):
    """The band binds: the last state stops where the band lets it, not at the pull."""
    _, vars_list, problem, constraint = _band_problem(cls, order, None)
    problem.addConstraintSet(constraint)
    pull = np.array([PULL, -PULL])
    problem.addCostSet(
        ti.JointPosConstraint(pull, vars_list[-1], np.ones(SOLVE_DOF), "Pull"),
        tsqp.CostPenaltyType.SQUARED,
    )
    problem.setup()
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())

    solver.solve(problem)

    assert solver.getStatus() == tsqp.SQPStatus.CONVERGED
    tolerance = solver.params.cnt_tolerance
    states = np.array([var.value() for var in vars_list])
    differences = np.diff(states, n=order, axis=0)
    assert np.all(np.abs(differences) <= BAND + tolerance)
    np.testing.assert_allclose(states[-1], [REACH[order], -REACH[order]], atol=tolerance)


@pytest.mark.parametrize(("cls", "order"), CLASSES)
def test_unsplit_band_is_refused_by_qp_problem(cls, order):
    """The negative control: KEEP_AS_IS leaves range rows, which TrajOptQPProblem refuses."""
    _, _, problem, constraint = _band_problem(cls, order, ti.RangeBoundHandling.KEEP_AS_IS)
    with pytest.raises(RuntimeError, match="rows must have equality or one-sided bounds"):
        problem.addConstraintSet(constraint)
