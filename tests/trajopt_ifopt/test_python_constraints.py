"""PyConstraint and the planning rules in trajopt_ifopt.python_constraints.

Each rule's Jacobian is checked against trajopt's own calcNumericalConstraintGradient over the
whole problem (so the column placement is checked too), a TrajOpt solve has to satisfy it, and
the two rules with a C++ counterpart (CartAxisConeConstraint, CartAxisAlignConstraint) have to
agree with it.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.planning import Robot
from tesseract_robotics.trajopt_ifopt.python_constraints import (
    Clearance,
    ElbowUp,
    HoldAxis,
    PointsDown,
    PyConstraint,
    StayDexterous,
    WatchTheTool,
    WorldHold,
)

# a forward difference of step 1e-6 on smooth kinematics agrees to about 1e-5
JAC_ATOL = 1e-4
# a generic, non-singular IRB2400 pose
Q = np.array([0.3, 0.4, -0.2, 0.5, -0.6, 0.7])


@pytest.fixture(scope="module")
def robot():
    return Robot.from_tesseract_support(
        "abb_irb2400", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )


@pytest.fixture(scope="module")
def manip(robot):
    return robot.env.getKinematicGroup("manipulator")


def _joint_names(manip):
    return [joint_id.name() for joint_id in manip.getJointIds()]


def _nodes(manip, states):
    limits = manip.getLimits().joint_limits
    return ti.createNodesVariables(
        "trajectory",
        _joint_names(manip),
        [np.asarray(s, float) for s in states],
        ti.toBounds(limits),
    )


def _rules(robot, manip, var):
    tcp_now = manip.calcFwdKin(Q)["tool0"].translation
    return {
        "points_down_analytic": PointsDown(var, manip, "tool0", np.radians(30)),
        "points_down_numeric": PointsDown(var, manip, "tool0", np.radians(30), analytic=False),
        "hold_axis_analytic": HoldAxis(var, manip, "tool0", [0.0, 0.6, -0.8], np.radians(1)),
        "hold_axis_numeric": HoldAxis(
            var, manip, "tool0", [0.0, 0.6, -0.8], np.radians(1), analytic=False
        ),
        "stay_dexterous": StayDexterous(var, manip, "tool0", 1e-3),
        "elbow_up": ElbowUp(var, manip, "link_3", "link_2", 0.05),
        "watch_the_tool": WatchTheTool(
            var, manip, "link_2", "tool0", np.radians(40), optical_axis=0
        ),
        "clearance": Clearance(
            var, robot.env, _joint_names(manip), ["link_6"], 0.0, 0.3, margin=0.5
        ),
        "world_hold": WorldHold(var, manip, "tool0", tcp_now, Isometry3d.Identity(), 1e-3),
    }


@pytest.mark.parametrize(
    "rule",
    [
        "points_down_analytic",
        "points_down_numeric",
        "hold_axis_analytic",
        "hold_axis_numeric",
        "stay_dexterous",
        "elbow_up",
        "watch_the_tool",
        "clearance",
        "world_hold",
    ],
)
def test_jacobian_matches_trajopt_numerical_gradient(robot, manip, rule):
    # three waypoints, the rule on the middle one: its block must land in columns 6..11
    nodes = _nodes(manip, [Q * 0.5, Q, Q * 1.1])
    var = nodes.getNodes()[1].getVar("joints")
    c = _rules(robot, manip, var)[rule]
    c.linkWithVariables(nodes)

    analytic = c.getJacobian().toarray()
    numerical = ti.calcNumericalConstraintGradient(nodes, c, 1e-7).toarray()
    assert analytic.shape == (c.getRows(), 18)
    assert not analytic[:, :6].any() and not analytic[:, 12:].any()
    np.testing.assert_allclose(analytic, numerical, atol=JAC_ATOL)


def test_values_and_bounds(robot, manip):
    nodes = _nodes(manip, [Q])
    var = nodes.getNodes()[0].getVar("joints")
    c = PointsDown(var, manip, "tool0", np.radians(30))
    c.linkWithVariables(nodes)
    z = manip.calcFwdKin(Q)["tool0"].rotation[:, 2]
    # one row, bounded below: a cosine needs no upper bound, and a row at 1 would sit on its
    # bound with a vanishing gradient whenever the axis is aligned
    np.testing.assert_allclose(c.getValues(), [-z[2]])
    (bound,) = c.getBounds()
    assert bound.getLower() == pytest.approx(np.cos(np.radians(30)))
    assert bound.getUpper() == np.inf


@pytest.mark.parametrize("rule", ["points_down_numeric", "watch_the_tool"])
def test_a_cosine_rule_is_one_row_bounded_below(robot, manip, rule):
    nodes = _nodes(manip, [Q])
    c = _rules(robot, manip, nodes.getNodes()[0].getVar("joints"))[rule]
    (bound,) = c.getBounds()
    assert np.isfinite(bound.getLower()) and bound.getUpper() == np.inf


def test_a_range_row_is_split_into_two_one_sided_rows(manip):
    class FirstJoint(PyConstraint):
        def f(self, q):
            return [q[0]]

    nodes = _nodes(manip, [Q])
    c = FirstJoint("first_joint", nodes.getNodes()[0].getVar("joints"), [-0.5], [0.5])
    c.linkWithVariables(nodes)
    np.testing.assert_allclose(c.getValues(), [Q[0], Q[0]])
    lo, hi = c.getBounds()
    assert lo.getLower() == -0.5 and lo.getUpper() == np.inf
    assert hi.getLower() == -np.inf and hi.getUpper() == 0.5


def test_mismatched_bounds_raise(manip):
    nodes = _nodes(manip, [Q])
    with pytest.raises(ValueError):
        PyConstraint("bad", nodes.getNodes()[0].getVar("joints"), [0.0, 1.0], [1.0])


def _solve_one_waypoint(manip, start, extra):
    """Stay close to `start` (squared cost) subject to `extra(var)`."""
    nodes = _nodes(manip, [start])
    var = nodes.getNodes()[0].getVar("joints")
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addCostSet(
        ti.JointPosConstraint(start, var, np.ones(6), "stay_close"), tsqp.CostPenaltyType.SQUARED
    )
    rule = extra(var)
    problem.addConstraintSet(rule)
    problem.setup()
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.solve(problem)
    return np.asarray(var.value()), rule


def test_solve_tilts_the_tool_down(manip):
    # the tool points sideways at the start, 90 degrees off vertical
    start = np.array([0.0, 0.2, 0.1, 0.0, -0.3, 0.0])
    tilt = np.degrees(np.arccos(-manip.calcFwdKin(start)["tool0"].rotation[2, 2]))
    assert tilt > 45

    q, rule = _solve_one_waypoint(
        manip, start, lambda var: PointsDown(var, manip, "tool0", np.radians(20))
    )
    tilt_after = np.degrees(np.arccos(-manip.calcFwdKin(q)["tool0"].rotation[2, 2]))
    assert tilt_after <= 20 + 0.5


def test_solve_raises_the_elbow(manip):
    start = np.array([0.0, 0.9, 0.6, 0.0, 0.3, 0.0])

    def elbow_height(q):
        tf = manip.calcFwdKin(q)
        return tf["link_3"].translation[2] - tf["link_2"].translation[2]

    delta = elbow_height(start) + 0.2  # 20 cm above where it starts
    q, _ = _solve_one_waypoint(
        manip, start, lambda var: ElbowUp(var, manip, "link_3", "link_2", delta)
    )
    assert elbow_height(q) >= delta - 1e-3


def test_solve_holds_an_axis_to_a_tenth_of_a_degree(manip):
    # the cosine form of PointsDown is too flat for this; HoldAxis is linear in the angle
    start = np.array([0.0, 0.2, 0.1, 0.0, -0.3, 0.0])
    direction = np.array([0.3, 0.0, -1.0])
    direction /= np.linalg.norm(direction)

    def off_axis_deg(q):
        cos = manip.calcFwdKin(q)["tool0"].rotation[:, 2] @ direction
        return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    assert off_axis_deg(start) > 45
    q, _ = _solve_one_waypoint(
        manip, start, lambda var: HoldAxis(var, manip, "tool0", direction, np.radians(0.1))
    )
    # the square cone reaches sqrt(2) * tol on its diagonal
    assert off_axis_deg(q) <= 0.1 * np.sqrt(2) + 0.02


TOOL_Z = np.array([0.0, 0.0, 1.0])
# a target direction off every base axis
DIRECTION = np.array([0.0, 0.6, -0.8])


def test_points_down_is_the_cosine_of_the_cpp_cone_angle(manip):
    nodes = _nodes(manip, [Q])
    var = nodes.getNodes()[0].getVar("joints")
    cone = ti.CartAxisConeConstraint(
        var, manip, "tool0", TOOL_Z, "base_link", DIRECTION, np.radians(30)
    )
    rule = PointsDown(var, manip, "tool0", np.radians(30), down=DIRECTION)

    angle = cone.calcValues(Q)[0]
    assert rule.f(Q)[0] == pytest.approx(np.cos(angle))


def test_hold_axis_is_the_sine_and_cosine_of_the_cpp_tilt(manip):
    nodes = _nodes(manip, [Q])
    var = nodes.getNodes()[0].getVar("joints")
    align = ti.CartAxisAlignConstraint(var, manip, "tool0", TOOL_Z, "base_link", DIRECTION)
    rule = HoldAxis(var, manip, "tool0", DIRECTION, np.radians(1))

    # the two C++ rows are the tilt angle times the unit tilt direction: their norm is the angle
    angle = np.linalg.norm(align.calcValues(Q))
    across_1, across_2, along = rule.f(Q)
    assert np.hypot(across_1, across_2) == pytest.approx(np.sin(angle))
    assert along == pytest.approx(np.cos(angle))
