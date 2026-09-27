"""The online SQP example builds the problem its C++ reference builds.

tesseract_planning's online_planning_example.cpp (0.35.0) builds a TrajOptQPProblem over the
node variables: the start and target poses as constraints, the joint velocity as a squared
cost, one collision constraint per step. The example used to add its velocity cost to a
wrapped IfoptProblem instead, where IfoptQPProblem neither squares it nor feeds it to its
Gauss-Newton Hessian: only IfoptQPProblem.addCostSet does either.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.examples.online_planning_sqp_example import build_optimization_problem
from tesseract_robotics.planning import Robot

STEPS = 12  # waypoints; run()'s default, as in the C++ reference
TARGET_JOINTS = np.array([5.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # run()'s target, rad/m
BOX_SIZE = 0.01  # run()'s initial trust box, as online_planning_example.cpp sets it
# float64 sums of 88 squared joint steps (11 steps x 8 joints) agree to about 1e-14 relative.
COST_ROUND_OFF = 1e-12


@pytest.fixture(scope="module")
def robot():
    robot = Robot.from_tesseract_support("online_planning_example")
    robot.set_joints(np.zeros(len(TARGET_JOINTS)), joint_names=robot.get_joint_names("manipulator"))
    return robot


@pytest.fixture
def problem_data(robot):
    joint_names = robot.get_joint_names("manipulator")
    start = np.zeros(len(joint_names))
    return build_optimization_problem(
        robot, joint_names, start, TARGET_JOINTS, STEPS, use_continuous_collision=False
    )


def _trajectory(problem_data) -> np.ndarray:
    values = np.array(problem_data["nodes_variables"].getValues())
    return values.reshape(STEPS, len(TARGET_JOINTS))


def test_builds_a_trajopt_qp_problem(problem_data):
    assert isinstance(problem_data["problem"], tsqp.TrajOptQPProblem)


def test_the_velocity_cost_is_the_squared_joint_steps(problem_data):
    """JointVelConstraint's residual is the joint step; unit weight, squared penalty."""
    steps = np.diff(_trajectory(problem_data), axis=0)
    assert problem_data["problem"].getTotalExactCost() == pytest.approx(
        np.sum(steps**2), rel=COST_ROUND_OFF
    )


def test_the_global_solve_reaches_the_target_from_the_pinned_start(robot, problem_data):
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.params.initial_trust_box_size = BOX_SIZE

    solver.solve(problem_data["problem"])

    assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
    # The solver accepts a set within cnt_tolerance, so each translational row of the target
    # pose is within it and the TCP within sqrt(3) of it.
    tolerance = solver.params.cnt_tolerance
    # best_var_vals holds the QP's slack variables after the joint values.
    joint_values = np.array(solver.getResults().best_var_vals)[: STEPS * len(TARGET_JOINTS)]
    trajectory = joint_values.reshape(STEPS, len(TARGET_JOINTS))
    manip = problem_data["manip"]
    target = manip.calcFwdKin(TARGET_JOINTS)["tool0"].translation
    reached = manip.calcFwdKin(trajectory[-1])["tool0"].translation
    assert np.linalg.norm(reached - target) <= np.sqrt(3.0) * tolerance
    assert np.max(np.abs(trajectory[0])) <= tolerance
