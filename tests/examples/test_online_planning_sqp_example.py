"""The online SQP example builds the problem its C++ reference builds, in both collision modes.

tesseract_planning's online_planning_example.cpp (0.35.0) builds a TrajOptQPProblem over the
node variables: the start and target poses as constraints, the joint velocity as a squared
cost, one collision constraint per step with the collision config's max_num_cnt rows. The
example targets the joint vector instead of the pose, and weighs the velocity cost per joint,
as the task composer variant plans the same task (online_planning_scene). The
example used to add its velocity cost without a penalty type, to the NLP layer of the
two-layer problem trajopt is removing. There the cost reached neither the QP's gradient nor
its Gauss-Newton Hessian, so the QP had no cost term.

The continuous mode builds LVSDiscreteCollisionEvaluator, which rejects a config whose check
type is not LVS_DISCRETE. The C++ reference configures DISCRETE for both modes, so its
continuous mode throws; the example sets LVS_DISCRETE there.
"""

from __future__ import annotations

import importlib

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
)
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.examples import online_planning_scene
from tesseract_robotics.examples.online_planning_sqp_example import (
    COLLISION_ROWS_PER_WAYPOINT,
    build_optimization_problem,
)
from tesseract_robotics.planning import Robot

STEPS = 12  # waypoints; run()'s default, as in the C++ reference
START_JOINTS = np.array(online_planning_scene.START_JOINTS)  # run()'s start, m and rad
TARGET_JOINTS = np.array(online_planning_scene.TARGET_JOINTS)  # run()'s target, m and rad
BOX_SIZE = 0.01  # run()'s initial trust box, as online_planning_example.cpp sets it
# float64 sums of 88 squared joint steps (11 steps x 8 joints) agree to about 1e-14 relative.
COST_ROUND_OFF = 1e-12


@pytest.fixture(scope="module")
def robot():
    robot = Robot.from_tesseract_support(
        "online_planning_example", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    robot.set_joints(START_JOINTS, joint_names=robot.get_joint_names("manipulator"))
    # As run() sets the scene up: the arm's links, not the robot's zone, meet the person
    online_planning_scene.check_arm_against_person(robot)
    return robot


@pytest.fixture(params=[False, True], ids=["discrete", "lvs_continuous"])
def problem_data(robot, request):
    joint_names = robot.get_joint_names("manipulator")
    return build_optimization_problem(
        robot,
        joint_names,
        START_JOINTS,
        TARGET_JOINTS,
        STEPS,
        use_continuous_collision=request.param,
    )


def _trajectory(problem_data) -> np.ndarray:
    values = np.array(problem_data["nodes_variables"].getValues())
    return values.reshape(STEPS, len(TARGET_JOINTS))


def test_builds_a_trajopt_qp_problem(problem_data):
    assert isinstance(problem_data["problem"], tsqp.TrajOptQPProblem)


def test_each_collision_constraint_has_the_configs_max_num_cnt_rows(problem_data):
    """As in the C++ reference: one set per step, sized by the config, not a literal 1.

    The config holds more rows than the C++ default 3, as several arm links meet the person.
    """
    constraints = problem_data["collision_constraints"]
    assert len(constraints) == STEPS - 1
    max_num_cnt = problem_data["collision_config"].max_num_cnt
    assert max_num_cnt == COLLISION_ROWS_PER_WAYPOINT
    assert [c.getRows() for c in constraints] == [max_num_cnt] * (STEPS - 1)


def test_the_velocity_cost_is_the_weighted_squared_joint_steps(robot, problem_data):
    """JointVelConstraint's residual is the joint step; each joint's weight times its square.

    The weights are the scene's, a gantry step costing more than an arm step; TrajOpt's
    velocity cost in the task composer variant sums the same terms.
    """
    steps = np.diff(_trajectory(problem_data), axis=0)
    weights = online_planning_scene.velocity_weights(robot)
    assert problem_data["problem"].getTotalExactCost() == pytest.approx(
        np.sum(weights * steps**2), rel=COST_ROUND_OFF
    )


def test_the_global_solve_reaches_the_target_from_the_pinned_start(robot, problem_data):
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.params.initial_trust_box_size = BOX_SIZE

    solver.solve(problem_data["problem"])

    assert solver.getStatus() == tsqp.SQPStatus.CONVERGED
    # The solver accepts a set within cnt_tolerance, so each joint row of the start and the
    # target is within it.
    tolerance = solver.params.cnt_tolerance
    # best_var_vals holds the QP's slack variables after the joint values.
    joint_values = np.array(solver.getResults().best_var_vals)[: STEPS * len(TARGET_JOINTS)]
    trajectory = joint_values.reshape(STEPS, len(TARGET_JOINTS))
    assert np.max(np.abs(trajectory[-1] - TARGET_JOINTS)) <= tolerance
    assert np.max(np.abs(trajectory[0] - START_JOINTS)) <= tolerance


# The package re-exports each example's `run` under the module's name: import the module.
sqp_example = importlib.import_module("tesseract_robotics.examples.online_planning_sqp_example")

# A seed bowed off the straight interpolation: the gantry's y axis 0.3 m further out at every
# interior waypoint [m], towards a person standing at the walk's near end, at (3.0, 3.3).
SEED_BOW_M = 0.3
PERSON_JOINTS_M = {"human_x_joint": 3.0 - 3.0, "human_y_joint": 3.3 - 4.0}
# How far inside the margin a converged plan may sit [m]: the solver accepts a constraint
# within cnt_tolerance, 1e-4.
MARGIN_TOLERANCE_M = 1e-4
# Contact distance reported up to this far [m]: past the whole cell, so every link reports.
REPORT_DISTANCE_M = 10.0


def test_a_seeded_problem_steps_from_its_seed(robot):
    """The seed reaches the collision constraints, as the C++ setupProblem's seed does.

    Seeded with `NodesVariables.setVariables` after `setup()`, the collision constraints still
    evaluated the straight interpolation: one step reported convergence and moved nothing,
    with the bowed seed deeper in the person's zone than the constraints said.
    """
    joint_names = robot.get_joint_names("manipulator")
    start = START_JOINTS
    seed = np.array(ti.interpolate(start, TARGET_JOINTS, STEPS))
    seed[1:-1, 1] += SEED_BOW_M
    robot.env.setState(PERSON_JOINTS_M)
    try:
        data = build_optimization_problem(
            robot, joint_names, start, TARGET_JOINTS, STEPS, False, initial_trajectory=seed
        )
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        solver.params.initial_trust_box_size = BOX_SIZE
        solver.init(data["problem"])

        converged = solver.stepSQPSolver()

        moved = np.array(solver.getResults().best_var_vals)[: seed.size] - seed.ravel()
        assert not converged
        assert np.abs(moved).max() > 0.0
    finally:
        robot.env.setState(dict.fromkeys(PERSON_JOINTS_M, 0.0))


def _arm_clearance_m(env, names, joints) -> float:
    """Closest distance between the robot's links and the person's cylinder at `joints` [m]."""
    env.setState(dict(zip(names, map(float, joints), strict=True)))
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkIds())
    manager.setDefaultCollisionMargin(REPORT_DISTANCE_M)
    manager.setCollisionObjectsTransform(env.getState().link_transforms)
    results = ContactResultMap()
    manager.contactTest(results, ContactRequest(ContactTestType.ALL))
    contacts = ContactResultVector()
    results.flattenMoveResults(contacts)
    return min(
        contact.distance
        for contact in contacts
        if online_planning_scene.ENVELOPE_LINK in [link.name() for link in contact.link_ids]
    )


def test_every_tick_of_the_walk_keeps_the_margin():
    """Each tick's plan keeps the arm the collision margin clear of the person.

    With three collision rows per waypoint (the config's default) a plan converged with a link
    0.03 m inside the margin.
    """
    results = sqp_example.run()
    env = results["robot"].env
    names = results["joint_names"]
    worst_m = np.inf
    for tick, plan in zip(results["ticks"], results["plans"], strict=True):
        online_planning_scene.move_person(env, tick)
        worst_m = min(worst_m, *(_arm_clearance_m(env, names, q) for q in plan))
    assert worst_m >= online_planning_scene.COLLISION_MARGIN_M - MARGIN_TOLERANCE_M
