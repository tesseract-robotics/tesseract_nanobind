"""
Online Planning with Low-Level SQP API

Demonstrates real-time trajectory replanning using the low-level SQP solver: tens of Hz
(docs/examples/online-planning.md has measured rates for both collision modes).

C++ Reference:
    tesseract_planning/examples/src/online_planning_example.cpp (0.35.0)

**Key Difference from online_planning_example.py**
- Uses TrustRegionSQPSolver.stepSQPSolver() for incremental optimization
- Builds the optimization problem directly (no TaskComposer)
- Warm-starts from previous solution
- Achieves real-time rates suitable for robot control

**Collision Checking Modes**
- Discrete: Single timestep collision check (SingleTimestepCollisionEvaluator)
- Continuous: LVS (longest valid segment) between timesteps (LVSDiscreteCollisionEvaluator)

**Architecture**
1. Build a TrajOptQPProblem over the node variables, as the C++ reference does:
   - Node/Var for each waypoint (replaces JointPosition)
   - NodesVariables container (passed to the TrajOptQPProblem constructor)
   - JointPosConstraint for the start joints (constraint)
   - JointPosConstraint for the target joints (constraint; the C++ reference uses a
     CartPosConstraint on the target pose)
   - JointVelConstraint for smoothness (squared cost, weighted per joint)
   - CollisionConstraint for safety (constraint; no CollisionCache — now internal)

2. Create TrustRegionSQPSolver with OSQPEigenSolver

3. Loop, one tick per `TICK_S`:
   - Move the person one step along its walk (online_planning_scene)
   - Rebuild the problem seeded with the last solution, as the C++ setupProblem does
   - stepSQPSolver() until converged, at most `SQP_STEPS_PER_TICK` times
   - Extract trajectory from results

**The task**
See online_planning_scene.py
"""

# --8<-- [start:setup]
import sys
import time

import numpy as np

# Low-level SQP imports
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.collision import CollisionEvaluatorType, ContactTestType
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.examples import online_planning_scene as scene
from tesseract_robotics.planning import Robot

# --8<-- [end:setup]

# Initial trust box of the SQP solver, online_planning_example.cpp's default [rad or m].
TRUST_BOX_SIZE = 0.01
# Most SQP steps per tick. The C++ loop takes one step per iteration (`num_steps = 1`) and
# iterates as fast as the solver allows; at walking speed one 0.01 trust-box step per 0.1 s
# tick runs the arm into the person, 0.28 m deep at worst. Ten keep the 0.1 m margin on every
# tick of the walk, measured, and take about 50 ms, inside the tick. The loop stops early when
# a step converges.
SQP_STEPS_PER_TICK = 10
# Contacts each collision constraint keeps as rows. Several of the arm's links can touch the
# person and the floor at once. With the config's default 3 the solver converged with a link
# 0.03 m inside the margin; with 10 every tick keeps it, measured.
COLLISION_ROWS_PER_WAYPOINT = 10


class SolverDidNotConvergeError(RuntimeError):
    """The initial global solve did not end in `SQPStatus.CONVERGED`."""


# --8<-- [start:problem]
def build_optimization_problem(
    robot,
    joint_names,
    start_pos,
    target_pos,
    steps=10,
    use_continuous_collision=True,
    extra_constraints=None,
    initial_trajectory=None,
):
    """Build the TrajOptQPProblem online_planning_example.cpp builds, with a joint target.

    Args:
        robot: Robot instance with environment
        joint_names: List of joint names
        start_pos: Starting joint positions
        target_pos: Target joint positions
        steps: Number of trajectory waypoints
        use_continuous_collision: If True, use LVS continuous collision.
                                  If False, use discrete single-timestep collision.
        extra_constraints: Optional callable (vars_list, manip, env) -> constraint sets,
                           added before setup(), e.g. Python-defined planning rules.
        initial_trajectory: `(steps, joints)` seed, e.g. the last solution; default the straight
                            interpolation from start to target. The C++ setupProblem seeds the
                            variables this way, when it creates them. `NodesVariables.
                            setVariables` after setup() only writes the values: the collision
                            constraints keep evaluating the interpolation, and the solver
                            reports convergence without moving, the person inside the margin.

    Returns:
        dict with problem and all objects that must stay alive
    """
    # Get kinematic group and joint limits
    manip = robot.env.getKinematicGroup("manipulator")
    joint_limits = manip.getLimits().joint_limits

    # Seed: the given trajectory, or the straight interpolation
    if initial_trajectory is None:
        initial_states = ti.interpolate(start_pos, target_pos, steps)
    else:
        initial_states = [np.asarray(row, dtype=np.float64) for row in initial_trajectory]
    bounds = ti.toBounds(joint_limits)

    # Create NodesVariables — factory handles Node/Var creation on C++ side
    nodes_variables = ti.createNodesVariables(
        "trajectory", list(joint_names), list(initial_states), bounds
    )
    # One layer: constraint and cost sets go straight into the QP problem
    problem = tsqp.TrajOptQPProblem(nodes_variables)

    # Get Var references for constraints
    vars_list = []
    for node in nodes_variables.getNodes():
        vars_list.append(node.getVar("joints"))

    # Add start position constraint (first waypoint = current position)
    joint_coeffs = np.ones(len(joint_names)) * 5.0
    home_constraint = ti.JointPosConstraint(start_pos, vars_list[0], joint_coeffs, "Home_Position")
    problem.addConstraintSet(home_constraint)

    # Add target position constraint (last waypoint = target joints), as the start's
    target_constraint = ti.JointPosConstraint(
        target_pos, vars_list[-1], joint_coeffs, "Target_Position"
    )
    problem.addConstraintSet(target_constraint)

    # Add velocity cost (smooth motion), squared: weight times squared step, per joint, a
    # gantry step costing more than an arm step
    vel_target = np.zeros(len(joint_names))
    vel_weights = scene.velocity_weights(robot)
    vel_constraint = ti.JointVelConstraint(vel_target, vars_list, vel_weights, "JointVelocity")
    problem.addCostSet(vel_constraint, tsqp.CostPenaltyType.SQUARED)

    # Add collision constraints, configured as the C++ reference configures them
    margin = scene.COLLISION_MARGIN_M  # 0.1 m safety margin
    margin_coeff = 10.0
    collision_config = ti.TrajOptCollisionConfig(margin, margin_coeff)
    # More rows than the C++ default 3: several arm links can be in contact at once
    collision_config.max_num_cnt = COLLISION_ROWS_PER_WAYPOINT
    collision_config.collision_check_config.contact_request.type = ContactTestType.ALL
    # LVSDiscreteCollisionEvaluator rejects any check type but LVS_DISCRETE. The C++ reference
    # sets DISCRETE for both modes, so its continuous mode throws on trajopt 0.35.0.
    collision_config.collision_check_config.type = (
        CollisionEvaluatorType.LVS_DISCRETE
        if use_continuous_collision
        else CollisionEvaluatorType.DISCRETE
    )
    collision_config.collision_margin_buffer = 0.10
    collision_evaluators = []
    collision_constraints = []

    if use_continuous_collision:
        # The first segment starts at the pinned current state, so its start is fixed
        fixed_start = True
        for i in range(1, steps):
            collision_evaluator = ti.LVSDiscreteCollisionEvaluator(
                manip,
                robot.env,
                collision_config,
                True,
            )
            collision_constraint = ti.ContinuousCollisionConstraint(
                collision_evaluator,
                vars_list[i - 1],
                vars_list[i],
                fixed_start,  # position_var0 fixed
                False,  # position_var1 fixed
                collision_config.max_num_cnt,
                False,  # fixed_sparsity
                f"LVSDiscreteCollision_{i}",
            )
            fixed_start = False
            problem.addConstraintSet(collision_constraint)
            collision_evaluators.append(collision_evaluator)
            collision_constraints.append(collision_constraint)
    else:
        for i in range(1, steps):
            collision_evaluator = ti.SingleTimestepCollisionEvaluator(
                manip,
                robot.env,
                collision_config,
                True,
            )
            collision_constraint = ti.DiscreteCollisionConstraint(
                collision_evaluator,
                vars_list[i],
                collision_config.max_num_cnt,
                False,  # fixed_sparsity
                f"SingleTimestepCollision_{i}",
            )
            problem.addConstraintSet(collision_constraint)
            collision_evaluators.append(collision_evaluator)
            collision_constraints.append(collision_constraint)

    # Extra constraint sets, e.g. Python-defined rules (python_constraints.PyConstraint)
    extra_sets = list(extra_constraints(vars_list, manip, robot.env)) if extra_constraints else []
    for extra in extra_sets:
        problem.addConstraintSet(extra)

    # Setup the problem (must be called after adding all sets)
    problem.setup()

    return {
        "problem": problem,
        "nodes_variables": nodes_variables,
        "vars_list": vars_list,
        "target_constraint": target_constraint,
        "home_constraint": home_constraint,
        "vel_constraint": vel_constraint,
        "collision_config": collision_config,
        "collision_evaluators": collision_evaluators,
        "collision_constraints": collision_constraints,
        "manip": manip,
        "extra_sets": extra_sets,
    }


# --8<-- [end:problem]


def run(
    steps=12,
    verbose=False,
    use_continuous_collision=False,
    extra_constraints=None,
    num_replan=None,
    sqp_steps_per_tick=SQP_STEPS_PER_TICK,
):
    """Run trajectory optimization with low-level SQP API.

    Args:
        steps: Trajectory waypoints per plan (C++ default: 12)
        verbose: Print debug info
        use_continuous_collision: Use LVS continuous collision (True) or discrete (False)
                                  C++ default: False (discrete collision)
        extra_constraints: see build_optimization_problem
        num_replan: replanning ticks after the initial solve; default `scene.ticks_per_run()`,
                    `scene.LAPS_PER_RUN` laps of the walk
        sqp_steps_per_tick: most SQP steps per tick; 1 is the C++ loop's single step

    Returns:
        dict with results: `trajectories` (solver values per solve, the initial one first),
        `plans` (the same as `(steps, joints)` arrays), `ticks` (the walk's tick per solve)
    """
    if num_replan is None:
        num_replan = scene.ticks_per_run()
    # Load robot model
    robot = Robot.from_tesseract_support(
        "online_planning_example", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Define start and target: the robot runs down the cell along x, the arm at zero at both ends
    start_pos = np.array(scene.START_JOINTS)
    target_pos = np.array(scene.TARGET_JOINTS)

    robot.set_joints(start_pos, joint_names=joint_names)
    # The person, at the walk's first point for the initial solve, checked against the arm
    scene.add_person(robot)
    scene.check_arm_against_person(robot)
    scene.move_person(robot.env, 0)

    # Build initial problem
    collision_mode = "LVS continuous" if use_continuous_collision else "discrete"
    print(f"Building optimization problem (collision: {collision_mode})...")
    problem_data = build_optimization_problem(
        robot,
        joint_names,
        start_pos,
        target_pos,
        steps,
        use_continuous_collision,
        extra_constraints,
    )
    problem = problem_data["problem"]
    problem.print()

    # --8<-- [start:sqp_loop]
    # Create SQP solver with OSQP
    box_size = TRUST_BOX_SIZE
    qp_solver = tsqp.OSQPEigenSolver()
    solver = tsqp.TrustRegionSQPSolver(qp_solver)
    solver.verbose = verbose
    solver.params.initial_trust_box_size = box_size

    # Initial global solve
    print("Running initial global solve...")
    t0 = time.perf_counter()
    solver.solve(problem)
    solve_time = time.perf_counter() - t0
    if solver.getStatus() != tsqp.SQPStatus.CONVERGED:
        raise SolverDidNotConvergeError(
            f"initial solve ended {tsqp.toString(solver.getStatus())}, expected SQP_CONVERGED"
        )
    x = solver.getResults().best_var_vals
    cost = problem.getTotalExactCost()
    print(f"Initial solve: cost={cost:.4f}, time={solve_time * 1000:.1f}ms")

    trajectories = [x.copy()]
    timings = [solve_time]

    # Online replanning loop
    print(f"\nOnline replanning ({num_replan} iterations)...")

    for tick in range(1, num_replan + 1):
        # The person takes one step of its walk
        scene.move_person(robot.env, tick)

        # The solver leaves the problem's variables at best_var_vals: one row per waypoint
        traj = ti.toTrajArray(problem_data["vars_list"])

        # Rebuild the problem seeded with the last solution
        t0 = time.perf_counter()
        problem_data = build_optimization_problem(
            robot,
            joint_names,
            traj[0],
            target_pos,
            steps,
            use_continuous_collision,
            extra_constraints,
            initial_trajectory=traj,
        )
        problem = problem_data["problem"]

        # Incremental optimization: a few SQP steps, from the seed
        solver.init(problem)
        for _ in range(sqp_steps_per_tick):
            converged = solver.stepSQPSolver()
            # Reset box size (trust region loop can shrink it to zero)
            solver.setBoxSize(box_size)
            if converged:
                break

        x = solver.getResults().best_var_vals
        dt = time.perf_counter() - t0
        timings.append(dt)
        trajectories.append(x.copy())

        if verbose:
            cost = problem.getTotalExactCost()
            print(f"  Tick {tick}: cost={cost:.4f}, time={dt * 1000:.1f}ms")
    # --8<-- [end:sqp_loop]

    avg_time = np.mean(timings[1:]) * 1000
    print(f"Replan timing: avg={avg_time:.1f}ms ({1000 / avg_time:.0f} Hz)")
    print(f"Trajectory: {steps} waypoints x {len(joint_names)} joints")

    n_joints = len(joint_names)
    plans = [np.asarray(t[: steps * n_joints]).reshape(steps, n_joints) for t in trajectories]
    return {
        "trajectories": trajectories,
        "plans": plans,
        "ticks": list(range(num_replan + 1)),
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "steps": steps,
        "success": True,
    }


def recording_rows(results):
    """The walk as `Trajectory.from_rows` wants it: one row per tick, the plan of that tick.

    Columns: the manipulator joints, then `scene.HUMAN_JOINTS`, then the time [s].
    """
    return scene.playback_rows(results["plans"], results["ticks"])


def recording_joint_names(results):
    """Column names of `recording_rows`, without the time."""
    return [*results["joint_names"], *scene.HUMAN_JOINTS]


def main() -> int:
    results = run(verbose=False)

    if results.get("trajectories"):
        n_joints = len(results["joint_names"])
        steps = results["steps"]
        robot = results["robot"]
        manip = robot.env.getKinematicGroup("manipulator")

        print("\n=== Trajectory Results ===")

        target_joints = np.array(scene.TARGET_JOINTS)
        target_tf = manip.calcFwdKin(target_joints)["tool0"]
        target_pos = target_tf.translation

        traj0 = results["trajectories"][0][: steps * n_joints].reshape(steps, n_joints)
        initial_tf = manip.calcFwdKin(traj0[-1])["tool0"]
        initial_pos = initial_tf.translation

        traj_final = results["trajectories"][-1][: steps * n_joints].reshape(steps, n_joints)
        final_tf = manip.calcFwdKin(traj_final[-1])["tool0"]
        final_pos = final_tf.translation

        print(f"Target TCP:  {target_pos}")
        print(
            f"Initial TCP: {initial_pos} (error: {np.linalg.norm(initial_pos - target_pos):.3f}m)"
        )
        print(f"Final TCP:   {final_pos} (error: {np.linalg.norm(final_pos - target_pos):.3f}m)")
        arm = np.abs(np.asarray(results["plans"])[:, :, 2:]).max(axis=(0, 1))
        print(
            f"Arm motion over the lap: up to {arm.max():.2f} rad "
            f"({results['joint_names'][2 + int(arm.argmax())]})"
        )

    if not results.get("trajectories"):
        return 0 if results["success"] else 1

    from tesseract_robotics.viewer import (
        TesseractViewer,
        Trajectory,
    )

    print("\n=== Viewer ===")
    print("Robot: 8-DOF gantry (2 linear + 6 rotational)")
    print("Person: walks up to the arm and back; the robot sweeps along the plan of each tick")

    env = results["robot"].env
    scene.style_for_viewer(env)
    walk = Trajectory.from_rows(recording_joint_names(results), recording_rows(results))

    with TesseractViewer() as viewer:
        viewer.update_environment(env)
        viewer.update_trajectory(walk)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")

    return 0 if results["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
