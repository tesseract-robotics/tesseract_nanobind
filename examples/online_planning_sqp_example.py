"""
Online Planning with Low-Level SQP API
======================================

Demonstrates real-time trajectory replanning using the low-level SQP solver.
This achieves 10-100+ Hz replanning rates, matching the C++ implementation.

C++ Reference:
    tesseract_planning/tesseract_examples/src/online_planning_example.cpp

Key Difference from online_planning_example.py
----------------------------------------------
- Uses TrustRegionSQPSolver.stepSQPSolver() for incremental optimization
- Builds the optimization problem directly (no TaskComposer)
- Warm-starts from previous solution
- Achieves real-time rates suitable for robot control

Collision Checking Modes
------------------------
- Discrete: Single timestep collision check (SingleTimestepCollisionEvaluator)
- Continuous: LVS (longest valid segment) between timesteps (LVSDiscreteCollisionEvaluator)

Architecture
------------
1. Build IFOPT problem with:
   - JointPosition variables for each waypoint
   - JointPosConstraint for start position
   - CartPosConstraint for target pose
   - JointVelConstraint for smoothness
   - CollisionConstraint for safety (discrete or continuous)

2. Create TrustRegionSQPSolver with OSQPEigenSolver

3. Loop:
   - Update obstacle position in environment
   - stepSQPSolver() for single SQP iteration
   - Extract trajectory from results
"""

import sys
import time

import numpy as np

# Low-level SQP imports
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_common import Isometry3d

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def build_optimization_problem(
    robot, joint_names, start_pos, target_pos, steps=10, use_continuous_collision=True
):
    """Build the IFOPT optimization problem.

    This manually constructs the trajectory optimization problem using:
    - JointPosition variables for each waypoint
    - Constraints: start position, target pose, velocity, collision

    Args:
        robot: Robot instance with environment
        joint_names: List of joint names
        start_pos: Starting joint positions
        target_pos: Target joint positions
        steps: Number of trajectory waypoints
        use_continuous_collision: If True, use LVS continuous collision (matches C++).
                                  If False, use discrete single-timestep collision.

    Returns:
        dict with problem and all objects that must stay alive
    """
    # Get kinematic group and joint limits
    manip = robot.env.getKinematicGroup("manipulator")
    joint_limits = manip.getLimits().joint_limits

    # Interpolate initial trajectory
    initial_states = ti.interpolate(start_pos, target_pos, steps)

    # Create QP problem
    problem = tsqp.IfoptQPProblem()

    # Add joint position variables for each timestep
    vars_list = []
    for i, state in enumerate(initial_states):
        var = ti.JointPosition(state, joint_names, f"Joint_Position_{i}")
        var.SetBounds(joint_limits)
        vars_list.append(var)
        problem.addVariableSet(var)

    # Add start position constraint (first waypoint = current position)
    home_coeffs = np.ones(len(joint_names)) * 5.0
    home_constraint = ti.JointPosConstraint(start_pos, [vars_list[0]], home_coeffs, "Home_Position")
    problem.addConstraintSet(home_constraint)

    # Add target pose constraint (last waypoint = target in Cartesian space)
    target_tf = manip.calcFwdKin(target_pos)["tool0"]
    cart_info = ti.CartPosInfo()
    cart_info.manip = manip
    cart_info.source_frame = "tool0"
    cart_info.target_frame = "world"
    cart_info.source_frame_offset = Isometry3d.Identity()
    cart_info.target_frame_offset = target_tf
    cart_info.type = ti.CartPosInfoType.TARGET_ACTIVE
    cart_info.indices = np.array([0, 1, 2, 3, 4, 5], dtype=np.int32)  # Full 6-DOF

    target_constraint = ti.CartPosConstraint(cart_info, vars_list[-1], "Target_Pose")
    problem.addConstraintSet(target_constraint)

    # Add velocity cost (smooth motion)
    vel_target = np.zeros(len(joint_names))
    vel_constraint = ti.JointVelConstraint(vel_target, vars_list, np.ones(1), "JointVelocity")
    problem.addCostSet(vel_constraint, tsqp.CostPenaltyType.SQUARED)

    # Add collision constraints
    # IMPORTANT: Keep references to collision_config and collision_cache alive
    # as the C++ side holds shared_ptr to them
    margin = 0.1  # 10cm safety margin
    margin_coeff = 10.0
    collision_config = ti.TrajOptCollisionConfig(margin, margin_coeff)
    collision_config.collision_margin_buffer = 0.10  # As in C++ example
    collision_cache = ti.CollisionCache(steps)
    collision_evaluators = []
    collision_constraints = []

    if use_continuous_collision:
        # Use LVS (longest valid segment) collision checking between consecutive states
        # This matches the C++ online_planning_example.cpp implementation
        for i in range(1, steps):
            # LVSDiscreteCollisionEvaluator checks collision at interpolated points
            collision_evaluator = ti.LVSDiscreteCollisionEvaluator(
                collision_cache,
                manip,
                robot.env,
                collision_config,
                True,  # dynamic_environment
            )
            # ContinuousCollisionConstraint takes two consecutive JointPosition variables
            collision_constraint = ti.ContinuousCollisionConstraint(
                collision_evaluator,
                vars_list[i - 1],  # position_var0
                vars_list[i],  # position_var1
                False,  # fixed0
                False,  # fixed1
                1,  # max_num_cnt
                False,  # fixed_sparsity
                f"LVSCollision_{i - 1}_{i}",
            )
            problem.addConstraintSet(collision_constraint)
            collision_evaluators.append(collision_evaluator)
            collision_constraints.append(collision_constraint)
    else:
        # Use discrete single-timestep collision checking
        for i in range(1, steps):
            collision_evaluator = ti.SingleTimestepCollisionEvaluator(
                collision_cache,
                manip,
                robot.env,
                collision_config,
                True,  # dynamic_environment
            )
            collision_constraint = ti.DiscreteCollisionConstraint(
                collision_evaluator, vars_list[i], 1, False, f"Collision_{i}"
            )
            problem.addConstraintSet(collision_constraint)
            collision_evaluators.append(collision_evaluator)
            collision_constraints.append(collision_constraint)

    # Setup the problem (must be called after adding all sets)
    problem.setup()

    # Return dict with all objects that must stay alive for the problem to work
    return {
        "problem": problem,
        "vars_list": vars_list,
        "target_constraint": target_constraint,
        "home_constraint": home_constraint,
        "vel_constraint": vel_constraint,
        "collision_config": collision_config,
        "collision_cache": collision_cache,
        "collision_evaluators": collision_evaluators,
        "collision_constraints": collision_constraints,
        "cart_info": cart_info,
        "manip": manip,
    }


def update_human_position(robot, human_x, human_y):
    """Update human obstacle position via joint values."""
    robot.env.setState({"human_x_joint": human_x, "human_y_joint": human_y})


def run(steps=12, verbose=False, use_continuous_collision=False):
    """Run trajectory optimization with low-level SQP API.

    Args:
        steps: Trajectory waypoints per plan (C++ default: 12)
        verbose: Print debug info
        use_continuous_collision: Use LVS continuous collision (True) or discrete (False)
                                  C++ default: False (discrete collision)

    Returns:
        dict with results
    """
    # Load robot model
    robot = Robot.from_tesseract_support("online_planning_example")
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Define start and target
    start_pos = np.zeros(len(joint_names))
    target_pos = np.array([5.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    robot.set_joints(start_pos, joint_names=joint_names)

    # Build initial problem
    collision_mode = "LVS continuous" if use_continuous_collision else "discrete"
    print(f"Building optimization problem (collision: {collision_mode})...")
    problem_data = build_optimization_problem(
        robot, joint_names, start_pos, target_pos, steps, use_continuous_collision
    )
    problem = problem_data["problem"]
    problem.print()

    # Create SQP solver with OSQP
    # Match C++ defaults: box_size=0.01
    box_size = 0.01
    qp_solver = tsqp.OSQPEigenSolver()
    solver = tsqp.TrustRegionSQPSolver(qp_solver)
    solver.verbose = verbose
    solver.params.initial_trust_box_size = box_size

    # Initial global solve
    print("Running initial global solve...")
    t0 = time.perf_counter()
    solver.solve(problem)
    solve_time = time.perf_counter() - t0
    x = solver.getResults().best_var_vals
    cost = problem.evaluateTotalExactCost(x)
    print(f"Initial solve: cost={cost:.4f}, time={solve_time * 1000:.1f}ms")

    trajectories = [x.copy()]
    timings = [solve_time]

    # Online replanning loop
    # Key insight from C++: rebuild problem each iteration with warm-start trajectory
    num_replan = 10
    print(f"\nOnline replanning ({num_replan} iterations)...")

    n_joints = len(joint_names)

    for iteration in range(num_replan):
        # Move obstacle
        human_x = 0.5 + 0.3 * np.sin(iteration * 0.3)
        update_human_position(robot, human_x, 0.0)

        # Extract joint values from solution (first steps*n_joints values, rest are slack)
        joint_vals = x[: steps * n_joints]
        traj = joint_vals.reshape(steps, n_joints)

        # Rebuild problem with warm-start (like C++ setupProblem)
        t0 = time.perf_counter()
        problem_data = build_optimization_problem(
            robot,
            joint_names,
            traj[0],  # Current start from trajectory
            target_pos,
            steps,
            use_continuous_collision,
        )
        problem = problem_data["problem"]

        # Set warm-start values
        for i, var in enumerate(problem_data["vars_list"]):
            var.SetVariables(traj[i])

        # Single SQP step (incremental optimization)
        solver.init(problem)
        solver.stepSQPSolver()

        # Reset box size (like C++ - trust region loop can shrink it to zero)
        solver.setBoxSize(box_size)

        x = solver.getResults().best_var_vals
        dt = time.perf_counter() - t0
        timings.append(dt)
        trajectories.append(x.copy())

        if verbose:
            cost = problem.evaluateTotalExactCost(x)
            print(f"  Iter {iteration + 1}: cost={cost:.4f}, time={dt * 1000:.1f}ms")

    avg_time = np.mean(timings[1:]) * 1000
    print(f"Replan timing: avg={avg_time:.1f}ms ({1000 / avg_time:.0f} Hz)")
    print(f"Trajectory: {steps} waypoints x {len(joint_names)} joints")

    return {
        "trajectories": trajectories,
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": len(trajectories) > 0,
    }


def main():
    results = run(verbose=False)  # Set True for solver iteration details

    # Print trajectory comparison info
    if results.get("trajectories"):
        n_joints = len(results["joint_names"])
        steps = 12  # C++ default
        robot = results["robot"]
        manip = robot.env.getKinematicGroup("manipulator")

        print("\n=== Trajectory Results ===")

        # Target pose (C++ reference)
        target_joints = np.array([5.5, 3.0, 0, 0, 0, 0, 0, 0])
        target_tf = manip.calcFwdKin(target_joints)["tool0"]
        target_pos = target_tf.translation()

        # Initial solve
        traj0 = results["trajectories"][0][: steps * n_joints].reshape(steps, n_joints)
        initial_tf = manip.calcFwdKin(traj0[-1])["tool0"]
        initial_pos = initial_tf.translation()

        # Final trajectory (after replanning)
        traj_final = results["trajectories"][-1][: steps * n_joints].reshape(steps, n_joints)
        final_tf = manip.calcFwdKin(traj_final[-1])["tool0"]
        final_pos = final_tf.translation()

        print(f"Target TCP:  {target_pos}")
        print(
            f"Initial TCP: {initial_pos} (error: {np.linalg.norm(initial_pos - target_pos):.3f}m)"
        )
        print(f"Final TCP:   {final_pos} (error: {np.linalg.norm(final_pos - target_pos):.3f}m)")

    if TesseractViewer is not None and results.get("trajectories"):
        print("\n=== Starting Viewer ===")
        print("Robot: 8-DOF gantry (2 linear + 6 rotational)")
        print("Human: red cylinder at FINAL position (static in viewer)")
        print("Note: Human moved during optimization but trajectory only animates robot")
        print("URL: http://localhost:8000")

        # Modify visuals for better viewing
        env = results["robot"].env
        scene = env.getSceneGraph()

        # Make human obstacle visible (URDF has 5% opacity by design)
        human_link = scene.getLink("human_estop_zone")
        for v in human_link.visual:
            if v.material:
                v.material.color = np.array([0.8, 0.0, 0.0, 0.7])  # 70% opaque

        # Remove robot_estop_zone visual (large cylinder obscures robot)
        robot_zone = scene.getLink("robot_estop_zone")
        robot_zone.visual.clear()

        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])

        # Convert trajectory to viewer format: [joint_vals..., timestamp]
        n_joints = len(results["joint_names"])
        steps = 12  # C++ default
        traj_final = results["trajectories"][-1][: steps * n_joints].reshape(steps, n_joints)
        dt = 0.1  # 100ms between waypoints
        trajectory_list = []
        for i, wp in enumerate(traj_final):
            # Append timestamp as last element, convert to Python list for JSON
            row = wp.tolist() + [i * dt]
            trajectory_list.append(row)
        viewer.update_trajectory_list(results["joint_names"], trajectory_list)

        viewer.start_serve_background()

        print("\nPress Enter to exit...")
        input()

    return results["success"]


if __name__ == "__main__":
    main()
