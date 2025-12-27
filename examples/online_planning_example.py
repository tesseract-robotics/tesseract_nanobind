"""
Online Planning Example
=======================

Demonstrates real-time trajectory replanning as a dynamic obstacle moves.

C++ Reference:
    tesseract_planning/tesseract_examples/src/online_planning_example.cpp

Overview
--------
This example simulates an industrial scenario where a robot must replan its
trajectory as a human worker moves through the workspace. The environment
changes between planning iterations, requiring continuous collision avoidance.

Robot Configuration
-------------------
The model combines:
- **Gantry**: 2 prismatic (linear) axes for XY positioning
- **ABB IRB2400**: 6-axis industrial arm mounted on gantry
- **Total**: 8 DOF manipulator (gantry_axis_1, gantry_axis_2, joint_1..joint_6)

The human obstacle is modeled as joints (human_x_joint, human_y_joint) in the
URDF, allowing its position to be updated via setState() rather than requiring
full environment modifications.

Key Concepts
------------
**Online/Real-Time Planning**:
    Planning while the environment changes. The planner must complete before
    the robot reaches the end of its current trajectory, else motion stalls.
    C++ version achieves 10-100 Hz replanning; Python version is slower due
    to TaskComposer overhead.

**Dynamic Obstacles**:
    Obstacles that move between planning cycles. Represented here as joints
    so position updates are efficient (no collision manager rebuilding).

**Warm Starting**:
    C++ version uses previous solution as initial guess for next planning cycle.
    This significantly speeds convergence when obstacles move slowly.

**TrajOpt SQP Solver**:
    The C++ version uses stepSQPSolver() for incremental optimization:
    - OSQP as the QP sub-solver
    - Trust region method for robust convergence
    - Can be interrupted if environment changes mid-solve

**Planning Rate vs Obstacle Velocity**:
    If obstacles move faster than the planner can track, the solver may get
    stuck in infeasible regions. Rule of thumb: replanning rate should be
    5-10x faster than obstacle motion in joint space.

Pipeline Comparison
-------------------
**C++ (Real-Time)**:
    1. Initialize TrustRegionSQPSolver with OSQP
    2. Loop: update obstacle -> stepSQPSolver() -> execute trajectory slice
    3. Warm-start from previous solution

**Python (This Example)**:
    1. Create TaskComposer with TrajOptPipeline
    2. Loop: update obstacle -> full planning pipeline -> log results
    3. No warm-start (each plan is independent)

The Python version demonstrates the concept but is not suitable for true
real-time control due to higher latency.

Collision Configuration
-----------------------
From C++ source:
- Collision margin: 0.1m (10cm safety buffer)
- Margin coefficient: 10 (cost weight for collision term)
- Discrete evaluator (vs continuous)

Related Examples
----------------
- freespace_ompl_example.py: Static environment planning with OMPL
- basic_cartesian_example.py: TrajOpt for Cartesian paths
- car_seat_example.py: Multi-step planning with profile customization
"""

import sys
import time
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def update_human_position(robot, human_x, human_y):
    """Update human obstacle position by changing joint values.

    The human obstacle is modeled as a kinematic chain in the URDF with
    prismatic joints (human_x_joint, human_y_joint). This allows efficient
    position updates without rebuilding the collision manager.

    Alternative approaches (not used here):
    - moveLink(): Move a link's origin (requires collision manager rebuild)
    - Add/remove links: Full environment modification (slowest)

    Args:
        robot: Robot instance with online_planning_example model
        human_x: X position of human obstacle in meters
        human_y: Y position of human obstacle in meters
    """
    # setState() efficiently updates joint values and recomputes transforms
    # The collision manager automatically uses the new positions
    joint_names = ["human_x_joint", "human_y_joint"]
    joint_values = np.array([human_x, human_y])
    robot.env.setState(joint_names, joint_values)


def run(pipeline="TrajOptPipeline", num_iterations=5, num_planners=None):
    """Run online planning example with moving human obstacle.

    Simulates repeated planning cycles as a human obstacle oscillates through
    the workspace. Measures planning time to characterize replanning rate.

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "TrajOptIfoptPipeline")
        num_iterations: Number of replanning cycles to run
        num_planners: Parallel planners (unused for TrajOpt, relevant for OMPL)

    Returns:
        dict with 'trajectories', 'timings', 'robot', 'joint_names', 'success'
    """
    # =========================================================================
    # STEP 1: Load Robot Model
    # =========================================================================
    # The online_planning_example model includes:
    # - Gantry: gantry_axis_1, gantry_axis_2 (prismatic, large travel)
    # - ABB IRB2400: joint_1..joint_6 (revolute, standard 6-axis arm)
    # - Human: human_x_joint, human_y_joint (prismatic, for obstacle motion)
    robot = Robot.from_tesseract_support("online_planning_example")

    # "manipulator" group contains only the 8 gantry+arm joints
    # Human joints are NOT part of the manipulator - they're environment state
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # =========================================================================
    # STEP 2: Define Start and Target States
    # =========================================================================
    # Start position: all zeros (gantry at origin, arm in home pose)
    start_position = np.zeros(len(joint_names))
    robot.set_joints(start_position, joint_names=joint_names)

    # Target position from C++ source line 169:
    # target_joint_position << 5.5, 3, 0, 0, 0, 0, 0, 0;
    # This moves the gantry to (5.5m, 3.0m) while keeping arm in home pose
    target_position = np.array([5.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # =========================================================================
    # STEP 3: Configure Planning Pipeline
    # =========================================================================
    # Default profiles provide reasonable TrajOpt settings:
    # - Collision margin and coefficients
    # - Joint velocity/acceleration limits
    # - Optimization parameters
    profiles = create_trajopt_default_profiles()

    # TaskComposer orchestrates the planning pipeline
    # TrajOptPipeline: interpolation -> TrajOpt optimization -> time parameterization
    composer = TaskComposer.from_config()

    trajectories = []
    timings = []

    # =========================================================================
    # STEP 4: Replanning Loop
    # =========================================================================
    for iteration in range(num_iterations):
        # --- Update Dynamic Obstacle ---
        # Oscillate human position using sine wave
        # IMPORTANT: Large displacements (>0.22m) can cause goal collisions
        # that make planning infeasible. Real systems need collision-free goals.
        t = iteration * 0.2
        human_x = 0.2 * np.sin(t)  # +/- 20cm oscillation
        human_y = 0.0
        update_human_position(robot, human_x, human_y)

        # --- Get Current Robot State ---
        # In a real system, this would come from robot encoders
        # Here we always start from the initial position (no execution)
        state = robot.get_state(joint_names)
        current_joints = state.joint_positions

        # --- Build Motion Program ---
        # StateTarget: plan in joint space (vs CartesianTarget for Cartesian)
        # First move_to sets start state, second sets goal
        program = (
            MotionProgram("manipulator", tcp_frame="tool0", profile="DEFAULT")
            .set_joint_names(joint_names)
            .move_to(StateTarget(current_joints, names=joint_names, profile="DEFAULT"))
            .move_to(StateTarget(target_position, names=joint_names, profile="DEFAULT"))
        )

        # --- Execute Planning with Timing ---
        start_time = time.time()
        try:
            result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
            planning_time = time.time() - start_time

            if result.successful:
                trajectories.append(result)
                timings.append(planning_time)
                print(
                    f"Iteration {iteration + 1}: Success in {planning_time:.3f}s, "
                    f"{len(result)} waypoints, human_x={human_x:.3f}"
                )
                # NOTE: In real online planning, the robot would execute the
                # first segment while planning the next. Here we just measure
                # planning time without actual execution.
            else:
                print(f"Iteration {iteration + 1}: Planning failed - {result.message}")
                timings.append(planning_time)
        except Exception as e:
            planning_time = time.time() - start_time
            timings.append(planning_time)
            print(f"Iteration {iteration + 1}: Exception - {e}")

    # =========================================================================
    # STEP 5: Report Statistics
    # =========================================================================
    # Planning rate determines how fast obstacles can move
    # For real-time: need >10 Hz replanning for typical industrial speeds
    if timings:
        avg_time = np.mean(timings)
        print("\nOnline Planning Summary:")
        print(f"  Iterations: {num_iterations}")
        print(f"  Successful: {len(trajectories)}")
        print(
            f"  Average planning time: {avg_time:.3f}s ({1.0 / avg_time if avg_time > 0 else 0:.1f} Hz)"
        )

    return {
        "trajectories": trajectories,
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": len(trajectories) > 0,
    }


def main():
    results = run()

    if TesseractViewer is not None and results.get("trajectories"):
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        # Show last trajectory
        if results["trajectories"][-1].raw_results is not None:
            viewer.update_trajectory(results["trajectories"][-1].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return results["success"]


if __name__ == "__main__":
    main()
