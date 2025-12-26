"""
Online Planning Example

Demonstrates trajectory replanning as the environment changes.
Uses TrajOpt for trajectory optimization with collision avoidance.

Based on: tesseract_planning/tesseract_examples/src/online_planning_example.cpp

Robot Setup:
- Gantry (2 prismatic axes) + ABB IRB2400 (6 revolute joints) = 8 DOF manipulator
- Human obstacle represented as joints (human_x_joint, human_y_joint)
- Target: gantry_axis_1=5.5, gantry_axis_2=3.0, arm at home position

Key Features:
- Trajectory replanning as human obstacle moves
- Uses TrajOpt for optimization
- Collision avoidance with moving human

Note: The C++ version uses low-level trajopt_sqp with stepSQPSolver() for true
real-time planning. This Python version uses the TaskComposer pipeline.
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

    Unlike the KUKA version which moves a separate collision object,
    the C++ example represents the human as joints in the model.
    """
    # Get current state and update human joints
    current_state = robot.env.getState()
    joint_names = ["human_x_joint", "human_y_joint"]
    joint_values = np.array([human_x, human_y])
    robot.env.setState(joint_names, joint_values)


def run(pipeline="TrajOptPipeline", num_iterations=5, num_planners=None):
    """Run online planning example with moving human obstacle.

    Args:
        pipeline: Planning pipeline to use (default: TrajOptPipeline)
        num_iterations: Number of replanning iterations
        num_planners: Number of parallel planners (unused for TrajOptIfopt)

    Returns:
        dict with results
    """
    # Load robot - use the dedicated online_planning_example model
    # This is a gantry (2 prismatic) + ABB IRB2400 (6 revolute) = 8 DOF
    robot = Robot.from_tesseract_support("online_planning_example")

    # Get manipulator joint names (8 DOF: gantry + arm)
    # Note: human_x_joint and human_y_joint are in the model but not part of manipulator
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Start position - from C++ example
    start_position = np.zeros(len(joint_names))
    robot.set_joints(start_position, joint_names=joint_names)

    # Target position - from C++ example line 169:
    # target_joint_position << 5.5, 3, 0, 0, 0, 0, 0, 0;
    target_position = np.array([5.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Create profiles for TrajOpt
    profiles = create_trajopt_default_profiles()

    # Create task composer
    composer = TaskComposer.from_config()

    trajectories = []
    timings = []

    for iteration in range(num_iterations):
        # Move the human obstacle to simulate dynamic environment
        # Note: Large movements cause goal collisions. Use small oscillations.
        t = iteration * 0.2
        human_x = 0.2 * np.sin(t)  # Limited oscillation - fails at human_x >= 0.22
        human_y = 0.0
        update_human_position(robot, human_x, human_y)

        # Get current state as start
        state = robot.get_state(joint_names)
        current_joints = state.joint_positions

        # Create motion program: current -> target
        program = (MotionProgram("manipulator", tcp_frame="tool0", profile="DEFAULT")
            .set_joint_names(joint_names)
            .move_to(StateTarget(current_joints, names=joint_names, profile="DEFAULT"))
            .move_to(StateTarget(target_position, names=joint_names, profile="DEFAULT"))
        )

        # Plan with timing
        start_time = time.time()
        try:
            result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
            planning_time = time.time() - start_time

            if result.successful:
                trajectories.append(result)
                timings.append(planning_time)
                print(f"Iteration {iteration + 1}: Success in {planning_time:.3f}s, "
                      f"{len(result)} waypoints, human_x={human_x:.3f}")
                # Note: In C++, the robot would execute while replanning.
                # Here we just replan from start each time as human moves.
            else:
                print(f"Iteration {iteration + 1}: Planning failed - {result.message}")
                timings.append(planning_time)
        except Exception as e:
            planning_time = time.time() - start_time
            timings.append(planning_time)
            print(f"Iteration {iteration + 1}: Exception - {e}")

    # Summary statistics
    if timings:
        avg_time = np.mean(timings)
        print(f"\nOnline Planning Summary:")
        print(f"  Iterations: {num_iterations}")
        print(f"  Successful: {len(trajectories)}")
        print(f"  Average planning time: {avg_time:.3f}s ({1.0/avg_time if avg_time > 0 else 0:.1f} Hz)")

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
