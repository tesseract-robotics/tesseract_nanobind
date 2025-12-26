"""
Freespace Hybrid Example

Demonstrates OMPL freespace planning with optional TrajOpt trajectory refinement.
Combines OMPL's sampling-based planning with trajectory optimization.

Based on: tesseract_planning/tesseract_examples/src/freespace_hybrid_example.cpp

Robot Setup:
- KUKA IIWA 7-DOF manipulator
- Sphere obstacle at (0.5, 0, 0.55) with radius 0.15
- Freespace motion from start to end joint configuration

Key Features:
- OMPL RRTConnect for path finding
- Optional TrajOpt/TrajOptIfopt for trajectory smoothing
- Configurable OMPL range and planning time
- Dynamic obstacle avoidance
"""

import sys
import time
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    StateTarget,
    TaskComposer,
    sphere,
    create_obstacle,
    Pose,
)
from tesseract_robotics.planning.profiles import (
    create_ompl_default_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="FreespacePipeline", use_ifopt=False, num_planners=None):
    """Run freespace hybrid planning example.

    Args:
        pipeline: Planning pipeline ("FreespacePipeline" or custom)
        use_ifopt: Use TrajOptIfopt for trajectory refinement
        num_planners: Number of parallel planners (unused)

    Returns:
        dict with results
    """
    # Load KUKA IIWA robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # Add sphere obstacle
    create_obstacle(
        robot,
        "sphere_attached",
        sphere(0.15),
        Pose.from_xyz(0.5, 0.0, 0.55)
    )

    # Joint configuration
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Start and end positions from C++ example
    joint_start = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    robot.set_joints(joint_start, joint_names=joint_names)

    # Create motion program
    program = (MotionProgram("manipulator", tcp_frame="tool0", profile="FREESPACE")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(joint_end, names=joint_names, profile="FREESPACE"))
    )

    # Create profiles - OMPL for freespace planning
    profiles = create_ompl_default_profiles()

    # Create task composer
    composer = TaskComposer.from_config()

    # Plan
    actual_pipeline = "FreespaceIfoptPipeline" if use_ifopt else pipeline
    print(f"Planning with {actual_pipeline}...")
    start_time = time.time()

    try:
        result = composer.plan(robot, program, pipeline=actual_pipeline, profiles=profiles)
        planning_time_actual = time.time() - start_time

        if result.successful:
            print(f"Success in {planning_time_actual:.2f}s, {len(result)} waypoints")
        else:
            print(f"Failed in {planning_time_actual:.2f}s: {result.message}")

    except Exception as e:
        planning_time_actual = time.time() - start_time
        print(f"Exception after {planning_time_actual:.2f}s: {e}")
        result = None

    return {
        "result": result,
        "robot": robot,
        "joint_names": joint_names,
        "planning_time": planning_time_actual,
        "success": result.successful if result else False,
    }


def main():
    results = run()

    if TesseractViewer is not None and results.get("result"):
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        if results["result"].raw_results is not None:
            viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return results["success"]


if __name__ == "__main__":
    main()
