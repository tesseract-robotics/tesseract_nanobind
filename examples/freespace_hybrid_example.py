"""
Freespace Hybrid Example - OMPL + TrajOpt Hybrid Planning

Demonstrates hybrid motion planning combining OMPL (path finding) with
TrajOpt or TrajOptIfopt (trajectory optimization).

Pipeline Overview:
1. OMPL Phase: RRTConnect bidirectional search for collision-free path
2. TrajOpt Phase (optional): Refines path for smoothness, adds velocity/jerk limits
3. Post-processing: Time parameterization

Key Concepts:
- OMPL RRTConnect: Bidirectional rapidly-exploring random tree, grows from both ends
- Hybrid planning: OMPL finds feasible path, TrajOpt optimizes it
- FreespaceIfoptPipeline: Adds TrajOptIfopt smoothing with velocity/acceleration/jerk costs
- Collision margin: 0.025m safety buffer for collision checking

Pipelines Available:
- FreespacePipeline: OMPL only (fast but less smooth)
- FreespaceIfoptPipeline: OMPL + TrajOptIfopt (smoother trajectories)

C++ Source: tesseract_examples/src/freespace_hybrid_example.cpp

Related Examples:
- freespace_ompl_example.py - pure OMPL planning
- basic_cartesian_example.py - pure TrajOpt planning
"""

import sys
import time

import numpy as np

from tesseract_robotics.planning import (
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    create_obstacle,
    sphere,
)
from tesseract_robotics.planning.profiles import (
    create_ompl_default_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


def run(pipeline="FreespacePipeline", use_ifopt=False, num_planners=None):
    """Run freespace hybrid planning example.

    Args:
        pipeline: "FreespacePipeline" (OMPL only) or custom
        use_ifopt: If True, use "FreespaceIfoptPipeline" (OMPL + TrajOptIfopt smoothing)
        num_planners: Number of parallel planners (unused, for API compat)

    Returns:
        dict with results including trajectory, robot, and timing
    """
    # KUKA LBR IIWA 14 R820: 7-DOF collaborative arm
    # Uses KDL (numeric) IK solver - slower than OPW but works for 7-DOF
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # Add sphere obstacle at (0.5, 0, 0.55) with radius 0.15m
    # This blocks the direct path, forcing OMPL to find collision-free route
    create_obstacle(
        robot,
        "sphere_attached",
        sphere(0.15),  # 15cm radius sphere
        Pose.from_xyz(0.5, 0.0, 0.55),  # Positioned in robot workspace
    )

    # Get joint names from "manipulator" kinematic group
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Start and end joint configurations from C++ example
    # Motion: swing joint 1 from -0.4 rad to +0.4 rad (others fixed)
    # Path must curve around sphere obstacle
    joint_start = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    robot.set_joints(joint_start, joint_names=joint_names)

    # Simple joint-to-joint motion program
    # FREESPACE profile uses OMPL (RRTConnect by default)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="FREESPACE")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(joint_end, names=joint_names, profile="FREESPACE"))
    )

    # OMPL profiles configure:
    # - Planner: RRTConnect (bidirectional, fast for freespace)
    # - Range: step size for tree extension
    # - Collision margin: 0.025m safety buffer (from C++ example)
    profiles = create_ompl_default_profiles()

    # TaskComposer manages pipeline execution
    composer = TaskComposer.from_config()

    # Select pipeline: OMPL-only or OMPL+TrajOptIfopt
    # TrajOptIfopt adds smoothing with coeff=1 for velocity cost
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
