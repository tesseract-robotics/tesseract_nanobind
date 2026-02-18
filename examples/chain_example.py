"""
Chain Example - Descartes + TrajOpt Chained Planning

Demonstrates chained motion planning using Descartes followed by TrajOpt:
1. FREESPACE: Home → first raster point (joint-space motion)
2. LINEAR: Raster pattern (3 passes, Cartesian straight-line paths)
3. FREESPACE: Back to home

Pipeline Overview:
1. Descartes: Samples IK solutions at each Cartesian waypoint, builds "ladder graph"
   of valid configurations, finds optimal path using Dijkstra/A* search
2. TrajOpt: Takes Descartes output, optimizes for smoothness, enforces constraints

Key Concepts:
- Descartes: Sampling-based Cartesian planner (fast for discrete waypoints)
- Ladder graph: Grid of valid IK solutions at each waypoint, connected by edges
- OPW IK solver: Fast closed-form 6-DOF inverse kinematics for industrial robots
- Chaining: Output of one planner becomes input to next planner

C++ Source: tesseract_motion_planners/examples/chain_example.cpp

Related Examples:
- basic_cartesian_example.py - single planner (TrajOpt only)
- puzzle_piece_example.py - complex Cartesian path from CSV
"""

import sys
import time

import numpy as np

from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import (
    create_cartesian_pipeline_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def run(pipeline="CartesianPipeline", num_planners=None):
    """Run chain planning example.

    Args:
        pipeline: Pipeline name (default "CartesianPipeline" = Descartes+TrajOpt)
        num_planners: Number of parallel planners (unused, for API compat)

    Returns:
        dict with results including trajectory, robot, and timing
    """
    # ABB IRB2400: 6-DOF industrial arm with OPW analytic IK solver
    # OPW = Ortho-Parallel Wrist kinematics (closed-form, ~100x faster than numeric)
    robot = Robot.from_tesseract_support("abb_irb2400")

    # Get joint names from "manipulator" kinematic group (defined in SRDF)
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Home position: all joints at 0 (typical industrial robot config)
    joint_start = np.zeros(6)
    robot.set_joints(joint_start, joint_names=joint_names)

    # Define raster waypoints (from C++ example)
    # C++ Quaterniond(0, 0, -1.0, 0) in (w,x,y,z) format = 180° rotation around Y-axis
    # Python from_xyz_quat uses (x, y, z, w) format → (0, -1.0, 0, 0)
    # This orientation points the tool straight down (Z-axis inverted)
    quat_down = (0.0, -1.0, 0.0, 0.0)  # Tool pointing down

    # Raster pattern: 3 parallel passes at different X positions
    # Each pass sweeps Y from -0.2m to +0.2m at constant Z=0.8m
    # Raster 1: x=0.8m
    wp1 = Pose.from_xyz_quat(0.8, -0.2, 0.8, *quat_down)  # Start of pass 1
    wp2 = Pose.from_xyz_quat(0.8, 0.2, 0.8, *quat_down)  # End of pass 1

    # Raster 2: x=0.9m (10cm step in X)
    wp3 = Pose.from_xyz_quat(0.9, -0.2, 0.8, *quat_down)
    wp4 = Pose.from_xyz_quat(0.9, 0.2, 0.8, *quat_down)

    # Raster 3: x=1.0m
    wp5 = Pose.from_xyz_quat(1.0, -0.2, 0.8, *quat_down)
    wp6 = Pose.from_xyz_quat(1.0, 0.2, 0.8, *quat_down)

    # Build motion program with FREESPACE and CARTESIAN segments
    # FREESPACE profile: uses OMPL sampling-based planner (handles obstacles)
    # CARTESIAN profile: uses Descartes ladder graph (optimal IK selection)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        # Segment 1: FREESPACE motion from home to first raster point
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
        .move_to(CartesianTarget(wp1, profile="FREESPACE"))
        # Segment 2: LINEAR raster pattern (Descartes optimizes IK redundancy)
        .move_to(CartesianTarget(wp2, profile="CARTESIAN"))  # Pass 1
        .move_to(CartesianTarget(wp3, profile="CARTESIAN"))  # Transition
        .move_to(CartesianTarget(wp4, profile="CARTESIAN"))  # Pass 2
        .move_to(CartesianTarget(wp5, profile="CARTESIAN"))  # Transition
        .move_to(CartesianTarget(wp6, profile="CARTESIAN"))  # Pass 3
        # Segment 3: FREESPACE return to home
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
    )

    # CartesianPipeline profiles configure:
    # - Descartes: IK sampling density, edge cost function
    # - TrajOpt: smoothing coefficients (coeff=1 for velocity), collision margin (0.025m)
    profiles = create_cartesian_pipeline_profiles()

    # TaskComposer loads pipeline from YAML config and manages execution
    composer = TaskComposer.from_config()

    # Execute planning - CartesianPipeline runs Descartes then TrajOpt
    print(f"Planning with {pipeline}...")
    start_time = time.time()

    try:
        result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
        planning_time = time.time() - start_time

        if result.successful:
            print(f"Success in {planning_time:.2f}s, {len(result)} waypoints")
        else:
            print(f"Failed in {planning_time:.2f}s: {result.message}")

    except Exception as e:
        planning_time = time.time() - start_time
        print(f"Exception after {planning_time:.2f}s: {e}")
        result = None

    return {
        "result": result,
        "robot": robot,
        "joint_names": joint_names,
        "planning_time": planning_time,
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
