"""
Chain Example

Demonstrates chained motion planning using Descartes + TrajOpt:
1. FREESPACE: Start position to first raster
2. LINEAR: Raster pattern (3 passes)
3. FREESPACE: Back to start

Descartes samples IK solutions at each Cartesian waypoint and finds
optimal path through ladder graph. TrajOpt refines for smoothness.

Based on: tesseract_motion_planners/examples/chain_example.cpp

Robot Setup:
- ABB IRB2400 6-DOF manipulator
- OPW inverse kinematics solver
- Raster passes at z=0.8, x=[0.8, 0.9, 1.0]
"""

import sys
import time
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    StateTarget,
    TaskComposer,
    Pose,
)
from tesseract_robotics.planning.profiles import (
    create_cartesian_pipeline_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="CartesianPipeline", num_planners=None):
    """Run chain planning example.

    Args:
        pipeline: Pipeline name (default "CartesianPipeline")
        num_planners: Number of parallel planners (unused, for API compat)

    Returns:
        dict with results
    """
    # Load ABB IRB2400 robot
    robot = Robot.from_tesseract_support("abb_irb2400")

    # Joint configuration
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # Start position (all zeros)
    joint_start = np.zeros(6)
    robot.set_joints(joint_start, joint_names=joint_names)

    # Define raster waypoints (from C++ example)
    # C++ Quaterniond(0, 0, -1.0, 0) = (w, x, y, z) = facing down
    # Python from_xyz_quat uses (x, y, z, w) format
    quat_down = (0.0, -1.0, 0.0, 0.0)  # Tool pointing down

    # Raster 1: x=0.8
    wp1 = Pose.from_xyz_quat(0.8, -0.2, 0.8, *quat_down)
    wp2 = Pose.from_xyz_quat(0.8, 0.2, 0.8, *quat_down)

    # Raster 2: x=0.9
    wp3 = Pose.from_xyz_quat(0.9, -0.2, 0.8, *quat_down)
    wp4 = Pose.from_xyz_quat(0.9, 0.2, 0.8, *quat_down)

    # Raster 3: x=1.0
    wp5 = Pose.from_xyz_quat(1.0, -0.2, 0.8, *quat_down)
    wp6 = Pose.from_xyz_quat(1.0, 0.2, 0.8, *quat_down)

    # Create motion program
    # - FREESPACE to first waypoint
    # - LINEAR through raster pattern
    # - FREESPACE back to start
    program = (MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        # Start from current joint state
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
        # FREESPACE to first raster point
        .move_to(CartesianTarget(wp1, profile="FREESPACE"))
        # LINEAR through raster pattern
        .move_to(CartesianTarget(wp2, profile="CARTESIAN"))
        .move_to(CartesianTarget(wp3, profile="CARTESIAN"))
        .move_to(CartesianTarget(wp4, profile="CARTESIAN"))
        .move_to(CartesianTarget(wp5, profile="CARTESIAN"))
        .move_to(CartesianTarget(wp6, profile="CARTESIAN"))
        # FREESPACE back to start
        .move_to(StateTarget(joint_start, names=joint_names, profile="FREESPACE"))
    )

    # Create profiles for CartesianPipeline (Descartes + TrajOpt)
    profiles = create_cartesian_pipeline_profiles()

    # Create task composer
    composer = TaskComposer.from_config()

    # Plan
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
