"""
Freespace OMPL Planning Example

Demonstrates freespace motion planning using OMPL RRTConnect algorithm with the
KUKA IIWA 7-DOF robot. Plans a collision-free path around a sphere obstacle.

Pipeline Overview:
1. Load robot environment (KUKA IIWA)
2. Add sphere obstacle at (0.5, 0, 0.55) with radius 0.15m
3. Define start/end joint configurations (only joint_a1 changes from -0.4 to 0.4 rad)
4. Execute FreespacePipeline (OMPL RRTConnect bidirectional search)
5. Post-process and return collision-free trajectory

Key Concepts:
- OMPL uses RRTConnect: bidirectional rapidly-exploring random tree algorithm
- FreespacePipeline: OMPL planning → interpolation → time parameterization
- Joint-space planning: no Cartesian constraints, just collision avoidance

C++ Source: tesseract_examples/src/freespace_ompl_example.cpp

Related Examples:
- freespace_hybrid_example.py - adds TrajOpt smoothing after OMPL
- basic_cartesian_example.py - uses TrajOpt for Cartesian paths
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    JointTarget,
    Pose,
    sphere,
    create_obstacle,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run():
    """Run freespace OMPL planning example.

    Returns:
        dict with result, robot, joint_names
    """
    # Load KUKA IIWA 7-DOF robot from tesseract_support package
    # This robot has joints: joint_a1 through joint_a7
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle for collision checking during motion planning
    # C++ uses: sphere at (0.5, 0, 0.55), radius=0.15m, attached to base_link
    # This forces the planner to find a path around the obstacle
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.15),  # 15cm radius sphere
        transform=Pose.from_xyz(0.5, 0, 0.55),  # In front of robot base
    )
    print("Added sphere obstacle at (0.5, 0, 0.55)")

    # Get joint names for the "manipulator" planning group defined in SRDF
    joint_names = robot.get_joint_names("manipulator")

    # Define start and end joint configurations (radians)
    # Note: Only joint_a1 changes (-0.4 → 0.4 rad), all others remain constant
    # This creates a sweeping motion that must avoid the sphere obstacle
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial robot state - required before planning so IK solver knows starting config
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # Create motion program with FREESPACE motion type (default for JointTarget)
    # FREESPACE = joint-space planning, no Cartesian path constraints
    # The planner will find any collision-free path between configurations
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(joint_start_pos))  # Start state
        .move_to(JointTarget(joint_end_pos))  # Goal state
    )
    print(f"\nCreated program with {len(program)} waypoints")

    # Execute FreespacePipeline via TaskComposer
    # Pipeline: OMPL RRTConnect → interpolation → time parameterization
    # RRTConnect uses bidirectional search: grows trees from both start and goal
    # C++ defaults: planning_time=60.0, planner_range=0.01, 2 planners
    print("\nRunning OMPL planner (FreespacePipeline)...")
    composer = TaskComposer.from_config()
    profiles = create_freespace_pipeline_profiles(
        num_planners=2,
        planning_time=60.0,
        planner_range=0.01,
    )
    result = composer.plan(
        robot, program, pipeline="FreespacePipeline", profiles=profiles
    )

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! {len(result)} waypoints")

    return {"result": result, "robot": robot, "joint_names": joint_names}


def main():
    results = run()

    if TesseractViewer is not None and results["result"].raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
