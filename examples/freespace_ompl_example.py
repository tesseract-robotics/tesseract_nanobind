"""
Freespace OMPL Planning Example (High-Level API)

Demonstrates joint-space freespace motion planning using the OMPL RRTConnect algorithm.
The robot (KUKA IIWA 7-DOF) plans a collision-free path around a sphere obstacle.

Pipeline Overview:
    1. Load KUKA IIWA robot from tesseract_support
    2. Add sphere obstacle at (0.5, 0, 0.55) with radius 0.15m
    3. Define start/end joint configurations (only joint_a1 changes: -0.4 to 0.4 rad)
    4. Execute FreespacePipeline: OMPL RRTConnect -> interpolation -> time parameterization
    5. Return collision-free joint trajectory

Key Concepts:
    - OMPL RRTConnect: Bidirectional rapidly-exploring random tree algorithm.
      Grows trees from both start and goal, efficient for high-DOF spaces.
    - FreespacePipeline: OMPL planning with post-processing (interpolation + timing)
    - JointTarget: Specifies goal as joint configuration (vs CartesianTarget for poses)
    - Joint-space planning: No Cartesian path constraints, only collision avoidance

Motion Types:
    - FREESPACE: Any collision-free path allowed (joint-space interpolation)
    - Uses JointTarget for both start and goal waypoints

C++ Source: tesseract_planning/tesseract_examples/src/freespace_ompl_example.cpp

C++ Parameters (verified):
    - Robot: KUKA LBR IIWA 14 R820 (7-DOF)
    - Obstacle: Sphere radius=0.15m at (0.5, 0, 0.55) attached to base_link
    - Start: joint_a1=-0.4 rad, others=[0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
    - End: joint_a1=0.4 rad (same other joints)
    - Planner: RRTConnect with configurable range and planning_time

Related Examples:
    - freespace_hybrid_example.py - OMPL + TrajOpt smoothing
    - basic_cartesian_example.py - TrajOpt for Cartesian paths
    - lowlevel/freespace_ompl_c_api_example.py - Same with low-level API
"""

import sys

import numpy as np

from tesseract_robotics.planning import (
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    TaskComposer,
    create_obstacle,
    sphere,
)
from tesseract_robotics.planning.profiles import (
    create_freespace_pipeline_profiles,
    create_trajopt_default_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


def run(pipeline="FreespacePipeline", num_planners=None):
    """Run freespace OMPL planning example.

    Plans a collision-free joint-space trajectory around a sphere obstacle using
    OMPL's RRTConnect algorithm. Only joint_a1 changes (-0.4 to 0.4 rad), creating
    a sweeping motion that must avoid the sphere.

    Args:
        pipeline: Planning pipeline to use. Options:
            - "FreespacePipeline" (default): OMPL RRTConnect + TrajOpt smoothing
            - "TrajOptPipeline": TrajOpt only (for comparison)
        num_planners: Number of parallel OMPL planners (default: all CPUs).
            More planners = faster solutions but higher CPU usage.

    Returns:
        dict with keys:
            - result: PlanningResult with trajectory and success status
            - robot: Robot instance with environment state
            - joint_names: List of 7 KUKA IIWA joint names
    """
    # Load KUKA IIWA 7-DOF robot from tesseract_support package
    # Joints: joint_a1 through joint_a7
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle for collision avoidance
    # C++ source: createSphere(0.15) at Translation3d(0.5, 0, 0.55)
    # Positioned in front of robot to force non-trivial path
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.15),  # 15cm radius
        transform=Pose.from_xyz(0.5, 0, 0.55),  # In front of robot base
    )
    print("Added sphere obstacle at (0.5, 0, 0.55)")

    # Get joint names for "manipulator" planning group (defined in SRDF)
    joint_names = robot.get_joint_names("manipulator")

    # Start/end configurations from C++ example
    # Only joint_a1 changes (-0.4 -> 0.4 rad = ~46 degrees)
    # This creates a base rotation that must navigate around the sphere
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial robot state - required before planning for IK seeding
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # Build motion program with JointTarget waypoints
    # FREESPACE motion type = joint-space planning, no Cartesian constraints
    # The planner finds ANY collision-free path (not necessarily shortest)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(joint_start_pos))  # Start configuration
        .move_to(JointTarget(joint_end_pos))  # Goal configuration
    )
    print(f"\nCreated program with {len(program)} waypoints")

    # Execute planning pipeline via TaskComposer
    # TrajOptPipeline: TrajOpt -> contact check -> time parameterization
    print(f"\nRunning planner ({pipeline})...")
    composer = TaskComposer.from_config()

    # Use appropriate profiles based on pipeline
    # C++ defaults: planning_time=60.0, planner_range=0.01, 2 planners
    if "OMPL" in pipeline or "Freespace" in pipeline:
        profiles = create_freespace_pipeline_profiles(
            num_planners=num_planners or 2,  # C++ uses 2
            planning_time=60.0,
            planner_range=0.01,  # C++ uses 0.01
        )
    else:
        profiles = create_trajopt_default_profiles()

    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

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


if __name__ == "__main__":
    main()
