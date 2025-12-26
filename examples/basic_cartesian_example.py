"""
Basic Cartesian Planning Example (High-Level API)

Demonstrates multi-phase Cartesian motion planning using TrajOpt trajectory optimization.
The robot (KUKA IIWA 7-DOF) executes a sequence of FREESPACE and LINEAR moves around an obstacle.

Pipeline Overview:
    1. Load KUKA IIWA robot from tesseract_support
    2. Add box obstacle at (1.0, 0, 0) - 50cm cube
    3. Create 4-phase motion program:
        - Phase 1: Start at known joint configuration
        - Phase 2: FREESPACE move to Cartesian waypoint 1 (0.5, -0.2, 0.62)
        - Phase 3: LINEAR move to Cartesian waypoint 2 (0.5, 0.3, 0.62)
        - Phase 4: FREESPACE return to start joint configuration
    4. Execute TrajOptPipeline: seed trajectory -> optimization -> time parameterization

Key Concepts:
    - TrajOpt: Trajectory optimizer minimizing costs (smoothness, collision) with constraints
    - FREESPACE motion: Any collision-free path to goal (joint-space interpolation)
    - LINEAR motion: Straight-line Cartesian path (tool pose interpolated along line)
    - Profile names: Control per-waypoint behavior ("freespace_profile", "RASTER")
    - StateTarget vs CartesianTarget: Joint config vs 6D pose goal specification

Motion Types:
    - move_to(CartesianTarget): FREESPACE motion to pose (any collision-free path)
    - linear_to(CartesianTarget): LINEAR motion (straight-line Cartesian path required)
    - move_to(StateTarget): Return to known joint configuration

Quaternion Note:
    Python uses (x, y, z, w) format; C++ Eigen::Quaterniond uses (w, x, y, z).
    This example uses (0, 0, 1.0, 0) = 180 deg rotation around Z-axis (tool pointing down).

C++ Source: tesseract_planning/tesseract_examples/src/basic_cartesian_example.cpp

C++ Parameters (verified):
    - Robot: KUKA LBR IIWA 14 R820 (7-DOF)
    - Obstacle: Octomap point cloud 1x1x1m at (1.0, 0, 0) - Python uses simplified box
    - Start joints: [-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
    - wp1: (0.5, -0.2, 0.62) with quat (0, 0, 1.0, 0)
    - wp2: (0.5, 0.3, 0.62) with same orientation
    - Profiles: "cartesian_program", "freespace_profile", "RASTER"

Related Examples:
    - glass_upright_example.py - Orientation-constrained TrajOpt
    - puzzle_piece_example.py - Complex Cartesian toolpath from CSV
    - freespace_ompl_example.py - Joint-space planning with OMPL
    - lowlevel/basic_cartesian_c_api_example.py - Same with low-level API
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    StateTarget,
    Pose,
    box,
    create_obstacle,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import (
    create_freespace_pipeline_profiles,
    create_trajopt_default_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run basic Cartesian planning example.

    Plans a multi-phase trajectory combining FREESPACE and LINEAR moves using
    TrajOpt trajectory optimization. Demonstrates mixing motion types and
    profile selection in a single motion program.

    Args:
        pipeline: Planning pipeline to use. Options:
            - "TrajOptPipeline" (default): TrajOpt trajectory optimization
            - "FreespacePipeline": OMPL for FREESPACE moves (ignores LINEAR)
        num_planners: Number of parallel OMPL planners (only for FreespacePipeline).

    Returns:
        dict with keys:
            - result: PlanningResult with trajectory and success status
            - robot: Robot instance with environment state
            - joint_names: List of 7 KUKA IIWA joint names
    """
    # Load KUKA IIWA 7-DOF robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # Add box obstacle at (1.0, 0, 0)
    # C++ uses octomap point cloud; Python uses simplified box geometry
    create_obstacle(robot, "box_obstacle", box(0.5, 0.5, 0.5), Pose.from_xyz(1.0, 0, 0))

    # Get joint names and set initial configuration
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (6D poses)
    # Quaternion (x=0, y=0, z=1.0, w=0) = 180 deg around Z = tool pointing down
    # C++ uses Eigen::Quaterniond(w, x, y, z) = (0, 0, 1.0, 0)
    wp1 = Pose.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 1.0, 0)  # First Cartesian waypoint
    wp2 = Pose.from_xyz_quat(
        0.5, 0.3, 0.62, 0, 0, 1.0, 0
    )  # Second waypoint (+0.5m in Y)

    # Build 4-phase motion program:
    # 1. StateTarget: Start from known joint configuration
    # 2. CartesianTarget + move_to: FREESPACE to wp1 (any collision-free path)
    # 3. CartesianTarget + linear_to: LINEAR to wp2 (straight-line Cartesian path)
    # 4. StateTarget: FREESPACE return to start joints
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1, profile="freespace_profile"))  # FREESPACE to pose
        .linear_to(CartesianTarget(wp2, profile="RASTER"))  # LINEAR Cartesian path
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
    )

    # Select profiles based on pipeline type
    # TrajOpt profiles include Cartesian constraint/cost configuration
    # OMPL profiles configure RRTConnect planner parameters
    if "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        profiles = create_trajopt_default_profiles()

    # Execute planning
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! Trajectory: {len(result)} waypoints")

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
    main()
