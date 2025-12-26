"""
Basic Cartesian Planning Example

Demonstrates Cartesian motion planning using TrajOpt trajectory optimization with
the KUKA IIWA 7-DOF robot. Plans a multi-phase trajectory combining freespace and
linear Cartesian moves around an obstacle.

Pipeline Overview:
1. Load robot and add box obstacle at (1.0, 0, 0)
2. Create 4-phase program: start → freespace → linear → freespace back
3. Execute TrajOptPipeline (trajectory optimization with collision/constraint costs)
4. Return smooth, collision-free trajectory

Key Concepts:
- TrajOpt: trajectory optimizer that minimizes costs while respecting constraints
- FREESPACE moves: joint-space interpolation, no Cartesian path constraints
- LINEAR moves: straight-line Cartesian path, tool pose interpolated along line
- Profile names: "freespace_profile" for unconstrained, "RASTER" for linear paths

Motion Types:
- move_to(CartesianTarget): FREESPACE motion (any collision-free path to pose)
- linear_to(CartesianTarget): LINEAR motion (straight-line path required)

C++ Source: tesseract_examples/src/basic_cartesian_example.cpp

Related Examples:
- glass_upright_example.py - orientation-constrained TrajOpt planning
- puzzle_piece_example.py - complex Cartesian toolpath from CSV
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

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run():
    """Run basic Cartesian planning example.

    Returns:
        dict with result, robot, joint_names
    """
    # Load KUKA IIWA 7-DOF robot from tesseract_support package
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot: {robot}")

    # Add box obstacle for collision checking
    # C++ uses an octomap (1x1x1m point cloud grid), Python uses simplified box
    # Position: 1m in front of robot base
    create_obstacle(
        robot,
        name="box_obstacle",
        geometry=box(0.5, 0.5, 0.5),  # 50cm cube
        transform=Pose.from_xyz(1.0, 0, 0),
    )
    print("Added box obstacle at (1.0, 0, 0)")

    # Get joint names and set initial robot configuration
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (6D tool poses in world frame)
    # Quaternion (x, y, z, w) = (0, 0, 1.0, 0) represents 180° rotation around Z-axis
    # This points the tool down toward the work surface
    # Note: Python uses (x,y,z,w) format; C++ Eigen uses (w,x,y,z)
    wp1_pose = Pose.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 1.0, 0)  # First waypoint
    wp2_pose = Pose.from_xyz_quat(0.5, 0.3, 0.62, 0, 0, 1.0, 0)   # Second waypoint (Y+0.5m)

    # Build 4-phase motion program using fluent API:
    # Phase 1: Start from known joint state (defines initial configuration)
    # Phase 2: FREESPACE move to wp1 (any collision-free path, uses IK)
    # Phase 3: LINEAR move to wp2 (straight-line tool path, Cartesian interpolation)
    # Phase 4: FREESPACE move back to start joint state
    program = (MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1_pose, profile="freespace_profile"))  # FREESPACE to pose
        .linear_to(CartesianTarget(wp2_pose, profile="RASTER"))           # LINEAR between poses
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
    )

    print("\nProgram created with TrajOpt Cartesian planning")
    print("  - Freespace to Cartesian wp1")
    print("  - Linear to Cartesian wp2")
    print("  - Freespace back to start")

    # Plan using TaskComposer
    print("\nRunning TrajOpt planner...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    assert result.successful, f"Planning failed: {result.message}"

    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints")

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
