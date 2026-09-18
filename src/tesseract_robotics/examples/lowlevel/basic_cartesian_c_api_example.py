"""
Basic Cartesian Planning Example

Demonstrates Cartesian motion planning using TrajOpt trajectory optimization with
the KUKA IIWA 7-DOF robot. Plans a multi-phase trajectory combining freespace and
linear Cartesian moves around an obstacle.

Pipeline Overview:
1. Load robot and build the C++ example's octomap from point-cloud data at (1.0, 0, 0):
   a 1 m cube of points at 0.05 m pitch, voxelised into 1000 boxes of 0.1 m
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
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    create_obstacle,
)
from tesseract_robotics.tesseract_geometry import Octree, OctreeSubType, PointCloud, createOctree

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def run():
    """Run basic Cartesian planning example.

    Returns:
        dict with result, robot, joint_names
    """
    # Load KUKA IIWA 7-DOF robot from tesseract_support package
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot: {robot}")

    # Build the C++ example's octomap (addPointCloud in basic_cartesian_example.cpp): points
    # every 0.05 m across a 1 m cube, voxelised at 0.1 m. Every voxel holds points, so this is a
    # solid cube of 1000 boxes — the collision load of a scanned part.
    pitch = 0.05
    count = round(1.0 / pitch)
    cloud = PointCloud()
    for i in range(count):
        for j in range(count):
            for k in range(count):
                cloud.addPoint(-0.5 + i * pitch, -0.5 + j * pitch, -0.5 + k * pitch)
    tree = createOctree(cloud, 2 * pitch, prune=False, binary=True)
    octree = Octree(tree, OctreeSubType.BOX, pruned=False, binary_octree=True)
    create_obstacle(
        robot,
        name="octomap_attached",
        geometry=octree,
        transform=Pose.from_xyz(1.0, 0, 0),
    )
    print(f"Added octomap of {octree.calcNumSubShapes()} boxes at (1.0, 0, 0)")

    # Get joint names and set initial robot configuration
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (6D tool poses in world frame), tool pointing
    # down. The C++ Eigen::Quaterniond(0, 0, 1.0, 0) is scalar-first (w=0, x=0,
    # y=1, z=0): 180° about Y. Project order is scalar-last, so [0, 1.0, 0, 0].
    # Copying the literal as [0, 0, 1.0, 0] turns the tool face-up instead.
    wp1_pose = Pose.from_xyz_quat([0.5, -0.2, 0.62], [0, 1.0, 0, 0])
    wp2_pose = Pose.from_xyz_quat([0.5, 0.3, 0.62], [0, 1.0, 0, 0])

    # Build 4-phase motion program using fluent API:
    # Phase 1: Start from known joint state (defines initial configuration)
    # Phase 2: FREESPACE move to wp1 (any collision-free path, uses IK)
    # Phase 3: LINEAR move to wp2 (straight-line tool path, Cartesian interpolation)
    # Phase 4: FREESPACE move back to start joint state
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1_pose, profile="freespace_profile"))  # FREESPACE to pose
        .linear_to(CartesianTarget(wp2_pose, profile="RASTER"))  # LINEAR between poses
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
