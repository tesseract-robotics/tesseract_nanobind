"""
Puzzle Piece Example - Low-level C API version

Demonstrates Cartesian toolpath planning using TrajOpt with custom profiles.
Ported from: tesseract_planning/tesseract_examples/src/puzzle_piece_example.cpp

Robot: 7-DOF KUKA IIWA in puzzle_piece_workcell
Task: Follow a toolpath (puzzle_bent.csv) with a grinding tool while allowing
      the tool to spin freely around its axis (useful for cylindrical tools).

Key concepts:
- CSV toolpath parsing with mm->m conversion
- Frame construction from position vector and surface normal
- TrajOpt profile configuration for Cartesian path following
- coeff[5]=0 allows rotation freedom around tool Z-axis

C++ reference values:
- Initial joints: [-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0]
- Cartesian constraint coeff: [10, 10, 10, 10, 10, 0]
- Collision cost: safety_margin=0.025m, coeff=20
- ManipulatorInfo: manipulator="manipulator", tcp="grinder_frame", working_frame="part"
"""

import sys
import csv
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    Pose,
    TaskComposer,
)
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def make_puzzle_tool_poses(robot):
    """
    Load toolpath poses from puzzle_bent.csv.

    CSV format: point_num, x, y, z, i, j, k (positions in mm, normals unitless)
    Returns list of Pose objects with full 6-DOF orientation.

    Frame construction algorithm (from C++):
    1. z-axis = surface normal (tool points into surface)
    2. temp_x = -position (pointing toward origin, gives consistent orientation)
    3. y-axis = cross(z, temp_x) -> perpendicular to both
    4. x-axis = cross(y, z) -> completes right-handed frame
    """
    # Locate CSV via resource locator (respects package:// URIs)
    resource = robot.locator.locateResource(
        "package://tesseract_support/urdf/puzzle_bent.csv"
    )
    csv_path = resource.getFilePath()

    poses = []

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        for lnum, row in enumerate(reader):
            # Skip header rows (lines 0-1 contain column names)
            if lnum < 2:
                continue

            if len(row) < 7:
                continue

            # CSV columns: [point_num, x, y, z, i, j, k]
            try:
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                i, j, k = float(row[4]), float(row[5]), float(row[6])
            except (ValueError, IndexError):
                continue

            # CSV is in millimeters, convert to meters for Tesseract
            pos = np.array([x, y, z]) / 1000.0

            # Surface normal becomes tool z-axis (pointing into workpiece)
            norm = np.array([i, j, k])
            norm = norm / np.linalg.norm(norm)

            # Frame construction: use -position as reference for x-axis direction
            # This creates consistent orientation as tool follows curved path
            temp_x = (
                -pos / np.linalg.norm(pos)
                if np.linalg.norm(pos) > 1e-6
                else np.array([1, 0, 0])
            )

            # Build orthonormal frame: y = z x temp_x, then x = y x z
            y_axis = np.cross(norm, temp_x)
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis = x_axis / np.linalg.norm(x_axis)

            # Rotation matrix: columns are frame axes in world coordinates
            rot = np.column_stack([x_axis, y_axis, norm])
            poses.append(Pose.from_matrix_position(rot, pos))

    return poses


def main():
    # Load puzzle_piece_workcell: KUKA IIWA 7-DOF arm with grinder tool
    # URDF defines: part frame (workpiece), grinder_frame (TCP)
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract_support/urdf/puzzle_piece_workcell.srdf",
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # KUKA IIWA joint naming convention: joint_a1 through joint_a7
    joint_names = [f"joint_a{i}" for i in range(1, 8)]

    # Initial joint position from C++ example - places tool near first waypoint
    # Values: [-45deg, 23deg, 0, -109deg, 0, 57deg, 0] approximately
    joint_pos = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0])

    # Set robot to initial configuration for IK seeding
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Load toolpath from CSV (positions + surface normals)
    try:
        tool_poses = make_puzzle_tool_poses(robot)
    except Exception as e:
        print(f"Failed to load toolpath: {e}")
        return False

    print(f"Loaded {len(tool_poses)} tool poses from CSV")

    if len(tool_poses) == 0:
        print("No poses loaded from CSV!")
        return False

    # Build motion program with Cartesian waypoints
    # - working_frame="part": poses are relative to workpiece
    # - tcp_frame="grinder_frame": tool center point for IK
    program = MotionProgram(
        "manipulator", tcp_frame="grinder_frame", working_frame="part"
    )
    program.set_joint_names(joint_names)

    # Add all waypoints as linear (Cartesian) moves with custom profile
    for pose in tool_poses:
        program.linear_to(CartesianTarget(pose, profile="CARTESIAN"))

    print(f"Program has {len(program)} Cartesian waypoints")

    # =========================================================================
    # TrajOpt Profile Configuration
    # =========================================================================
    # Two profile types:
    # - PlanProfile: per-waypoint settings (Cartesian/joint constraints)
    # - CompositeProfile: trajectory-wide settings (collision avoidance)
    profiles = ProfileDictionary()

    # --- Plan Profile: Cartesian constraint configuration ---
    trajopt_plan_profile = TrajOptDefaultPlanProfile()

    # Disable joint-space cost (we want Cartesian accuracy, not joint smoothness)
    trajopt_plan_profile.joint_cost_config.enabled = False

    # Disable Cartesian cost (use hard constraint instead of soft cost)
    trajopt_plan_profile.cartesian_cost_config.enabled = False

    # Enable Cartesian constraint - hard constraint on pose accuracy
    trajopt_plan_profile.cartesian_constraint_config.enabled = True

    # Constraint coefficients: [x, y, z, rx, ry, rz]
    # - coeff[0-2]=10: strict position constraint (x, y, z)
    # - coeff[3-4]=10: strict rotation constraint (rx, ry)
    # - coeff[5]=0: ZERO means rotation around tool Z-axis is FREE
    #   This is critical for cylindrical tools (grinders, drills) where
    #   spinning around the tool axis doesn't affect the operation
    trajopt_plan_profile.cartesian_constraint_config.coeff = np.array(
        [10.0, 10.0, 10.0, 10.0, 10.0, 0.0]
    )

    # --- Composite Profile: collision avoidance configuration ---
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()

    # Use cost (soft) rather than constraint (hard) for collision
    # Hard constraints can make optimization infeasible
    trajopt_composite_profile.collision_constraint_config.enabled = False
    trajopt_composite_profile.collision_cost_config.enabled = True

    # 0.33 API: TrajOptCollisionConfig replaces CollisionCostConfig
    # collision_margin_buffer: additional margin beyond contact (25mm)
    # collision_check_config.type: DISCRETE (was SINGLE_TIMESTEP)
    # collision_coeff_data: per-pair coefficients (default coeff = 20.0)
    trajopt_composite_profile.collision_cost_config.collision_margin_buffer = 0.025
    trajopt_composite_profile.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.DISCRETE
    )
    trajopt_composite_profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(
        20.0
    )

    # Register profiles with TrajOpt namespace
    # "CARTESIAN" profile used by waypoints, "DEFAULT" for trajectory-wide settings
    ProfileDictionary_addTrajOptPlanProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile
    )
    ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile
    )

    print("Running TrajOpt planner...")

    # Plan using TaskComposer - handles IK seeding and optimization
    # TrajOptPipeline: MinLengthTask -> DescartesMotionPlannerTask -> TrajOptMotionPlannerTask
    composer = TaskComposer.from_config()
    result = composer.plan(
        robot, program, pipeline="TrajOptPipeline", profiles=profiles
    )

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")
    print(f"Trajectory has {len(result)} waypoints")

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
