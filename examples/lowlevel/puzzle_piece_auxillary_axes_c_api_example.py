"""
Puzzle Piece Auxiliary Axes Example

Cartesian path planning with 9-DOF (KUKA IIWA 7-DOF + 2-DOF positioner).
This demonstrates planning for a multi-chain kinematic system where auxiliary
axes (positioner) can orient the workpiece to improve reachability.

C++ Source: tesseract_examples/src/puzzle_piece_auxillary_axes_example.cpp

Key C++ Implementation Details:
- 9-DOF system: 7 arm joints + 2 auxiliary positioner joints
- CSV toolpath positions are in millimeters (converted to meters here)
- TrajOpt Cartesian constraint coeff[5]=0 allows yaw rotation freedom
  so auxiliary axes can optimize orientation while maintaining position
- Uses "manipulator_aux" kinematic group (defined in SRDF)
- puzzle_bent.csv contains ~50 waypoints for grinding puzzle piece edge

Robot Configuration:
- KUKA IIWA 7-DOF arm (joint_a1 through joint_a7)
- 2-DOF positioner (joint_aux1=Z rotation, joint_aux2=tilt)
- "grinder_frame" TCP attached to arm end-effector
- "part" working frame on positioner (moves with aux axes)
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
    Load toolpath poses from the puzzle_bent.csv file.

    The CSV contains position (x, y, z) and normal direction (i, j, k) for each pose.
    We construct a full orientation from the normal using the position for the x-axis.

    CSV Format (puzzle_bent.csv):
        Row 0-1: Headers
        Columns: point_num, x, y, z, i, j, k
        Units: millimeters (converted to meters below)

    Frame Construction:
        - Z-axis: surface normal (i, j, k) from CSV
        - X-axis: computed from cross products for orthogonality
        - Y-axis: Z x X for right-handed frame
    """
    # Locate CSV via tesseract resource system (resolves package:// URIs)
    resource = robot.locator.locateResource(
        "package://tesseract_support/urdf/puzzle_bent.csv"
    )
    csv_path = resource.getFilePath()

    poses = []

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        for lnum, row in enumerate(reader):
            # Skip header rows (first 2 lines in puzzle_bent.csv)
            if lnum < 2:
                continue

            if len(row) < 7:
                continue

            # Parse values: skip first column (point number), then x, y, z, i, j, k
            try:
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                i, j, k = float(row[4]), float(row[5]), float(row[6])
            except (ValueError, IndexError):
                continue

            # CRITICAL: CSV positions in mm, tesseract uses meters
            pos = np.array([x, y, z]) / 1000.0

            # Normalize the surface normal vector (tool Z-axis)
            norm = np.array([i, j, k])
            norm = norm / np.linalg.norm(norm)

            # Construct orthogonal frame from normal:
            # Use negative position as reference to create X-axis pointing inward
            temp_x = (
                -pos / np.linalg.norm(pos)
                if np.linalg.norm(pos) > 1e-6
                else np.array([1, 0, 0])
            )
            y_axis = np.cross(norm, temp_x)
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis = x_axis / np.linalg.norm(x_axis)

            # Build rotation matrix [X|Y|Z] and create pose
            rot = np.column_stack([x_axis, y_axis, norm])
            poses.append(Pose.from_matrix_position(rot, pos))

    return poses


def main():
    """
    Main planning workflow:
    1. Load 9-DOF workcell (arm + positioner)
    2. Load CSV toolpath (~50 waypoints)
    3. Configure TrajOpt with yaw-free constraint (coeff[5]=0)
    4. Plan Cartesian path through all waypoints
    """
    # Load puzzle piece workcell - contains KUKA IIWA + 2-DOF positioner
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract_support/urdf/puzzle_piece_workcell.srdf",
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # 9-DOF joint names: 7 arm + 2 auxiliary (must match URDF order)
    joint_names = [
        "joint_a1",
        "joint_a2",
        "joint_a3",
        "joint_a4",
        "joint_a5",
        "joint_a6",
        "joint_a7",
        "joint_aux1",
        "joint_aux2",  # positioner axes
    ]

    # Initial configuration from C++ example
    # Arm slightly bent, positioner at neutral (0,0)
    joint_pos = np.array(
        [
            -0.785398,  # joint_a1: -45deg
            0.4,  # joint_a2
            0.0,  # joint_a3
            -1.9,  # joint_a4
            0.0,  # joint_a5
            1.0,  # joint_a6
            0.0,  # joint_a7
            0.0,  # joint_aux1: positioner Z-rotation
            0.0,  # joint_aux2: positioner tilt
        ]
    )

    # Set initial state
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Load tool poses from CSV
    try:
        tool_poses = make_puzzle_tool_poses(robot)
    except Exception as e:
        print(f"Failed to load toolpath: {e}")
        return False

    print(f"Loaded {len(tool_poses)} tool poses from CSV")

    if len(tool_poses) == 0:
        print("No poses loaded from CSV!")
        return False

    # Build motion program using "manipulator_aux" kinematic group
    # This group includes both arm and positioner chains (defined in SRDF)
    # - tcp_frame: grinder tool on arm end-effector
    # - working_frame: "part" frame attached to positioner (moves with aux axes)
    program = MotionProgram(
        "manipulator_aux", tcp_frame="grinder_frame", working_frame="part"
    )
    program.set_joint_names(joint_names)

    # Add all CSV waypoints as Cartesian targets
    for pose in tool_poses:
        program.linear_to(CartesianTarget(pose, profile="CARTESIAN"))

    print(f"Program has {len(program)} Cartesian waypoints")

    # Configure TrajOpt for 9-DOF Cartesian planning
    profiles = ProfileDictionary()

    # TrajOpt plan profile: waypoint-level constraints
    trajopt_plan_profile = TrajOptDefaultPlanProfile()
    trajopt_plan_profile.joint_cost_config.enabled = False  # No joint-space costs
    trajopt_plan_profile.cartesian_cost_config.enabled = (
        False  # Using constraints instead
    )
    trajopt_plan_profile.cartesian_constraint_config.enabled = True

    # CRITICAL: coeff[5]=0 for yaw (rz) freedom
    # This allows aux axes to rotate workpiece while maintaining TCP position
    # [x, y, z, rx, ry, rz] - position constrained, roll/pitch constrained, yaw FREE
    trajopt_plan_profile.cartesian_constraint_config.coeff = np.array(
        [5.0, 5.0, 5.0, 2.0, 2.0, 0.0]
    )

    # TrajOpt composite profile: trajectory-level collision settings
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()
    trajopt_composite_profile.collision_constraint_config.enabled = (
        False  # Soft cost, not hard constraint
    )
    trajopt_composite_profile.collision_cost_config.enabled = True
    # 0.33 API: TrajOptCollisionConfig replaces CollisionCostConfig
    trajopt_composite_profile.collision_cost_config.collision_margin_buffer = (
        0.025  # 25mm collision buffer
    )
    trajopt_composite_profile.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.DISCRETE  # was SINGLE_TIMESTEP
    )
    trajopt_composite_profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(
        1.0
    )

    # Register profiles with TrajOpt task namespace
    ProfileDictionary_addTrajOptPlanProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile
    )
    ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile
    )

    print("Running TrajOpt planner with 9 DOF (7 arm + 2 auxiliary axes)...")

    # Plan via TaskComposer - handles seeding and post-processing
    # TrajOptPipeline: seeds Cartesian waypoints with IK, then optimizes
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
