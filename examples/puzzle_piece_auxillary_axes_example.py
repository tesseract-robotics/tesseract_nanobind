"""
Puzzle Piece Auxiliary Axes Example - 9-DOF Cartesian Path with Positioner

This example demonstrates Cartesian path planning for a multi-chain kinematic
system: a 7-DOF KUKA IIWA arm combined with a 2-DOF workpiece positioner.
The positioner (auxiliary axes) can reorient the workpiece to improve
reachability and avoid singularities during surface following operations.

PIPELINE OVERVIEW
-----------------
1. LOAD WORKCELL: 9-DOF system (7-DOF arm + 2-DOF positioner)
2. LOAD TOOLPATH: Parse CSV file with ~50 Cartesian waypoints (puzzle piece edge)
3. CONFIGURE TRAJOPT: Enable yaw freedom (coeff[5]=0) for auxiliary axis optimization
4. PLAN: TrajOpt optimizes 9-DOF trajectory following Cartesian path

KEY CONCEPTS DEMONSTRATED
-------------------------
1. Multi-Chain Kinematics:
   - "manipulator_aux" group: combines arm and positioner kinematic chains
   - Arm chain: grinder_frame (TCP) on KUKA IIWA end-effector
   - Positioner chain: "part" frame that moves with auxiliary axes

2. Yaw Freedom for Auxiliary Axes:
   - TrajOpt constraint coeff = [5,5,5,2,2,0] (x,y,z,rx,ry,rz)
   - coeff[5]=0: yaw (rz) is FREE, not constrained
   - Positioner can rotate workpiece around tool axis while maintaining TCP position
   - Enables better arm configurations and singularity avoidance

3. CSV Toolpath Format:
   - puzzle_bent.csv: surface points with normals for grinding operation
   - Columns: point_num, x, y, z, i, j, k (position + normal vector)
   - Units: millimeters (converted to meters in code)
   - ~50 waypoints tracing puzzle piece edge

4. Frame Construction from Normal:
   - Z-axis: surface normal from CSV (tool approach direction)
   - X-axis: computed orthogonal to normal
   - Y-axis: Z x X for right-handed frame

ROBOT CONFIGURATION (from C++)
------------------------------
9 DOF joint order:
- joint_a1 through joint_a7: KUKA IIWA arm (7 DOF)
- joint_aux1: positioner Z-rotation (turntable)
- joint_aux2: positioner tilt (tilting table)

Initial configuration:
- joint_a1=-0.785398 (-45deg), joint_a2=0.4, joint_a4=-1.9, joint_a6=1.0
- Positioner at neutral (0, 0)

Frames:
- tcp_frame="grinder_frame": tool center point on arm end-effector
- working_frame="part": workpiece frame attached to positioner

TRAJOPT SETTINGS (from C++)
---------------------------
Plan profile (CARTESIAN):
- cartesian_constraint_config.coeff = [5,5,5,2,2,0]
- Position (x,y,z) tightly constrained (coeff=5)
- Roll/pitch (rx,ry) moderately constrained (coeff=2)
- Yaw (rz) FREE (coeff=0) - allows tool rotation

Composite profile (DEFAULT):
- collision_cost_config.collision_margin_buffer = 0.025m
- collision_cost_config.collision_check_config.type = DISCRETE
- Solver: OSQP, max_iter=200

C++ SOURCE
----------
tesseract_planning/tesseract_examples/src/puzzle_piece_auxillary_axes_example.cpp
Author: Levi Armstrong, Southwest Research Institute, July 2019

USE CASES
---------
- Surface grinding/polishing with tool orientation freedom
- Welding with auxiliary rotation for weld pool control
- Machining with coordinated workpiece repositioning
- Any process where auxiliary axes extend workspace or improve reachability

RELATED EXAMPLES
----------------
- basic_cartesian_example.py: simple Cartesian path without auxiliary axes
- glass_upright_example.py: orientation constraints for upright maintenance
- raster_example.py: industrial raster patterns

PERFORMANCE NOTE
----------------
First planning call takes ~10-15s due to dynamic plugin loading (dlopen).
This is inherent to the TaskComposer plugin architecture (same in C++).
Subsequent calls in the same process are fast (~8s for this 50-waypoint path).

For interactive use, call composer.warmup() or use warmup=True:
    composer = TaskComposer.from_config(warmup=True)  # loads plugins upfront
"""

import csv
import sys

import numpy as np

from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    TaskComposer,
)
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    CollisionEvaluatorType,
    ProfileDictionary_addTrajOptCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptSolverProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultPlanProfile,
    TrajOptOSQPSolverProfile,
)

TRAJOPT_NS = "TrajOptMotionPlannerTask"

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def make_puzzle_tool_poses(robot):
    """Load toolpath poses from puzzle_bent.csv.

    From C++ makePuzzleToolPoses(): parses CSV with position and normal data.

    CSV Format:
        Row 0-1: Headers (skipped)
        Columns: point_num, x, y, z, i, j, k
        Units: millimeters (converted to meters)

    Frame Construction (from C++):
        - Z-axis: surface normal (i, j, k) from CSV - tool approach direction
        - X-axis: computed orthogonal, pointing inward toward origin
        - Y-axis: Z x X for right-handed coordinate frame

    Returns:
        list[Pose]: ~50 Cartesian waypoints for puzzle piece edge
    """
    resource = robot.locator.locateResource("package://tesseract_support/urdf/puzzle_bent.csv")
    csv_path = resource.getFilePath()

    poses = []
    with open(csv_path) as f:
        reader = csv.reader(f)
        for lnum, row in enumerate(reader):
            if lnum < 2 or len(row) < 7:  # Skip header rows
                continue

            try:
                # Parse x,y,z (mm) and normal i,j,k - convert mm to meters
                x, y, z = (
                    float(row[1]) / 1000,
                    float(row[2]) / 1000,
                    float(row[3]) / 1000,
                )
                i, j, k = float(row[4]), float(row[5]), float(row[6])
            except (ValueError, IndexError):
                continue

            pos = np.array([x, y, z])
            norm = np.array([i, j, k])
            norm /= np.linalg.norm(norm)

            # Build orthogonal frame from surface normal
            # Use negative position as reference to create X-axis pointing inward
            temp_x = (
                -pos / np.linalg.norm(pos) if np.linalg.norm(pos) > 1e-6 else np.array([1, 0, 0])
            )
            y_axis = np.cross(norm, temp_x)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis /= np.linalg.norm(x_axis)

            # Build rotation matrix [X|Y|Z] and create pose
            rot = np.column_stack([x_axis, y_axis, norm])
            poses.append(Pose.from_matrix_position(rot, pos))

    return poses


def create_profiles():
    """Create TrajOpt profiles for Cartesian path following with yaw freedom.

    From C++ puzzle_piece_auxillary_axes_example.cpp profile configuration.

    Key settings:
    - cartesian_constraint_config.coeff = [5,5,5,2,2,0]
      Position (x,y,z) tightly constrained, roll/pitch moderate, yaw FREE
    - collision_cost (not constraint) with 25mm safety margin
    - solver: OSQP, max_iter=200, min_approx_improve=1e-3, min_trust_box_size=1e-3
    """
    profiles = ProfileDictionary()

    # Plan profile: Cartesian constraint with yaw (rz) freedom
    # coeff = [x, y, z, rx, ry, rz] weights for pose error
    # CRITICAL: coeff[5]=0 allows yaw rotation freedom for auxiliary axes
    plan = TrajOptDefaultPlanProfile()
    plan.joint_cost_config.enabled = False
    plan.cartesian_cost_config.enabled = False
    plan.cartesian_constraint_config.enabled = True
    plan.cartesian_constraint_config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 0.0])

    # Composite profile: soft collision cost (not hard constraint)
    composite = TrajOptDefaultCompositeProfile()
    composite.collision_constraint_config.enabled = False
    composite.collision_cost_config.enabled = True
    composite.collision_cost_config.collision_margin_buffer = 0.025  # 25mm buffer
    composite.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE

    # Solver profile: OSQP with C++ settings
    solver = TrajOptOSQPSolverProfile()
    solver.opt_params.max_iter = 200
    solver.opt_params.min_approx_improve = 1e-3
    solver.opt_params.min_trust_box_size = 1e-3

    ProfileDictionary_addTrajOptPlanProfile(profiles, TRAJOPT_NS, "CARTESIAN", plan)
    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_NS, "DEFAULT", composite)
    ProfileDictionary_addTrajOptSolverProfile(profiles, TRAJOPT_NS, "DEFAULT", solver)
    return profiles


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run 9-DOF Cartesian path planning with auxiliary axes.

    Args:
        pipeline: Pipeline name (default "TrajOptPipeline")
        num_planners: Number of parallel planners (unused, for API compat)

    Returns:
        dict with result, robot, joint_names, planning_time, success
    """
    import time

    # === LOAD WORKCELL ===
    # puzzle_piece_workcell: KUKA IIWA arm + 2-DOF positioner
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract_support/urdf/puzzle_piece_workcell.srdf",
    )

    # 9 DOF: KUKA IIWA (7) + auxiliary axes (2)
    # Joint order must match URDF kinematic chain
    joint_names = [
        "joint_a1",
        "joint_a2",
        "joint_a3",
        "joint_a4",
        "joint_a5",
        "joint_a6",
        "joint_a7",
        "joint_aux1",
        "joint_aux2",  # Positioner: Z-rotation + tilt
    ]
    # Initial configuration from C++ (arm slightly bent, positioner neutral)
    joint_pos = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0, 0.0, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # === LOAD TOOLPATH ===
    # Parse CSV with ~50 Cartesian waypoints (puzzle piece edge grinding)
    tool_poses = make_puzzle_tool_poses(robot)
    print(f"Loaded {len(tool_poses)} tool poses")
    assert tool_poses, "No poses loaded from CSV"

    # === BUILD MOTION PROGRAM ===
    # "manipulator_aux": combined kinematic group (arm + positioner)
    # tcp_frame="grinder_frame": tool on arm end-effector
    # working_frame="part": workpiece frame attached to positioner
    program = MotionProgram(
        "manipulator_aux", tcp_frame="grinder_frame", working_frame="part"
    ).set_joint_names(joint_names)

    for pose in tool_poses:
        program.linear_to(CartesianTarget(pose, profile="CARTESIAN"))

    print(f"Program: {len(program)} waypoints")

    # === PLAN WITH TRAJOPT ===
    # Custom profiles enable yaw freedom for auxiliary axis optimization
    print(f"Planning with {pipeline} (9 DOF: 7 arm + 2 aux)...")
    composer = TaskComposer.from_config()

    # Warmup: pre-load plugins (dlopen overhead)
    warmup_start = time.time()
    composer.warmup([pipeline])
    warmup_time = time.time() - warmup_start
    print(f"Plugin warmup: {warmup_time:.2f}s")

    profiles = create_profiles()

    # Plan: actual optimization time
    plan_start = time.time()
    try:
        result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
        planning_time = time.time() - plan_start

        if result.successful:
            print(f"Planning: {planning_time:.2f}s ({len(result)} waypoints)")
        else:
            print(f"Planning failed: {planning_time:.2f}s - {result.message}")

    except Exception as e:
        planning_time = time.time() - plan_start
        print(f"Planning exception: {planning_time:.2f}s - {e}")
        result = None

    total_time = warmup_time + planning_time
    print(f"Total: {total_time:.2f}s (warmup {warmup_time:.2f}s + plan {planning_time:.2f}s)")

    return {
        "result": result,
        "robot": robot,
        "joint_names": joint_names,
        "planning_time": planning_time,
        "warmup_time": warmup_time,
        "success": result.successful if result else False,
    }


def main():
    """Execute example and optionally visualize."""
    results = run()

    if TesseractViewer is not None and results.get("result"):
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        if results["result"].raw_results is not None:
            viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return results["success"]


if __name__ == "__main__":
    main()
