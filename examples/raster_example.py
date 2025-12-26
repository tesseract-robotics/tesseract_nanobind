"""
Raster Example - Industrial Raster/Process Path Planning

This example demonstrates the classic industrial raster pattern used in
manufacturing processes like welding, painting, milling, and surface
treatment. The robot executes multiple linear passes (raster strokes)
with freespace transitions between them.

PIPELINE OVERVIEW
-----------------
1. LOAD ROBOT: ABB IRB2400 6-DOF industrial robot
2. DEFINE WAYPOINTS: Linear raster path along Y-axis at constant X,Z
3. BUILD PROGRAM: Multiple raster segments with approach/transition/retract
4. PLAN: TrajOpt optimizes collision-free trajectory

KEY CONCEPTS DEMONSTRATED
-------------------------
1. Raster Pattern Structure:
   - Raster segment: linear moves through Cartesian waypoints (process path)
   - Transition: freespace move between raster segments (no process)
   - Approach: freespace move from start to first raster point
   - Retract: return to start position after final segment

2. Motion Types:
   - FREESPACE (move_to): unconstrained joint-space motion for transitions
   - LINEAR (linear_to): Cartesian-constrained motion for process path

3. Tool Orientation:
   - Quaternion (w=0, x=0, y=-1, z=0): 180deg rotation around Y-axis
   - Tool pointing downward (-Z direction in world frame)

4. Raster Geometry (from C++):
   - X = 0.8m (constant distance from robot base)
   - Z = 0.8m (constant height)
   - Y varies from -0.3m to +0.3m (7 waypoints, 0.1m spacing)

ROBOT CONFIGURATION (from C++)
------------------------------
ABB IRB2400:
- 6-DOF industrial manipulator
- Home position: all joints at zero
- TCP frame: "tool0" (standard ABB tool mount)
- Working frame: "base_link" (robot base)

The C++ example uses OPWInvKin for analytical inverse kinematics
(specific to ABB robot geometry).

PLANNING APPROACH
-----------------
The C++ example uses a two-stage planning approach:
1. Descartes: samples valid IK solutions along Cartesian path
2. TrajOpt: optimizes trajectory using Descartes solution as seed

This Python example uses TrajOpt directly for simplicity. For the
full Descartes+TrajOpt pipeline, see chain_example.py which uses
CartesianPipeline with create_cartesian_pipeline_profiles().

C++ SOURCE
----------
tesseract_planning/tesseract_motion_planners/examples/raster_example.cpp
Author: Levi Armstrong, Southwest Research Institute, August 2020

USE CASES
---------
- Welding seams (linear weld paths with travel moves between)
- Spray painting (parallel strokes covering surface area)
- Surface milling (raster cuts for material removal)
- Grinding/polishing (overlapping passes for surface finish)
- Laser processing (engraving, cutting, ablation)

RELATED EXAMPLES
----------------
- basic_cartesian_example.py: single Cartesian path
- puzzle_piece_auxillary_axes_example.py: complex surface following
- glass_upright_example.py: orientation-constrained motion
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
)
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Execute industrial raster pattern planning.

    Builds and plans a raster pattern with multiple parallel strokes:
    - 3 raster segments (process paths)
    - Freespace transitions between segments
    - Approach from and return to home position

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "FreespacePipeline")
        num_planners: Number of parallel threads (unused for TrajOpt)

    Returns:
        dict: {result, robot, joint_names} for testing/visualization
    """
    # === LOAD ROBOT ===
    # ABB IRB2400: 6-DOF industrial manipulator (same as C++ example)
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/abb_irb2400.urdf",
        "package://tesseract_support/urdf/abb_irb2400.srdf",
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Get joint names from SRDF-defined "manipulator" group
    joint_names = robot.get_joint_names("manipulator")

    # Home position: all joints at zero (from C++ example)
    home_pos = np.zeros(6)
    robot.set_joints(home_pos, joint_names=joint_names)

    # === DEFINE RASTER GEOMETRY ===
    # Tool orientation from C++: Quaterniond(w=0, x=0, y=-1.0, z=0)
    # This is 180deg rotation around Y-axis (tool pointing down)
    # Python format: (qx, qy, qz, qw)
    tool_quat = [0.0, -1.0, 0.0, 0.0]

    # Raster path geometry (from C++ example):
    # - X = 0.8m constant (distance from robot base)
    # - Z = 0.8m constant (height above floor)
    # - Y varies from -0.3m to +0.3m (7 waypoints, 0.1m spacing)
    x_const = 0.8
    z_const = 0.8
    y_values = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]

    # Create Cartesian waypoints with constant orientation
    waypoints = [Pose.from_xyz_quat(x_const, y, z_const, *tool_quat) for y in y_values]
    print(f"Created {len(waypoints)} waypoints")

    # === BUILD MOTION PROGRAM ===
    # Raster structure (from C++ example):
    # - from_start: home -> first waypoint (freespace)
    # - raster_segment: linear moves through waypoints (3 times)
    # - transitions: freespace moves between segments
    # - to_end: final waypoint -> home (freespace)
    num_segments = 3

    program = MotionProgram(
        "manipulator", tcp_frame="tool0", profile="RASTER"
    ).set_joint_names(joint_names)

    # Start from home position
    program.move_to(StateTarget(home_pos, names=joint_names, profile="FREESPACE"))

    for seg in range(num_segments):
        # Approach: freespace move to first raster waypoint
        program.move_to(CartesianTarget(waypoints[0], profile="FREESPACE"))

        # Raster segment: linear moves through all waypoints (process path)
        for wp in waypoints[1:]:
            program.linear_to(CartesianTarget(wp, profile="RASTER"))

        # Transition: freespace move back to start of next segment
        if seg < num_segments - 1:
            program.move_to(CartesianTarget(waypoints[0], profile="FREESPACE"))

    # Return to home position
    program.move_to(StateTarget(home_pos, names=joint_names, profile="FREESPACE"))

    print(f"Created program with {len(program)} waypoints ({num_segments} segments)")

    # === PLAN WITH TRAJOPT ===
    # TrajOpt optimizes the trajectory for smooth motion and collision avoidance
    print(f"\nRunning planner ({pipeline})...")
    composer = TaskComposer.from_config()
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
