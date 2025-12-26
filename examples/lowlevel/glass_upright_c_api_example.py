"""
Glass Upright Example - Orientation-Constrained Motion Planning

Demonstrates TrajOpt planning with orientation constraints: the robot moves while
keeping the tool "upright" (like carrying a glass of water without spilling).

C++ Source: tesseract_planning/tesseract_examples/src/glass_upright_example.cpp

Concept: "Upright" Constraint
-----------------------------
The UPRIGHT profile applies Cartesian constraints with coeff=[0,0,0,5,5,5]:
  - Position (x,y,z): coeff=0 -> unconstrained, robot can move freely in space
  - Orientation (rx,ry,rz): coeff=5 -> strongly constrained, must stay upright

This is the inverse of typical constraints that fix position but allow rotation.
The high orientation weights (5) penalize any deviation from the initial tool
orientation during trajectory optimization.

Pipeline Overview
-----------------
1. Load KUKA IIWA 7-DOF robot
2. Add sphere obstacle to force non-trivial path (robot must go around it)
3. Define start/end via joint states (only joint_a1 changes: -0.4 -> 0.4 rad)
4. Plan with TrajOpt using UPRIGHT profile for orientation constraints
5. TrajOpt optimizes trajectory while respecting orientation + collision constraints
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    StateTarget,
    Pose,
    sphere,
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
    """Run glass upright example with orientation-constrained planning.

    Returns:
        dict with result, robot, joint_names for testing/visualization
    """
    # KUKA IIWA 14 R820: 7-DOF redundant arm (joints: joint_a1 through joint_a7)
    # 7-DOF allows maintaining orientation while repositioning - essential for upright constraint
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Sphere obstacle near path to influence trajectory, not block it
    # Forces TrajOpt to find trajectory that avoids sphere while keeping tool upright
    # Without obstacle, robot could just rotate joint_a1 directly (trivial solution)
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.1),  # 10cm radius - smaller to avoid blocking
        transform=Pose.from_xyz(0.55, 0, 0.45),  # Slightly forward and lower
    )
    print("Added sphere obstacle at (0.55, 0, 0.45)")

    # joint_a1 through joint_a7 for IIWA manipulator group
    joint_names = robot.get_joint_names("manipulator")

    # Start/end positions from C++ example - only joint_a1 changes (-0.4 -> 0.4 rad)
    # This represents ~46 degrees rotation of the base joint while all other joints
    # maintain the same configuration. The arm effectively "sweeps" horizontally
    # while the tool orientation must remain constant (upright constraint)
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set robot to start configuration for planning
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # MotionProgram with "UPRIGHT" profile - this profile name maps to TrajOpt config that sets:
    #   coeff = [0, 0, 0, 5, 5, 5]  (position free, orientation constrained)
    #   lower_tolerance / upper_tolerance for orientation bounds
    #
    # LINEAR motion type (vs FREESPACE) tells TrajOpt to interpolate in Cartesian space,
    # essential for maintaining orientation at intermediate waypoints
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="UPRIGHT")
        .set_joint_names(joint_names)
        # StateTarget specifies full joint configuration (not just Cartesian pose)
        # TrajOpt will optimize intermediate states while enforcing UPRIGHT constraints
        .linear_to(StateTarget(joint_start_pos, names=joint_names, profile="UPRIGHT"))
        .linear_to(StateTarget(joint_end_pos, names=joint_names, profile="UPRIGHT"))
    )

    print(
        "\nProgram: LINEAR motion with UPRIGHT profile (orientation locked, position free)"
    )
    print(
        "TrajOpt will optimize trajectory to avoid sphere while maintaining tool orientation"
    )

    # TrajOptPipeline: trajectory optimization with collision avoidance
    # C++ uses safety_margin=0.01, contact_buffer=0.01 for collision checking
    print("\nRunning TrajOpt planner with upright constraint...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    assert result.successful, f"Planning failed: {result.message}"
    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints:")
    for i, point in enumerate(result.trajectory):
        print(f"  [{i}] {point.positions}")

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
