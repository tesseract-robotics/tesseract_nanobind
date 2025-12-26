"""
Glass Upright Example - Orientation-Constrained Motion Planning (High-Level API)

Demonstrates TrajOpt planning with orientation constraints: the robot moves while
keeping the tool "upright" (like carrying a glass of water without spilling).
Only position changes; orientation remains constant throughout the trajectory.

Pipeline Overview:
    1. Load KUKA IIWA 7-DOF robot from tesseract_support
    2. Add sphere obstacle at (0.5, 0, 0.55) with radius 0.15m to force non-trivial path
    3. Define start/end via joint states (only joint_a1 changes: -0.4 -> 0.4 rad)
    4. Plan with TrajOptPipeline using UPRIGHT profile for orientation constraints
    5. TrajOpt optimizes trajectory while respecting orientation + collision constraints

Concept: "UPRIGHT" Profile Constraint
    The UPRIGHT profile applies Cartesian constraints with coeff=[0,0,0,5,5,5]:
        - Position (x,y,z): coeff=0 -> unconstrained, robot can move freely in space
        - Orientation (rx,ry,rz): coeff=5 -> strongly constrained, must stay upright

    This is the INVERSE of typical constraints that fix position but allow rotation.
    The high orientation weights (5) penalize any deviation from the initial tool
    orientation during trajectory optimization.

Key Concepts:
    - UPRIGHT profile: Custom TrajOpt profile constraining orientation, not position
    - 7-DOF redundancy: Allows maintaining orientation while repositioning (essential!)
    - LINEAR motion type: Cartesian interpolation at intermediate waypoints
    - StateTarget: Full joint configuration (FK determines Cartesian pose)

Motion Type:
    - linear_to(StateTarget): LINEAR motion with Cartesian interpolation
    - UPRIGHT profile applied to enforce orientation during optimization

C++ Source: tesseract_planning/tesseract_examples/src/glass_upright_example.cpp

C++ Profile Configuration (verified):
    Composite Profile (UPRIGHT):
        - collision_cost: enabled, safety_margin=0.01, type=DISCRETE_CONTINUOUS
        - collision_constraint: enabled, safety_margin=0.01
        - smooth_velocities: true
    Plan Profile (UPRIGHT):
        - cartesian_constraint: enabled
        - coeff = [0, 0, 0, 5, 5, 5]  (position free, orientation constrained)
        - joint_cost: disabled

C++ Parameters (verified):
    - Robot: KUKA LBR IIWA 14 R820 (7-DOF)
    - Obstacle: Sphere r=0.15m at (0.5, 0, 0.55)
    - Start: joint_a1=-0.4 rad, others=[0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
    - End: joint_a1=0.4 rad (same other joints)
    - Motion: LINEAR with UPRIGHT profile

Related Examples:
    - basic_cartesian_example.py - FREESPACE + LINEAR without orientation constraint
    - freespace_ompl_example.py - Joint-space planning (no Cartesian constraints)
    - lowlevel/glass_upright_c_api_example.py - Same with low-level API
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
from tesseract_robotics.planning.profiles import (
    create_freespace_pipeline_profiles,
    create_trajopt_upright_profiles,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run glass upright example with orientation-constrained planning.

    Plans a trajectory where the tool orientation remains constant while the
    robot moves around a sphere obstacle. The 7-DOF KUKA IIWA has sufficient
    redundancy to maintain orientation while changing position.

    Args:
        pipeline: Planning pipeline to use. Options:
            - "TrajOptPipeline" (default): TrajOpt with UPRIGHT constraints
            - "FreespacePipeline": OMPL (ignores orientation constraints!)
        num_planners: Number of parallel OMPL planners (only for FreespacePipeline).

    Returns:
        dict with keys:
            - result: PlanningResult with trajectory and success status
            - robot: Robot instance with environment state
            - joint_names: List of 7 KUKA IIWA joint names

    Note:
        FreespacePipeline does NOT enforce orientation constraints - it only
        respects collision avoidance. Use TrajOptPipeline for orientation control.
    """
    # Load KUKA IIWA 7-DOF robot
    # 7-DOF is essential for orientation constraints - allows maintaining tool pose
    # while repositioning (not possible with 6-DOF arm without external axis)
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle to force non-trivial trajectory
    # Without obstacle, robot could just rotate joint_a1 (trivial solution)
    # Sphere positioned near path to influence trajectory, not block it
    # C++ params: sphere(0.15) at (0.5, 0, 0.55)
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.15),  # 15cm radius (C++ default)
        transform=Pose.from_xyz(0.5, 0, 0.55),  # C++ position
    )
    print("Added sphere obstacle r=0.15 at (0.5, 0, 0.55)")

    # Get joint names for manipulator group
    joint_names = robot.get_joint_names("manipulator")

    # Start/end configurations from C++ example
    # Only joint_a1 changes (-0.4 -> 0.4 rad = ~46 degrees)
    # This creates a horizontal "sweep" while maintaining tool orientation
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial state for planning
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # Build motion program with UPRIGHT profile
    # UPRIGHT profile: coeff=[0,0,0,5,5,5] constrains orientation, frees position
    # LINEAR motion type ensures Cartesian interpolation at intermediate points
    # StateTarget specifies joint config - TrajOpt uses FK for Cartesian constraint
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="UPRIGHT")
        .set_joint_names(joint_names)
        .linear_to(StateTarget(joint_start_pos, names=joint_names, profile="UPRIGHT"))
        .linear_to(StateTarget(joint_end_pos, names=joint_names, profile="UPRIGHT"))
    )

    print("\nProgram: LINEAR motion with UPRIGHT profile")
    print("Orientation locked (coeff=[0,0,0,5,5,5]), position free")

    # Select profiles - TrajOpt required for orientation constraints
    if "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        # UPRIGHT profile: C++ glass_upright collision+cartesian constraint settings
        profiles = create_trajopt_upright_profiles()

    # Execute TrajOpt planning with orientation constraints
    print(f"\nRunning planner ({pipeline})...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

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
    main()
