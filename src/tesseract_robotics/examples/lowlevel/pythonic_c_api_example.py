#!/usr/bin/env python
"""
Pythonic API Example - High-Level Wrapper over C++ Bindings

This example demonstrates how the tesseract_robotics.planning module provides a
Pythonic wrapper layer over the low-level C++ bindings (nanobind). The goal is to
reduce boilerplate and hide the complexity of poly-type wrapping while preserving
full access to the underlying Tesseract capabilities.

Pipeline Overview
-----------------
The high-level API abstracts three major pain points of the raw C++ bindings:

1. Environment Setup (Robot class)
   Low-level: 10+ lines with FilesystemPath, GeneralResourceLocator, Environment.init()
   High-level: Robot.from_tesseract_support("abb_irb2400")

2. Poly-Type Wrapping (MotionProgram class)
   Low-level: Manual wrapping with CartesianWaypointPoly_wrap_CartesianWaypoint(),
              MoveInstructionPoly_wrap_MoveInstruction(), AnyPoly_wrap_*(), etc.
   High-level: program.move_to(CartesianTarget(pose)) - wrapping is automatic

3. Obstacle Creation (create_obstacle helper)
   Low-level: 20+ lines with Link, Joint, Visual, Collision, AddLinkCommand
   High-level: create_obstacle(robot, name="box", geometry=box(1,1,1), transform=pose)

Key Concepts
------------
Robot: Wraps tesseract_environment.Environment with:
  - from_urdf()/from_files()/from_tesseract_support() factory methods
  - fk()/ik() for kinematics (wraps KinematicGroup)
  - set_joints()/get_state() for state management
  - add_link()/remove_link() for scene modifications

MotionProgram: Fluent builder that produces CompositeInstruction:
  - move_to()/linear_to() for adding waypoints
  - Accepts CartesianTarget, JointTarget, StateTarget
  - Automatically wraps all types in their Poly containers
  - set_joint_names() binds joint ordering for JointTarget

Pose: Pythonic wrapper around Isometry3d:
  - from_xyz()/from_xyz_quat() factory methods
  - @ operator for chaining transforms (rotation_z(1.57) @ translation(0.5, 0, 0))
  - .position/.quaternion properties for numpy access

plan_freespace(): Convenience function that:
  - Creates TaskComposer from default config
  - Converts MotionProgram to CompositeInstruction
  - Handles AnyPoly wrapping for TaskComposerDataStorage
  - Returns PlanningResult with trajectory extraction

Why Use This API?
-----------------
- Reduces typical planning setup from 100+ lines to ~20 lines
- No need to understand poly-type system (AnyPoly, WaypointPoly, InstructionPoly)
- Numpy integration for trajectories and transforms
- Type hints and IDE autocomplete support
- Still allows dropping to low-level API when needed via .env, .to_composite_instruction()

When to Use Low-Level API Instead
---------------------------------
- Custom planner pipelines not in TaskComposer config
- Direct access to planner-specific options (OMPL algorithm selection)
- Custom time parameterization settings
- Performance-critical code (avoid Python object overhead)

Related Examples
----------------
- tesseract_planning_composer_c_api_example.py: TaskComposer with high-level API
- tesseract_planning_lowlevel_c_api_example.py: Raw planner access without TaskComposer
- freespace_ompl_c_api_example.py: OMPL planning with pythonic wrappers
- basic_cartesian_c_api_example.py: TrajOpt Cartesian planning
"""

import sys

from tesseract_robotics.planning import (
    CartesianTarget,
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    box,
    create_obstacle,
    plan_freespace,
    rotation_z,
    sphere,
    translation,
)


def main():
    print("=" * 60)
    print("Pythonic API Example")
    print("=" * 60)

    # =========================================================================
    # 1. Load Robot
    # =========================================================================
    # WHY: The low-level API requires FilesystemPath, GeneralResourceLocator,
    # and explicit Environment.init() calls. Robot.from_tesseract_support()
    # encapsulates all this and resolves package:// URLs automatically.
    # Compare with tesseract_planning_lowlevel_c_api_example.py lines 52-62.
    print("\n1. Loading robot...")

    robot = Robot.from_tesseract_support("abb_irb2400")

    print(f"   Loaded: {robot}")
    print(f"   Links: {robot.get_link_names()[:5]}...")
    print(f"   Manipulator joints: {robot.get_joint_names('manipulator')}")

    # =========================================================================
    # 2. Robot State - Simple state access
    # =========================================================================
    # WHY: Low-level requires env.setState(joint_names, np.array(...)) with
    # explicit numpy array creation. The Robot wrapper accepts dicts or arrays
    # and handles the conversion, preventing common dtype/order errors.
    print("\n2. Accessing robot state...")

    state = robot.get_state()
    print(f"   Current state: {state}")

    # WHY dict input: Avoids joint ordering errors - the wrapper extracts
    # names and values in correct order for Environment.setState()
    robot.set_joints(
        {
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "joint_6": 0.0,
        }
    )
    print("   Set joints to zero position")

    # =========================================================================
    # 3. Forward Kinematics - One-liner
    # =========================================================================
    # WHY: Low-level requires getKinematicGroup(), then calcFwdKin(), then
    # extracting the right link from the result dict. robot.fk() handles
    # group lookup and returns a Pose object with convenient accessors.
    print("\n3. Forward kinematics...")

    pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0])
    print(f"   FK at zeros: {pose}")
    print(f"   Position: x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}")

    # =========================================================================
    # 4. Poses - Clean API
    # =========================================================================
    # WHY: Tesseract uses Eigen Isometry3d internally, requiring Translation3d,
    # Quaterniond, and matrix multiplication. The Pose class provides:
    # - Factory methods that accept separate x,y,z,qx,qy,qz,qw args
    # - @ operator for composing transforms (maps to matrix multiplication)
    # - .position/.quaternion properties returning numpy arrays
    print("\n4. Pose helpers...")

    # WHY from_xyz_quat: Quaternion order is (x,y,z,w) to match scipy/ROS
    # convention. Internally converts to Quaterniond(w,x,y,z) for Eigen.
    t1 = Pose.from_xyz_quat(0.5, 0, 0.8, 0, 0, 0.707, 0.707)
    print(f"   From xyz_quat: {t1}")

    # WHY @ chaining: Reads left-to-right as "translate then rotate"
    # Equivalent to t2 = Isometry3d.Identity() * Translation3d(...) * Quaterniond(...)
    t2 = translation(0.5, 0, 0.8) @ rotation_z(1.57)
    print(f"   From factories: {t2}")

    # WHY numpy properties: Direct interop with numpy/scipy without manual extraction
    print(f"   Position array: {t1.position}")
    print(f"   Quaternion: {t1.quaternion}")

    # =========================================================================
    # 5. Add Obstacles
    # =========================================================================
    # WHY: Low-level obstacle creation requires:
    # 1. Create geometry (tesseract_geometry.Box)
    # 2. Create Visual/Collision with geometry
    # 3. Create Link containing Visual/Collision
    # 4. Create fixed Joint connecting to parent link
    # 5. Create AddLinkCommand and apply to environment
    # create_obstacle() does all this in one call, with sensible defaults.
    print("\n5. Adding obstacles...")

    # WHY single function call: Encapsulates all the Link/Joint/Command boilerplate
    create_obstacle(
        robot,
        name="table",
        geometry=box(0.8, 0.8, 0.05),
        transform=Pose.from_xyz(0.5, 0, 0.3),
        color=(0.6, 0.4, 0.2, 1.0),
    )
    print("   Added table obstacle")

    # Add a sphere obstacle
    create_obstacle(
        robot,
        name="ball",
        geometry=sphere(0.1),
        transform=Pose.from_xyz(0.4, 0.2, 0.6),
        color=(1.0, 0.0, 0.0, 1.0),
    )
    print("   Added ball obstacle")

    # =========================================================================
    # 6. Motion Program - Fluent builder API
    # =========================================================================
    # WHY: The low-level C++ bindings require explicit poly-type wrapping:
    #   CartesianWaypointPoly_wrap_CartesianWaypoint(waypoint)
    #   MoveInstructionPoly_wrap_MoveInstruction(instruction)
    # This is because C++ templates don't translate directly to Python.
    # MotionProgram handles all wrapping internally via to_composite_instruction().
    print("\n6. Building motion program...")

    # WHY fluent API: Method chaining (.move_to().move_to()) mirrors how motion
    # programs are conceptualized - a sequence of waypoints to visit.
    # Returns CompositeInstruction with all poly types properly wrapped.
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(robot.get_joint_names("manipulator"))
        # Start at current position (joints)
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        # Move to Cartesian target
        .move_to(CartesianTarget(Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.707, 0, 0.707, 0)))
        # Another Cartesian target
        .move_to(
            CartesianTarget(
                position=[0.8, 0.2, 0.8],
                quaternion=[0.707, 0, 0.707, 0],
            )
        )
        # Back to joint target
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    )

    print(f"   Created program with {len(program)} targets")

    # =========================================================================
    # 7. Plan Motion
    # =========================================================================
    # WHY: TaskComposer requires wrapping data in AnyPoly containers:
    #   AnyPoly_wrap_CompositeInstruction(program)
    #   AnyPoly_wrap_ProfileDictionary(profiles)
    #   AnyPoly_wrap_EnvironmentConst(environment)
    # Then setting up TaskComposerDataStorage, running the executor, and
    # extracting results via AnyPoly_as_CompositeInstruction().
    # plan_freespace() handles all this and returns a PlanningResult with
    # trajectory points extracted from the raw CompositeInstruction.
    print("\n7. Planning motion...")

    # WHY plan_freespace: Creates default TrajOpt profiles and executes
    # the TrajOptPipeline. For OMPL+TrajOpt, use TaskComposer directly.
    result = plan_freespace(robot, program)

    if result.successful:
        print("   Planning successful!")
        print(f"   Trajectory has {len(result)} waypoints")

        # WHY result iteration: PlanningResult implements __iter__ and __getitem__
        # so you can treat it like a list of TrajectoryPoint objects
        if result.trajectory:
            print(f"   First point: {result[0].positions}")
            print(f"   Last point: {result[-1].positions}")

            # WHY to_numpy: Common workflow is to pass trajectory to numpy/scipy
            # for analysis, interpolation, or export. to_numpy() returns (N, num_joints).
            trajectory_array = result.to_numpy()
            print(f"   Trajectory shape: {trajectory_array.shape}")
    else:
        print(f"   Planning failed: {result.message}")

    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
