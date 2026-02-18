"""
Task Composer Planning Example - High-Level Pipeline Orchestration

This example demonstrates the TaskComposer system, which orchestrates multi-stage
planning pipelines. TaskComposer is Tesseract's workflow engine for combining
planners (OMPL, TrajOpt, Descartes) with post-processing (interpolation, time
parameterization) into reusable pipelines defined in YAML configuration.

Pipeline Overview
-----------------
TaskComposer executes task graphs loaded from YAML config files. Each pipeline
is a directed graph of tasks:

FreespacePipeline (used in this example):
  1. OMPL planner (RRTConnect/RRT*/PRM) - finds collision-free path
  2. Interpolation - adds intermediate waypoints for smoothness
  3. TrajOpt optimization - refines path for smoothness and collision margins
  4. Time parameterization - adds velocity/acceleration respecting limits

CartesianPipeline:
  1. Descartes graph search - finds IK solutions along Cartesian path
  2. TrajOpt optimization - smooths joint-space trajectory
  3. Time parameterization

TrajOptPipeline:
  1. TrajOpt only - optimization-based planning (may fail if initial guess is poor)

Key Concepts
------------
TaskComposerPluginFactory: Loads pipeline definitions and planner plugins from YAML.
  - Default config: tesseract_planning/config/task_composer_plugins.yaml
  - Plugins are loaded dynamically (libtesseract_task_composer_planning_nodes.dylib)
  - Environment variable: TESSERACT_TASK_COMPOSER_CONFIG_FILE

TaskComposerExecutor: Runs task graphs (usually TaskflowTaskComposerExecutor).
  - Thread pool for parallel task execution
  - Can be configured in YAML or created programmatically

TaskComposerDataStorage: Type-erased data container for task I/O.
  - All data wrapped in AnyPoly (type erasure for plugin boundary crossing)
  - Keys: "planning_input", "environment", "profiles", "program" (output)

ProfileDictionary: Collection of planner-specific configuration profiles.
  - Namespace pattern: "{PlannerName}Task" (e.g., "OMPLMotionPlannerTask")
  - Profiles can be overridden per-waypoint using profile names in MoveInstruction

AnyPoly Wrapping: Required at plugin boundaries because C++ RTTI doesn't work
  across shared library boundaries. The TaskComposer uses type erasure (AnyPoly)
  to pass data between dynamically loaded planner plugins.
  - AnyPoly_wrap_CompositeInstruction(program)
  - AnyPoly_wrap_ProfileDictionary(profiles)
  - AnyPoly_wrap_EnvironmentConst(environment)

The Pythonic TaskComposer class hides all this complexity:
  composer = TaskComposer.from_config()
  result = composer.plan(robot, program, pipeline="FreespacePipeline")

Configuration Files
-------------------
task_composer_plugins.yaml structure:
  executors:
    TaskflowExecutor: {threads: 4}
  tasks:
    FreespacePipeline:
      class: GraphTaskFactory
      config:
        nodes: [OMPL, Interpolate, TrajOpt, TimeParam]
        edges: [[OMPL, Interpolate], [Interpolate, TrajOpt], ...]

Plugin Resolution:
  - Bundled wheels: plugins in .dylibs/ (macOS) or .libs/ (Linux)
  - Editable install: uses $CONDA_PREFIX/lib paths
  - __init__.py patches @PLUGIN_PATH@ placeholder at import time

Why Use TaskComposer?
---------------------
- Encapsulates complex multi-stage planning workflows
- YAML-configurable without code changes
- Plugin architecture for custom planners
- Handles data marshaling between planners
- Parallel task execution where graph allows

When to Bypass TaskComposer
---------------------------
- Need direct planner access (custom OMPL algorithms, TrajOpt cost functions)
- Want to avoid plugin loading overhead
- Custom time parameterization settings
See: tesseract_planning_lowlevel_c_api_example.py

Related Examples
----------------
- pythonic_c_api_example.py: Robot, MotionProgram, plan_freespace() wrappers
- tesseract_planning_lowlevel_c_api_example.py: Direct planner access
- freespace_ompl_c_api_example.py: OMPL planning via TaskComposer
- basic_cartesian_c_api_example.py: TrajOpt Cartesian planning
"""

import sys

import numpy as np

from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    TaskComposer,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def main():
    # Load ABB IRB2400 robot from tesseract_support package
    # WHY: ABB IRB2400 is a standard 6-DOF industrial robot with well-tested
    # kinematic model. SRDF defines "manipulator" group with tool0 TCP frame.
    robot = Robot.from_tesseract_support("abb_irb2400")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # WHY set initial state: Planners need a seed configuration for IK solving.
    # Cartesian waypoints are converted to joint space using current state as seed.
    # Setting all joints to 0.1 rad avoids singularities at zeros.
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(dict(zip(joint_names, [0.1] * 6)))

    # WHY Cartesian waypoints: Specifying poses in Cartesian space is natural
    # for end-effector tasks. The planner handles IK internally.
    # Quaternion (0.707, 0, 0.707, 0) = 90 deg rotation around Y axis,
    # pointing tool downward (common for pick/place operations).
    wp1 = Pose.from_xyz_quat(0.8, -0.3, 1.455, 0.707, 0, 0.707, 0)
    wp2 = Pose.from_xyz_quat(0.8, 0.3, 1.455, 0.707, 0, 0.707, 0)

    # WHY MotionProgram builder: Abstracts the low-level CompositeInstruction
    # construction. Internally creates CartesianWaypoint, wraps in poly types,
    # and adds to CompositeInstruction with proper ManipulatorInfo.
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)  # WHY: Required for JointTarget, optional for CartesianTarget
        .move_to(
            CartesianTarget(wp1)
        )  # WHY move_to: Sets FREESPACE motion type (collision avoidance)
        .move_to(CartesianTarget(wp2))
    )

    print(f"\nProgram has {len(program)} waypoints")

    # WHY TaskComposer: Orchestrates multi-stage planning pipeline without
    # manual planner setup. from_config() loads YAML defining pipeline graphs.
    # Internally handles: AnyPoly wrapping, ProfileDictionary creation,
    # TaskComposerDataStorage setup, executor management, result extraction.
    print("Running FreespacePipeline (OMPL + TrajOpt)...")
    composer = TaskComposer.from_config()

    # WHY FreespacePipeline: Combines OMPL (global search) + TrajOpt (local optimization)
    # OMPL finds feasible path through complex configuration space,
    # TrajOpt smooths it while enforcing collision/joint limit constraints.
    # Alternative: "TrajOptPipeline" for optimization-only (faster but may fail).
    result = composer.plan(robot, program, pipeline="FreespacePipeline")

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")

    # WHY iterate result: PlanningResult wraps the raw CompositeInstruction
    # and provides TrajectoryPoint objects with numpy arrays for positions,
    # velocities, accelerations, and timestamps. Much easier than manual
    # InstructionPoly_as_MoveInstructionPoly() + WaypointPoly_as_StateWaypointPoly().
    print(f"\nTrajectory has {len(result)} waypoints:")
    for i, point in enumerate(result.trajectory):
        # WHY point.time: FreespacePipeline includes time parameterization,
        # so timestamps are computed from joint velocity/acceleration limits.
        time_str = f" t={point.time:.3f}" if point.time else ""
        print(f"  [{i:2d}] {point.positions}{time_str}")

    # WHY raw_results: PlanningResult stores both extracted TrajectoryPoints
    # (for easy iteration) and the raw CompositeInstruction (for visualization
    # and low-level access). The viewer requires the raw CompositeInstruction format.
    if TesseractViewer is not None and result.raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        # WHY robot.env: Access underlying Environment for viewer
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_joint_positions(joint_names, np.array([1, -0.2, 0.01, 0.3, -0.5, 1]))
        viewer.start_serve_background()
        # WHY raw_results: Viewer animates the CompositeInstruction directly
        viewer.update_trajectory(result.raw_results)
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
