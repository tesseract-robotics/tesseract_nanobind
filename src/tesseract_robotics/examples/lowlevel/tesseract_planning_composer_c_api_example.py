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

This example does each step with the bindings; `tesseract_robotics.planning.TaskComposer`
wraps the same steps for the high-level examples.

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
- tesseract_planning_lowlevel_c_api_example.py: Direct planner access
- freespace_ompl_c_api_example.py: OMPL planning via TaskComposer
- basic_cartesian_c_api_example.py: TrajOpt Cartesian planning
"""

import sys

import numpy as np

from tesseract_robotics import get_task_composer_config_path
from tesseract_robotics.examples.lowlevel.example_support import locate_file
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLRealVectorMoveProfile,
    RRTConnectConfigurator,
)
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerPluginFactory,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
PIPELINE = "FreespacePipeline"
NUM_OMPL_PLANNERS = 2
OMPL_PLANNING_TIME_S = 10.0

# Target orientation, scalar-last (x, y, z, w): 180 deg about the (1, 0, 1) axis.
TOOL_XYZW = (0.707, 0.0, 0.707, 0.0)


def run():
    locator = GeneralResourceLocator()
    urdf = locate_file(locator, "package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locate_file(locator, "package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    if not env.init(urdf, srdf, locator):
        raise RuntimeError("failed to initialize abb_irb2400")
    print(f"Loaded {env.getName()}")

    # A seed away from the all-zero singularity.
    joint_names = [jid.name() for jid in env.getJointGroup("manipulator").getJointIds()]
    env.setState(joint_names, np.full(len(joint_names), 0.1))

    # Two Cartesian FREESPACE targets: the pipeline solves IK for them.
    program = CompositeInstruction("DEFAULT", ManipulatorInfo("manipulator", "base_link", "tool0"))
    for y in (-0.3, 0.3):
        pose = (
            Isometry3d.Identity() * Translation3d(0.8, y, 1.455) * Quaterniond.from_xyzw(*TOOL_XYZW)
        )
        program.push_back(
            MoveInstructionPoly(
                MoveInstruction(
                    CartesianWaypointPoly(CartesianWaypoint(pose)),
                    MoveInstructionType.FREESPACE,
                    "DEFAULT",
                )
            )
        )
    print(f"\nProgram has {program.size()} instructions")

    # OMPL needs at least one planner configurator; TrajOpt and the post-processing
    # tasks run their default profiles.
    ompl_profile = OMPLRealVectorMoveProfile()
    ompl_profile.solver_config.planning_time = OMPL_PLANNING_TIME_S
    ompl_profile.solver_config.planners = [
        RRTConnectConfigurator() for _ in range(NUM_OMPL_PLANNERS)
    ]
    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "DEFAULT", ompl_profile)

    # The factory reads the pipeline graphs and plugin libraries from the YAML config.
    factory = TaskComposerPluginFactory(get_task_composer_config_path(), locator)
    task = factory.createTaskComposerNode(PIPELINE)
    input_key = task.getInputPortMappings().single("planning_input")
    output_key = task.getOutputPortMappings().single("program")
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Task data crosses the plugin boundary type-erased, as AnyPoly.
    data = TaskComposerDataStorage()
    data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
    data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print(f"Running {PIPELINE} (OMPL + TrajOpt)...")
    future = executor.run(task, data)
    future.wait()
    if not future.context.isSuccessful():
        print("Planning failed")
        return False
    print("Planning successful!")

    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))
    states = [
        instruction.asMoveInstruction().getWaypoint().asStateWaypoint()
        for instruction in results
        if instruction.isMoveInstruction()
        and instruction.asMoveInstruction().getWaypoint().isStateWaypoint()
    ]
    print(f"\nTrajectory has {len(states)} waypoints:")
    for i, state in enumerate(states):
        print(f"  [{i:2d}] {np.asarray(state.getPosition())} t={state.getTime():.3f}")

    return {"env": env, "success": True, "trajectory": results}


def main() -> int:
    results = run()
    if not results:
        return 1

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        if results["trajectory"] is not None:
            viewer.update_trajectory(results["trajectory"])
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0 if results["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
