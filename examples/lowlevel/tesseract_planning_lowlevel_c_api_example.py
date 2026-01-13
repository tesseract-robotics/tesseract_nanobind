"""
Low-Level Planning Example (Advanced)

This example demonstrates using individual Tesseract planners directly WITHOUT
the TaskComposer. This gives fine-grained control over each planning step.

For most use cases, prefer the high-level API (see tesseract_planning_example_composer.py):
    from tesseract_robotics.planning import Robot, MotionProgram, plan_freespace
    result = plan_freespace(robot, program)

Use this low-level approach when you need:
- Custom planner pipelines not available in TaskComposer
- Fine-grained control over individual planners (OMPL, TrajOpt)
- Custom time parameterization settings
- To bypass the TaskComposer config system
"""

import sys

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorPlanProfile,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    generateInterpolatedProgram,
)
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultPlanProfile,
    TrajOptMotionPlanner,
)
from tesseract_robotics.tesseract_time_parameterization import (
    TimeOptimalTrajectoryGeneration,
    TOTGCompositeProfile,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def main():
    # Initialize the resource locator and environment
    locator = GeneralResourceLocator()
    abb_irb2400_urdf_package_url = "package://tesseract_support/urdf/abb_irb2400.urdf"
    abb_irb2400_srdf_package_url = "package://tesseract_support/urdf/abb_irb2400.srdf"
    abb_irb2400_urdf_fname = FilesystemPath(
        locator.locateResource(abb_irb2400_urdf_package_url).getFilePath()
    )
    abb_irb2400_srdf_fname = FilesystemPath(
        locator.locateResource(abb_irb2400_srdf_package_url).getFilePath()
    )

    t_env = Environment()

    # locator_fn must be kept alive by maintaining a reference
    assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

    # Fill in the manipulator information. This is used to find the kinematic chain
    # for the manipulator. This must match the SRDF, although the exact tcp_frame
    # can differ if a tool is used.
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    # Set the initial state of the robot
    joint_names = [f"joint_{i + 1}" for i in range(6)]
    t_env.setState(joint_names, np.ones(6) * 0.1)

    # Create the input command program waypoints
    wp1 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, -0.3, 1.455)
        * Quaterniond(0.70710678, 0, 0.70710678, 0)
    )
    wp2 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, 0.3, 1.455)
        * Quaterniond(0.70710678, 0, 0.70710678, 0)
    )

    # Create the input command program instructions. Note the use of explicit
    # construction of the CartesianWaypointPoly using *_wrap_CartesianWaypoint
    # functions. This is required because the Python bindings do not support
    # implicit conversion from CartesianWaypoint to CartesianWaypointPoly.
    start_instruction = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )
    plan_f1 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )

    # Create the input command program. Note the use of *_wrap_MoveInstruction
    # functions. This is required because the Python bindings do not support
    # implicit conversion from MoveInstruction to MoveInstructionPoly.
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    # Initialize the OMPL planner for RRTConnect algorithm
    plan_profile = OMPLRealVectorPlanProfile()

    # Create the profile dictionary. Profiles can be used to customize the behavior
    # of the planner. The ProfileDictionary.addProfile method is used to add a
    # profile to the dictionary. All profile types have associated namespace strings.
    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

    # Create the planning request and run the planner
    request = PlannerRequest()
    request.instructions = program
    request.env = t_env
    request.profiles = profiles

    ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)

    response = ompl_planner.solve(request)
    assert response.successful
    results_instruction = response.results

    # The OMPL program does not generate dense waypoints. This function will
    # interpolate the results to generate a dense set of waypoints.
    interpolated_results_instruction = generateInterpolatedProgram(
        results_instruction, t_env, 3.14, 1.0, 3.14, 10
    )

    # Create the TrajOpt planner profile configurations. TrajOpt is used to
    # optimize the sparse random program generated by OMPL.
    trajopt_plan_profile = TrajOptDefaultPlanProfile()
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()

    trajopt_profiles = ProfileDictionary()
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_plan_profile)
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile)

    # Create the TrajOpt planner
    trajopt_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

    # Create the TrajOpt planning request and run the planner
    trajopt_request = PlannerRequest()
    trajopt_request.instructions = interpolated_results_instruction
    trajopt_request.env = t_env
    trajopt_request.profiles = trajopt_profiles

    trajopt_response = trajopt_planner.solve(trajopt_request)
    assert trajopt_response.successful

    trajopt_results_instruction = trajopt_response.results

    # The TrajOpt planner does not assign timestamps to the results. This function
    # will assign timestamps using the TimeOptimalTrajectoryGeneration class.
    # 0.33 API: TimeParameterization.compute() takes (CompositeInstruction, Environment, ProfileDictionary)
    # instead of (InstructionsTrajectory, velocity, acceleration, jerk) arrays.
    # Velocity/acceleration limits are set via TOTGCompositeProfile or taken from Environment.
    time_parameterization = TimeOptimalTrajectoryGeneration()

    # Create profile with velocity/acceleration scaling
    totg_profile = TOTGCompositeProfile()
    totg_profile.max_velocity_scaling_factor = 1.0
    totg_profile.max_acceleration_scaling_factor = 1.0

    # Create profile dictionary for time parameterization
    # Note: namespace is "TOTG" (or any string), profile name is "DEFAULT"
    time_profiles = ProfileDictionary()
    time_profiles.addProfile("TOTG", "DEFAULT", totg_profile)

    assert time_parameterization.compute(trajopt_results_instruction, t_env, time_profiles)

    # Get the results as a list of instructions
    trajopt_results = trajopt_results_instruction.getInstructions()

    # Print out the resulting waypoints
    for instr in trajopt_results:
        assert instr.isMoveInstruction()
        move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
        wp1 = move_instr1.getWaypoint()
        assert wp1.isStateWaypoint()
        wp = WaypointPoly_as_StateWaypointPoly(wp1)
        print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")

    # Visualize if viewer available
    if TesseractViewer is not None:
        viewer = TesseractViewer()
        viewer.update_environment(t_env, [0, 0, 0])
        viewer.update_joint_positions(joint_names, np.array([1, -0.2, 0.01, 0.3, -0.5, 1]))
        viewer.start_serve_background()
        viewer.update_trajectory(trajopt_results)
        viewer.plot_trajectory(trajopt_results, manip_info, axes_length=0.05)
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
