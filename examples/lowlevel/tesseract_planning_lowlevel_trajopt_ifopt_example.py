"""
Low-Level Planning with TrajOptIfopt (OSQP-based SQP)

This example demonstrates using OMPL + TrajOptIfopt directly WITHOUT TaskComposer.
TrajOptIfopt uses OSQP (a fast QP solver) instead of sco (sequential convex optimizer).

For real-time/online planning at 100+ Hz, see online_planning_sqp_example.py which
uses the even lower-level trajopt_sqp.TrustRegionSQPSolver API.

Comparison:
- TrajOpt: sco-based optimizer, mature, well-tested
- TrajOptIfopt: OSQP-based optimizer, faster for QP subproblems, IFOPT interface
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
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    ProfileDictionary_addTrajOptIfoptCompositeProfile,
    ProfileDictionary_addTrajOptIfoptPlanProfile,
    ProfileDictionary_addTrajOptIfoptSolverProfile,
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultPlanProfile,
    TrajOptIfoptMotionPlanner,
    TrajOptIfoptOSQPSolverProfile,
)
from tesseract_robotics.tesseract_time_parameterization import (
    TimeOptimalTrajectoryGeneration,
    TOTGCompositeProfile,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_IFOPT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"


def main():
    # Initialize the resource locator and environment
    locator = GeneralResourceLocator()
    urdf_url = "package://tesseract_support/urdf/abb_irb2400.urdf"
    srdf_url = "package://tesseract_support/urdf/abb_irb2400.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator)

    # Manipulator info
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    # Set initial state
    joint_names = [f"joint_{i + 1}" for i in range(6)]
    t_env.setState(joint_names, np.ones(6) * 0.1)

    # Create Cartesian waypoints
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

    # Create move instructions
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

    # Create program
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    # === OMPL Planning ===
    print("Running OMPL planner...")
    ompl_profiles = ProfileDictionary()
    ompl_profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "DEFAULT", OMPLRealVectorPlanProfile())

    ompl_request = PlannerRequest()
    ompl_request.instructions = program
    ompl_request.env = t_env
    ompl_request.profiles = ompl_profiles

    ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
    ompl_response = ompl_planner.solve(ompl_request)
    assert ompl_response.successful, f"OMPL failed: {ompl_response.message}"
    print(f"OMPL found path with {ompl_response.results.size()} instructions")

    # Interpolate OMPL results
    interpolated = generateInterpolatedProgram(ompl_response.results, t_env, 3.14, 1.0, 3.14, 10)

    # === TrajOptIfopt Optimization ===
    print("Running TrajOptIfopt optimizer...")

    # Create profiles with velocity/accel coefficients sized for 6-DOF robot
    plan_profile = TrajOptIfoptDefaultPlanProfile()
    composite_profile = TrajOptIfoptDefaultCompositeProfile()
    composite_profile.velocity_coeff = np.ones(6)
    composite_profile.acceleration_coeff = np.ones(6)
    composite_profile.jerk_coeff = np.ones(6)
    solver_profile = TrajOptIfoptOSQPSolverProfile()

    trajopt_profiles = ProfileDictionary()
    ProfileDictionary_addTrajOptIfoptPlanProfile(
        trajopt_profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", plan_profile
    )
    ProfileDictionary_addTrajOptIfoptCompositeProfile(
        trajopt_profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", composite_profile
    )
    ProfileDictionary_addTrajOptIfoptSolverProfile(
        trajopt_profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", solver_profile
    )

    trajopt_request = PlannerRequest()
    trajopt_request.instructions = interpolated
    trajopt_request.env = t_env
    trajopt_request.profiles = trajopt_profiles

    trajopt_planner = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE)
    trajopt_response = trajopt_planner.solve(trajopt_request)
    assert trajopt_response.successful, f"TrajOptIfopt failed: {trajopt_response.message}"
    print("TrajOptIfopt optimization complete")

    trajopt_results_instruction = trajopt_response.results

    # === Time Parameterization ===
    # 0.33 API: TimeParameterization.compute() takes (CompositeInstruction, Environment, ProfileDictionary)
    # instead of (InstructionsTrajectory, velocity, acceleration, jerk) arrays.
    print("Running time parameterization...")
    time_param = TimeOptimalTrajectoryGeneration()

    # Create profile with velocity/acceleration scaling
    totg_profile = TOTGCompositeProfile()
    totg_profile.max_velocity_scaling_factor = 1.0
    totg_profile.max_acceleration_scaling_factor = 1.0

    # Create profile dictionary for time parameterization
    # Note: namespace is "TOTG" (or any string), profile name is "DEFAULT"
    time_profiles = ProfileDictionary()
    time_profiles.addProfile("TOTG", "DEFAULT", totg_profile)

    assert time_param.compute(trajopt_results_instruction, t_env, time_profiles)
    print("Time parameterization complete")

    # Print results
    trajopt_results = trajopt_results_instruction.getInstructions()
    print(f"\nTrajectory has {len(trajopt_results)} waypoints:")
    for i, instr in enumerate(trajopt_results):
        move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
        wp_poly = move_instr.getWaypoint()
        wp = WaypointPoly_as_StateWaypointPoly(wp_poly)
        pos = wp.getPosition().flatten()
        time = wp.getTime()
        if i < 3 or i >= len(trajopt_results) - 2:
            print(f"  [{i:2d}] t={time:.3f}s  joints={np.array2string(pos, precision=3)}")
        elif i == 3:
            print("  ...")

    # Visualize if viewer available
    if TesseractViewer is not None:
        viewer = TesseractViewer()
        viewer.update_environment(t_env, [0, 0, 0])
        viewer.start_serve_background()
        viewer.update_trajectory(trajopt_results)
        viewer.plot_trajectory(trajopt_results, manip_info, axes_length=0.05)
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
