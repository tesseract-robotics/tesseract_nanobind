"""
ABB IRB2400 motion planning example with viewer.

This example demonstrates:
- Loading robot from URDF/SRDF
- OMPL motion planning
- TrajOpt trajectory optimization
- Time parameterization with TOTG
- Trajectory visualization
"""

import sys
from pathlib import Path

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
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
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorMoveProfile,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    generateInterpolatedProgram,
)
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
    TrajOptMotionPlanner,
)
from tesseract_robotics.tesseract_time_parameterization import (
    TimeOptimalTrajectoryGeneration,
    TimeOptimalTrajectoryGenerationCompositeProfile,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def run():
    # --8<-- [start:load_robot]
    # Load robot
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    abb_irb2400_urdf_package_url = "package://tesseract/support/urdf/abb_irb2400.urdf"
    abb_irb2400_srdf_package_url = "package://tesseract/support/urdf/abb_irb2400.srdf"
    abb_irb2400_urdf_fname = Path(
        locator.locateResource(abb_irb2400_urdf_package_url).getFilePath()
    )
    abb_irb2400_srdf_fname = Path(
        locator.locateResource(abb_irb2400_srdf_package_url).getFilePath()
    )

    t_env = Environment()
    assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    # --8<-- [end:load_robot]

    joint_names = [f"joint_{i + 1}" for i in range(6)]
    t_env.setState(joint_names, np.ones(6) * 0.1)

    # Define waypoints
    wp1 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, -0.3, 1.455)
        * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
    )
    wp2 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(0.8, 0.3, 1.455)
        * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
    )
    start_instruction = MoveInstruction(
        CartesianWaypointPoly(wp1),
        MoveInstructionType.FREESPACE,
        "DEFAULT",
    )
    plan_f1 = MoveInstruction(
        CartesianWaypointPoly(wp2),
        MoveInstructionType.FREESPACE,
        "DEFAULT",
    )
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.push_back(MoveInstructionPoly(start_instruction))
    program.push_back(MoveInstructionPoly(plan_f1))

    # --8<-- [start:plan_and_animate]
    # OMPL planning
    plan_profile = OMPLRealVectorMoveProfile()
    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

    request = PlannerRequest()
    request.instructions = program
    request.env = t_env
    request.profiles = profiles

    ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
    response = ompl_planner.solve(request)
    assert response.successful
    results_instruction = response.results

    interpolated_results_instruction = generateInterpolatedProgram(
        results_instruction, t_env, 3.14, 1.0, 3.14, 10
    )
    # --8<-- [end:plan_and_animate]

    # TrajOpt trajectory optimization for smooth trajectories
    trajopt_plan_profile = TrajOptDefaultMoveProfile()
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()

    trajopt_profiles = ProfileDictionary()
    trajopt_profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_plan_profile)
    trajopt_profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile)

    trajopt_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

    trajopt_request = PlannerRequest()
    trajopt_request.instructions = interpolated_results_instruction
    trajopt_request.env = t_env
    trajopt_request.profiles = trajopt_profiles

    trajopt_response = trajopt_planner.solve(trajopt_request)
    assert trajopt_response.successful, f"TrajOpt failed: {trajopt_response.message}"
    final_results_instruction = trajopt_response.results

    # Time parameterization of the TrajOpt output (StateWaypoints)
    time_parameterization = TimeOptimalTrajectoryGeneration()

    # Create profile with velocity/acceleration scaling
    totg_profile = TimeOptimalTrajectoryGenerationCompositeProfile()
    totg_profile.max_velocity_scaling_factor = 1.0
    totg_profile.max_acceleration_scaling_factor = 1.0

    # Create profile dictionary
    time_profiles = ProfileDictionary()
    time_profiles.addProfile("TOTG", "DEFAULT", totg_profile)

    assert time_parameterization.compute(final_results_instruction, t_env, time_profiles)

    print("OMPL + TrajOpt planning completed successfully!")
    print(f"Number of waypoints: {final_results_instruction.size()}")
    return {"env": t_env, "trajectory": final_results_instruction, "manip_info": manip_info}


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        viewer.update_trajectory(results["trajectory"])
        viewer.plot_trajectory(results["trajectory"], results["manip_info"], axes_length=0.05)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
