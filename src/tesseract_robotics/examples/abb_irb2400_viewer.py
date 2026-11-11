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
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
)
from tesseract_robotics.common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.environment import Environment
from tesseract_robotics.examples.failures import TimeParameterizationFailedError, require_success
from tesseract_robotics.motion_planners import PlannerRequest
from tesseract_robotics.motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorMoveProfile,
)
from tesseract_robotics.motion_planners_simple import (
    generateInterpolatedProgram,
)
from tesseract_robotics.motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
    TrajOptMotionPlanner,
)
from tesseract_robotics.planning.errors import EnvironmentInitError, ResourceNotFoundError
from tesseract_robotics.time_parameterization import (
    TimeOptimalTrajectoryGeneration,
    TimeOptimalTrajectoryGenerationCompositeProfile,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

# The ABB IRB 2400's six joints, and the pose the environment starts in [rad].
JOINT_COUNT = 6
START_JOINT_RAD = 0.1
# The two Cartesian waypoints on base_link [m]: shared x and z, then each one's y.
WAYPOINT_X_M = 0.8
WAYPOINT_Z_M = 1.455
WAYPOINT_1_Y_M = -0.3
WAYPOINT_2_Y_M = 0.3
# Tool orientation, scalar-last [qx, qy, qz, qw]: a quarter turn about y (sin and cos of
# pi/4 to eight digits).
TOOL_QUAT_XYZW = (0.0, 0.70710678, 0.0, 0.70710678)
# Interpolation limits for generateInterpolatedProgram: state, translation and rotation
# longest valid segments [rad, m, rad], and the minimum steps per segment [-].
INTERP_STATE_LVS = 3.14
INTERP_TRANSLATION_LVS_M = 1.0
INTERP_ROTATION_LVS_RAD = 3.14
INTERP_MIN_STEPS = 10
# Velocity and acceleration scaling of the time parameterisation [-].
SCALING_FACTOR = 1.0
# Axes drawn on each plotted waypoint [m].
AXES_LENGTH_M = 0.05


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource.

    Raises:
        ResourceNotFoundError: the locator resolves nothing for `url`.
    """
    resource = locator.locateResource(url)
    if resource is None:
        raise ResourceNotFoundError(f"Could not locate resource: {url}")
    return Path(resource.getFilePath())


def run():
    # --8<-- [start:load_robot]
    # Load robot
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    abb_irb2400_urdf_package_url = "package://tesseract/support/urdf/abb_irb2400.urdf"
    abb_irb2400_srdf_package_url = "package://tesseract/support/urdf/abb_irb2400.srdf"
    abb_irb2400_urdf_fname = locate(locator, abb_irb2400_urdf_package_url)
    abb_irb2400_srdf_fname = locate(locator, abb_irb2400_srdf_package_url)

    t_env = Environment()
    if not t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator):
        raise EnvironmentInitError(
            f"failed to initialize the environment from {abb_irb2400_urdf_fname}"
        )

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    # --8<-- [end:load_robot]

    joint_names = [f"joint_{i + 1}" for i in range(JOINT_COUNT)]
    t_env.setState(joint_names, np.ones(JOINT_COUNT) * START_JOINT_RAD)

    # Define waypoints
    wp1 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(WAYPOINT_X_M, WAYPOINT_1_Y_M, WAYPOINT_Z_M)
        * Quaterniond.from_xyzw(*TOOL_QUAT_XYZW)
    )
    wp2 = CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(WAYPOINT_X_M, WAYPOINT_2_Y_M, WAYPOINT_Z_M)
        * Quaterniond.from_xyzw(*TOOL_QUAT_XYZW)
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
    require_success(response, "OMPL")
    results_instruction = response.results

    interpolated_results_instruction = generateInterpolatedProgram(
        results_instruction,
        t_env,
        INTERP_STATE_LVS,
        INTERP_TRANSLATION_LVS_M,
        INTERP_ROTATION_LVS_RAD,
        INTERP_MIN_STEPS,
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
    require_success(trajopt_response, "TrajOpt")
    final_results_instruction = trajopt_response.results

    # Time parameterization of the TrajOpt output (StateWaypoints)
    time_parameterization = TimeOptimalTrajectoryGeneration()

    # Create profile with velocity/acceleration scaling
    totg_profile = TimeOptimalTrajectoryGenerationCompositeProfile()
    totg_profile.max_velocity_scaling_factor = SCALING_FACTOR
    totg_profile.max_acceleration_scaling_factor = SCALING_FACTOR

    # Create profile dictionary
    time_profiles = ProfileDictionary()
    time_profiles.addProfile("TOTG", "DEFAULT", totg_profile)

    if not time_parameterization.compute(final_results_instruction, t_env, time_profiles):
        raise TimeParameterizationFailedError("time parameterization failed")

    print("OMPL + TrajOpt planning completed successfully!")
    print(f"Number of waypoints: {final_results_instruction.size()}")
    return {"env": t_env, "trajectory": final_results_instruction, "manip_info": manip_info}


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        viewer.update_trajectory(results["trajectory"])
        viewer.plot_trajectory(
            results["trajectory"], results["manip_info"], axes_length=AXES_LENGTH_M
        )
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
