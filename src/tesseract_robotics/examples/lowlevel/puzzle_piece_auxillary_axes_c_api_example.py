"""
Puzzle Piece Auxiliary Axes Example

Cartesian path planning with 9-DOF (KUKA IIWA 7-DOF + 2-DOF positioner).
This demonstrates planning for a multi-chain kinematic system where auxiliary
axes (positioner) can orient the workpiece to improve reachability.

C++ Source: tesseract_planning/examples/src/puzzle_piece_auxillary_axes_example.cpp

Bindings only, line for line with the C++ (#42).

Key C++ Implementation Details:
- 9-DOF system: 7 arm joints + 2 auxiliary positioner joints
- CSV toolpath positions are in millimeters (converted to meters here)
- TrajOpt Cartesian constraint coeff[5]=0 allows yaw rotation freedom
  so auxiliary axes can optimize orientation while maintaining position
- Uses "manipulator_aux" kinematic group (defined in SRDF)
- puzzle_bent.csv contains ~50 waypoints for grinding puzzle piece edge

Robot Configuration:
- KUKA IIWA 7-DOF arm (joint_a1 through joint_a7)
- 2-DOF positioner (joint_aux1=Z rotation, joint_aux2=tilt)
- "grinder_frame" TCP attached to arm end-effector
- "part" working frame on positioner (moves with aux axes)
"""

import sys

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import CollisionEvaluatorType
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
)
from tesseract_robotics.common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.examples.lowlevel.example_support import (
    load_support_environment,
    run_pipeline,
)
from tesseract_robotics.examples.lowlevel.puzzle_piece_c_api_example import (
    MAX_ITER,
    MIN_APPROX_IMPROVE,
    MIN_TRUST_BOX_SIZE,
    TRAJOPT_DEFAULT_NAMESPACE,
    TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
    make_puzzle_tool_poses,
)
from tesseract_robotics.motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
    TrajOptOSQPSolverProfile,
)
from tesseract_robotics.motion_planners_trajopt_ifopt import (
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultMoveProfile,
    TrajOptIfoptOSQPSolverProfile,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)] + ["joint_aux1", "joint_aux2"]
JOINT_POS = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0, 0.0, 0.0])

# Cartesian constraint coeff [x, y, z, rx, ry, rz]: rotation about the tool z-axis is free,
# so the positioner can take it up.
CARTESIAN_COEFF = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 0.0])


def create_profiles(ifopt: bool = False) -> ProfileDictionary:
    """The C++ example's profiles, TrajOpt or TrajOptIfopt."""
    if ifopt:
        move_profile = TrajOptIfoptDefaultMoveProfile()
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        solver_profile = TrajOptIfoptOSQPSolverProfile()
        namespace = TRAJOPT_IFOPT_DEFAULT_NAMESPACE
    else:
        move_profile = TrajOptDefaultMoveProfile()
        composite_profile = TrajOptDefaultCompositeProfile()
        solver_profile = TrajOptOSQPSolverProfile()
        solver_profile.settings.adaptive_rho = 0
        namespace = TRAJOPT_DEFAULT_NAMESPACE

    move_profile.joint_cost_config.enabled = False
    move_profile.cartesian_cost_config.enabled = False
    move_profile.cartesian_constraint_config.enabled = True
    move_profile.cartesian_constraint_config.coeff = CARTESIAN_COEFF
    composite_profile.collision_constraint_config.enabled = False
    composite_profile.collision_cost_config = TrajOptCollisionConfig(0.025, 1)
    composite_profile.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.DISCRETE
    )
    solver_profile.opt_params.max_iter = MAX_ITER
    solver_profile.opt_params.min_approx_improve = MIN_APPROX_IMPROVE
    solver_profile.opt_params.min_trust_box_size = MIN_TRUST_BOX_SIZE

    profiles = ProfileDictionary()
    profiles.addProfile(namespace, "CARTESIAN", move_profile)
    profiles.addProfile(namespace, "DEFAULT", composite_profile)
    profiles.addProfile(namespace, "DEFAULT", solver_profile)
    return profiles


def run(ifopt: bool = False):
    """Run the auxiliary-axes puzzle piece example (C++ PuzzlePieceAuxillaryAxesExample::run).

    Args:
        ifopt: Plan with TrajOptIfoptPipeline instead of TrajOptPipeline.

    Returns:
        dict with env, success, trajectory, result
    """
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    env = load_support_environment("puzzle_piece_workcell", locator)
    env.setState(JOINT_NAMES, JOINT_POS)

    tool_poses = make_puzzle_tool_poses(locator)
    print(f"Loaded {len(tool_poses)} tool poses")

    mi = ManipulatorInfo()
    mi.manipulator = "manipulator_aux"
    mi.working_frame = "part"
    mi.tcp_frame = "grinder_frame"

    program = CompositeInstruction("DEFAULT", mi)
    for i, pose in enumerate(tool_poses):
        plan_instruction = MoveInstruction(
            CartesianWaypointPoly(CartesianWaypoint(pose)), MoveInstructionType.LINEAR, "CARTESIAN"
        )
        plan_instruction.setDescription(f"waypoint_{i}")
        program.push_back(MoveInstructionPoly(plan_instruction))
    assignCurrentStateAsSeed(program, env)

    pipeline = "TrajOptIfoptPipeline" if ifopt else "TrajOptPipeline"
    print(f"Running {pipeline} (9 DOF)...")
    result = run_pipeline(env, program, create_profiles(ifopt), pipeline)
    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False
    print(f"Planning successful! {len(result)} waypoints")

    return {"env": env, "success": True, "trajectory": result.raw_results, "result": result}


def main() -> int:
    results = run()
    if not results:
        return 1

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        viewer.update_trajectory(results["trajectory"])
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0 if results["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
