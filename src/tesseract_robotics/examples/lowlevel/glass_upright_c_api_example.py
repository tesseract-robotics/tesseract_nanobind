"""
Glass Upright Example - Orientation-Constrained Motion Planning

Demonstrates TrajOpt planning with orientation constraints: the robot moves while
keeping the tool "upright" (like carrying a glass of water without spilling).

C++ Source: tesseract_planning/examples/src/glass_upright_example.cpp

**Concept: "Upright" Constraint**
The UPRIGHT move profile makes the Cartesian waypoint a constraint with
coeff = [0, 0, 0, 1, 1, 1]: position free, orientation held. The composite profile
adds an LVS-discrete collision cost and constraint (margin 0.01 m, buffer 0.01 m)
and velocity smoothing.

**Pipeline Overview**
1. Load KUKA IIWA 7-DOF robot
2. Add the 0.15 m sphere at (0.5, 0, 0.55)
3. LINEAR moves between two joint states (only joint_a1 changes: -0.4 -> 0.4 rad)
4. Run TrajOptPipeline (or TrajOptIfoptPipeline with ifopt=True)

Bindings only, line for line with the C++ (#42).
"""

import sys

import numpy as np

from tesseract_robotics.collision import CollisionEvaluatorType
from tesseract_robotics.command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.common import Isometry3d, ManipulatorInfo, Translation3d
from tesseract_robotics.examples.failures import require_success
from tesseract_robotics.examples.lowlevel.example_support import (
    add_fixed_link,
    load_support_environment,
    run_pipeline,
)
from tesseract_robotics.geometry import Sphere
from tesseract_robotics.motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
)
from tesseract_robotics.motion_planners_trajopt_ifopt import (
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultMoveProfile,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"
TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"

JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)]
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

# addSphere() (glass_upright_example.cpp:104-105): radius [m] and position on base_link [m].
SPHERE_RADIUS_M = 0.15
SPHERE_XYZ_M = (0.5, 0.0, 0.55)

# Collision margin and margin buffer of both the cost and the constraint (m).
COLLISION_MARGIN_M = 0.01
COLLISION_MARGIN_BUFFER_M = 0.01
# Cartesian constraint coefficients [x, y, z, rx, ry, rz]: position free, orientation held.
UPRIGHT_COEFF = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])


def _collision_config() -> TrajOptCollisionConfig:
    config = TrajOptCollisionConfig(COLLISION_MARGIN_M, 1)
    config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
    config.collision_margin_buffer = COLLISION_MARGIN_BUFFER_M
    return config


def _profiles(ifopt: bool) -> ProfileDictionary:
    """The C++ example's UPRIGHT profiles, TrajOpt or TrajOptIfopt."""
    if ifopt:
        namespace = TRAJOPT_IFOPT_DEFAULT_NAMESPACE
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        move_profile = TrajOptIfoptDefaultMoveProfile()
    else:
        namespace = TRAJOPT_DEFAULT_NAMESPACE
        composite_profile = TrajOptDefaultCompositeProfile()
        move_profile = TrajOptDefaultMoveProfile()

    composite_profile.collision_cost_config = _collision_config()
    composite_profile.collision_constraint_config = _collision_config()
    composite_profile.smooth_velocities = True
    composite_profile.smooth_accelerations = False
    composite_profile.smooth_jerks = False
    composite_profile.velocity_coeff = np.ones(1)
    composite_profile.acceleration_coeff = np.ones(1)

    move_profile.joint_cost_config.enabled = False
    move_profile.cartesian_cost_config.enabled = False
    move_profile.cartesian_constraint_config.enabled = True
    move_profile.cartesian_constraint_config.coeff = UPRIGHT_COEFF

    profiles = ProfileDictionary()
    profiles.addProfile(namespace, "UPRIGHT", composite_profile)
    profiles.addProfile(namespace, "UPRIGHT", move_profile)
    return profiles


def run(ifopt: bool = False):
    """Run the glass upright example (C++ GlassUprightExample::run).

    Args:
        ifopt: Plan with TrajOptIfoptPipeline instead of TrajOptPipeline.

    Returns:
        dict with result, env, joint_names
    """
    env = load_support_environment("lbr_iiwa_14_r820")
    # addSphere(): a 0.15 m sphere at (0.5, 0, 0.55), fixed to base_link.
    add_fixed_link(
        env,
        "sphere_attached",
        "joint_sphere_attached",
        Sphere(SPHERE_RADIUS_M),
        Isometry3d.Identity() * Translation3d(*SPHERE_XYZ_M),
    )
    env.setState(JOINT_NAMES, JOINT_START)

    program = CompositeInstruction("UPRIGHT", ManipulatorInfo("manipulator", "base_link", "tool0"))
    start_instruction = MoveInstruction(
        StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_START)),
        MoveInstructionType.LINEAR,
        "UPRIGHT",
    )
    start_instruction.setDescription("Start Instruction")
    plan_f0 = MoveInstruction(
        StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_END)),
        MoveInstructionType.LINEAR,
        "UPRIGHT",
    )
    plan_f0.setDescription("freespace_plan")
    program.push_back(MoveInstructionPoly(start_instruction))
    program.push_back(MoveInstructionPoly(plan_f0))

    pipeline = "TrajOptIfoptPipeline" if ifopt else "TrajOptPipeline"
    print(f"Running {pipeline} with the upright constraint...")
    result = run_pipeline(env, program, _profiles(ifopt), pipeline)
    require_success(result, "glass upright")
    print(f"Planning successful! {len(result)} waypoints:")
    for i, state in enumerate(result.trajectory):
        print(f"  [{i}] {state.positions}")

    return {"result": result, "env": env, "joint_names": JOINT_NAMES}


def main() -> int:
    results = run()
    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        if results["result"].raw_results is not None:
            viewer.update_trajectory(results["result"].raw_results)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0 if results["result"].successful else 1


if __name__ == "__main__":
    sys.exit(main())
