"""
Basic Cartesian Planning Example

Demonstrates Cartesian motion planning using TrajOpt trajectory optimization with
the KUKA IIWA 7-DOF robot. Plans a multi-phase trajectory combining freespace and
linear Cartesian moves around an obstacle.

Pipeline Overview:
1. Load the KUKA IIWA and add the C++ example's octomap at (1.0, 0, 0): a 1 m cube of
   points at 0.05 m pitch, voxelised into 1000 boxes of 0.1 m
2. Program: start state, FREESPACE to wp1, LINEAR to wp2, FREESPACE back to start
3. TrajOpt profiles exactly as the C++: LVS-discrete collision cost (0.025 m) and
   constraint (0 m), velocity smoothing, Cartesian and joint waypoints as constraints
4. Run TrajOptPipeline (or TrajOptIfoptPipeline with ifopt=True)

Bindings only, line for line with the C++ (#42).

C++ Source: tesseract_planning/examples/src/basic_cartesian_example.cpp

Related Examples:
- glass_upright_example.py - orientation-constrained TrajOpt planning
- puzzle_piece_example.py - complex Cartesian toolpath from CSV
"""

import sys

import numpy as np

from tesseract_robotics.collision import CollisionEvaluatorType
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.common import (
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.examples.lowlevel.example_support import (
    add_fixed_link,
    load_support_environment,
    run_pipeline,
)
from tesseract_robotics.geometry import Octree, OctreeSubType, PointCloud, createOctree
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
JOINT_POS = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

# addPointCloud() (example.cpp): points every 0.05 m across a 1 m cube, voxelised at
# 2 x 0.05 m, the octomap link at (1, 0, 0).
POINT_PITCH_M = 0.05
OCTOMAP_POSITION_M = (1.0, 0.0, 0.0)

# The C++ Eigen::Quaterniond(0, 0, 1.0, 0) is scalar-first (w=0, x=0, y=1, z=0): 180 deg
# about Y, tool down. Scalar-last, as Quaterniond.from_xyzw takes it: (0, 1, 0, 0).
TOOL_DOWN_XYZW = (0.0, 1.0, 0.0, 0.0)


def _add_point_cloud(env) -> None:
    count = round(1.0 / POINT_PITCH_M)
    cloud = PointCloud()
    for i in range(count):
        for j in range(count):
            for k in range(count):
                cloud.addPoint(
                    -0.5 + i * POINT_PITCH_M, -0.5 + j * POINT_PITCH_M, -0.5 + k * POINT_PITCH_M
                )
    tree = createOctree(cloud, 2 * POINT_PITCH_M, prune=False, binary=True)
    octree = Octree(tree, OctreeSubType.BOX, pruned=False, binary_octree=True)
    add_fixed_link(
        env,
        "octomap_attached",
        "joint_octomap_attached",
        octree,
        Isometry3d.Identity() * Translation3d(*OCTOMAP_POSITION_M),
    )
    print(f"Added octomap of {octree.calcNumSubShapes()} boxes at {OCTOMAP_POSITION_M}")


def _cartesian(y: float) -> CartesianWaypointPoly:
    return CartesianWaypointPoly(
        CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.5, y, 0.62)
            * Quaterniond.from_xyzw(*TOOL_DOWN_XYZW)
        )
    )


def _collision_config(margin: float) -> TrajOptCollisionConfig:
    config = TrajOptCollisionConfig(margin, 1)
    config.enabled = True
    config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
    return config


def _profiles(ifopt: bool) -> ProfileDictionary:
    """The C++ example's profiles, TrajOpt or TrajOptIfopt."""
    if ifopt:
        namespace = TRAJOPT_IFOPT_DEFAULT_NAMESPACE
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        move_profile = TrajOptIfoptDefaultMoveProfile()
    else:
        namespace = TRAJOPT_DEFAULT_NAMESPACE
        composite_profile = TrajOptDefaultCompositeProfile()
        move_profile = TrajOptDefaultMoveProfile()

    composite_profile.collision_cost_config = _collision_config(0.025)
    composite_profile.collision_constraint_config = _collision_config(0.0)
    composite_profile.smooth_velocities = True
    composite_profile.smooth_accelerations = False
    composite_profile.smooth_jerks = False
    composite_profile.velocity_coeff = np.ones(1)

    move_profile.cartesian_cost_config.enabled = False
    move_profile.cartesian_constraint_config.enabled = True
    move_profile.cartesian_constraint_config.coeff = np.ones(6)
    move_profile.joint_cost_config.enabled = False
    move_profile.joint_constraint_config.enabled = True
    move_profile.joint_constraint_config.coeff = np.ones(7)

    profiles = ProfileDictionary()
    profiles.addProfile(namespace, "cartesian_program", composite_profile)
    profiles.addProfile(namespace, "RASTER", move_profile)
    profiles.addProfile(namespace, "freespace_profile", move_profile)
    return profiles


def run(ifopt: bool = False):
    """Run the basic Cartesian example (C++ BasicCartesianExample::run).

    Args:
        ifopt: Plan with TrajOptIfoptPipeline instead of TrajOptPipeline.

    Returns:
        dict with result, env, joint_names
    """
    env = load_support_environment("lbr_iiwa_14_r820")
    _add_point_cloud(env)
    env.setState(JOINT_NAMES, JOINT_POS)

    program = CompositeInstruction(
        "cartesian_program", ManipulatorInfo("manipulator", "base_link", "tool0")
    )
    wp0 = StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_POS))
    start_instruction = MoveInstruction(wp0, MoveInstructionType.FREESPACE, "freespace_profile")
    start_instruction.setDescription("Start Instruction")
    plan_f0 = MoveInstruction(
        _cartesian(-0.2), MoveInstructionType.FREESPACE, "RASTER", "freespace_profile"
    )
    plan_f0.setDescription("from_start_plan")
    plan_c0 = MoveInstruction(_cartesian(0.3), MoveInstructionType.LINEAR, "RASTER")
    plan_f1 = MoveInstruction(wp0, MoveInstructionType.FREESPACE, "freespace_profile")
    plan_f1.setDescription("to_end_plan")
    for instruction in (start_instruction, plan_f0, plan_c0, plan_f1):
        program.push_back(MoveInstructionPoly(instruction))

    pipeline = "TrajOptIfoptPipeline" if ifopt else "TrajOptPipeline"
    print(f"Running {pipeline}...")
    result = run_pipeline(env, program, _profiles(ifopt), pipeline)
    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! {len(result)} waypoints")

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
