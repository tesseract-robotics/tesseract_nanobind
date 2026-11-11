"""
Freespace OMPL Planning Example

Demonstrates freespace motion planning using OMPL RRTConnect algorithm with the
KUKA IIWA 7-DOF robot. Plans a collision-free path around a sphere obstacle.

Pipeline Overview:
1. Load robot environment (KUKA IIWA)
2. Add sphere obstacle at (0.5, 0, 0.55) with radius 0.15m
3. Define start/end joint configurations (only joint_a1 changes from -0.4 to 0.4 rad)
4. Execute OMPLPipeline (two RRTConnect planners, range 0.01, 60 s budget)

Bindings only, line for line with the C++ (#42): the environment, program, profile
dictionary and task composer run are built from the binding types.

C++ Source: tesseract_planning/examples/src/freespace_ompl_example.cpp

Related Examples:
- freespace_hybrid_example.py - adds TrajOpt smoothing after OMPL
- basic_cartesian_example.py - uses TrajOpt for Cartesian paths
"""

import sys

import numpy as np

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
from tesseract_robotics.examples.lowlevel.example_support import (
    add_fixed_link,
    load_support_environment,
    run_pipeline,
)
from tesseract_robotics.geometry import Sphere
from tesseract_robotics.motion_planners_ompl import (
    OMPLRealVectorMoveProfile,
    RRTConnectConfigurator,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"

# FreespaceOMPLExample constructor defaults (freespace_ompl_example.h).
RANGE = 0.01
PLANNING_TIME_S = 60.0

JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)]
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])


def run():
    """Run the freespace OMPL example (C++ FreespaceOMPLExample::run).

    Returns:
        dict with result, env, joint_names
    """
    env = load_support_environment("lbr_iiwa_14_r820")

    # addSphere(): a 0.15 m sphere at (0.5, 0, 0.55), fixed to base_link.
    add_fixed_link(
        env,
        "sphere_attached",
        "joint_sphere_attached",
        Sphere(0.15),
        Isometry3d.Identity() * Translation3d(0.5, 0, 0.55),
    )
    env.setState(JOINT_NAMES, JOINT_START)

    program = CompositeInstruction(
        "FREESPACE", ManipulatorInfo("manipulator", "base_link", "tool0")
    )
    start_instruction = MoveInstruction(
        StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_START)),
        MoveInstructionType.FREESPACE,
        "FREESPACE",
    )
    start_instruction.setDescription("Start Instruction")
    plan_f0 = MoveInstruction(
        StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_END)),
        MoveInstructionType.FREESPACE,
        "FREESPACE",
    )
    plan_f0.setDescription("freespace_plan")
    program.push_back(MoveInstructionPoly(start_instruction))
    program.push_back(MoveInstructionPoly(plan_f0))

    # Two RRTConnect planners sharing one configurator, as the C++ does.
    ompl_planner_config = RRTConnectConfigurator()
    ompl_planner_config.range = RANGE
    ompl_profile = OMPLRealVectorMoveProfile()
    ompl_profile.solver_config.planning_time = PLANNING_TIME_S
    ompl_profile.solver_config.planners = [ompl_planner_config, ompl_planner_config]

    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "FREESPACE", ompl_profile)

    print("Running OMPLPipeline...")
    result = run_pipeline(env, program, profiles, "OMPLPipeline")
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
