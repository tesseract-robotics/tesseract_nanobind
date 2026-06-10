"""
Descartes Raster Example (Low-Level Planner API)

Port of the upstream C++ example
tesseract_planning/motion_planners/examples/raster_example.cpp (gh-87):
plan a 3-pass raster program by calling the Descartes ladder-graph planner
DIRECTLY (no TaskComposer), then refine the Descartes seed with TrajOpt —
the canonical chained workflow for Cartesian process paths.

Pipeline:
1. generateInterpolatedProgram: densify the sparse program (Descartes needs
   one waypoint per ladder rung)
2. DescartesMotionPlannerD: sample IK at every Cartesian waypoint (OPW
   closed-form solver), build the ladder graph, search the cheapest path
3. TrajOptMotionPlanner: smooth the Descartes joint path

Program structure (mirrors the C++ exactly):
- from_start: zeros joint state -> approach pose (FREESPACE)
- 3x raster_segment: 6 LINEAR moves sweeping y=-0.2..0.3 at x=0.8, z=0.8
- transitions between rasters: UNORDERED composite (planner may pick
  direction) back to the approach pose
- to_end: return to approach pose (FREESPACE)

Related Examples:
- chain_example.py - same Descartes+TrajOpt chaining via the high-level
  TaskComposer API (CartesianPipeline)
- tesseract_planning_lowlevel_c_api_example.py - OMPL+TrajOpt via planner API
"""

import sys

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    DEFAULT_PROFILE_KEY,
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    CompositeInstructionOrder,
    InstructionPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
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
from tesseract_robotics.tesseract_motion_planners_descartes import (
    DescartesDefaultMoveProfileD,
    DescartesMotionPlannerD,
    cast_DescartesMoveProfileD,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    generateInterpolatedProgram,
)
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addTrajOptSolverProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
    TrajOptMotionPlanner,
    TrajOptOSQPSolverProfile,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer

DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

# C++ Eigen::Quaterniond(0, 0, -1.0, 0) is scalar-FIRST (w, x, y, z); the
# project canon is scalar-last [qx, qy, qz, qw]. 180 deg about Y = tool
# pointing straight down.
TOOL_DOWN_XYZW = (0.0, -1.0, 0.0, 0.0)

# Raster sweep from the C++ example: x=0.8, z=0.8, y from -0.3 to +0.3 in
# 0.1 steps. First pose is the approach point, the remaining six are the
# LINEAR raster targets.
RASTER_X = 0.8
RASTER_Z = 0.8
RASTER_YS = (-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3)


def _cartesian_waypoint(y: float) -> CartesianWaypoint:
    return CartesianWaypoint(
        Isometry3d.Identity()
        * Translation3d(RASTER_X, y, RASTER_Z)
        * Quaterniond.from_xyzw(*TOOL_DOWN_XYZW)
    )


def _freespace_move(waypoint: CartesianWaypoint, description: str) -> MoveInstruction:
    instruction = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(waypoint),
        MoveInstructionType_FREESPACE,
        DEFAULT_PROFILE_KEY,
    )
    instruction.setDescription(description)
    return instruction


def _raster_segment() -> CompositeInstruction:
    """One LINEAR pass over the six raster targets (C++ plan_c0..plan_c5)."""
    segment = CompositeInstruction(DEFAULT_PROFILE_KEY)
    segment.setDescription("raster_segment")
    for y in RASTER_YS[1:]:
        segment.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(
                MoveInstruction(
                    CartesianWaypointPoly_wrap_CartesianWaypoint(_cartesian_waypoint(y)),
                    MoveInstructionType_LINEAR,
                    DEFAULT_PROFILE_KEY,
                )
            )
        )
    return segment


def _transitions() -> CompositeInstruction:
    """UNORDERED transition composite back to the approach pose.

    UNORDERED tells consumers the two child composites (transition_from_start /
    transition_from_end) are alternatives without a fixed sequence — the
    structure raster pipelines expect between passes.
    """
    transition_from_start = CompositeInstruction(DEFAULT_PROFILE_KEY)
    transition_from_start.setDescription("transition_from_start")
    transition_from_start.appendMoveInstruction(
        MoveInstructionPoly_wrap_MoveInstruction(
            _freespace_move(_cartesian_waypoint(RASTER_YS[0]), "transition_from_end_plan")
        )
    )

    transition_from_end = CompositeInstruction(DEFAULT_PROFILE_KEY)
    transition_from_end.setDescription("transition_from_end")
    transition_from_end.appendMoveInstruction(
        MoveInstructionPoly_wrap_MoveInstruction(
            _freespace_move(_cartesian_waypoint(RASTER_YS[0]), "transition_from_end_plan")
        )
    )

    transitions = CompositeInstruction(
        DEFAULT_PROFILE_KEY, order=CompositeInstructionOrder.UNORDERED
    )
    transitions.setDescription("transitions")
    transitions.push_back(transition_from_start)
    transitions.push_back(transition_from_end)
    return transitions


def _iter_move_instructions(composite):
    """Depth-first MoveInstructionPoly iteration over nested composites."""
    for instruction in composite:
        if instruction.isCompositeInstruction():
            yield from _iter_move_instructions(InstructionPoly_as_CompositeInstruction(instruction))
        elif instruction.isMoveInstruction():
            yield InstructionPoly_as_MoveInstructionPoly(instruction)


def main():
    # ABB IRB2400 with the OPW closed-form IK solver — Descartes samples IK at
    # every waypoint, so an analytic solver is what makes the ladder graph fast.
    locator = GeneralResourceLocator()
    env = Environment()
    urdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )
    assert env.init(urdf_path, srdf_path, locator)

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    manip_info.manipulator = "manipulator"
    manip_info.manipulator_ik_solver = "OPWInvKin"

    kin_group = env.getKinematicGroup(manip_info.manipulator, manip_info.manipulator_ik_solver)
    joint_names = list(kin_group.getJointNames())

    # Build the raster program: from_start, 3 raster passes with UNORDERED
    # transitions between them, to_end. Mirrors the C++ program exactly.
    program = CompositeInstruction(DEFAULT_PROFILE_KEY, manip_info)

    start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(StateWaypoint(joint_names, np.zeros(6))),
        MoveInstructionType_FREESPACE,
        DEFAULT_PROFILE_KEY,
    )
    start_instruction.setDescription("Start")

    from_start = CompositeInstruction(DEFAULT_PROFILE_KEY)
    from_start.setDescription("from_start")
    from_start.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    from_start.appendMoveInstruction(
        MoveInstructionPoly_wrap_MoveInstruction(
            _freespace_move(_cartesian_waypoint(RASTER_YS[0]), "from_start_plan")
        )
    )
    program.push_back(from_start)

    program.push_back(_raster_segment())
    program.push_back(_transitions())
    program.push_back(_raster_segment())
    program.push_back(_transitions())
    program.push_back(_raster_segment())

    to_end = CompositeInstruction(DEFAULT_PROFILE_KEY)
    to_end.setDescription("to_end")
    to_end.appendMoveInstruction(
        MoveInstructionPoly_wrap_MoveInstruction(
            _freespace_move(_cartesian_waypoint(RASTER_YS[0]), "to_end_plan")
        )
    )
    program.push_back(to_end)

    # Descartes plans one ladder rung per waypoint — interpolate the sparse
    # program first (defaults match the C++: 5 deg state/rotation, 0.15 m
    # translation, min 1 step).
    interpolated_program = generateInterpolatedProgram(program, env)

    # One shared profile dictionary for both planners, like the C++ example.
    descartes_move_profile = DescartesDefaultMoveProfileD()
    trajopt_move_profile = TrajOptDefaultMoveProfile()
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()
    trajopt_solver_profile = TrajOptOSQPSolverProfile()

    profiles = ProfileDictionary()
    profiles.addProfile(
        DESCARTES_DEFAULT_NAMESPACE,
        DEFAULT_PROFILE_KEY,
        cast_DescartesMoveProfileD(descartes_move_profile),
    )
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_move_profile)
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_composite_profile)
    ProfileDictionary_addTrajOptSolverProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_solver_profile
    )

    request = PlannerRequest()
    request.instructions = interpolated_program
    request.env = env
    request.profiles = profiles

    # Solve the whole program with Descartes (ladder graph over IK samples).
    descartes_planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)
    descartes_response = descartes_planner.solve(request)
    assert descartes_response.successful, f"Descartes failed: {descartes_response.message}"
    descartes_moves = list(_iter_move_instructions(descartes_response.results))
    print(f"Descartes solved {len(descartes_moves)} waypoints")

    # Chain: the Descartes joint path becomes the TrajOpt seed.
    request.instructions = descartes_response.results

    trajopt_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
    trajopt_response = trajopt_planner.solve(request)
    assert trajopt_response.successful, f"TrajOpt failed: {trajopt_response.message}"

    trajopt_moves = list(_iter_move_instructions(trajopt_response.results))
    print(f"TrajOpt refined {len(trajopt_moves)} waypoints:")
    for move in trajopt_moves:
        waypoint = move.getWaypoint()
        if waypoint.isStateWaypoint():
            state = WaypointPoly_as_StateWaypointPoly(waypoint)
            print(f"  {np.array2string(state.getPosition().flatten(), precision=3)}")

    if TesseractViewer is not None:
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.start_serve_background()
        viewer.update_trajectory(trajopt_response.results.getInstructions())
        input("Press Enter to exit...")

    return trajopt_response.successful


if __name__ == "__main__":
    main()
