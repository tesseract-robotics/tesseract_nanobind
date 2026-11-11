"""
Puzzle Piece Example - Low-level C API version

Demonstrates Cartesian toolpath planning using TrajOpt with custom profiles.
Ported from: tesseract_planning/examples/src/puzzle_piece_example.cpp

Robot: 7-DOF KUKA IIWA in puzzle_piece_workcell
Task: Follow a toolpath (puzzle_bent.csv) with a grinding tool while allowing
      the tool to spin freely around its axis (useful for cylindrical tools).

Key concepts:
- CSV toolpath parsing with mm->m conversion
- Frame construction from position vector and surface normal
- coeff[5]=0 allows rotation freedom around tool Z-axis

C++ reference values:
- Initial joints: [-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0]
- Cartesian constraint coeff: [10, 10, 10, 10, 10, 0] (ifopt: [1, 1, 1, 1, 1, 0])
- Collision cost: TrajOptCollisionConfig(0.025, 20) (ifopt: (0.025, 2)), DISCRETE
- OSQP solver: max_iter 200, min_approx_improve 1e-3, min_trust_box_size 1e-3
- ManipulatorInfo: manipulator="manipulator", tcp="grinder_frame", working_frame="part"

Bindings only, line for line with the C++ (#42).
"""

import csv
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
from tesseract_robotics.common import GeneralResourceLocator, Isometry3d, ManipulatorInfo
from tesseract_robotics.examples.lowlevel.example_support import (
    load_support_environment,
    locate_file,
    run_pipeline,
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

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"
TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"

JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)]
JOINT_POS = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0])

TOOLPATH_URL = "package://tesseract/support/urdf/puzzle_bent.csv"
CSV_HEADER_LINES = 2
MM_PER_M = 1000.0

# Solver settings of both branches (opt_params).
MAX_ITER = 200
MIN_APPROX_IMPROVE = 1e-3
MIN_TRUST_BOX_SIZE = 1e-3


def make_puzzle_tool_poses(locator: GeneralResourceLocator) -> list[Isometry3d]:
    """Tool poses from puzzle_bent.csv (C++ makePuzzleToolPoses).

    Each row is `point, x, y, z, i, j, k` (mm, unit normal). The normal is the tool z-axis,
    y = normalize(z x normalize(-p)), x = normalize(y x z).
    """
    path = locate_file(locator, TOOLPATH_URL)
    poses = []
    with path.open() as f:
        for row in list(csv.reader(f))[CSV_HEADER_LINES:]:
            xyzijk = np.array([float(cell) for cell in row[1:7]])
            pos = xyzijk[:3] / MM_PER_M
            norm = xyzijk[3:] / np.linalg.norm(xyzijk[3:])
            temp_x = -pos / np.linalg.norm(pos)
            y_axis = np.cross(norm, temp_x)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis /= np.linalg.norm(x_axis)
            matrix = np.eye(4)
            matrix[:3, 0], matrix[:3, 1], matrix[:3, 2], matrix[:3, 3] = x_axis, y_axis, norm, pos
            poses.append(Isometry3d(matrix))
    return poses


def _profiles(ifopt: bool) -> ProfileDictionary:
    """The C++ example's profiles, TrajOpt or TrajOptIfopt."""
    profiles = ProfileDictionary()
    if ifopt:
        move_profile = TrajOptIfoptDefaultMoveProfile()
        # Cartesian constraint coeff [1, 1, 1, 1, 1, 0]: rotation about the tool z-axis is free.
        move_profile.cartesian_constraint_config.coeff = np.array([1.0, 1, 1, 1, 1, 0])
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        composite_profile.collision_cost_config = TrajOptCollisionConfig(0.025, 2)
        solver_profile = TrajOptIfoptOSQPSolverProfile()
        namespace = TRAJOPT_IFOPT_DEFAULT_NAMESPACE
    else:
        move_profile = TrajOptDefaultMoveProfile()
        move_profile.cartesian_constraint_config.coeff = np.array([10.0, 10, 10, 10, 10, 0])
        composite_profile = TrajOptDefaultCompositeProfile()
        composite_profile.collision_cost_config = TrajOptCollisionConfig(0.025, 20)
        solver_profile = TrajOptOSQPSolverProfile()
        solver_profile.opt_params.num_threads = 0
        namespace = TRAJOPT_DEFAULT_NAMESPACE

    move_profile.joint_cost_config.enabled = False
    move_profile.cartesian_cost_config.enabled = False
    move_profile.cartesian_constraint_config.enabled = True
    composite_profile.collision_constraint_config.enabled = False
    composite_profile.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.DISCRETE
    )
    solver_profile.opt_params.max_iter = MAX_ITER
    solver_profile.opt_params.min_approx_improve = MIN_APPROX_IMPROVE
    solver_profile.opt_params.min_trust_box_size = MIN_TRUST_BOX_SIZE

    profiles.addProfile(namespace, "CARTESIAN", move_profile)
    profiles.addProfile(namespace, "DEFAULT", composite_profile)
    profiles.addProfile(namespace, "DEFAULT", solver_profile)
    return profiles


def run(ifopt: bool = False):
    """Run the puzzle piece example (C++ PuzzlePieceExample::run).

    Args:
        ifopt: Plan with TrajOptIfoptPipeline instead of TrajOptPipeline.

    Returns:
        dict with env, success, trajectory, result
    """
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    env = load_support_environment("puzzle_piece_workcell", locator)
    env.setState(JOINT_NAMES, JOINT_POS)

    tool_poses = make_puzzle_tool_poses(locator)
    print(f"Loaded {len(tool_poses)} tool poses from {TOOLPATH_URL}")

    mi = ManipulatorInfo()
    mi.manipulator = "manipulator"
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
    print(f"Running {pipeline}...")
    result = run_pipeline(env, program, _profiles(ifopt), pipeline)
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
