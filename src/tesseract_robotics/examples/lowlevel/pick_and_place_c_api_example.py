"""
Pick and Place Example - Low-Level C API Implementation

Demonstrates a complete pick-and-place workflow using TrajOpt motion planning.
A KUKA IIWA 7-DOF robot picks a box from a table and places it on a shelf.

Based on: tesseract_planning/examples/src/pick_and_place_example.cpp
Robot: KUKA IIWA 7-DOF with workcell (pick_and_place_plan.urdf)

Workflow Phases:
    1. PICK: freespace to 15 cm above the box, linear descent to grasp
    2. ATTACH: move the box link to the tool flange, allow its contacts
    3. PLACE: linear retreat, freespace to the shelf approach, linear to place

C++ values: default collision margin 5 mm; box 0.2 m at (0.15, 0.4) on the table
(z = 0.772 m), OFFSET 5 mm; place on middle_left_shelf (-0.148856, 0.73085, 1.16).

Bindings only, line for line with the C++ (#42).
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
    AllowedCollisionMatrix,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.environment import (
    ChangeCollisionMarginsCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    MoveLinkCommand,
)
from tesseract_robotics.examples.failures import require_success
from tesseract_robotics.examples.lowlevel.example_support import (
    add_fixed_link,
    load_support_environment,
    run_pipeline,
)
from tesseract_robotics.geometry import Box
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
from tesseract_robotics.scene_graph import Joint, JointType
from tesseract_robotics.task_composer_planning import ContactCheckProfile
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"
TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"
DISCRETE_CONTACT_CHECK_TASK_NAME = "DiscreteContactCheckTask"

# pick_and_place_example.cpp constants.
OFFSET = 0.005  # clearance between the box and the table (m)
LINK_BOX_NAME = "box"
LINK_BASE_NAME = "world"
LINK_END_EFFECTOR_NAME = "iiwa_tool0"
TABLE_HEIGHT_M = 0.772
DEFAULT_COLLISION_MARGIN_M = 0.005
LONGEST_VALID_SEGMENT_LENGTH_M = 0.05
APPROACH_HEIGHT_M = 0.15
PLACE_APPROACH_OFFSET_M = (0.0, -0.25, 0.0)

# PickAndPlaceExample constructor defaults (pick_and_place_example.h).
BOX_SIZE_M = 0.2
BOX_POSITION_M = (0.15, 0.4)

JOINT_NAMES = [f"iiwa_joint_a{i}" for i in range(1, 8)]
# Move profile coefficients [-] (pick_and_place_example.cpp:280-282): joint constraint 5 on
# the one-wide velocity weight, Cartesian constraint 10 on all six axes.
JOINT_CONSTRAINT_COEFF = 5.0
CARTESIAN_CONSTRAINT_COEFF = 10.0
CARTESIAN_DOF = 6
# Collision constraint and cost (cpp:285-297): margins [m], coefficients [-], buffers [m].
CONSTRAINT_MARGIN_M = 0.0
CONSTRAINT_COEFF = 10
CONSTRAINT_BUFFER_M = 0.005
COST_MARGIN_M = 0.005
COST_COEFF = 50
COST_BUFFER_M = 0.01
# OSQP iteration limit [-] (pick_and_place_example.cpp, solver profile).
SOLVER_MAX_ITER = 100


JOINT_START = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])

# C++ Eigen::Quaterniond(w, x, y, z) is scalar-first; Quaterniond.from_xyzw takes scalar-last.
# Pick: Quaterniond(0, 0, 1, 0), 180 deg about Y (tool down).
PICK_XYZW = (0.0, 1.0, 0.0, 0.0)
# Shelves: Quaterniond(0, 0, 0.7071068, 0.7071068).
SHELF_XYZW = (0.0, 0.7071068, 0.7071068, 0.0)
MIDDLE_LEFT_SHELF_M = (-0.148856, 0.73085, 1.16)


def _pose(position, xyzw) -> Isometry3d:
    return Isometry3d.Identity() * Translation3d(*position) * Quaterniond.from_xyzw(*xyzw)


def _collision_config(margin: float, coeff: float, buffer: float) -> TrajOptCollisionConfig:
    config = TrajOptCollisionConfig(margin, coeff)
    config.enabled = True
    config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
    config.collision_check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH_M
    config.collision_margin_buffer = buffer
    return config


def create_pick_and_place_profiles(ifopt: bool = False) -> ProfileDictionary:
    """The C++ example's profiles, TrajOpt or TrajOptIfopt, plus the contact post-check.

    Move profile: joint constraint coeff 5, Cartesian constraint coeff 10 on all six axes.
    Composite profile: LVS-discrete collision constraint (margin 0, coeff 10, buffer 5 mm)
    and cost (margin 5 mm, coeff 50, buffer 10 mm), velocity, acceleration and jerk
    smoothing. Solver: OSQP, max_iter 100.
    """
    if ifopt:
        namespace = TRAJOPT_IFOPT_DEFAULT_NAMESPACE
        move_profile = TrajOptIfoptDefaultMoveProfile()
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        solver_profile = TrajOptIfoptOSQPSolverProfile()
    else:
        namespace = TRAJOPT_DEFAULT_NAMESPACE
        move_profile = TrajOptDefaultMoveProfile()
        composite_profile = TrajOptDefaultCompositeProfile()
        solver_profile = TrajOptOSQPSolverProfile()

    move_profile.joint_cost_config.enabled = False
    move_profile.cartesian_cost_config.enabled = False
    move_profile.joint_constraint_config.enabled = True
    move_profile.joint_constraint_config.coeff = np.full(1, JOINT_CONSTRAINT_COEFF)
    move_profile.cartesian_constraint_config.enabled = True
    move_profile.cartesian_constraint_config.coeff = np.full(
        CARTESIAN_DOF, CARTESIAN_CONSTRAINT_COEFF
    )

    composite_profile.collision_constraint_config = _collision_config(
        CONSTRAINT_MARGIN_M, CONSTRAINT_COEFF, CONSTRAINT_BUFFER_M
    )
    composite_profile.collision_cost_config = _collision_config(
        COST_MARGIN_M, COST_COEFF, COST_BUFFER_M
    )
    composite_profile.smooth_velocities = True
    composite_profile.velocity_coeff = np.ones(1)
    composite_profile.smooth_accelerations = True
    composite_profile.acceleration_coeff = np.ones(1)
    composite_profile.smooth_jerks = True
    composite_profile.jerk_coeff = np.ones(1)

    solver_profile.opt_params.max_iter = SOLVER_MAX_ITER

    profiles = ProfileDictionary()
    profiles.addProfile(namespace, "CARTESIAN", move_profile)
    profiles.addProfile(namespace, "DEFAULT", move_profile)
    profiles.addProfile(namespace, "DEFAULT", composite_profile)
    profiles.addProfile(namespace, "DEFAULT", solver_profile)

    post_check_profile = ContactCheckProfile()
    profiles.addProfile(DISCRETE_CONTACT_CHECK_TASK_NAME, "CARTESIAN", post_check_profile)
    profiles.addProfile(DISCRETE_CONTACT_CHECK_TASK_NAME, "DEFAULT", post_check_profile)
    return profiles


def run(ifopt: bool = False):
    """Run the pick and place example (C++ PickAndPlaceExample::run).

    Args:
        ifopt: Plan with TrajOptIfoptPipeline instead of TrajOptPipeline.

    Returns:
        dict with pick_result, place_result, env, joint_names
    """
    env = load_support_environment("pick_and_place_plan")
    if not env.applyCommand(ChangeCollisionMarginsCommand(DEFAULT_COLLISION_MARGIN_M)):
        raise RuntimeError("ChangeCollisionMarginsCommand rejected")
    env.setState(JOINT_NAMES, JOINT_START)

    # addBox(): the box on the table (workcell_base), its centre OFFSET above half its side.
    add_fixed_link(
        env,
        LINK_BOX_NAME,
        "joint_box",
        Box(BOX_SIZE_M, BOX_SIZE_M, BOX_SIZE_M),
        Isometry3d.Identity(),
        parent_link="workcell_base",
        joint_origin=Isometry3d.Identity()
        * Translation3d(BOX_POSITION_M[0], BOX_POSITION_M[1], BOX_SIZE_M / 2.0 + OFFSET),
    )

    manip_info = ManipulatorInfo("manipulator", LINK_BASE_NAME, LINK_END_EFFECTOR_NAME)
    profiles = create_pick_and_place_profiles(ifopt)
    pipeline = "TrajOptIfoptPipeline" if ifopt else "TrajOptPipeline"

    # ==================== PICK ====================
    print("\n=== PICK PHASE ===")
    pick_position = (BOX_POSITION_M[0], BOX_POSITION_M[1], BOX_SIZE_M + TABLE_HEIGHT_M + OFFSET)
    pick_final_pose = _pose(pick_position, PICK_XYZW)
    pick_approach_position = (*pick_position[:2], pick_position[2] + APPROACH_HEIGHT_M)
    pick_approach_pose = _pose(pick_approach_position, PICK_XYZW)

    pick_program = CompositeInstruction("DEFAULT", manip_info)
    start_instruction = MoveInstruction(
        StateWaypointPoly(StateWaypoint(JOINT_NAMES, JOINT_START)),
        MoveInstructionType.FREESPACE,
        "FREESPACE",
    )
    start_instruction.setDescription("Start Instruction")
    pick_plan_a0 = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(pick_approach_pose)),
        MoveInstructionType.FREESPACE,
        "CARTESIAN",
        "FREESPACE",
    )
    pick_plan_a0.setDescription("From start to pick Approach")
    pick_plan_a1 = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(pick_final_pose)),
        MoveInstructionType.LINEAR,
        "CARTESIAN",
    )
    pick_plan_a1.setDescription("Pick Approach")
    for instruction in (start_instruction, pick_plan_a0, pick_plan_a1):
        pick_program.push_back(MoveInstructionPoly(instruction))

    pick_result = run_pipeline(env, pick_program, profiles, pipeline)
    if not pick_result.successful:
        raise RuntimeError(f"PICK planning failed: {pick_result.message}")
    print(f"PICK successful! {len(pick_result)} waypoints")

    # ==================== ATTACH ====================
    # Reparent the box to the tool flange, half its side along the tool z-axis, and
    # allow its contacts with the flange and the last two links.
    print("\n=== ATTACHING BOX ===")
    joint_box2 = Joint("joint_box2")
    joint_box2.parent_link_id = LINK_END_EFFECTOR_NAME
    joint_box2.child_link_id = LINK_BOX_NAME
    joint_box2.type = JointType.FIXED
    joint_box2.parent_to_joint_origin_transform = Isometry3d.Identity() * Translation3d(
        0.0, 0.0, BOX_SIZE_M / 2.0
    )
    add_ac = AllowedCollisionMatrix()
    add_ac.addAllowedCollision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never")
    add_ac.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_7", "Never")
    add_ac.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_6", "Never")
    if not env.applyCommands(
        [
            MoveLinkCommand(joint_box2),
            ModifyAllowedCollisionsCommand(add_ac, ModifyAllowedCollisionsType.ADD),
        ]
    ):
        raise RuntimeError("attaching the box was rejected")

    # ==================== PLACE ====================
    print("\n=== PLACE PHASE ===")
    if pick_result.raw_results is None:
        raise RuntimeError("PICK returned no program")
    pick_final_state = pick_result.raw_results.getLastMoveInstruction()
    place_pose = _pose(MIDDLE_LEFT_SHELF_M, SHELF_XYZW)
    place_approach_pose = _pose(np.add(MIDDLE_LEFT_SHELF_M, PLACE_APPROACH_OFFSET_M), SHELF_XYZW)

    place_program = CompositeInstruction("DEFAULT", manip_info)
    place_plan_a0 = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(pick_approach_pose)),
        MoveInstructionType.LINEAR,
        "CARTESIAN",
    )
    place_plan_a0.setDescription("Place retraction")
    place_plan_a1 = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(place_approach_pose)),
        MoveInstructionType.FREESPACE,
        "FREESPACE",
    )
    place_plan_a1.setDescription("Place Freespace")
    place_plan_a2 = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(place_pose)),
        MoveInstructionType.LINEAR,
        "CARTESIAN",
    )
    place_plan_a2.setDescription("Place approach")
    place_program.push_back(pick_final_state)
    for instruction in (place_plan_a0, place_plan_a1, place_plan_a2):
        place_program.push_back(MoveInstructionPoly(instruction))

    place_result = run_pipeline(env, place_program, profiles, pipeline)
    require_success(place_result, "PLACE")
    print(f"PLACE successful! {len(place_result)} waypoints")

    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "env": env,
        "joint_names": JOINT_NAMES,
    }


def main() -> int:
    """Entry point - run the workflow, then animate the place trajectory in the viewer."""
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        viewer.update_trajectory(results["place_result"].raw_results)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")

    print("\nDone!")
    return 0 if results["pick_result"].successful and results["place_result"].successful else 1


if __name__ == "__main__":
    sys.exit(main())
