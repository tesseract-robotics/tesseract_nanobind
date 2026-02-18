"""
Pick and Place Example (High-Level API)

Demonstrates a complete pick-and-place workflow using TrajOpt motion planning.
A KUKA IIWA 7-DOF robot picks a box from a table and places it on a shelf.

This example showcases key capabilities:
    - Multi-phase planning with state continuity between pick and place
    - Scene graph manipulation (attach object to end-effector)
    - Allowed collision matrix modification for attached objects
    - Mixed FREESPACE/LINEAR motion types in a single program

Pipeline Overview:
    1. Load KUKA IIWA workcell (robot + table + shelf)
    2. Add box to table surface
    3. PICK phase: approach (FREESPACE) -> grasp (LINEAR descent)
    4. ATTACH: Reparent box link to end-effector, update collision matrix
    5. PLACE phase: retreat (LINEAR) -> transit (FREESPACE) -> place (LINEAR)

Workflow Phases:
    PICK:
        - Start from home joint configuration
        - FREESPACE move to 15cm above box
        - LINEAR descent to grasp position (Cartesian precision)

    ATTACH:
        - Reparent box link from workcell_base to iiwa_tool0 via move_link()
        - Add allowed collisions: box<->tool0, box<->link_7, box<->link_6
        - Required to prevent self-collision with attached object

    PLACE:
        - LINEAR retreat to 15cm above original position
        - FREESPACE transit to shelf approach pose
        - LINEAR approach to shelf placement location

Key Concepts:
    - create_fixed_joint(): High-level helper to create FIXED joint for attach
    - move_link(): Scene graph reparenting (changes kinematic parent)
    - add_allowed_collision(): ACM modification for attached objects
    - CartesianTarget: 6D pose goal with profile selection
    - StateTarget: Joint configuration goal for state continuity

TrajOpt Profile Configuration (0.33 API):
    - Cartesian constraint: coeff=[10,10,10,10,10,10] for all 6 DOF
    - Collision cost: enabled, collision_margin_buffer=0.025m, coeff=20
    - collision_check_config.longest_valid_segment_length = 0.05m

C++ Source: tesseract_planning/tesseract_examples/src/pick_and_place_example.cpp

C++ Parameters (verified):
    - Robot: KUKA IIWA with pick_and_place_plan.urdf workcell
    - Box: 10cm cube at (-0.2, 0.55) on workcell_base (table z=0.772m)
    - Pick rotation: Ry(180) = gripper pointing down
    - Place location: middle_left_shelf at (-0.149, 0.731, 1.160)
    - Place rotation: Rz(90) = gripper approaching shelf from front
    - OFFSET = 0.005m (5mm clearance)

Quaternion Note:
    - C++ Eigen::Quaterniond(w, x, y, z) = (0, 0, 0.7071, 0.7071) for shelf
    - Equivalent rotation matrix: [[-1,0,0], [0,0,1], [0,1,0]]

Related Examples:
    - basic_cartesian_example.py - Simpler FREESPACE/LINEAR mixing
    - car_seat_example.py - Complex multi-phase planning
    - lowlevel/pick_and_place_c_api_example.py - Same with low-level API
"""

import sys

import numpy as np

from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    box,
    create_fixed_joint,
    create_obstacle,
)
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addTrajOptCompositeProfile,
    TrajOptCollisionConfig,
    TrajOptDefaultCompositeProfile,
)

TRAJOPT_NS = "TrajOptMotionPlannerTask"

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer

OFFSET = 0.005
BOX_SIZE = 0.1
LINK_BOX = "box"
LINK_BASE = "world"
LINK_TCP = "iiwa_tool0"


def create_profiles():
    """Create TrajOpt profiles for pick-and-place matching C++ settings.

    C++ profile settings (from pick_and_place_example.cpp):
        - collision_constraint: margin=0, coeff=10, LVS_DISCRETE, lvs=0.05, buffer=0.005
        - collision_cost: margin=0.005, coeff=50, LVS_DISCRETE, lvs=0.05, buffer=0.01

    Returns:
        ProfileDictionary with TrajOpt composite profile for collision handling.
    """
    profiles = ProfileDictionary()
    composite = TrajOptDefaultCompositeProfile()

    # Collision constraint (hard): margin=0, coeff=10 (matches C++)
    composite.collision_constraint_config = TrajOptCollisionConfig(0.0, 10)
    composite.collision_constraint_config.collision_check_config.type = (
        CollisionEvaluatorType.LVS_DISCRETE
    )
    composite.collision_constraint_config.collision_check_config.longest_valid_segment_length = 0.05
    composite.collision_constraint_config.collision_margin_buffer = 0.005

    # Collision cost (soft): margin=0.005, coeff=50 (matches C++)
    composite.collision_cost_config = TrajOptCollisionConfig(0.005, 50)
    composite.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.LVS_DISCRETE
    )
    composite.collision_cost_config.collision_check_config.longest_valid_segment_length = 0.05
    composite.collision_cost_config.collision_margin_buffer = 0.01

    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_NS, "DEFAULT", composite)
    return profiles


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Execute complete pick-and-place workflow.

    Performs two-phase motion planning:
        1. PICK: Navigate to box and grasp
        2. PLACE: Transport and place on shelf

    Between phases, the box is attached to the end-effector via scene graph
    manipulation and collision matrix updates.

    Args:
        pipeline: Planning pipeline to use. Options:
            - "TrajOptPipeline" (default): TrajOpt trajectory optimization
            - "FreespacePipeline": OMPL (ignores LINEAR motion requirements)
        num_planners: Number of parallel OMPL planners (only for FreespacePipeline).

    Returns:
        dict with keys:
            - pick_result: PlanningResult for pick phase
            - place_result: PlanningResult for place phase
            - robot: Robot instance with attached box
            - joint_names: List of 7 KUKA IIWA joint names
    """
    # Box position on table (X, Y relative to workcell_base)
    box_pos = [-0.2, 0.55]

    # Load KUKA IIWA workcell from tesseract_support
    # Includes: robot, table (z=0.772m), shelf structure
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/pick_and_place_plan.urdf",
        "package://tesseract_support/urdf/pick_and_place_plan.srdf",
    )
    # Set default collision margin (distance for contact reporting)
    robot.set_collision_margin(0.005)  # 5mm

    # KUKA IIWA joint names (7-DOF)
    joint_names = [f"iiwa_joint_a{i}" for i in range(1, 8)]
    # Initial configuration: elbow bent down (-90 deg on joint 4)
    start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
    robot.set_joints(start_pos, joint_names=joint_names)

    # Add box to table surface
    # Z = box_size/2 (center) + OFFSET (5mm clearance)
    # Parent: workcell_base (table frame, z=0 is table surface)
    create_obstacle(
        robot,
        name=LINK_BOX,
        geometry=box(BOX_SIZE, BOX_SIZE, BOX_SIZE),  # 10cm cube
        transform=Pose.from_xyz(box_pos[0], box_pos[1], BOX_SIZE / 2 + OFFSET),
        parent_link="workcell_base",
    )
    print(f"Added box at {box_pos}")

    composer = TaskComposer.from_config()

    # Select profiles based on pipeline type
    if "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        profiles = create_profiles()

    # ==================== PICK PHASE ====================
    # Motion: start -> approach (15cm above) -> grasp (on box)
    print("\n=== PICK ===")

    # Pick pose calculation:
    # - Z = table_height (0.772m) + box_size + offset
    # - Rotation: Ry(180) = gripper pointing down
    # - Rotation matrix: [[-1,0,0], [0,1,0], [0,0,-1]]
    pick_z = BOX_SIZE + 0.772 + OFFSET
    pick_rotation = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    pick_pose = Pose.from_matrix_position(pick_rotation, [box_pos[0], box_pos[1], pick_z])

    # Approach pose: 15cm directly above pick pose (same orientation)
    approach_pose = Pose.from_matrix_position(
        pick_rotation, [box_pos[0], box_pos[1], pick_z + 0.15]
    )

    # Build PICK program:
    # 1. Start at home joint configuration
    # 2. FREESPACE to approach pose (any collision-free path)
    # 3. LINEAR descent to grasp pose (Cartesian precision for gripper alignment)
    pick_program = (
        MotionProgram("manipulator", tcp_frame=LINK_TCP, working_frame=LINK_BASE)
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(CartesianTarget(approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(pick_pose, profile="CARTESIAN"))
    )

    pick_result = composer.plan(robot, pick_program, pipeline=pipeline, profiles=profiles)
    assert pick_result.successful, f"PICK failed: {pick_result.message}"
    print(f"PICK OK: {len(pick_result)} waypoints")

    # ==================== ATTACH BOX TO END EFFECTOR ====================
    # Reparent box link from workcell_base to iiwa_tool0
    print("\n=== ATTACH ===")

    # Update robot to final pick configuration
    pick_final = pick_result.trajectory[-1].positions
    robot.set_joints(pick_final, joint_names=joint_names)

    # Create FIXED joint to attach box to tool flange
    # Origin offset: box center is box_size/2 below tool (in tool Z direction)
    attach_joint = create_fixed_joint(
        "joint_box2", LINK_TCP, LINK_BOX, origin=Pose.from_xyz(0, 0, BOX_SIZE / 2)
    )
    # move_link() reparents the box - removes old joint, adds new one
    robot.move_link(attach_joint)

    # Update Allowed Collision Matrix (ACM)
    # Without this, planner reports self-collision with attached object
    # "Never" = permanently ignore collisions (vs "Adjacent" for kinematic neighbors)
    robot.add_allowed_collision(LINK_BOX, LINK_TCP, "Never")
    robot.add_allowed_collision(LINK_BOX, "iiwa_link_7", "Never")  # Wrist
    robot.add_allowed_collision(LINK_BOX, "iiwa_link_6", "Never")  # Forearm
    print("Box attached with collision exceptions")

    # ==================== PLACE PHASE ====================
    # Motion: grasp -> retreat -> transit -> place
    print("\n=== PLACE ===")

    # Place pose: middle_left_shelf from C++ example
    # Quaternion (w=0, x=0, y=0.7071, z=0.7071) = 90 deg around Z
    # Rotation matrix: [[-1,0,0], [0,0,1], [0,1,0]]
    place_rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    place_pos = [-0.148856, 0.73085, 1.16]  # middle_left_shelf coordinates
    place_pose = Pose.from_matrix_position(place_rotation, place_pos)

    # Approach: 25cm back from shelf in -Y direction (approach from front)
    place_approach_pos = [place_pos[0], place_pos[1] - 0.25, place_pos[2]]
    place_approach_pose = Pose.from_matrix_position(place_rotation, place_approach_pos)

    # Build PLACE program:
    # 1. Start from final pick configuration (box now attached)
    # 2. LINEAR retreat to approach pose (controlled extraction)
    # 3. FREESPACE transit to shelf approach (collision-aware path)
    # 4. LINEAR approach to shelf (precise placement)
    place_program = (
        MotionProgram("manipulator", tcp_frame=LINK_TCP, working_frame=LINK_BASE)
        .set_joint_names(joint_names)
        .move_to(StateTarget(pick_final, names=joint_names))
        .linear_to(CartesianTarget(approach_pose, profile="CARTESIAN"))  # Retreat
        .move_to(CartesianTarget(place_approach_pose, profile="FREESPACE"))  # Transit
        .linear_to(CartesianTarget(place_pose, profile="CARTESIAN"))  # Place
    )

    place_result = composer.plan(robot, place_program, pipeline=pipeline, profiles=profiles)
    assert place_result.successful, f"PLACE failed: {place_result.message}"
    print(f"PLACE OK: {len(place_result)} waypoints")

    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main():
    results = run()

    if TesseractViewer is not None:
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["place_result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")


if __name__ == "__main__":
    main()
