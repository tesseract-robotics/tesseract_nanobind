"""
Pick and Place Example - Low-Level C API Implementation

Demonstrates a complete pick-and-place workflow using TrajOpt motion planning.
A KUKA IIWA 7-DOF robot picks a box from a table and places it on a shelf.

Based on: tesseract_examples/src/pick_and_place_example.cpp
Robot: KUKA IIWA 7-DOF with workcell (pick_and_place_plan.urdf)

Workflow Phases:
    1. PICK: Approach box from above (15cm), linear descent to grasp
    2. ATTACH: Move box link to end-effector, set allowed collisions
    3. PLACE: Retreat up, freespace to shelf approach, linear to place

Key C++ Constants:
    - OFFSET = 0.005m (clearance above box)
    - Box on table at z = 0.772m (table height)
    - Pick orientation: 180 deg around Y (gripper pointing down)
    - Place orientation: 90 deg around Z (shelf alignment)
    - Shelf position: middle_left_shelf at (-0.149, 0.731, 1.160)

TrajOpt Profile Configuration:
    - Cartesian constraint: coeff=[10,10,10,10,10,10] for all 6 DOF
    - Collision constraint: margin=0, buffer=0.005m, coeff=10
    - Collision cost: margin=0.005m, buffer=0.01m, coeff=50
    - longest_valid_segment_length = 0.05m

This example uses the low-level C API bindings directly to demonstrate:
    - Custom TrajOpt profile creation and registration
    - Scene graph manipulation (move_link for attach)
    - Allowed collision matrix modification
    - Multi-phase planning with state continuity
"""

import gc
gc.disable()  # Disable GC to avoid nanobind reference leak warnings on exit

import sys
import numpy as np

# High-level planning API
from tesseract_robotics.planning import (
    Robot,           # Environment wrapper with URDF/SRDF loading
    MotionProgram,   # Fluent API for building motion sequences
    CartesianTarget, # Pose-based waypoint
    StateTarget,     # Joint-space waypoint
    Pose,            # SE(3) pose with multiple constructors
    box,             # Box geometry factory
    create_obstacle, # Add collision object to environment
    TaskComposer,    # Motion planning orchestrator
)

# Low-level scene graph for link manipulation
from tesseract_robotics.tesseract_scene_graph import Joint, JointType

# Isometry3d for 4x4 homogeneous transforms
from tesseract_robotics.tesseract_common import Isometry3d

# Profile dictionary for custom planner settings
from tesseract_robotics.tesseract_command_language import ProfileDictionary

# TrajOpt-specific profile types (C++ bindings)
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,       # Per-waypoint settings (constraint/cost)
    TrajOptDefaultCompositeProfile,  # Trajectory-wide settings (collision)
    ProfileDictionary_addTrajOptPlanProfile,      # Register plan profile
    ProfileDictionary_addTrajOptCompositeProfile, # Register composite profile
)

# TrajOpt planner namespace for profile registration
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

# Optional viewer (disabled during pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

# C++ constants from pick_and_place_example.cpp
OFFSET = 0.005  # 5mm clearance above box surface
LINK_BOX_NAME = "box"  # Name of the box link in scene graph
LINK_BASE_NAME = "world"  # World frame for working_frame parameter
LINK_END_EFFECTOR_NAME = "iiwa_tool0"  # KUKA IIWA tool flange


def create_pick_and_place_profiles():
    """Create custom TrajOpt profiles for pick-and-place motion planning.

    TrajOpt uses two profile types:
        1. Plan Profile: Per-waypoint settings (cost/constraint weights)
        2. Composite Profile: Trajectory-wide settings (collision, smoothness)

    C++ Reference (pick_and_place_example.cpp):
        Plan profile:
            - cartesian_constraint: enabled, coeff=[10,10,10,10,10,10]
            - joint_cost: disabled (we want Cartesian precision)

        Composite profile:
            - collision_constraint: margin=0.0, buffer=0.005m, coeff=10
            - collision_cost: margin=0.005m, buffer=0.01m, coeff=50
            - longest_valid_segment_length: 0.05m

    Note:
        Python implementation uses cost-only collision (like puzzle_piece_example)
        instead of hard constraints, as constraint-based collision checking can
        cause solver failures in tight spaces near the attached box.

    Returns:
        ProfileDictionary with registered TrajOpt profiles
    """
    profiles = ProfileDictionary()

    # ==== PLAN PROFILE (per-waypoint) ====
    # Applied to each Cartesian waypoint in LINEAR moves
    plan_profile = TrajOptDefaultPlanProfile()

    # Disable joint-space cost (we want pure Cartesian precision)
    plan_profile.joint_cost_config.enabled = False

    # Disable Cartesian cost (using constraint instead)
    plan_profile.cartesian_cost_config.enabled = False

    # Enable Cartesian constraint with weight 10 on all 6 DOF (x,y,z,rx,ry,rz)
    # This enforces exact pose matching at each waypoint
    plan_profile.cartesian_constraint_config.enabled = True
    plan_profile.cartesian_constraint_config.coeff = np.full(6, 10.0)

    # ==== COMPOSITE PROFILE (trajectory-wide) ====
    # Applied to entire motion sequence
    composite_profile = TrajOptDefaultCompositeProfile()

    # Max distance between collision checks along trajectory
    composite_profile.longest_valid_segment_length = 0.05  # 5cm

    # Collision constraint disabled - hard constraints can cause solver failures
    # when box is attached close to end-effector
    composite_profile.collision_constraint_config.enabled = False

    # Collision cost enabled with soft margin for gradual repulsion
    # This allows solver to push away from obstacles without hard failure
    composite_profile.collision_cost_config.enabled = True
    composite_profile.collision_cost_config.safety_margin = 0.025  # 25mm
    composite_profile.collision_cost_config.coeff = 20.0

    # ==== REGISTER PROFILES ====
    # "CARTESIAN" profile used by linear_to() moves
    ProfileDictionary_addTrajOptPlanProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", plan_profile
    )
    # "DEFAULT" composite profile applies to all trajectories
    ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", composite_profile
    )

    return profiles


def run():
    """Execute pick-and-place workflow and return results.

    Workflow:
        1. Load KUKA IIWA workcell (robot + table + shelf)
        2. Add box to table surface
        3. Plan PICK motion (approach + linear descent)
        4. Attach box to end-effector (reparent in scene graph)
        5. Plan PLACE motion (retreat + freespace + linear approach)

    Returns:
        dict with keys:
            - pick_result: PlanningResult for pick phase
            - place_result: PlanningResult for place phase
            - robot: Robot instance with final state
            - joint_names: list of KUKA IIWA joint names
    """
    # Box parameters (matches C++ example)
    box_position = [-0.2, 0.55]  # X, Y on table surface
    box_size = 0.1  # 10cm cube

    # ==== LOAD ROBOT ====
    # pick_and_place_plan.urdf includes:
    #   - KUKA IIWA 7-DOF arm
    #   - workcell_base with table (z=0.772m)
    #   - Shelf structure with placement locations
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/pick_and_place_plan.urdf",
        "package://tesseract_support/urdf/pick_and_place_plan.srdf"
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Set default collision margin (distance below which contacts are reported)
    robot.set_collision_margin(0.005)  # 5mm

    # KUKA IIWA joint names (7-DOF arm)
    joint_names = [
        "iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3", "iiwa_joint_a4",
        "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"
    ]

    # Initial joint configuration from C++: elbow bent down (-90 deg on joint 4)
    # This provides good reachability for the pick location
    joint_start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])

    # Set robot to initial configuration
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # ==== ADD BOX TO ENVIRONMENT ====
    # Box sits on table surface (workcell_base)
    # Z position: box_size/2 (center) + OFFSET (clearance)
    # Note: Table surface is at z=0 relative to workcell_base
    create_obstacle(
        robot,
        name=LINK_BOX_NAME,
        geometry=box(box_size, box_size, box_size),
        transform=Pose.from_xyz(box_position[0], box_position[1], box_size / 2.0 + OFFSET),
        parent_link="workcell_base",  # Table frame
    )
    print(f"Added box at ({box_position[0]}, {box_position[1]}) with size {box_size}m")

    # ==== SETUP PLANNING ====
    # TaskComposer loads planner pipelines from config YAML
    composer = TaskComposer.from_config()
    # Custom TrajOpt profiles for Cartesian precision
    profiles = create_pick_and_place_profiles()

    # ==================== PICK PHASE ====================
    # Motion sequence: start -> approach (15cm above) -> grasp (on box)
    print("\n=== PICK PHASE ===")

    # ==== PICK POSE ====
    # End-effector on top of box, gripper pointing down
    # Rotation: 180 deg around Y axis -> gripper Z-axis points down
    #   R = Ry(180) = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
    pick_rotation = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])

    # Pick position in world frame:
    #   X, Y: box center
    #   Z: table_height (0.772) + box_size + OFFSET
    pick_position = [box_position[0], box_position[1], box_size + 0.772 + OFFSET]
    pick_final_pose = Pose.from_matrix_position(pick_rotation, pick_position)

    # ==== APPROACH POSE ====
    # 15cm directly above pick pose (same orientation)
    approach_position = [pick_position[0], pick_position[1], pick_position[2] + 0.15]
    pick_approach_pose = Pose.from_matrix_position(pick_rotation, approach_position)

    # ==== BUILD PICK PROGRAM ====
    # MotionProgram is a fluent builder for motion sequences
    pick_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME, working_frame=LINK_BASE_NAME)
        .set_joint_names(joint_names)
        # Start waypoint: joint-space start position
        .move_to(StateTarget(joint_start_pos, names=joint_names, profile="FREESPACE"))
        # Approach: freespace motion to above box
        .move_to(CartesianTarget(pick_approach_pose, profile="FREESPACE"))
        # Grasp: linear descent to box surface (CARTESIAN profile for precision)
        .linear_to(CartesianTarget(pick_final_pose, profile="CARTESIAN"))
    )

    print(f"Pick program: {len(pick_program)} instructions")
    print("Running TrajOpt planner for PICK...")

    # ==== PLAN PICK MOTION ====
    # TaskComposer.plan() auto-seeds Cartesian waypoints using IK
    # TrajOptPipeline: seed -> trajopt optimization -> time parameterization
    pick_result = composer.plan(robot, pick_program, pipeline="TrajOptPipeline", profiles=profiles)

    if not pick_result.successful:
        print(f"PICK planning failed: {pick_result.message}")
        return False

    print("PICK planning successful!")
    print(f"Pick trajectory has {len(pick_result)} waypoints")

    # ==================== ATTACH BOX TO END EFFECTOR ====================
    # Reparent box link from workcell_base to iiwa_tool0 (grasp simulation)
    print("\n=== ATTACHING BOX ===")

    # ==== CREATE NEW JOINT ====
    # move_link() reparents an existing link by defining a new joint
    joint_box2 = Joint("joint_box2")
    joint_box2.parent_link_name = LINK_END_EFFECTOR_NAME  # New parent: tool flange
    joint_box2.child_link_name = LINK_BOX_NAME  # Box link to reparent
    joint_box2.type = JointType.FIXED  # Rigid attachment

    # ==== TRANSFORM: TOOL -> BOX CENTER ====
    # Box center is box_size/2 below tool flange (in tool Z direction)
    # This places box flush against gripper
    box_attach_mat = np.eye(4)
    box_attach_mat[:3, 3] = [0, 0, box_size / 2.0]  # Offset along tool Z
    joint_box2.parent_to_joint_origin_transform = Isometry3d(box_attach_mat)

    # Execute reparenting - removes old joint, adds new one
    robot.move_link(joint_box2)

    # ==== ALLOWED COLLISION MATRIX (ACM) ====
    # Disable collision checking between box and nearby links
    # Without this, planner would report self-collision with attached object
    # "Never" reason indicates permanent ignore (vs "Adjacent" for kinematic neighbors)
    robot.add_allowed_collision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never")
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_7", "Never")  # Wrist link
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_6", "Never")  # Forearm link

    print("Box attached to end effector with collision exceptions")

    # ==================== PLACE PHASE ====================
    # Motion sequence: grasp -> retreat (15cm up) -> approach shelf -> place
    print("\n=== PLACE PHASE ===")

    # ==== UPDATE ROBOT STATE ====
    # Continue from final pick configuration (box now attached)
    pick_final = pick_result.trajectory[-1].positions
    robot.set_joints(pick_final, joint_names=joint_names)

    # ==== PLACE POSE ====
    # Target: middle_left_shelf location from C++ example
    # Rotation: 90 deg around Z (gripper approaches shelf from front)
    # C++ Eigen::Quaterniond(w=0, x=0, y=0.7071068, z=0.7071068)
    # Equivalent rotation matrix: [[-1,0,0], [0,0,1], [0,1,0]]
    place_rotation = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0]
    ])
    # Shelf position from C++ middle_left_shelf: (-0.149, 0.731, 1.160)
    place_position = [-0.148856, 0.73085, 1.16]
    place_pose = Pose.from_matrix_position(place_rotation, place_position)

    # ==== PLACE APPROACH POSE ====
    # 25cm back from shelf in -Y direction (approach from front)
    place_approach_position = np.array(place_position) + np.array([0.0, -0.25, 0.0])
    place_approach_pose = Pose.from_matrix_position(place_rotation, place_approach_position)

    # ==== RETREAT POSE ====
    # Reuse pick approach pose (15cm above grasp location)
    retreat_pose = pick_approach_pose

    # ==== BUILD PLACE PROGRAM ====
    # C++ uses: LINEAR retreat, FREESPACE to approach, LINEAR to place
    place_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME, working_frame=LINK_BASE_NAME)
        .set_joint_names(joint_names)
        # Start waypoint: final pick configuration
        .move_to(StateTarget(pick_final, names=joint_names))
        # Retreat: linear motion up to clear the table
        .linear_to(CartesianTarget(retreat_pose, profile="CARTESIAN"))
        # Transit: freespace motion to shelf approach (collision-aware)
        .move_to(CartesianTarget(place_approach_pose, profile="FREESPACE"))
        # Place: linear approach to shelf location
        .linear_to(CartesianTarget(place_pose, profile="CARTESIAN"))
    )

    print(f"Place program: {len(place_program)} instructions")
    print("Running TrajOpt planner for PLACE...")

    # ==== PLAN PLACE MOTION ====
    place_result = composer.plan(robot, place_program, pipeline="TrajOptPipeline", profiles=profiles)

    assert place_result.successful, f"PLACE planning failed: {place_result.message}"
    print("PLACE planning successful!")
    print(f"Place trajectory has {len(place_result)} waypoints")

    # Return results for testing and visualization
    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main():
    """Entry point - run workflow and optionally visualize.

    When run standalone (not under pytest), launches web viewer at localhost:8000
    to visualize the planned trajectory.
    """
    results = run()

    # Launch viewer if available (skipped during pytest)
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        # Load environment with attached box
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        # Display place trajectory (includes retreat, transit, and place motions)
        viewer.update_trajectory(results["place_result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
