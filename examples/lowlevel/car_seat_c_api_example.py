"""
Car Seat Example - Complex Multi-Step Pick & Place with Dynamic Mesh Objects

This example demonstrates advanced Tesseract features:
- Dynamic mesh loading from DAE (visual) and STL (collision) files
- Convex hull decomposition: 10 STL collision meshes per seat for efficient collision
- Runtime scene graph modification: AddLinkCommand, MoveLinkCommand
- Allowed collision matrix updates during planning
- Multi-phase motion planning with TrajOpt

Robot: 8-DOF system (1 linear carriage_rail + 7 arm joints: s,l,e,u,r,b,t)

Workflow (from C++ car_seat_example.cpp):
1. Add 3 seats to environment at positions (0.5+i, 2.15, 0.45), rotated 180deg around Z
2. Plan Home -> Pick1 (freespace motion to grasp position)
3. Attach seat_1 to end_effector via MoveLinkCommand (dynamic scene modification)
4. Plan Pick1 -> Place1 (transport with attached object, collision-aware)

TrajOpt Profile (from C++):
- Collision constraint: margin=0, buffer=0.005m, coeff=10
- Collision cost: margin=0.005m, buffer=0.01m, coeff=50
- Solver: max_iter=200, min_approx_improve=1e-3

Mesh decomposition rationale:
- Visual: single seat.dae for rendering
- Collision: 10 convex hulls (seat_1.stl through seat_10.stl) for efficient GJK/EPA

Based on: tesseract_examples/src/car_seat_example.cpp
"""

import sys

import numpy as np

from tesseract_robotics.planning import (
    MotionProgram,
    Robot,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType, makeConvexMesh
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    MoveLinkCommand,
)
from tesseract_robotics.tesseract_geometry import createMeshFromPath
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addTrajOptCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptSolverProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultPlanProfile,
    TrajOptOSQPSolverProfile,
)
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Visual,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


TRAJOPT_NS = "TrajOptMotionPlannerTask"


def create_car_seat_profiles():
    """Create TrajOpt profiles for car seat motion planning.

    Matches C++ car_seat_example.cpp profile settings exactly:
    - collision_constraint: margin=0, coeff=10, LVS_CONTINUOUS, LVS=0.1, buffer=0.005
    - collision_cost: margin=0.005, coeff=50, LVS_CONTINUOUS, LVS=0.1, buffer=0.01
    - solver: OSQP, max_iter=200, min_approx_improve=1e-3, min_trust_box_size=1e-3

    Returns:
        ProfileDictionary with TrajOpt profiles for FREESPACE motions.
    """
    profiles = ProfileDictionary()

    # === Composite Profile (collision config) ===
    composite = TrajOptDefaultCompositeProfile()

    # Collision constraint: margin=0, coeff=10
    composite.collision_constraint_config = TrajOptCollisionConfig(0.0, 10)
    composite.collision_constraint_config.collision_check_config.type = (
        CollisionEvaluatorType.LVS_CONTINUOUS
    )
    composite.collision_constraint_config.collision_check_config.longest_valid_segment_length = 0.1
    composite.collision_constraint_config.collision_margin_buffer = 0.005

    # Collision cost: margin=0.005, coeff=50
    composite.collision_cost_config = TrajOptCollisionConfig(0.005, 50)
    composite.collision_cost_config.collision_check_config.type = (
        CollisionEvaluatorType.LVS_CONTINUOUS
    )
    composite.collision_cost_config.collision_check_config.longest_valid_segment_length = 0.1
    composite.collision_cost_config.collision_margin_buffer = 0.01

    # === Plan/Move Profile ===
    plan = TrajOptDefaultPlanProfile()
    plan.cartesian_cost_config.enabled = False
    plan.cartesian_constraint_config.enabled = True
    plan.joint_cost_config.enabled = False
    plan.joint_constraint_config.enabled = True

    # === Solver Profile (OSQP) ===
    solver = TrajOptOSQPSolverProfile()
    solver.opt_params.max_iter = 200
    solver.opt_params.min_approx_improve = 1e-3
    solver.opt_params.min_trust_box_size = 1e-3

    for name in ["DEFAULT", "FREESPACE"]:
        ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_NS, name, composite)
        ProfileDictionary_addTrajOptPlanProfile(profiles, TRAJOPT_NS, name, plan)
        ProfileDictionary_addTrajOptSolverProfile(profiles, TRAJOPT_NS, name, solver)

    return profiles


def get_predefined_positions():
    """Get predefined joint positions for the 8-DOF car seat robot.

    All values in radians (from C++ car_seat_example.cpp).
    Joint order: carriage_rail (linear), joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t

    Returns:
        dict: Named configurations mapping joint names to values
    """
    positions = {}

    # Default: carriage centered, arm in typical working pose
    positions["Default"] = {
        "carriage_rail": 1.0,
        "joint_b": 0.0,
        "joint_e": 0.0,
        "joint_l": 0.0,
        "joint_r": 0.0,
        "joint_s": -1.5707,  # -pi/2: shoulder rotated
        "joint_t": 0.0,
        "joint_u": -1.5707,  # -pi/2: elbow bent
    }

    # Pick1: above seat_1 at X=0.5, carriage at 2.22m
    # joint_s=-3.14 (pi rotation), arm reaching down
    positions["Pick1"] = {
        "carriage_rail": 2.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,  # ~-pi: shoulder fully rotated for downward reach
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    # Pick2: above seat_2 at X=1.5, carriage shifted 1m left
    positions["Pick2"] = {
        "carriage_rail": 1.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    # Pick3: above seat_3 at X=2.5, carriage at 0.22m
    positions["Pick3"] = {
        "carriage_rail": 0.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    # Place1: carriage at 4.15m (far end of rail), complex arm config
    # This places the seat at the car body mounting position
    positions["Place1"] = {
        "carriage_rail": 4.15466,  # Far end of linear rail
        "joint_b": 0.537218,
        "joint_e": 0.0189056,
        "joint_l": 0.801223,
        "joint_r": 0.0580309,
        "joint_s": -0.0481182,  # Near zero: arm pointing forward
        "joint_t": -0.325783,
        "joint_u": -1.2813,
    }

    # Home: all joints at zero (robot at origin, arm extended)
    positions["Home"] = {
        "carriage_rail": 0.0,
        "joint_b": 0.0,
        "joint_e": 0.0,
        "joint_l": 0.0,
        "joint_r": 0.0,
        "joint_s": 0.0,
        "joint_t": 0.0,
        "joint_u": 0.0,
    }

    return positions


def get_position_vector(joint_names, pos_dict):
    """Get joint position vector from a position dictionary."""
    return np.array([pos_dict[name] for name in joint_names])


def add_seats(robot):
    """Create and add 3 seat links to the environment with visual and collision meshes.

    From C++ addSeats():
    - Creates seats at positions (0.5+i, 2.15, 0.45) for i=0,1,2
    - Each seat rotated 180deg around Z (facing robot)
    - Visual: single seat.dae mesh for rendering
    - Collision: 10 convex hulls (seat_1.stl to seat_10.stl) for GJK/EPA collision

    Mesh decomposition rationale:
    - Complex concave seat geometry cannot use GJK directly
    - Pre-decomposed into 10 convex pieces for fast narrow-phase collision
    - makeConvexMesh() creates ConvexMesh type from regular Mesh

    Args:
        robot: Robot instance with environment and resource locator
    """
    locator = robot.locator

    # Resolve package:// URI to filesystem path for visual mesh
    # seat.dae contains full visual detail (curves, materials)
    visual_mesh_path = locator.locateResource(
        "package://tesseract_support/meshes/car_seat/visual/seat.dae"
    ).getFilePath()

    for i in range(3):
        seat_name = f"seat_{i + 1}"
        link_seat = Link(seat_name)

        # === VISUAL GEOMETRY ===
        # Single DAE mesh for rendering (high detail, not used for collision)
        visual = Visual()
        visual.origin = Isometry3d.Identity()

        # createMeshFromPath returns vector<Mesh> (may have multiple meshes in file)
        visual_meshes = createMeshFromPath(visual_mesh_path)
        if visual_meshes:
            visual.geometry = visual_meshes[0]  # Use first mesh from DAE
        link_seat.addVisual(visual)

        # === COLLISION GEOMETRY ===
        # 10 pre-decomposed convex hulls for efficient collision checking
        # Each STL is a convex piece of the original seat geometry
        for m in range(1, 11):  # seat_1.stl through seat_10.stl
            collision_mesh_url = (
                f"package://tesseract_support/meshes/car_seat/collision/seat_{m}.stl"
            )
            collision_mesh_path = locator.locateResource(collision_mesh_url).getFilePath()

            meshes = createMeshFromPath(collision_mesh_path)
            for mesh in meshes:
                collision = Collision()
                collision.origin = visual.origin
                # makeConvexMesh: converts Mesh -> ConvexMesh for GJK/EPA algorithms
                # ConvexMesh stores convex hull representation for fast collision
                collision.geometry = makeConvexMesh(mesh)
                link_seat.addCollision(collision)

        # === JOINT: attach seat to world frame ===
        joint_seat = Joint(f"joint_seat_{i + 1}")
        joint_seat.parent_link_name = "world"
        joint_seat.child_link_name = seat_name
        joint_seat.type = JointType.FIXED  # Static object, not actuated

        # Transform: position (0.5+i, 2.15, 0.45) with 180deg Z rotation
        # Seats are spaced 1m apart along X axis, Y=2.15 (pick station)
        transform_mat = np.eye(4)
        transform_mat[0, 0] = -1.0  # cos(pi) = -1: 180deg rotation
        transform_mat[1, 1] = -1.0  # rotation affects X and Y axes
        transform_mat[:3, 3] = [0.5 + i, 2.15, 0.45]  # Position: X varies, Y/Z fixed
        joint_seat.parent_to_joint_origin_transform = Isometry3d(transform_mat)

        # AddLinkCommand: adds link+joint to scene graph at runtime
        cmd = AddLinkCommand(link_seat, joint_seat)
        if not robot.env.applyCommand(cmd):
            raise RuntimeError(f"Failed to add {seat_name}")

    print("Added 3 seats to environment")


def attach_seat_to_effector(robot, seat_name="seat_1"):
    """Attach a seat to the end effector using MoveLinkCommand.

    This demonstrates dynamic scene graph modification during planning:
    1. Get current world transforms for end_effector and seat
    2. Compute relative transform (seat in end_effector frame)
    3. MoveLinkCommand: reparent seat from world to end_effector
    4. Update allowed collision matrix (adjacent links don't collide)

    This is the "attach" operation in pick-and-place workflows:
    - After picking, object becomes child of gripper
    - Object moves with robot during transport
    - Collision checking accounts for attached geometry

    Args:
        robot: Robot instance with environment
        seat_name: Name of seat link to attach (default: "seat_1")
    """
    state = robot.env.getState()

    # Get current world-frame transforms from environment state
    end_effector_tf = state.link_transforms["end_effector"]
    seat_tf = state.link_transforms[seat_name]

    # === REPARENT SEAT: world -> end_effector ===
    # Create new joint that makes seat a child of end_effector
    joint_seat_robot = Joint(f"joint_{seat_name}_robot")
    joint_seat_robot.parent_link_name = "end_effector"
    joint_seat_robot.child_link_name = seat_name
    joint_seat_robot.type = JointType.FIXED  # Rigidly attached

    # Compute relative transform: T_ee_seat = T_world_ee^-1 * T_world_seat
    # This preserves the seat's world position when reparenting
    relative_tf = np.linalg.inv(end_effector_tf.matrix()) @ seat_tf.matrix()
    joint_seat_robot.parent_to_joint_origin_transform = Isometry3d(relative_tf)

    # MoveLinkCommand: changes parent of existing link in scene graph
    # Seat is removed from world and becomes child of end_effector
    move_cmd = MoveLinkCommand(joint_seat_robot)
    if not robot.env.applyCommand(move_cmd):
        raise RuntimeError(f"Failed to attach {seat_name}")

    # === UPDATE ALLOWED COLLISION MATRIX ===
    # Adjacent links (in contact) should not trigger collision violations
    # "Adjacent" = expected contact, "Never" = explicitly disabled check
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision(seat_name, "end_effector", "Adjacent")  # Gripper-object contact OK
    acm.addAllowedCollision(seat_name, "cell_logo", "Never")  # Static environment
    acm.addAllowedCollision(seat_name, "fence", "Never")  # Workcell boundary
    acm.addAllowedCollision(seat_name, "link_b", "Never")  # Arm links
    acm.addAllowedCollision(seat_name, "link_r", "Never")
    acm.addAllowedCollision(seat_name, "link_t", "Never")

    # ModifyAllowedCollisionsType.ADD merges with existing ACM
    acm_cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
    if not robot.env.applyCommand(acm_cmd):
        raise RuntimeError("Failed to modify allowed collisions")

    print(f"Attached {seat_name} to end effector")


def run():
    """Execute the car seat pick-and-place workflow.

    Workflow phases (from C++ car_seat_example.cpp):
    1. Load 8-DOF robot (carriage_rail + 7-axis arm)
    2. Add 3 seats to environment with mesh geometry
    3. PICK: Plan Home -> Pick1 (freespace motion)
    4. ATTACH: Reparent seat_1 to end_effector
    5. PLACE: Plan Pick1 -> Place1 (collision-aware transport)

    TrajOpt configuration (from C++):
    - Collision constraint: margin=0, buffer=0.005m, coeff=10
    - Collision cost: margin=0.005m, buffer=0.01m, coeff=50
    - Solver: BPMPD, max_iter=200, min_approx_improve=1e-3

    Returns:
        dict: {pick_result, place_result, robot, joint_names} for testing/visualization
    """
    # === PHASE 0: LOAD ROBOT ===
    # car_seat_demo: 8-DOF system with linear carriage + 7-axis arm
    # URDF defines kinematic chain, SRDF defines planning groups and ACM
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/car_seat_demo.urdf",
        "package://tesseract_support/urdf/car_seat_demo.srdf",
    )
    print(f"Loaded robot: {robot.env.getName()}")

    # Get predefined joint configurations (from C++ source)
    positions = get_predefined_positions()

    # Get joint ordering from SRDF-defined "manipulator" group
    joint_group = robot.env.getJointGroup("manipulator")
    joint_names = list(joint_group.getJointNames())
    print(f"Joint names: {joint_names}")

    # === PHASE 1: ADD SEATS TO ENVIRONMENT ===
    # Low-level mesh loading: visual (DAE) + collision (10 convex STLs per seat)
    print("\nAdding seats to environment...")
    add_seats(robot)

    # Initialize robot at home position (all zeros)
    robot.set_joints(positions["Home"])

    # Create task composer for TrajOpt planning
    composer = TaskComposer.from_config()

    # Create TrajOpt profiles matching C++ settings
    profiles = create_car_seat_profiles()

    # ==================== PHASE 2: PICK MOTION ====================
    # Plan freespace motion from Home to Pick1 (above seat_1)
    print("\n=== PICK SEAT 1 ===")

    start_pos = get_position_vector(joint_names, positions["Home"])
    pick_pos = get_position_vector(joint_names, positions["Pick1"])

    # MotionProgram: high-level motion specification
    # - "manipulator": SRDF-defined joint group
    # - tcp_frame: end_effector link for Cartesian operations
    # - StateTarget: joint-space waypoints with planning profile
    pick_program = (
        MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(pick_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"Pick program: {len(pick_program)} instructions")
    print("Running TrajOpt planner for PICK...")

    # TrajOptPipeline: sequential convex optimization for collision-free motion
    pick_result = composer.plan(robot, pick_program, pipeline="TrajOptPipeline", profiles=profiles)

    assert pick_result.successful, f"PICK planning failed: {pick_result.message}"
    print(f"PICK successful! {len(pick_result)} waypoints")

    # ==================== PHASE 3: ATTACH SEAT ====================
    # Update environment state to Pick1, then reparent seat_1 to end_effector
    # This simulates gripper activation after reaching pick position
    print("\n=== ATTACHING SEAT ===")
    robot.set_joints(positions["Pick1"])
    attach_seat_to_effector(robot, "seat_1")

    # ==================== PHASE 4: PLACE MOTION ====================
    # Plan transport motion Pick1 -> Place1 with attached seat
    # TrajOpt now includes seat_1 geometry in collision checking
    print("\n=== PLACE SEAT 1 ===")

    place_start_pos = get_position_vector(joint_names, positions["Pick1"])
    place_end_pos = get_position_vector(joint_names, positions["Place1"])

    place_program = (
        MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(place_start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(place_end_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"Place program: {len(place_program)} instructions")
    print("Running TrajOpt planner for PLACE...")

    # Planning now accounts for attached seat_1 collision geometry
    place_result = composer.plan(
        robot, place_program, pipeline="TrajOptPipeline", profiles=profiles
    )

    assert place_result.successful, f"PLACE planning failed: {place_result.message}"
    print(f"PLACE successful! {len(place_result)} waypoints")

    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main():
    results = run()

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["place_result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
