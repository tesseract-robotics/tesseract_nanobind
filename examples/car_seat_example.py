"""
Car Seat Example - Complex Multi-Step Pick & Place with Dynamic Mesh Objects

This example demonstrates advanced Tesseract motion planning for automotive
manufacturing: picking car seats from a staging area and placing them in a
vehicle body. It showcases dynamic scene modification, mesh handling, and
multi-phase planning workflows.

PIPELINE OVERVIEW
-----------------
1. LOAD ENVIRONMENT: 8-DOF robot (linear carriage + 7-axis arm), workcell
2. ADD SEATS: Dynamic mesh loading - visual (DAE) + collision (10 convex STLs per seat)
3. PICK PHASE: Plan freespace motion Home -> Pick1 (above seat_1)
4. ATTACH: Reparent seat_1 from world to end_effector (MoveLinkCommand)
5. PLACE PHASE: Plan transport motion Pick1 -> Place1 with attached object

KEY CONCEPTS DEMONSTRATED
-------------------------
1. Dynamic Scene Graph Modification:
   - AddLinkCommand: add objects at runtime
   - MoveLinkCommand: reparent links (attach/detach objects)
   - ModifyAllowedCollisionsCommand: update collision rules

2. Mesh Decomposition for Collision:
   - Visual: single DAE mesh for rendering (high detail)
   - Collision: 10 convex hulls per seat for efficient GJK/EPA collision
   - makeConvexMesh(): converts triangle mesh to convex representation

3. Multi-Phase Planning:
   - Sequential planning phases with environment modifications between
   - Collision checking includes attached geometry after attachment

4. 8-DOF System:
   - carriage_rail: linear prismatic joint (extends workspace)
   - joint_s, l, e, u, r, b, t: 7-DOF arm (Motoman-style naming)

ROBOT CONFIGURATION (from C++)
------------------------------
Joint positions defined in getPredefinedPosition():
- Home: all zeros (robot at origin)
- Pick1: carriage=2.22m, arm configured for downward reach at seat_1
- Pick2/Pick3: similar configurations for seats 2 and 3
- Place1: carriage=4.15m (far end), arm oriented for vehicle mounting

TRAJOPT SETTINGS (from C++)
---------------------------
Collision constraint: margin=0, buffer=0.005m, coeff=10
Collision cost: margin=0.005m, buffer=0.01m, coeff=50
Solver: OSQP, max_iter=200, min_approx_improve=1e-3

C++ SOURCE
----------
tesseract_planning/tesseract_examples/src/car_seat_example.cpp
Author: Levi Armstrong, Southwest Research Institute, July 2019

RELATED EXAMPLES
----------------
- pick_and_place_example.py: simpler pick-and-place workflow
- glass_upright_example.py: orientation-constrained motion
- lowlevel/car_seat_c_api_example.py: low-level API version with detailed comments
"""
import sys
import numpy as np

from tesseract_robotics.planning import Robot, MotionProgram, StateTarget, TaskComposer
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles, create_trajopt_default_profiles
from tesseract_robotics.tesseract_common import Isometry3d, AllowedCollisionMatrix
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    MoveLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
)
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType, Visual, Collision
from tesseract_robotics.tesseract_geometry import createMeshFromPath
from tesseract_robotics.tesseract_collision import makeConvexMesh

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

# Predefined joint positions from C++ getPredefinedPosition()
# All values in radians except carriage_rail (meters)
# Joint order varies - use dict keys for order-independent access
POSITIONS = {
    # Home: all joints at zero (robot at origin, arm extended)
    "Home": {
        "carriage_rail": 0.0, "joint_b": 0.0, "joint_e": 0.0, "joint_l": 0.0,
        "joint_r": 0.0, "joint_s": 0.0, "joint_t": 0.0, "joint_u": 0.0,
    },
    # Pick1: carriage at 2.22m, arm reaching down to seat_1 at (0.5, 2.15, 0.45)
    # joint_s=-3.14 (~-pi): shoulder fully rotated for downward reach
    "Pick1": {
        "carriage_rail": 2.22, "joint_b": 0.39, "joint_e": 0.0, "joint_l": 0.5,
        "joint_r": 0.0, "joint_s": -3.14, "joint_t": -0.29, "joint_u": -1.45,
    },
    # Place1: carriage at 4.15m (far end of rail), arm oriented for vehicle mounting
    "Place1": {
        "carriage_rail": 4.15466, "joint_b": 0.537218, "joint_e": 0.0189056, "joint_l": 0.801223,
        "joint_r": 0.0580309, "joint_s": -0.0481182, "joint_t": -0.325783, "joint_u": -1.2813,
    },
}


def get_position_vector(joint_names, pos_dict):
    """Get joint position vector from a position dictionary."""
    return np.array([pos_dict[name] for name in joint_names])


def add_seats(robot):
    """Create and add 3 seat links with visual and collision meshes.

    From C++ addSeats(): creates seats at positions (0.5+i, 2.15, 0.45)
    for i=0,1,2, each rotated 180deg around Z (facing robot).

    Mesh decomposition rationale:
    - Visual: single seat.dae for rendering (high polygon, materials)
    - Collision: 10 pre-decomposed convex hulls for efficient GJK/EPA
    - Complex concave geometry cannot use GJK directly
    """
    locator = robot.locator
    visual_path = locator.locateResource(
        "package://tesseract_support/meshes/car_seat/visual/seat.dae"
    ).getFilePath()

    for i in range(3):
        seat_name = f"seat_{i + 1}"
        link = Link(seat_name)

        # Visual mesh
        visual = Visual()
        visual.origin = Isometry3d.Identity()
        visual_meshes = createMeshFromPath(visual_path)
        if visual_meshes:
            visual.geometry = visual_meshes[0]
        link.visual.append(visual)

        # Collision meshes (10 convex hulls)
        for m in range(1, 11):
            collision_url = f"package://tesseract_support/meshes/car_seat/collision/seat_{m}.stl"
            collision_path = locator.locateResource(collision_url).getFilePath()
            for mesh in createMeshFromPath(collision_path):
                collision = Collision()
                collision.origin = visual.origin
                collision.geometry = makeConvexMesh(mesh)
                link.collision.append(collision)

        # Fixed joint to world (rotated 180deg around Z, positioned along X)
        joint = Joint(f"joint_seat_{i + 1}")
        joint.parent_link_name = "world"
        joint.child_link_name = seat_name
        joint.type = JointType.FIXED
        transform = np.eye(4)
        transform[0, 0] = transform[1, 1] = -1.0  # 180deg Z rotation
        transform[:3, 3] = [0.5 + i, 2.15, 0.45]
        joint.parent_to_joint_origin_transform = Isometry3d(transform)

        if not robot.env.applyCommand(AddLinkCommand(link, joint)):
            raise RuntimeError(f"Failed to add {seat_name}")

    print("Added 3 seats to environment")


def attach_seat(robot, seat_name="seat_1"):
    """Attach seat to end effector using MoveLinkCommand.

    This demonstrates dynamic scene graph modification during planning:
    1. Get current world transforms for end_effector and seat
    2. Compute relative transform (seat in end_effector frame)
    3. MoveLinkCommand: reparent seat from world to end_effector
    4. Update allowed collision matrix (adjacent links don't collide)

    After this, the seat moves with the robot during transport and
    collision checking accounts for the attached geometry.
    """
    state = robot.env.getState()
    ee_tf = state.link_transforms["end_effector"]
    seat_tf = state.link_transforms[seat_name]

    # Create fixed joint: reparent seat from world to end_effector
    # Compute relative transform: T_ee_seat = T_world_ee^-1 * T_world_seat
    joint = Joint(f"joint_{seat_name}_robot")
    joint.parent_link_name = "end_effector"
    joint.child_link_name = seat_name
    joint.type = JointType.FIXED
    relative_tf = np.linalg.inv(ee_tf.matrix()) @ seat_tf.matrix()
    joint.parent_to_joint_origin_transform = Isometry3d(relative_tf)

    # MoveLinkCommand: changes parent of existing link in scene graph
    if not robot.env.applyCommand(MoveLinkCommand(joint)):
        raise RuntimeError(f"Failed to attach {seat_name}")

    # Update allowed collision matrix - adjacent links should not collide
    # "Adjacent" = expected contact (gripper-object), "Never" = disabled check
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision(seat_name, "end_effector", "Adjacent")
    for link in ["cell_logo", "fence", "link_b", "link_r", "link_t"]:
        acm.addAllowedCollision(seat_name, link, "Never")

    robot.env.applyCommand(ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD))
    print(f"Attached {seat_name} to end effector")


def plan_motion(robot, composer, joint_names, start_pos, end_pos, phase_name, pipeline="TrajOptPipeline", profiles=None):
    """Plan a motion from start to end position."""
    program = (MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(end_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"\n=== {phase_name} ===")
    print(f"Planning with {pipeline}...")
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

    assert result.successful, f"{phase_name} failed: {result.message}"
    print(f"{phase_name} OK: {len(result)} waypoints")
    return result


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Execute the car seat pick-and-place workflow.

    Workflow phases (from C++ CarSeatExample::run()):
    1. Load 8-DOF robot (carriage_rail + 7-axis arm)
    2. Add 3 seats to environment with mesh geometry
    3. PICK: Plan Home -> Pick1 (freespace motion)
    4. ATTACH: Reparent seat_1 to end_effector
    5. PLACE: Plan Pick1 -> Place1 (collision-aware transport)

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "FreespacePipeline")
        num_planners: Number of parallel OMPL planners (for FreespacePipeline)

    Returns:
        dict: {pick_result, place_result, robot, joint_names} for testing/viz
    """
    # === PHASE 0: LOAD ROBOT ===
    # car_seat_demo: 8-DOF system with linear carriage + 7-axis arm
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/car_seat_demo.urdf",
        "package://tesseract_support/urdf/car_seat_demo.srdf"
    )
    print(f"Loaded robot: {robot.env.getName()}")

    # Get joint ordering from SRDF-defined "manipulator" group
    joint_group = robot.env.getJointGroup("manipulator")
    joint_names = list(joint_group.getJointNames())
    print(f"Joint names: {joint_names}")

    # === PHASE 1: ADD SEATS TO ENVIRONMENT ===
    # Low-level mesh loading: visual (DAE) + collision (10 convex STLs per seat)
    add_seats(robot)

    # Get position vectors in correct joint order (from SRDF group)
    home_pos = get_position_vector(joint_names, POSITIONS["Home"])
    pick_pos = get_position_vector(joint_names, POSITIONS["Pick1"])
    place_pos = get_position_vector(joint_names, POSITIONS["Place1"])

    robot.set_joints(POSITIONS["Home"])

    composer = TaskComposer.from_config()

    # Create profiles based on pipeline type
    if "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        profiles = create_trajopt_default_profiles()

    # === PHASE 2: PICK MOTION ===
    # Plan freespace motion from Home to Pick1 (above seat_1)
    pick_result = plan_motion(robot, composer, joint_names, home_pos, pick_pos, "PICK", pipeline, profiles)

    # === PHASE 3: ATTACH SEAT ===
    # Update environment state to Pick1, then reparent seat_1 to end_effector
    print("\n=== ATTACH SEAT ===")
    robot.set_joints(POSITIONS["Pick1"])
    attach_seat(robot, "seat_1")

    # === PHASE 4: PLACE MOTION ===
    # Plan transport motion Pick1 -> Place1 with attached seat
    # TrajOpt now includes seat_1 geometry in collision checking
    place_result = plan_motion(robot, composer, joint_names, pick_pos, place_pos, "PLACE", pipeline, profiles)

    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main():
    results = run()

    # Visualize
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
