"""
Collision Checking Example
==========================

Demonstrates collision detection using Tesseract's discrete contact manager.

C++ Reference:
    tesseract_collision/examples/bullet_discrete_simple_example.cpp

Overview
--------
This example shows the collision checking workflow:
1. Load robot and add obstacle geometry to the environment
2. Get DiscreteContactManager from environment
3. Sweep through robot configurations checking for collisions
4. Interpret contact results (distances, link pairs)

Key Concepts
------------
**Contact Managers**:
    - DiscreteContactManager: Check collision at single poses (this example)
    - ContinuousContactManager: Check collision along trajectory (motion planning)

**Collision Margin**:
    The margin expands collision geometry by specified distance. A margin of 0.1m
    means objects report contact when within 10cm of each other. This provides:
    - Safety buffer for planning
    - Gradient information for optimizers (TrajOpt)
    - Detection before actual collision

**Contact Result Fields**:
    - distance: Signed penetration depth (negative = overlap, positive = separation)
    - link_names: Pair of colliding link names [link_a, link_b]
    - nearest_points: Closest points on each geometry
    - normal: Contact normal vector

Pipeline
--------
1. Environment Setup:
   - Load URDF/SRDF defining robot links and collision geometry
   - Add obstacle objects with collision components

2. Contact Manager Configuration:
   - setActiveCollisionObjects(): Which links to check
   - setCollisionMarginData(): Safety buffer distance

3. Query Loop:
   - Update robot joint state
   - setCollisionObjectsTransform(): Sync manager with new link poses
   - contactTest(): Run collision query
   - Iterate ContactResultMap for collision pairs

Related Examples
----------------
- geometry_showcase_example.py: Geometry types for collision
- freespace_ompl_example.py: Collision-free motion planning
- basic_cartesian_example.py: TrajOpt uses collision gradients
"""

import numpy as np
from tesseract_robotics.planning import Robot, Pose, sphere, create_fixed_joint
from tesseract_robotics.tesseract_scene_graph import Link, Visual, Collision
from tesseract_robotics.tesseract_collision import (
    ContactResultMap,
    ContactTestType_ALL,
    ContactRequest,
    ContactResultVector,
)
from tesseract_robotics.tesseract_common import CollisionMarginData


def main():
    # =========================================================================
    # STEP 1: Environment Setup
    # =========================================================================
    # Load ABB IRB2400 robot - a common 6-axis industrial manipulator
    robot = Robot.from_tesseract_support("abb_irb2400")

    # Create an obstacle sphere in the robot's workspace
    # This sphere will be positioned where the robot may collide during motion
    sphere_link = Link("sphere_link")
    sphere_geom = sphere(0.1)  # 10cm radius sphere

    # Visual component for rendering (optional but helpful for debugging)
    visual = Visual()
    visual.geometry = sphere_geom
    sphere_link.visual.append(visual)

    # Collision component - this is what the contact manager actually checks
    # Without this, the sphere would be visible but not cause collisions
    collision = Collision()
    collision.geometry = sphere_geom
    sphere_link.collision.append(collision)

    # Attach sphere to environment at fixed position
    # Position chosen to be within robot reach for collision demonstration
    sphere_joint = create_fixed_joint(
        name="sphere_joint",
        parent_link="base_link",
        child_link="sphere_link",
        origin=Pose.from_xyz(0.7, 0, 1.5),  # 70cm forward, 1.5m high
    )
    robot.add_link(sphere_link, sphere_joint)

    # =========================================================================
    # STEP 2: Contact Manager Configuration
    # =========================================================================
    # DiscreteContactManager checks collision at instantaneous poses
    # (vs ContinuousContactManager which checks along motion paths)
    manager = robot.env.getDiscreteContactManager()

    # Specify which links to include in collision checking
    # getActiveLinkNames() returns all links with collision geometry
    manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())

    # Collision margin: report contacts when objects are within 10cm
    # Larger margin = earlier detection but more false positives
    # TrajOpt uses margin for gradient-based optimization
    manager.setCollisionMarginData(CollisionMarginData(0.1))

    # =========================================================================
    # STEP 3: Collision Query Loop
    # =========================================================================
    # ABB IRB2400 has 6 revolute joints named joint_1 through joint_6
    joint_names = [f"joint_{i+1}" for i in range(6)]
    joint_pos = np.zeros(6)

    # Sweep joint_1 (base rotation) to move arm toward/away from obstacle
    for i in range(-5, 5):
        joint_pos[0] = i * np.deg2rad(5)  # -25 to +20 degrees
        print(f"Contact check at robot position: {joint_pos}")

        # Update environment state with new joint positions
        # This recomputes all link transforms through forward kinematics
        robot.set_joints(joint_pos, joint_names=joint_names)
        scene_state = robot.env.getState()

        # CRITICAL: Sync contact manager with updated link transforms
        # The manager caches transforms for performance - must update manually
        manager.setCollisionObjectsTransform(scene_state.link_transforms)

        # Debug: show current poses of relevant links
        print(f"Link 6 Pose:\n{scene_state.link_transforms['link_6'].matrix()}")
        print(f"Sphere Link Pose:\n{scene_state.link_transforms['sphere_link'].matrix()}")

        # Execute collision query
        # ContactTestType_ALL finds all collision pairs (vs FIRST for early-out)
        contact_result_map = ContactResultMap()
        manager.contactTest(contact_result_map, ContactRequest(ContactTestType_ALL))

        # Flatten nested map structure into simple vector for iteration
        result_vector = ContactResultVector()
        contact_result_map.flattenMoveResults(result_vector)

        # Interpret results
        print(f"Found {len(result_vector)} contact results")
        for j in range(len(result_vector)):
            contact_result = result_vector[j]
            print(f"Contact {j}:")
            # distance < 0: penetration, distance > 0: within margin but separated
            print(f"\tDistance: {contact_result.distance}")
            print(f"\tLink A: {contact_result.link_names[0]}")
            print(f"\tLink B: {contact_result.link_names[1]}")
        print()


if __name__ == "__main__":
    main()
