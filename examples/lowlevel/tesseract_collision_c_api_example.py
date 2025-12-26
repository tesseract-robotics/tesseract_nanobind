"""
Low-Level Collision Checking Example (C-API Style)
===================================================

Demonstrates collision detection using Tesseract's discrete contact manager
via the low-level C++ bindings without high-level abstractions.

C++ Reference:
    tesseract_collision/examples/bullet_discrete_simple_example.cpp

High-Level Alternative:
    examples/tesseract_collision_example.py (uses Robot wrapper class)

Overview
--------
This example directly uses the C++ API bindings for maximum control:
1. Manually load URDF/SRDF and initialize Environment
2. Construct Link/Joint/Collision objects explicitly
3. Use OFKTStateSolver for forward kinematics
4. Query DiscreteContactManager for collision detection

Why Use Low-Level API?
----------------------
- Direct access to all C++ functionality
- Fine-grained control over object lifetimes
- Performance-critical applications
- When the high-level Robot API doesn't expose needed features

Key Concepts
------------
**DiscreteContactManager**:
    Checks collision at instantaneous poses (single point in time).
    Backed by Bullet physics engine for fast broad-phase/narrow-phase queries.
    Must be refreshed after environment changes (new links, removed links).

**Collision Margin (CollisionMarginData)**:
    Expands collision geometry by specified distance. With margin=0.1m:
    - Objects report contact when within 10cm of each other
    - distance > 0: separated but within margin
    - distance < 0: actual penetration/overlap
    - distance = 0: touching at surface

    Margin enables:
    - Safety buffer for motion planning
    - Gradient information for optimization (TrajOpt needs smooth cost)
    - Early collision warning before actual contact

**ContactResultMap / ContactResultVector**:
    - ContactResultMap: Nested map keyed by (link_a, link_b) pairs
    - ContactResultVector: Flattened list for simpler iteration
    - Each ContactResult contains: distance, link_names, nearest_points, normal

**OFKTStateSolver (Optimized Forward Kinematics Tree)**:
    Computes link transforms from joint positions. More efficient than full
    dynamics solver when only poses are needed (not velocities/accelerations).

**Environment Commands**:
    AddLinkCommand, RemoveLinkCommand, etc. modify the scene graph.
    After modifications, call getDiscreteContactManager() again to refresh.

Pipeline
--------
1. Environment Setup:
   - GeneralResourceLocator finds URDF/SRDF via TESSERACT_RESOURCE_PATH
   - Environment.init() parses robot description and loads collision plugins
   - AddLinkCommand adds obstacle geometry to scene

2. State Solver Configuration:
   - OFKTStateSolver wraps the scene graph
   - setStateByNamesAndValues() updates joint positions
   - getState() returns SceneState with all link_transforms

3. Contact Manager Configuration:
   - getDiscreteContactManager() returns Bullet-based manager
   - setActiveCollisionObjects() filters which links to check
   - setCollisionMarginData() sets safety buffer distance

4. Query Loop:
   - Update joint positions via state solver
   - Sync manager with setCollisionObjectsTransform()
   - contactTest() populates ContactResultMap
   - Flatten and iterate results

Related Examples
----------------
- tesseract_collision_example.py: Same workflow with Robot API
- geometry_showcase_example.py: All supported collision geometry types
- scene_graph_example.py: Direct scene graph manipulation
"""

from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, Isometry3d, Translation3d, \
    CollisionMarginData
from tesseract_robotics.tesseract_environment import Environment, AddLinkCommand
from tesseract_robotics.tesseract_scene_graph import Joint, Link, Visual, Collision, JointType_FIXED
from tesseract_robotics.tesseract_geometry import Sphere
from tesseract_robotics.tesseract_collision import ContactResultMap, ContactTestType_ALL, \
    ContactRequest, ContactResultVector
from tesseract_robotics.tesseract_state_solver import OFKTStateSolver
import numpy as np

# Initialize Environment with a robot from URDF file
# The collision checker is configured using a yaml configuration file specified by the SRDF file. This configuration
# file must be configured for collision checking to work. This example uses the `contact_manager_plugins.yaml` file
# to configure the plugins using Bullet for collision checking. This configuration file can be copied and
# used for most scenes.

# This example uses the GeneralResourceLocator to find resources on the file system. The GeneralResourceLocator
# uses the TESSERACT_RESOURCE_PATH environmental variable.
#
# TESSERACT_RESOURCE_PATH must be set to the directory containing the `tesseract_support` package. This can be done
# by running:
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# export TESSERACT_RESOURCE_PATH="$(pwd)/tesseract/"
#
# or on Windows
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# set TESSERACT_RESOURCE_PATH=%cd%\tesseract\


def main():
    # =========================================================================
    # STEP 1: Environment Initialization
    # =========================================================================
    # GeneralResourceLocator resolves "package://" URIs using TESSERACT_RESOURCE_PATH
    # This env var should point to a directory containing tesseract_support/urdf/
    locator = GeneralResourceLocator()
    env = Environment()

    # Locate URDF (robot geometry) and SRDF (semantic info: groups, ACM, plugins)
    urdf_path_str = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf").getFilePath()
    srdf_path_str = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.srdf").getFilePath()

    # FilesystemPath wraps std::filesystem::path for cross-platform compatibility
    urdf_path = FilesystemPath(urdf_path_str)
    srdf_path = FilesystemPath(srdf_path_str)

    # init() parses URDF/SRDF and loads configured plugins (collision, kinematics)
    # Returns False if parsing fails - always check return value
    assert env.init(urdf_path, srdf_path, locator)

    # ABB IRB2400 is a 6-axis industrial robot with joints named joint_1..joint_6
    robot_joint_names = [f"joint_{i+1}" for i in range(6)]
    robot_joint_pos = np.zeros(6)

    # =========================================================================
    # STEP 2: Add Obstacle Geometry via Environment Commands
    # =========================================================================
    # Create a sphere obstacle to demonstrate collision detection
    # Links must have both Visual (rendering) and Collision (physics) components
    sphere_link = Link("sphere_link")

    # Visual component: for rendering/visualization (optional for collision)
    sphere_link_visual = Visual()
    sphere_link_visual.geometry = Sphere(0.1)  # 10cm radius
    sphere_link.visual.append(sphere_link_visual)

    # Collision component: this is what the contact manager actually checks
    # A link without collision geometry is invisible to the collision checker
    sphere_link_collision = Collision()
    sphere_link_collision.geometry = Sphere(0.1)
    sphere_link.collision.append(sphere_link_collision)

    # Joint connects child link to parent link with a transform
    # JointType_FIXED means no relative motion (obstacle is stationary)
    sphere_joint = Joint("sphere_joint")
    sphere_joint.parent_link_name = "base_link"
    sphere_joint.child_link_name = sphere_link.getName()
    sphere_joint.type = JointType_FIXED

    # Position sphere at (0.7, 0, 1.5)m relative to base_link
    # This is within the robot's workspace to trigger collision during sweep
    sphere_link_joint_transform = Isometry3d.Identity() * Translation3d(0.7, 0, 1.5)
    sphere_joint.parent_to_joint_origin_transform = sphere_link_joint_transform

    # Apply command to modify environment - this updates the scene graph
    add_sphere_command = AddLinkCommand(sphere_link, sphere_joint)
    env.applyCommand(add_sphere_command)

    # =========================================================================
    # STEP 3: Configure State Solver
    # =========================================================================
    # OFKTStateSolver computes forward kinematics for all links efficiently
    # Using OFKTStateSolver directly since getStateSolver() returns a different type
    scene_graph = env.getSceneGraph()
    solver = OFKTStateSolver(scene_graph)

    # =========================================================================
    # STEP 4: Configure Discrete Contact Manager
    # =========================================================================
    # IMPORTANT: getDiscreteContactManager() returns a clone - call again after
    # any environment modifications to get an updated manager
    manager = env.getDiscreteContactManager()

    # Specify which links participate in collision checking
    # getActiveLinkNames() returns all links with collision geometry
    manager.setActiveCollisionObjects(env.getActiveLinkNames())

    # Collision margin: report contacts when objects are within this distance
    # margin=0.1 means contacts reported when separation < 10cm
    margin_data = CollisionMarginData(0.1)
    manager.setCollisionMarginData(margin_data)

    # =========================================================================
    # STEP 5: Collision Query Loop
    # =========================================================================
    # Sweep joint_1 (base rotation) to move robot toward/away from obstacle
    for i in range(-5, 5):
        robot_joint_pos[0] = i * np.deg2rad(5)  # -25 to +20 degrees
        print(f"Contact check at robot position: {robot_joint_pos}")

        # Update state solver with new joint positions
        # This recomputes all link transforms via forward kinematics
        solver.setStateByNamesAndValues(robot_joint_names, robot_joint_pos)
        scene_state = solver.getState()

        # CRITICAL: Sync contact manager with updated link transforms
        # Manager caches transforms for performance - must update manually
        manager.setCollisionObjectsTransform(scene_state.link_transforms)

        # Debug output: verify link poses are updating correctly
        print(f"Link 6 Pose: {scene_state.link_transforms['link_6'].matrix()}")
        print(f"Sphere Link Pose: {scene_state.link_transforms[sphere_link.getName()].matrix()}")

        # Execute collision query
        # ContactTestType_ALL: find all collision pairs (vs FIRST for early-out)
        contact_result_map = ContactResultMap()
        manager.contactTest(contact_result_map, ContactRequest(ContactTestType_ALL))

        # ContactResultMap is nested: map<link_a, map<link_b, vector<ContactResult>>>
        # Flatten to simple vector for easier iteration
        result_vector = ContactResultVector()
        contact_result_map.flattenMoveResults(result_vector)

        # Interpret contact results
        print(f"Found {len(result_vector)} contact results")
        for j in range(len(result_vector)):
            contact_result = result_vector[j]
            print(f"Contact {j}:")
            # distance interpretation:
            #   < 0: penetration (overlapping geometry)
            #   = 0: touching at surface
            #   > 0: separated but within collision margin
            print(f"\tDistance: {contact_result.distance}")
            print(f"\tLink A: {contact_result.link_names[0]}")
            print(f"\tLink B: {contact_result.link_names[1]}")
        print()


if __name__ == "__main__":
    main()
   