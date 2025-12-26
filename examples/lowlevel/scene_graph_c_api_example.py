"""
Scene Graph Example

Demonstrates low-level scene graph manipulation using tesseract commands:
- MoveJointCommand: Change a joint's parent link
- MoveLinkCommand: Re-attach a link with a new joint and transform

C++ Source: tesseract_examples/src/scene_graph_example.cpp

Key C++ Implementation Details:
- MoveJointCommand("joint_a4", "base_link"): Reparents joint to base
- MoveLinkCommand uses transform: -pi/2 rotation around Y, then translate (0.15, 0, 0)
- Order matters: rotation first, then translation (Isometry3d composition)

Use Cases:
- Reconfiguring robot for tool changes
- Dynamic workcell modifications
- Testing kinematic reachability with modified chains
"""

import sys
import numpy as np
import math

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    Translation3d,
    AngleAxisd,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    MoveJointCommand,
    MoveLinkCommand,
)
from tesseract_robotics.tesseract_scene_graph import Joint, JointType

# Viewer (skip in pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def main():
    """
    Demonstrates three scene graph operations:
    1. MoveJointCommand - change joint's parent link
    2. MoveLinkCommand - re-attach link with new joint and transform
    3. Scene graph queries - traverse kinematic tree
    """
    # Resource locator resolves package:// URIs to filesystem paths
    locator = GeneralResourceLocator()

    # Load KUKA IIWA 7-DOF robot
    urdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
    srdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    # Initialize environment (loads URDF/SRDF, builds scene graph)
    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")
    print(f"Root link: {env.getRootLinkName()}")
    print(f"Links: {list(env.getLinkNames())}")
    print(f"Joints: {list(env.getJointNames())}")

    # Scene graph is a directed acyclic graph of links connected by joints
    scene_graph = env.getSceneGraph()
    print(f"\nScene graph name: {scene_graph.getName()}")

    # DEBUG: Uncomment to visualize graph structure
    # scene_graph.saveDOT("scene_graph_example_initial.dot")

    # =========================================================================
    # Example 1: MoveJointCommand
    # Reparents an existing joint to a different parent link
    # Use case: Shortcut kinematic chain (e.g., for testing reachability)
    # =========================================================================
    print("\n--- Example 1: MoveJointCommand ---")
    print("Moving joint 'joint_a4' to have parent 'base_link' instead of 'link_3'")

    # Query current joint configuration before modification
    joint = scene_graph.getJoint("joint_a4")
    if joint:
        print(f"  Current parent: {joint.parent_link_name}")
        print(f"  Current child: {joint.child_link_name}")

    # MoveJointCommand(joint_name, new_parent_link)
    # This preserves the joint but changes parent_link_name
    move_joint_cmd = MoveJointCommand("joint_a4", "base_link")
    if env.applyCommand(move_joint_cmd):
        print("  Command applied successfully!")
        # Verify the change took effect
        joint = env.getSceneGraph().getJoint("joint_a4")
        if joint:
            print(f"  New parent: {joint.parent_link_name}")
    else:
        print("  Failed to apply command")

    # Reinitialize environment for clean state in next example
    env = Environment()
    env.init(urdf_path, srdf_path, locator)

    # =========================================================================
    # Example 2: MoveLinkCommand
    # Re-attaches a link using a NEW joint with custom transform
    # More powerful than MoveJointCommand - can specify full joint properties
    # =========================================================================
    print("\n--- Example 2: MoveLinkCommand ---")
    print("Moving link_4 to be attached to link_1 with a new fixed joint")

    # Create the new joint that will connect link_1 -> link_4
    new_joint = Joint("moved_link_joint")
    new_joint.parent_link_name = "link_1"
    new_joint.child_link_name = "link_4"
    new_joint.type = JointType.FIXED  # Could also be REVOLUTE, PRISMATIC, etc.

    # Build transform: rotate -90deg around Y, then translate (0.15, 0, 0)
    # C++ equivalent: Isometry3d::Identity() * AngleAxisd(-pi/2, Y) * Translation3d(0.15, 0, 0)
    # Order: rotation applied first, then translation in rotated frame
    transform = Isometry3d.Identity()
    transform = transform * AngleAxisd(
        -math.pi / 2, np.array([0, 1, 0], dtype=np.float64)
    )
    transform = transform * Translation3d(0.15, 0.0, 0.0)
    new_joint.parent_to_joint_origin_transform = transform

    # MoveLinkCommand replaces the joint connecting to child_link_name
    move_link_cmd = MoveLinkCommand(new_joint)
    if env.applyCommand(move_link_cmd):
        print("  Command applied successfully!")
        print("  Link 4 now attached to link_1 via joint 'moved_link_joint'")
        # Verify the new joint exists in scene graph
        joint = env.getSceneGraph().getJoint("moved_link_joint")
        if joint:
            print(f"  New joint parent: {joint.parent_link_name}")
            print(f"  New joint child: {joint.child_link_name}")
    else:
        print("  Failed to apply command")

    # =========================================================================
    # Example 3: Scene Graph Queries
    # Demonstrates traversal and inspection of the kinematic structure
    # =========================================================================
    print("\n--- Example 3: Scene Graph Queries ---")
    print(f"Number of links: {scene_graph.getLinks().__len__()}")
    print(f"Number of joints: {scene_graph.getJoints().__len__()}")

    # getAdjacentLinkNames returns links directly connected by a joint
    root_link = env.getRootLinkName()
    adjacent = scene_graph.getAdjacentLinkNames(root_link)
    print(f"\nLinks adjacent to '{root_link}': {list(adjacent)}")

    # Traverse kinematic tree using DFS
    # Note: scene graph may have multiple branches (tool, sensors, etc.)
    print("\nKinematic chain from root:")
    visited = set()
    stack = [(root_link, 0)]

    while stack:
        link_name, depth = stack.pop()
        if link_name in visited:
            continue
        visited.add(link_name)
        print(f"  {'  ' * depth}{link_name}")

        # Find child links by iterating joints where this is parent
        for joint in scene_graph.getJoints():
            if joint.parent_link_name == link_name:
                stack.append((joint.child_link_name, depth + 1))

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
