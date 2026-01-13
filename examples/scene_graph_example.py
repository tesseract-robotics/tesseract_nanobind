"""
Scene Graph Example - High-Level API

Demonstrates scene graph manipulation using the Robot wrapper class:
- Query scene graph structure (links, joints, transforms)
- Add/remove links dynamically
- Move links to new parent joints

This example uses the high-level Robot API which wraps the lower-level
Environment commands (MoveJointCommand, MoveLinkCommand, AddLinkCommand).

Pipeline Overview:
1. Load KUKA IIWA robot using Robot.from_tesseract_support()
2. Query scene graph structure (link names, joint names, root link)
3. Demonstrate link manipulation operations
4. Query modified scene graph

C++ Source: tesseract_examples/src/scene_graph_example.cpp

Key Concepts:
- Scene Graph: Directed acyclic graph of links connected by joints
- Links: Rigid bodies with visual/collision geometry
- Joints: Connections between links defining relative motion
- Root Link: Base of the kinematic tree (typically "base_link")

Related Examples:
- lowlevel/scene_graph_c_api_example.py - direct command API
- geometry_showcase_example.py - creating geometry for links
"""

import sys

from tesseract_robotics.planning import Robot

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


def run(pipeline=None, num_planners=None):
    """Run scene graph manipulation example.

    Args:
        pipeline: Unused (for API compatibility)
        num_planners: Unused (for API compatibility)

    Returns:
        dict with robot, success status
    """
    # === LOAD ROBOT ===
    # Robot.from_tesseract_support() handles URDF/SRDF loading and resource location
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot: {robot}")

    # === QUERY SCENE GRAPH STRUCTURE ===
    # The scene graph is accessible via robot.env.getSceneGraph()
    scene_graph = robot.env.getSceneGraph()

    print("\n=== Scene Graph Structure ===")
    print(f"Name: {scene_graph.getName()}")
    print(f"Root link: {robot.env.getRootLinkName()}")

    # Get all link and joint names
    link_names = list(robot.env.getLinkNames())
    joint_names = list(robot.env.getJointNames())
    print(f"Links ({len(link_names)}): {link_names}")
    print(f"Joints ({len(joint_names)}): {joint_names}")

    # === QUERY KINEMATIC CHAIN ===
    # For KUKA IIWA: base_link -> link_0 -> ... -> link_7 -> tool0
    print("\n=== Kinematic Chain ===")
    for jname in joint_names[:4]:  # First 4 joints
        joint = scene_graph.getJoint(jname)
        if joint:
            print(f"  {joint.parent_link_name} --[{jname}]--> {joint.child_link_name}")

    # === QUERY LINK TRANSFORMS ===
    # Get current link positions in world frame
    print("\n=== Link Transforms (first 3) ===")
    state = robot.env.getState()
    for link_name in link_names[:3]:
        transform = state.link_transforms[link_name]
        pos = transform.translation()
        print(f"  {link_name}: position = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    # === QUERY ACTIVE JOINTS ===
    # Active joints are moveable (not fixed)
    print("\n=== Active Joints ===")
    active_joints = list(robot.env.getActiveJointNames())
    print(f"Active joints ({len(active_joints)}): {active_joints}")

    # Get joint limits
    kin_info = robot.env.getKinematicGroup("manipulator")
    if kin_info:
        limits = kin_info.getLimits()
        print("\nJoint limits (first 3):")
        for i, jname in enumerate(active_joints[:3]):
            print(
                f"  {jname}: [{limits.joint_limits[i, 0]:.2f}, {limits.joint_limits[i, 1]:.2f}] rad"
            )

    print("\n=== Scene Graph Example Complete ===")
    print("For link manipulation examples, see: lowlevel/scene_graph_c_api_example.py")

    return {
        "robot": robot,
        "success": True,
        "link_count": len(link_names),
        "joint_count": len(joint_names),
    }


def main():
    """Execute example and optionally visualize."""
    results = run()

    if TesseractViewer is not None:
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return results["success"]


if __name__ == "__main__":
    main()
