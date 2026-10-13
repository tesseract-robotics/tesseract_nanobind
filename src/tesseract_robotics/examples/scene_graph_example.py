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

Building a Scene Graph (low-level section)
------------------------------------------
`build_scene_graph()` builds upstream's five-link test graph directly with
`SceneGraph`, `Link` and `Joint`, without an `Environment`, and queries it.

C++ Reference:
    scene_graph/examples/build_scene_graph_example.cpp  (graph, queries, tree checks, cycle)
    scene_graph/test/tesseract_scene_graph_unit.cpp:40-140  (createTestSceneGraph, runTest)
    scene_graph/test/tesseract_scene_graph_joint_unit.cpp  (joint value types, JointType output)
    scene_graph/src/joint.cpp:49-198, graph.cpp:1295-1309  (operator<<, printed by str())

1. Links `link_1`..`link_5` and joints `joint_1`..`joint_4` of each `JointType`,
   with `JointLimits` on the planar and revolute joints, and four allowed collisions.
2. Adjacent links, children, `getAdjacencyMap`, `isAcyclic` / `isTree`.
3. `str()` of each `JointType`, of the joint value types and of a `ShortestPath`.
4. `setAllowedCollisionMatrix` / `isCollisionAllowed`, `Link.visible` /
   `Link.collision_enabled`.
5. A disconnected `link_6`, then upstream's `joint_5`, which gives `link_4` a
   second parent: still acyclic, no longer a tree.

The `ShortestPath` from `link_1` to `link_4` is taken while the graph is a tree.
Upstream prints it after adding `joint_5`, where both paths weigh 3.75 m (edge
weight = origin translation norm, graph.cpp:599) and the one returned is a tie-break.
"""

import sys

import numpy as np

from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix, Isometry3d
from tesseract_robotics.tesseract_scene_graph import (
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafety,
    JointType,
    Link,
    SceneGraph,
)

# createTestSceneGraph: each joint origin sits 1.25 m along one axis of its parent, m
JOINT_OFFSET = 1.25

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def make_joint(name, parent, child, joint_type, axis, limits=None):
    """A joint `JOINT_OFFSET` along `axis` (0 = x, 1 = y) of its parent, as createTestSceneGraph builds them."""
    joint = Joint(name)
    offset = np.zeros(3, dtype=np.float64)
    offset[axis] = JOINT_OFFSET
    origin = Isometry3d.Identity()
    origin.translate(offset)
    joint.parent_to_joint_origin_transform = origin
    joint.parent_link_name = parent
    joint.child_link_name = child
    joint.type = joint_type
    if limits is not None:
        joint.limits = limits
    return joint


def build_scene_graph():
    """Build upstream's five-link test graph and query it; return what the test asserts on."""
    # --8<-- [start:build]
    graph = SceneGraph()
    for i in range(1, 6):
        graph.addLink(Link(f"link_{i}"))
    # tesseract_scene_graph_unit.cpp:57-90, limits (lower, upper, effort, velocity, acceleration, jerk)
    graph.addJoint(make_joint("joint_1", "link_1", "link_2", JointType.FIXED, 0))
    graph.addJoint(
        make_joint(
            "joint_2", "link_2", "link_3", JointType.PLANAR, 0, JointLimits(-1, 1, 0, 2, 3, 6)
        )
    )
    graph.addJoint(make_joint("joint_3", "link_3", "link_4", JointType.FLOATING, 0))
    graph.addJoint(
        make_joint(
            "joint_4", "link_2", "link_5", JointType.REVOLUTE, 1, JointLimits(-1, 1, 0, 2, 3, 4)
        )
    )
    for first, second in [
        ("link_1", "link_2"),
        ("link_2", "link_3"),
        ("link_3", "link_5"),
        ("link_2", "link_5"),
    ]:
        graph.addAllowedCollision(first, second, "Adjacent")
    # --8<-- [end:build]

    print("\n=== Building a Scene Graph ===")
    # --8<-- [start:queries]
    adjacent = graph.getAdjacentLinkNames("link_3")
    inv_adjacent = graph.getInvAdjacentLinkNames("link_3")
    children = sorted(graph.getLinkChildrenNames("link_2"))
    joint_children = graph.getJointChildrenNames("joint_4")
    # each link of the subtrees below link_2 and link_3, mapped to the nearest of the two above it
    adjacency_map = graph.getAdjacencyMap(["link_2", "link_3"])
    acyclic, tree = graph.isAcyclic(), graph.isTree()
    # --8<-- [end:queries]
    print(f"Adjacency map of link_2, link_3: {dict(sorted(adjacency_map.items()))}")

    # --8<-- [start:strings]
    joint_types = {member.name: str(member) for member in JointType}
    shortest_path = str(graph.getShortestPath("link_1", "link_4"))
    joint_values = {
        "joint_2.limits": str(graph.getJoint("joint_2").limits),
        "joint_4.limits": str(graph.getJoint("joint_4").limits),
        # tesseract_scene_graph_joint_unit.cpp: the values each test sets
        "dynamics": str(JointDynamics(10, 5)),
        "safety": str(JointSafety(1, 2, 3, 4)),
        "calibration": str(JointCalibration(1, 2, 3)),
        "mimic": str(JointMimic(1, 2, "joint_name")),
    }
    # --8<-- [end:strings]
    print(f"Shortest path link_1 -> link_4:\n{shortest_path}")
    for name, text in joint_values.items():
        print(f"  {name}: {text}")

    # --8<-- [start:acm]
    initial_entries = len(graph.getAllowedCollisionMatrix().getAllAllowedCollisions())
    allowed_before = graph.isCollisionAllowed("link_1", "link_2")
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("link_3", "link_4", "Adjacent")
    graph.setAllowedCollisionMatrix(acm)
    allowed_after = graph.isCollisionAllowed("link_1", "link_2")
    entries_after = len(graph.getAllowedCollisionMatrix().getAllAllowedCollisions())
    # the graph keeps `acm` itself: an edit after the call is an edit to the graph's matrix
    acm.addAllowedCollision("link_1", "link_5", "Never")
    # --8<-- [end:acm]
    acm_result = {
        "initial_entries": initial_entries,
        "link_1_link_2_before": allowed_before,
        "link_1_link_2_after": allowed_after,
        "link_3_link_4_after": graph.isCollisionAllowed("link_3", "link_4"),
        "entries_after": entries_after,
        "later_edit_seen": graph.isCollisionAllowed("link_1", "link_5"),
    }
    print(f"Allowed collisions: {acm_result}")

    # --8<-- [start:link_flags]
    link_5 = graph.getLink("link_5")
    defaults = (link_5.visible, link_5.collision_enabled)
    graph.setLinkVisibility("link_5", False)
    graph.setLinkCollisionEnabled("link_5", False)
    link_6 = Link("link_6")
    link_6.visible = False
    link_6.collision_enabled = False
    graph.addLink(link_6)  # no joint: the graph is no longer a tree
    # --8<-- [end:link_flags]
    link_flags = {
        "defaults": defaults,
        "link_5_after_graph_set": (link_5.visible, link_5.collision_enabled),
        "link_6_from_link": (
            graph.getLinkVisibility("link_6"),
            graph.getLinkCollisionEnabled("link_6"),
        ),
    }
    print(f"Link flags (visible, collision_enabled): {link_flags}")

    tree_with_link_6 = graph.isTree()
    graph.removeLink("link_6")
    tree_after_remove = graph.isTree()
    # build_scene_graph_example.cpp: joint_5 gives link_4 a second parent
    graph.addJoint(make_joint("joint_5", "link_5", "link_4", JointType.CONTINUOUS, 1))
    graph_checks = {
        "acyclic": acyclic,
        "tree": tree,
        "tree_with_link_6": tree_with_link_6,
        "tree_after_remove": tree_after_remove,
        "acyclic_with_joint_5": graph.isAcyclic(),
        "tree_with_joint_5": graph.isTree(),
    }
    print(f"Graph checks: {graph_checks}")

    return {
        "joint_types": joint_types,
        "adjacent": adjacent,
        "inv_adjacent": inv_adjacent,
        "children": children,
        "joint_children": joint_children,
        "adjacency_map": adjacency_map,
        "graph_checks": graph_checks,
        "shortest_path": shortest_path,
        "joint_values": joint_values,
        "acm": acm_result,
        "link_flags": link_flags,
    }


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
        pos = transform.translation
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

    # === BUILDING A SCENE GRAPH (low-level) ===
    built = build_scene_graph()

    print("\n=== Scene Graph Example Complete ===")
    print("For link manipulation examples, see: lowlevel/scene_graph_c_api_example.py")

    return {
        "robot": robot,
        "success": True,
        "link_count": len(link_names),
        "joint_count": len(joint_names),
        "built": built,
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
