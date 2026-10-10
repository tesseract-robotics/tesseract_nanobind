"""
State Solver Example
====================

Turns the IIWA's first joint into a floating joint, drives the state solver
through every native `setState` / `getState` / `getJacobian` /
`getLinkTransforms` overload, and attaches a sub-graph with `insertSceneGraph`.
After every change a `KDLStateSolver` rebuilt from the same scene graph is the
oracle.

C++ Reference:
    state_solver/test/state_solver_test_suite.h  (runSetFloatingJointStateTest :766-820,
        runAddSceneGraphTest :1120-1215, getSubSceneGraph :28-57, runCompareStateSolver)

Overview
--------
1. `OFKTStateSolver.replaceJoint` makes `joint_a1` FLOATING at x = 1.25, then the
   floating value moves to y = 1.5 and z = 1.5, as upstream.
2. Every native overload of `setState`, `getState`, `getJacobian` and
   `getLinkTransforms`, each with `floating_joint_values` bound.
3. `insertSceneGraph` attaches upstream's two-link sub-graph through a FIXED
   joint, then again with the prefix `prefix_` through a FLOATING joint.

Key Concepts
------------
**Floating joints**:
    A FLOATING joint has no joint value; its pose relative to the parent is set
    directly as an `Isometry3d` through `floating_joint_values`. It is not an
    active joint, so it adds no Jacobian column.

**The KDL oracle**:
    KDL treats a FLOATING joint as fixed at its origin
    (scene_graph/src/kdl_parser.cpp:133-160) and ignores floating joint values.
    Moving the joint origin in the scene graph with `changeJointOrigin` and
    rebuilding a `KDLStateSolver` therefore gives an independent computation of
    the same link transforms. PLANAR joints are unsupported by KDL, so this
    oracle is valid for FLOATING only.

**Read-back gap**:
    Upstream's third step reads `state.floating_joints` from a `SceneState`,
    edits it and passes it back. `SceneState.floating_joints` is not bound in
    Python, so this example builds the `floating_joint_values` dict itself.
"""

from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    SceneGraph,
    Visual,
)
from tesseract_robotics.tesseract_state_solver import KDLStateSolver, OFKTStateSolver, StateSolver
from tesseract_robotics.tesseract_urdf import parseURDFFile

URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
FLOATING_JOINT = "joint_a1"
TIP_LINK = "tool0"
# Upstream jvals (kinematics_test_utils.h:598-604) for the joints that stay
# active, joint_a2..joint_a7, rad
JOINT_VALUES = np.array([0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
# Floating-joint origins, runSetFloatingJointStateTest (state_solver_test_suite.h:774, 797, 812), m
ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1.25, 1.5, 1.5
# getSubSceneGraph fixed-joint offset (state_solver_test_suite.h:47), m
SUBGRAPH_JOINT_X = 1.25
PREFIX = "prefix_"
# runCompareStateSolver / runCompareSceneStates isApprox precision
# (state_solver_test_suite.h:73, 83, 165-194)
STATE_PRECISION = 1e-6


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def translation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Isometry3d:
    """Pure translation."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def is_approx(a: Isometry3d, b: Isometry3d) -> bool:
    """Eigen's `a.isApprox(b, STATE_PRECISION)` on the 4x4 matrices."""
    diff = np.linalg.norm(a.matrix - b.matrix)
    return bool(diff <= STATE_PRECISION * min(np.linalg.norm(a.matrix), np.linalg.norm(b.matrix)))


def agree(transforms: dict[str, Isometry3d], oracle: dict[str, Isometry3d]) -> bool:
    """Every oracle link transform is matched, as upstream's runCompareStateSolver."""
    return all(is_approx(transforms[link], tf) for link, tf in oracle.items())


def kdl_oracle(
    scene_graph: SceneGraph, joint_names: list[str], values: np.ndarray
) -> dict[str, Isometry3d]:
    """Link transforms of a KDLStateSolver rebuilt from `scene_graph`."""
    return KDLStateSolver(scene_graph).getState(joint_names, values).link_transforms


def sub_scene_graph() -> SceneGraph:
    """Upstream getSubSceneGraph: a unit box link and a link 1.25 m along x, FIXED."""
    subgraph = SceneGraph()
    subgraph.setName("subgraph")
    visual, collision = Visual(), Collision()
    visual.geometry = Box(1, 1, 1)
    collision.geometry = Box(1, 1, 1)
    base = Link("subgraph_base_link")
    base.addVisual(visual)
    base.addCollision(collision)
    joint = Joint("subgraph_joint1")
    joint.parent_to_joint_origin_transform = translation(SUBGRAPH_JOINT_X)
    joint.parent_link_name = "subgraph_base_link"
    joint.child_link_name = "subgraph_link_1"
    joint.type = JointType.FIXED
    subgraph.addLink(base)
    subgraph.addLink(Link("subgraph_link_1"))
    subgraph.addJoint(joint)
    return subgraph


def check_step(solver: StateSolver, scene_graph: SceneGraph, origin: Isometry3d) -> dict[str, Any]:
    """Compare the solver with a KDL solver rebuilt from `scene_graph`, with `origin` moved in."""
    names = solver.getActiveJointNames()
    # No floating_joint_values: the solver uses the floating value it holds
    state = solver.getState(names, JOINT_VALUES)
    base_link = state.link_transforms["base_link"]
    return {
        "origin": origin.translation.tolist(),
        "agrees_with_kdl": agree(
            state.link_transforms, kdl_oracle(scene_graph, names, JOINT_VALUES)
        ),
        # link_1 = base_link ∘ origin: a FLOATING joint has no motion of its own
        "link_1_at_origin": is_approx(state.link_transforms["link_1"], base_link * origin),
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator()
    scene_graph = parseURDFFile(str(locate(locator, URDF_URL)), locator)
    solver = OFKTStateSolver(scene_graph)

    # --8<-- [start:floating_joint]
    origin = translation(ORIGIN_X)
    joint = Joint(FLOATING_JOINT)
    joint.parent_to_joint_origin_transform = origin
    joint.parent_link_name = "base_link"
    joint.child_link_name = "link_1"
    joint.type = JointType.FLOATING

    edits = [
        scene_graph.removeJoint(FLOATING_JOINT),
        scene_graph.addJoint(joint),
        solver.replaceJoint(joint),
    ]
    # --8<-- [end:floating_joint]
    floating_joint_names = solver.getFloatingJointNames()
    steps = [check_step(solver, scene_graph, origin)]

    # --8<-- [start:move_floating]
    # Floating value only: the active joints keep their values
    origin = translation(ORIGIN_X, ORIGIN_Y)
    solver.setState({FLOATING_JOINT: origin})
    edits.append(scene_graph.changeJointOrigin(FLOATING_JOINT, origin))
    steps.append(check_step(solver, scene_graph, origin))

    # Joints and floating values together
    origin = translation(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    solver.setState(solver.getState().joints, {FLOATING_JOINT: origin})
    edits.append(scene_graph.changeJointOrigin(FLOATING_JOINT, origin))
    steps.append(check_step(solver, scene_graph, origin))
    # --8<-- [end:move_floating]

    names = solver.getActiveJointNames()
    floating = {FLOATING_JOINT: origin}
    joints = dict(zip(names, JOINT_VALUES))
    oracle = kdl_oracle(scene_graph, names, JOINT_VALUES)

    # --8<-- [start:overloads]
    # Each setState starts from another state (zero joints, the step-1 floating
    # origin), so a call that changed nothing would disagree with the oracle
    other_joints, other_floating = np.zeros(len(names)), {FLOATING_JOINT: translation(ORIGIN_X)}
    overloads = {}
    solver.setState(other_joints, other_floating)
    solver.setState(JOINT_VALUES, floating)
    overloads["setState(values, floating)"] = agree(solver.getState().link_transforms, oracle)
    solver.setState(other_joints, other_floating)
    solver.setState(joints, floating)
    overloads["setState(dict, floating)"] = agree(solver.getState().link_transforms, oracle)
    solver.setState(other_joints, other_floating)
    solver.setState(names, JOINT_VALUES, floating)
    overloads["setState(names, values, floating)"] = agree(
        solver.getState().link_transforms, oracle
    )
    solver.setState(JOINT_VALUES, other_floating)
    solver.setState(floating)  # floating values only: the joints keep JOINT_VALUES
    overloads["setState(floating)"] = agree(solver.getState().link_transforms, oracle)

    # getState computes a state without changing the solver's; it holds other_floating
    solver.setState(JOINT_VALUES, other_floating)
    overloads["getState(values, floating)"] = agree(
        solver.getState(JOINT_VALUES, floating).link_transforms, oracle
    )
    overloads["getState(dict, floating)"] = agree(
        solver.getState(joints, floating).link_transforms, oracle
    )
    overloads["getState(names, values, floating)"] = agree(
        solver.getState(names, JOINT_VALUES, floating).link_transforms, oracle
    )
    overloads["getState(floating)"] = agree(solver.getState(floating).link_transforms, oracle)
    overloads["getLinkTransforms(names, values, floating)"] = agree(
        solver.getLinkTransforms(names, JOINT_VALUES, floating), oracle
    )

    # getLinkTransforms() lists the current state in getLinkNames() order
    solver.setState(JOINT_VALUES, floating)
    overloads["getLinkTransforms()"] = agree(
        dict(zip(solver.getLinkNames(), solver.getLinkTransforms())), oracle
    )
    # --8<-- [end:overloads]

    # --8<-- [start:jacobian]
    jacobians = [
        solver.getJacobian(JOINT_VALUES, TIP_LINK, floating),
        solver.getJacobian(joints, TIP_LINK, floating),
        solver.getJacobian(names, JOINT_VALUES, TIP_LINK, floating),
    ]
    kdl_jacobian = KDLStateSolver(scene_graph).getJacobian(names, JOINT_VALUES, TIP_LINK)
    # --8<-- [end:jacobian]
    jacobian = {
        "overloads_identical": all(np.array_equal(jacobians[0], j) for j in jacobians[1:]),
        "agrees_with_kdl": bool(
            np.linalg.norm(jacobians[0] - kdl_jacobian)
            <= STATE_PRECISION * np.linalg.norm(kdl_jacobian)
        ),
    }

    # --8<-- [start:insert_scene_graph]
    subgraph = sub_scene_graph()
    root = scene_graph.getRoot()

    attach = Joint("attach_subgraph_joint")
    attach.parent_link_name = root
    attach.child_link_name = subgraph.getRoot()
    attach.type = JointType.FIXED

    prefix_attach = Joint(PREFIX + "attach_subgraph_joint")
    prefix_attach.parent_link_name = root
    prefix_attach.child_link_name = PREFIX + subgraph.getRoot()
    prefix_attach.type = JointType.FLOATING

    inserted = {}
    for attach_joint, prefix in [(attach, ""), (prefix_attach, PREFIX)]:
        edits.append(scene_graph.insertSceneGraph(subgraph, attach_joint, prefix))
        inserted[attach_joint.getName()] = solver.insertSceneGraph(subgraph, attach_joint, prefix)
    # --8<-- [end:insert_scene_graph]

    state = solver.getState()
    transforms = state.link_transforms
    prefixed_root, prefixed_child = PREFIX + "subgraph_base_link", PREFIX + "subgraph_link_1"
    moved = translation(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    moved_root = solver.getState(
        {FLOATING_JOINT: origin, prefix_attach.getName(): moved}
    ).link_transforms[prefixed_root]
    insert = {
        "inserted": inserted,
        "prefixed_links": sorted(name for name in solver.getLinkNames() if name.startswith(PREFIX)),
        "floating_joint_names": sorted(solver.getFloatingJointNames()),
        "agrees_with_kdl": agree(transforms, kdl_oracle(scene_graph, names, JOINT_VALUES)),
        # the attach joints have the identity origin: inserted root = parent ∘ origin = parent
        "root_is_parent_times_origin": is_approx(
            transforms[prefixed_root],
            transforms[root] * prefix_attach.parent_to_joint_origin_transform,
        ),
        "child_is_root_times_fixed_origin": is_approx(
            transforms[prefixed_child], transforms[prefixed_root] * translation(SUBGRAPH_JOINT_X)
        ),
        # a floating value replaces the attach joint's origin
        "floating_value_moves_root": is_approx(moved_root, transforms[root] * moved),
    }

    return {
        "floating_joint_names": floating_joint_names,
        "scene_graph_edits_succeeded": all(edits),
        "active_joint_names": names,
        "floating_steps": steps,
        "state_overloads": overloads,
        "jacobian": jacobian,
        "insert_scene_graph": insert,
    }


def main() -> None:
    """Console entry point (`tesseract_state_solver_example`): run() and print the results."""
    result = run()
    print(
        f"Floating joints {result['floating_joint_names']}; active joints {result['active_joint_names']}"
    )
    for step in result["floating_steps"]:
        print(
            f"joint_a1 origin {step['origin']}: agrees with KDL {step['agrees_with_kdl']}, "
            f"link_1 = base_link ∘ origin {step['link_1_at_origin']}"
        )
    for name, ok in result["state_overloads"].items():
        print(f"{name}: agrees with KDL {ok}")
    print(f"getJacobian: three overloads identical, agrees with KDL: {result['jacobian']}")
    insert = result["insert_scene_graph"]
    print(f"insertSceneGraph: {insert['inserted']}; prefixed links {insert['prefixed_links']}")
    print(
        f"  agrees with KDL {insert['agrees_with_kdl']}; inserted root = parent ∘ origin "
        f"{insert['root_is_parent_times_origin']}; child = root ∘ 1.25 m x {insert['child_is_root_times_fixed_origin']}; "
        f"floating value moves the prefixed root {insert['floating_value_moves_root']}"
    )


if __name__ == "__main__":
    main()
