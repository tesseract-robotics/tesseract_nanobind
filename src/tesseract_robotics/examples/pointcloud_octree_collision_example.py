"""
Point Cloud → Octree Collision + Planning Example
=================================================

Builds a tesseract ``Octree`` from a point-cloud file, attaches it as a
collision obstacle in front of an ABB IRB2400, and demonstrates the difference
between a TrajOpt run with collision disabled (drives the arm through the
obstacle) and one with the default collision-aware profile (routes around it).

Pipeline
--------
1. Load ``docs/assets/sample.ply`` via
   ``tesseract_geometry.createMeshFromPath`` → an ``(N, 3)`` xyz array.
   The shipped PLY is already centred at the origin and pre-scaled into
   metres, so no per-run transform is needed.
2. Push points into a ``tesseract_geometry.PointCloud`` and call
   ``createOctree(...)`` to produce an ``octomap::OcTree``.
3. Wrap in a ``tesseract_geometry.Octree`` and attach as a fixed-joint link
   with both Collision and Visual components (the viewer renders each leaf
   as an instanced cube).
4. Run ``TrajOptPipeline`` twice on the same ``MotionProgram``: once with the
   collision cost/constraint disabled, once with the default profile. The two
   trajectories share start/goal/waypoint-count; only collision handling
   differs.
5. Animate the chosen trajectory in ``TesseractViewer``; press ``t``-Enter to
   toggle (robot tints red for the colliding plan, green for the collision-
   aware plan).

Notes
-----
- The cloud file ships in ``docs/assets/`` of the source repo. Installed
  wheels do not include it; pass ``--cloud PATH`` or set
  ``TESSERACT_NANOBIND_SAMPLE_CLOUD`` to point at any file Assimp can read
  (PLY / STL / OBJ / DAE — only the vertex positions are used).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

from tesseract_robotics.planning import (
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    create_obstacle,
    plan_freespace,
)
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_ALL,
)
from tesseract_robotics.tesseract_common import CollisionMarginData
from tesseract_robotics.tesseract_geometry import (
    Octree,
    OctreeSubType,
    PointCloud,
    createMeshFromPath,
    createOctree,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer


def _load_cloud_xyz(path: Path) -> np.ndarray:
    """Return an ``(N, 3)`` float64 array of xyz points from any Assimp-loadable file.

    Uses ``tesseract_geometry.createMeshFromPath`` so PLY / STL / OBJ / DAE
    all work — we only keep the vertex positions.
    """
    meshes = createMeshFromPath(str(path))
    if not meshes:
        raise ValueError(f"{path}: no meshes loaded")
    verts = meshes[0].getVertices()
    if len(verts) == 0:
        raise ValueError(f"{path}: mesh has no vertices")
    return np.asarray(verts, dtype=np.float64)


def _default_cloud_path() -> Path:
    """Locate the shipped sample.ply relative to the source repo."""
    env = os.environ.get("TESSERACT_NANOBIND_SAMPLE_CLOUD")
    if env:
        return Path(env)
    # src/tesseract_robotics/examples/this_file.py -> repo_root/docs/assets/sample.ply
    return Path(__file__).resolve().parents[3] / "docs" / "assets" / "sample.ply"


def build_octree_obstacle(points_m: np.ndarray, resolution: float) -> Octree:
    """Build a tesseract Octree geometry from an (N, 3) point array in metres."""
    pc = PointCloud()
    for x, y, z in points_m:
        pc.addPoint(float(x), float(y), float(z))
    ot = createOctree(pc, resolution, prune=True, binary=True)
    return Octree(ot, OctreeSubType.BOX, pruned=True, binary_octree=True)


# Place the cloud just inside the IRB24t00's outer reach, slightly off-axis,
# so the arm at full forward extension can sweep sideways through it.
PIECE_ORIGIN = (1.3, 0.3, 0.0)

# Start/goal: a base-rotation sweep aimed at the cloud's bearing
# (atan2(0.3, 1.3) ≈ 0.23 rad). Wrist sits at tool0 z ≈ 0.13 m — right at
# the top of the cloud (whose z-range after the 2× bake is roughly
# [-0.04, 0.13]). Both endpoints are clear; the joint-space straight line
# drags the forearm through the cloud's upper half. TrajOpt has room above
# to bend the arc up and over, which is what makes the avoidance visually
# obvious in the viewer.
START_JOINTS = np.array([+0.7, 1.45, -0.52, 0.0, 0.0, 0.0])
GOAL_JOINTS  = np.array([-0.2, 1.45, -0.52, 0.0, 0.0, 0.0])


def _count_contacts(robot, manager, joint_names, joint_pos):
    """Update robot state and return the number of contact pairs."""
    robot.set_joints(joint_pos, joint_names=joint_names)
    state = robot.env.getState()
    manager.setCollisionObjectsTransform(state.link_transforms)
    contacts = ContactResultMap()
    manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))
    results = ContactResultVector()
    contacts.flattenMoveResults(results)
    return len(results), results


def _count_traj_contacts(robot, manager, joint_names, planning_result) -> int:
    """Return the number of trajectory waypoints in contact with the env."""
    if not planning_result.successful:
        return 0
    n_collide = 0
    for pt in planning_result.trajectory:
        n, _ = _count_contacts(robot, manager, joint_names, np.asarray(pt.positions))
        if n > 0:
            n_collide += 1
    return n_collide


def _trajopt_profiles_without_collision():
    """Build a TrajOpt ProfileDictionary with collision cost + constraint disabled.

    Same composite/plan layout as ``create_trajopt_default_profiles`` but with
    ``collision_cost_config.enabled = False`` and
    ``collision_constraint_config.enabled = False``. The optimiser converges
    to a smoothed joint-space straight line — the "naive" path that drives
    the wrist through the cloud.
    """
    from tesseract_robotics.planning.profiles import (
        TRAJOPT_DEFAULT_NAMESPACE,
        TRAJOPT_PROFILE_NAMES,
        _create_trajopt_profiles,
    )
    from tesseract_robotics.tesseract_command_language import ProfileDictionary
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        ProfileDictionary_addTrajOptCompositeProfile,
        ProfileDictionary_addTrajOptPlanProfile,
    )
    from tesseract_robotics.tesseract_task_composer_planning import ContactCheckProfile

    composite, plan = _create_trajopt_profiles()
    composite.collision_cost_config.enabled = False
    composite.collision_constraint_config.enabled = False

    profiles = ProfileDictionary()
    for name in TRAJOPT_PROFILE_NAMES:
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, composite
        )
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, plan
        )

    # The TrajOptPipeline task graph runs a DiscreteContactCheckTask AFTER the
    # planner, and that task aborts the pipeline if any waypoint is in contact.
    # Register a permissive ContactCheckProfile (huge negative margin so no
    # realistic penetration registers) so the naive plan is allowed to finish.
    permissive = ContactCheckProfile()
    permissive.contact_manager_config.default_margin = -10.0
    for name in TRAJOPT_PROFILE_NAMES:
        profiles.addProfile("DiscreteContactCheckTask", name, permissive)

    return profiles


def run(cloud_path: Path | None = None, resolution: float = 0.04) -> dict:
    """Build the scene, demonstrate collision vs. TrajOpt-planned trajectory.

    Returns a dict with ``colliding_contacts``, ``aware_contacts``, ``result``
    (the ``PlanningResult``), and ``robot`` so callers (tests, viewer) can
    inspect outcomes.
    """
    cloud_path = cloud_path or _default_cloud_path()
    if not cloud_path.exists():
        raise FileNotFoundError(
            f"Cloud file not found: {cloud_path}\n"
            "Pass --cloud PATH, set TESSERACT_NANOBIND_SAMPLE_CLOUD, or run "
            "from a source checkout that has docs/assets/sample.ply."
        )

    points_m = _load_cloud_xyz(cloud_path)
    octree_geom = build_octree_obstacle(points_m, resolution=resolution)
    print(
        f"Loaded {points_m.shape[0]:,} points from {cloud_path.name}; "
        f"octree has {octree_geom.calcNumSubShapes()} sub-shapes at "
        f"resolution {resolution} m"
    )

    # Attach the octree as a fixed-joint obstacle on the robot's base.
    robot = Robot.from_tesseract_support("abb_irb2400")
    create_obstacle(
        robot,
        name="pcd_octree",
        geometry=octree_geom,
        transform=Pose.from_xyz(*PIECE_ORIGIN),
    )

    manager = robot.env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    # Zero margin: only report actual penetrations, not near-misses. This
    # makes the colliding-vs-collision-aware comparison binary — the
    # collision-aware trajectory should report 0 contacts.
    manager.setCollisionMarginData(CollisionMarginData(0.0))

    joint_names = robot.get_joint_names("manipulator")

    # Confirm both endpoints are clear before planning. If either endpoint
    # is in contact the comparison is meaningless — TrajOpt can't satisfy a
    # waypoint constraint that already collides, so the collision-aware plan
    # would just abort.
    endpoint_contacts = {}
    for label, q in [("start", START_JOINTS), ("goal", GOAL_JOINTS)]:
        n, _ = _count_contacts(robot, manager, joint_names, q)
        endpoint_contacts[label] = n
        print(f"[{label}] {n} contact pair(s)")
    assert endpoint_contacts == {"start": 0, "goal": 0}, (
        f"start/goal joint configs must be collision-free, got {endpoint_contacts}"
    )

    # Same MotionProgram for both runs: TrajOpt with the default (collision-
    # aware) profiles and TrajOpt with collision disabled, so the only
    # difference is whether collision pulls the trajectory away from the cloud.
    robot.set_joints(START_JOINTS, joint_names=joint_names)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(START_JOINTS))
        .move_to(JointTarget(GOAL_JOINTS))
    )
    # --8<-- [start:colliding]
    # "Colliding" plan: TrajOpt with both collision cost and collision constraint
    # disabled. With nothing pushing the trajectory away from obstacles, the
    # optimiser converges to a smooth joint-space line — straight through
    # the cloud.
    print("Running plan_freespace (collision disabled)...")
    colliding_result = plan_freespace(
        robot, program, profiles=_trajopt_profiles_without_collision()
    )
    colliding_contacts = _count_traj_contacts(robot, manager, joint_names, colliding_result)
    if colliding_result.successful:
        print(
            f"Collision-disabled trajectory: {colliding_contacts}/{len(colliding_result)} "
            "waypoints collide with the cloud"
        )
    else:
        print(f"Collision-disabled plan failed: {colliding_result.message}")
    # --8<-- [end:colliding]

    # --8<-- [start:aware]
    # Collision-aware plan: TrajOpt with default profiles — collision-cost
    # (soft, coeff=50) plus collision-constraint (hard, coeff=10) bend the
    # path up and over the cloud.
    print("Running plan_freespace (collision enabled)...")
    result = plan_freespace(robot, program, profiles=create_trajopt_default_profiles())
    aware_contacts = _count_traj_contacts(robot, manager, joint_names, result)
    if result.successful:
        print(
            f"Collision-aware trajectory: {aware_contacts}/{len(result)} waypoints collide"
        )
    else:
        print(f"plan_freespace failed: {result.message}")
    # --8<-- [end:aware]

    # Demo invariants — these are the whole point of the example. If they
    # ever fail (e.g. solver tuning drifts, or the cloud gets repositioned)
    # the printed numbers above stop matching the user-guide narrative.
    assert colliding_result.successful, (
        f"collision-disabled plan failed: {colliding_result.message}"
    )
    assert result.successful, f"collision-aware plan failed: {result.message}"
    assert colliding_contacts > 0, (
        "collision-disabled plan should drive through the cloud, but "
        f"reported {colliding_contacts}/{len(colliding_result)} waypoints in contact"
    )
    assert aware_contacts == 0, (
        "collision-aware plan should clear the cloud, but "
        f"reported {aware_contacts}/{len(result)} waypoints in contact"
    )

    return {
        "colliding_contacts": colliding_contacts,
        "aware_contacts": aware_contacts,
        "result": result,
        "colliding_result": colliding_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main() -> int:
    import argparse

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cloud", type=Path, default=None,
                   help="path to a point-cloud file Assimp can read — PLY / STL "
                        "/ OBJ / DAE (default: docs/assets/sample.ply)")
    p.add_argument("--resolution", type=float, default=0.02,
                   help="octree leaf resolution in metres (default: 0.02)")
    args = p.parse_args()

    results = run(args.cloud, resolution=args.resolution)
    result = results["result"]
    robot = results["robot"]

    if TesseractViewer is not None and "pytest" not in sys.modules:
        viewer = TesseractViewer()

        # Tint every robot link visual so the colour reflects which path
        # the viewer is currently animating. The octree link has no
        # Visual.material set, so it stays at the gltf-default grey.
        red   = np.array([0.85, 0.15, 0.15, 1.0])
        green = np.array([0.15, 0.75, 0.25, 1.0])

        colliding_result = results["colliding_result"]
        have_aware = result.successful and result.raw_results is not None
        have_colliding = (
            colliding_result.successful and colliding_result.raw_results is not None
        )

        # The viewer prefers Visual.material.color over the mesh's intrinsic
        # material when both are set. Mutating link.visual[i].material.color
        # and re-calling update_environment is enough to recolor a URDF-loaded
        # robot live — that's how we flip the IRB2400 between red and green
        # without rebuilding anything.
        def _tint_robot(rgba: np.ndarray) -> None:
            for name in robot.env.getLinkNames():
                if name == "pcd_octree":
                    continue
                for v in robot.env.getLink(name).visual:
                    if v.material is not None:
                        v.material.color = rgba

        def render(show_colliding: bool) -> str:
            # Trajectory is bundled into the same update_environment call
            # rather than firing update_environment + update_trajectory
            # separately. The two separate pushes race on the browser side:
            # refresh_scene resets the joints to rest, and the trajectory
            # message that arrives milliseconds later doesn't restart the
            # animation cleanly.
            if show_colliding:
                _tint_robot(red)
                traj = colliding_result.raw_results if have_colliding else None
                viewer.update_environment(robot.env, [0, 0, 0], trajectory=traj)
                return "colliding (red)"
            _tint_robot(green)
            viewer.update_environment(robot.env, [0, 0, 0], trajectory=result.raw_results)
            return "collision-aware (green)"

        # Start on the colliding variant so toggling visibly turns collision
        # avoidance on. Fall through to the collision-aware view if the
        # colliding plan failed (e.g. ContactCheckTask aborted).
        show_colliding = have_colliding
        showing = render(show_colliding)
        viewer.start_serve_background()
        print(f"Viewer at http://localhost:8000 — showing {showing}")
        if have_aware and have_colliding:
            print(
                "Press 't' + Enter to toggle colliding / collision-aware; "
                "Enter alone to exit."
            )
            while True:
                cmd = input("> ").strip().lower()
                if not cmd:
                    break
                if cmd in ("t", "toggle"):
                    show_colliding = not show_colliding
                    showing = render(show_colliding)
                    print(f"Now showing: {showing}")
                else:
                    print("Unknown command; 't'+Enter to toggle, Enter to exit.")
        else:
            input("Press Enter to exit...")

    return 0 if result.successful else 1


if __name__ == "__main__":
    sys.exit(main())
