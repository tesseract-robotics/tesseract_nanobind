# Basic Examples

Fundamental examples demonstrating kinematics, collision detection, and scene graph operations.

All examples install as console scripts. Run them with:

```bash
# Installed console script (preferred)
tesseract_kinematics_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.tesseract_kinematics_example
```

## Kinematics Example

Forward and inverse kinematics with the KUKA LBR IIWA 14 R820 (7-DOF). The snippet
below is the core of the example — loaded directly from the shipped source.

```python title="tesseract_kinematics_example.py"
--8<-- "src/tesseract_robotics/examples/tesseract_kinematics_example.py:fk_ik"
```

Run it:

```bash
tesseract_kinematics_example
```

!!! tip "Quaternion convention — scalar-last everywhere"
    Project canonical order is **scalar-last** `[qx, qy, qz, qw]`. Build
    with `Quaterniond.from_xyzw(qx, qy, qz, qw)`; access components via
    `q.x`, `q.y`, `q.z`, `q.w` properties. The Eigen-style
    `Quaterniond(w, x, y, z)` ctor exists for direct interop but is not the
    preferred call shape.

## Collision Detection Example

Check for collisions in the environment:

```bash
tesseract_collision_example
```

See the source at
[`tesseract_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_collision_example.py).

## Scene Graph Example

Manipulate the scene graph (add/remove objects):

```bash
tesseract_scene_graph_example
```

See the source at
[`scene_graph_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/scene_graph_example.py).

### Building a scene graph

The example's second part builds a `SceneGraph` directly, without an
`Environment`: upstream's five-link test graph (`tesseract_scene_graph_unit.cpp:40–93`,
`build_scene_graph_example.cpp`), with one joint of each kind and `JointLimits`
on the planar and revolute joints.

```python title="scene_graph_example.py"
--8<-- "src/tesseract_robotics/examples/scene_graph_example.py:build"
```

`getAdjacencyMap(link_names)` maps every link at or below the given links to the
nearest of them above it: `link_4` to `link_3`, `link_5` to `link_2`.

```python title="scene_graph_example.py"
--8<-- "src/tesseract_robotics/examples/scene_graph_example.py:queries"
```

`str()` prints what upstream's `operator<<` prints: `JointType.REVOLUTE` is
`Revolute`, a `JointLimits` is `lower=-1 upper=1 effort=0 velocity=2
acceleration=3 jerk=4`, and a `ShortestPath` lists its links, joints and active
joints. The `JointType_*` module constants are gone; use the enum
([breaking changes](../breaking-changes.md#swig-era-jointtype_-constants-removed)).

```python title="scene_graph_example.py"
--8<-- "src/tesseract_robotics/examples/scene_graph_example.py:strings"
```

`setAllowedCollisionMatrix` replaces the graph's allowed collision matrix:
after it, `link_1`/`link_2` is no longer allowed. The graph keeps the matrix
object itself (`graph.cpp:746–749`), so an edit to `acm` after the call shows up
in `isCollisionAllowed`.

```python title="scene_graph_example.py"
--8<-- "src/tesseract_robotics/examples/scene_graph_example.py:acm"
```

`Link.visible` and `Link.collision_enabled` are the flags that
`SceneGraph.setLinkVisibility` / `setLinkCollisionEnabled` set and
`getLinkVisibility` / `getLinkCollisionEnabled` read (`graph.cpp:417–432`).
They default to `True`; set on a `Link` before `addLink`, the graph reads them back.

```python title="scene_graph_example.py"
--8<-- "src/tesseract_robotics/examples/scene_graph_example.py:link_flags"
```

!!! note "Shortest path before the cycle"
    Upstream's example prints `getShortestPath("link_1", "link_4")` after
    `joint_5` gives `link_4` a second parent. Edges weigh the length of their
    joint origin (`graph.cpp:599`), and both paths then weigh 3.75 m, so which one
    is returned is a tie-break. The example takes the path while the graph is a
    tree, where it is unique.

## Geometry Showcase

All available geometry types (Box, Sphere, Cylinder, Capsule, Cone, Mesh,
ConvexMesh, Plane, Octree):

```bash
tesseract_geometry_showcase_example
```

See the source at
[`geometry_showcase_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/geometry_showcase_example.py).

??? tip "Loading Mesh Files"
    ```python
    import numpy as np

    from tesseract_robotics.tesseract_geometry import createMeshFromPath

    # From file (STL, OBJ, DAE, PLY, glTF): one Mesh per mesh in the file
    meshes = createMeshFromPath("/path/to/model.stl")

    # With scale
    meshes = createMeshFromPath("/path/to/model.stl", np.array([0.001, 0.001, 0.001]))  # mm to m
    ```

    Loading from bytes, normals and textures, and writing PLY: [Geometry Examples](geometry.md).

## Point cloud → Octree Example

Loads a binary PLY point cloud, builds a `tesseract_geometry.Octree`, attaches
it as a collision obstacle in front of an ABB IRB2400, and then runs **the
same `MotionProgram` through TrajOpt twice** — once with collision disabled
(robot tints red, drags through the cloud) and once with the default
collision-aware profile (robot tints green, routes around). 

```bash
tesseract_pointcloud_octree_collision_example
# custom cloud / leaf size
tesseract_pointcloud_octree_collision_example --cloud path/to/scan.ply --resolution 0.01
```

!!! tip "Toggle in the viewer"
    The viewer starts on the colliding trajectory; press **`t` + Enter** in the terminal to swap between the colliding (red) and collision-aware (green) trajectories. Press **Enter** alone to exit.

Typical output:

```text
[start] 0 contact pair(s)
[goal]  0 contact pair(s)
Running plan_freespace (collision disabled)...
Collision-disabled trajectory: 4/11 waypoints collide with the cloud
Running plan_freespace (collision enabled)...
Collision-aware trajectory: 0/11 waypoints collide
```

Source:
[`pointcloud_octree_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pointcloud_octree_collision_example.py).
For the generic "octree as a collision obstacle" recipe, see the user-guide
section [Point Clouds as Collision Obstacles](../user-guide/collision.md#point-clouds-as-collision-obstacles).

**How the comparison is built**

`run()` constructs a single two-waypoint `MotionProgram` and feeds it to
`plan_freespace(...)` twice with different `ProfileDictionary` objects —
the default `create_trajopt_default_profiles()` for the collision-aware
run, and a copy with `collision_cost_config.enabled` /
`collision_constraint_config.enabled` flipped off for the colliding run.

Same start/goal/waypoint count, same robot, same env; only the profile
differs, which is what makes the result fair to eyeball.

!!! note "Why a permissive `ContactCheckProfile`?"
    `TrajOptPipeline` runs a `DiscreteContactCheckTask` *after* the
    optimizer; that task aborts the whole pipeline if any waypoint is in
    contact. Disabling collision in the TrajOpt profile only changes what
    TrajOpt optimises for — the post-check still fires. The example
    sidesteps that by registering a `ContactCheckProfile` under namespace
    `"DiscreteContactCheckTask"` with `default_margin = -10.0`, so only
    penetrations deeper than 10 m would register and the colliding plan is
    allowed to finish and return a real `CompositeInstruction`.

## Reeds-Shepp Example

Reeds-Shepp path for a differential-drive vehicle — a 2D motion planning
demonstration using OMPL:

```bash
tesseract_reeds_shepp_example
```

See the source at
[`reeds_shepp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/reeds_shepp_example.py).

## Next Steps

- [Planning Examples](planning.md) - Motion planning
- [Online Planning](online-planning.md) - Real-time replanning
