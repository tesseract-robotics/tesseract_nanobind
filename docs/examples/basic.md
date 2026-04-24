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

!!! tip "Quaternion convention"
    The Python API uses **scalar-last** quaternions `[x, y, z, w]` to match
    scipy. C++ Eigen uses scalar-first `[w, x, y, z]` — reorder when porting
    C++ examples.

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
    from tesseract_robotics.tesseract_geometry import Mesh

    # From file (STL, OBJ, DAE)
    mesh = Mesh.fromFile("/path/to/model.stl")

    # With scale
    mesh = Mesh.fromFile("/path/to/model.stl", scale=[0.001, 0.001, 0.001])  # mm to m
    ```

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
