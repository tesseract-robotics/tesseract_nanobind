# Python and C++

`tesseract_robotics` is a [nanobind](https://github.com/wjakob/nanobind) binding of the
Tesseract C++ libraries, plus a Python layer (`tesseract_robotics.planning`, `viewer`,
`emitters`) on top. The modules mirror the C++ libraries one to one, and the C++ names are kept
(`getLinkTransform`, `calcFwdKin`), so the upstream C++ documentation and examples apply. This
page lists the rules by which C++ types appear in Python; [Binding
conventions](../developer/binding-conventions.md) has the full rule set for names, errors and
lifetimes.

## Arrays and transforms

Eigen vectors and matrices arrive as NumPy `float64` arrays (`int32` for integer types). An Eigen
argument also accepts a list or a tuple of the exact shape, nested for a matrix; integers
convert to `float64`. A function that writes into its argument in place needs the NumPy array
itself.

`Eigen::Isometry3d` is the class `common.Isometry3d`; `planning.Pose` subclasses it, so a `Pose`
is accepted wherever an `Isometry3d` is.

```python
import numpy as np
from tesseract_robotics.common import Isometry3d

pose = Isometry3d.Identity()
pose.translate([0.5, 0.2, 0.3])  # m

position = pose.translation  # (3,) array, m
rotation = pose.rotation     # (3, 3) rotation matrix
matrix = pose.matrix         # (4, 4) homogeneous matrix

mat = np.eye(4)
mat[:3, 3] = [1.0, 2.0, 3.0]
assert np.allclose(Isometry3d(mat).translation, [1.0, 2.0, 3.0])
```

Quaternions are scalar-last at every Python surface: `[qx, qy, qz, qw]`. See
[Frames and poses](frames.md#quaternions).

## Containers

C++ containers (`std::vector`, `std::map`, `std::unordered_map`) convert from Python lists and
dicts on the way in. Some are bound as their own classes (`common.StringVector`, …) with the
list or dict protocol on the way out.

!!! warning "A vector member returns a copy"
    Reading a `std::vector` member returns a new Python list. Appending to it changes nothing in
    C++. Use the method the class provides:

    ```python
    from tesseract_robotics.scene_graph import Link, Visual

    link = Link("part")
    link.visual.append(Visual())  # appends to a copy
    assert len(link.visual) == 0
    link.addVisual(Visual())
    assert len(link.visual) == 1
    ```

## Ownership

- Objects passed by value are copied into Python.
- `std::shared_ptr` objects are reference-counted across the boundary and live while either
  side holds them.
- A `std::unique_ptr` return transfers ownership to Python; there is no `release()` to call.

## Polymorphic instructions and waypoints

The command language erases types: a program holds `MoveInstructionPoly` and
`CartesianWaypointPoly` wrappers rather than the concrete classes. The `*Poly` constructors
wrap; the `as*` methods unwrap. `MotionProgram` does both for you.

```python
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
)
from tesseract_robotics.planning import Pose

waypoint = CartesianWaypointPoly(CartesianWaypoint(Pose.from_xyz(0.8, 0.0, 1.0)))
program = CompositeInstruction("DEFAULT")
program.push_back(MoveInstructionPoly(MoveInstruction(waypoint, MoveInstructionType.LINEAR, "DEFAULT")))

move = program[0].asMoveInstruction()
assert move.getWaypoint().isCartesianWaypoint()
```

## Resource locators

A `GeneralResourceLocator` resolves `package://` URLs to files. It searches the `paths` it is
given (`addPath()` adds more), then the directories in the `TESSERACT_RESOURCE_PATH`,
`ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH` environment variables. Importing
`tesseract_robotics` sets no environment variable and adds no search path.

```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
assert locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf") is not None
```

`get_tesseract_resource_path()` is the directory of tesseract's example robots (the
`[support]` extra), `get_tesseract_support_path()` is its `support/` directory, and
`get_task_composer_config_path()` is the task composer configuration the wheel bundles.

## Plugins

Kinematics solvers, contact managers and task composer tasks are plugins: shared libraries
loaded at run time from the paths in a YAML configuration. The wheel ships the plugins and their
configurations, and `tesseract_robotics` points the loaders at them on import. Loading the
task composer plugins takes seconds; hold one `TaskComposer` per process (see
[Task composer](../user-guide/task-composer.md#plugin-warmup)).

## The GIL

Long-running C++ calls (planning, IK, collision checks, serialization) release the GIL, so
Python threads run while they do. [Threading and the GIL](../developer/threading.md) lists
which calls release it and what a Python callback costs.

## Logging

Tesseract logs through spdlog. The level of its default logger, `"tesseract"`:

```python
from tesseract_robotics.common import LogLevel, getLogger

getLogger().set_level(LogLevel.warn)
```

[Logging](../api/common.md#logging) shows how to route records to Python handlers.
