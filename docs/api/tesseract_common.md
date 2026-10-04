# tesseract_robotics.tesseract_common

Common types, utilities, and resource handling.

## Transforms

### Isometry3d

Eigen rigid-body transform (rotation + translation). Raw C++ binding.

```python
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Identity transform
pose = Isometry3d.Identity()

# From a 4x4 matrix
mat = np.eye(4)
mat[:3, 3] = [0.5, 0.2, 0.3]
pose = Isometry3d(mat)

# Access components (translation() / rotation() / matrix() are METHODS)
position = pose.translation()  # np.array([x, y, z])
rotation = pose.rotation()     # 3x3 rotation matrix
matrix = pose.matrix()         # 4x4 homogeneous matrix

# Compose
combined = pose1 * pose2
```

!!! tip "Prefer `planning.Pose` for authoring"
    For building/serializing transforms in Python, use
    `tesseract_robotics.planning.Pose` — it accepts scalar-last quaternions
    (`qx, qy, qz, qw`) and offers `from_xyz_quat`, `from_rpy`, etc. Convert
    to an `Isometry3d` when calling the raw C++ API.

    ```python
    from tesseract_robotics.planning import Pose
    pose = Pose.from_xyz_quat(0.5, 0.2, 0.3, 0, 0, 0, 1)
    iso = Isometry3d(pose.matrix)
    ```

### Quaterniond

Eigen quaternion. **Project canonical order is scalar-last
`[qx, qy, qz, qw]`** — use `Quaterniond.from_xyzw(qx, qy, qz, qw)` to build
and access components via the `q.x`, `q.y`, `q.z`, `q.w` properties.

```python
from tesseract_robotics.tesseract_common import Quaterniond

q = Quaterniond.from_xyzw(0.0, 0.0, 0.0, 1.0)  # identity: scalar-last

# Component access — properties, not method calls.
print(f"x={q.x}, y={q.y}, z={q.z}, w={q.w}")

# Flat coefficient vector is also scalar-last (Eigen storage order).
qx, qy, qz, qw = q.coeffs()

rotation_matrix = q.toRotationMatrix()
```

!!! warning "Eigen's 4-double ctor is scalar-first — avoid in Python"
    `Quaterniond(w, x, y, z)` exists for direct Eigen interop but is the
    one signature in the binding that breaks the project convention. Use
    `Quaterniond.from_xyzw(qx, qy, qz, qw)` everywhere else.

    `q.coeffs()`, `q.vec()`, the `Quaterniond(Vector4d)` ctor, and `Pose`'s
    quaternion-returning factories all use scalar-last `[qx, qy, qz, qw]`.

### AngleAxisd

Axis-angle rotation representation.

```python
from tesseract_robotics.tesseract_common import AngleAxisd
import numpy as np

# 90 degrees around Z axis
aa = AngleAxisd(np.pi/2, np.array([0, 0, 1]))
rotation_matrix = aa.toRotationMatrix()
```

## Ids

Links and joints are addressed by id (upstream `IDENTITY_DESIGN.md`). An id is
built from a name; anywhere an id is expected a `str` converts implicitly.

```python
from tesseract_robotics.tesseract_common import JointId, LinkId, LinkIdPair

tool0 = LinkId("tool0")
tool0.name()                 # "tool0"; str(tool0) is the same
tool0 == "tool0"             # True: ids compare equal to their name
{tool0: 1}["tool0"]          # 1: hash(id) == hash(name), so str keys find id-keyed entries
LinkIdPair("a", "b").orderedNameView()   # ("a", "b"), alphabetical
```

Id-keyed results (`SceneState.link_transforms`, `calcFwdKin`) are plain
`dict[LinkId, ...]`. `LinkId` and `JointId` do not convert into each other.

!!! note "Type checkers"
    The stubs accept `LinkId | str` wherever a parameter takes an id. Looking up an
    id-keyed dict with a literal `str` (`transforms["tool0"]`) works at runtime but
    is flagged by type checkers; use `transforms[LinkId("tool0")]` to keep them quiet.

## Resource Locators

### GeneralResourceLocator

Resolves `package://` URLs to file paths.

```python
from tesseract_robotics.tesseract_common import GeneralResourceLocator

locator = GeneralResourceLocator()

# Resolve package URL
resource = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")
path = resource.getFilePath()
```

### BytesResource

In-memory resource from bytes.

```python
from tesseract_robotics.tesseract_common import BytesResource

data = b"<robot name='test'></robot>"
resource = BytesResource("robot.urdf", data)
```

## Collision

### AllowedCollisionMatrix

Defines which link pairs to skip during collision checking.

```python
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix

acm = AllowedCollisionMatrix()

# Allow collision between links
acm.addAllowedCollision("link_1", "link_2", "Adjacent links")

# Check if allowed
is_allowed = acm.isCollisionAllowed(("link_1", "link_2"))

# Remove entry
acm.removeAllowedCollision("link_1", "link_2")

# Clear all
acm.clearAllowedCollisions()
```

### CollisionMarginData

Configure collision margins per link pair.

```python
from tesseract_robotics.tesseract_common import CollisionMarginData

margins = CollisionMarginData()
margins.setDefaultCollisionMargin(0.025)
margins.setPairCollisionMargin("link_a", "link_b", 0.05)

default = margins.getDefaultCollisionMargin()
pair_margin = margins.getPairCollisionMargin("link_a", "link_b")
```

## State

### JointState

Joint positions and velocities.

```python
from tesseract_robotics.tesseract_common import JointState

state = JointState()
state.joint_ids = ["j1", "j2", "j3"]
state.position = np.array([0.0, 0.5, -0.5])
state.velocity = np.array([0.0, 0.0, 0.0])
```

### KinematicLimits

Joint position, velocity, acceleration limits.

```python
from tesseract_robotics.tesseract_common import KinematicLimits

limits = kin_group.getLimits()
print(f"Position min: {limits.joint_limits.col(0)}")
print(f"Position max: {limits.joint_limits.col(1)}")
print(f"Velocity: {limits.velocity_limits}")
print(f"Acceleration: {limits.acceleration_limits}")
```

## Manipulator Info

### ManipulatorInfo

Describes a kinematic group configuration.

```python
from tesseract_robotics.tesseract_common import ManipulatorInfo

info = ManipulatorInfo()
info.manipulator = "manipulator"        # group name
info.working_frame = "base_link"        # reference frame
info.tcp_frame = "tool0"                # tool center point
info.tcp_offset = Isometry3d.Identity() # optional TCP offset
```

## Logging

Tesseract logs through spdlog (upstream #1367). The default logger is named
`"tesseract"`; its level controls tesseract's console output.

```python
from tesseract_robotics.tesseract_common import (
    LoggerLevel, addLogRecordHandler, getLogger, isLogLevelEnabled, removeLogRecordHandler,
)

logger = getLogger()                 # getLogger("tesseract")
logger.set_level(LoggerLevel.err)    # suppress warnings
logger.set_level(LoggerLevel.debug)  # enable debug output
assert isLogLevelEnabled(LoggerLevel.debug)

# Route records into Python (any thread; the handler runs with the GIL held)
records = []
handler_id = addLogRecordHandler(records.append)
...
removeLogRecordHandler(handler_id)
```

A `LogRecord` carries `level`, `message`, `logger_name`, `component_name`,
`attributes`, `timestamp`, `filename`, `line` and `function_name`. An exception
raised by a handler is reported through `sys.unraisablehook`.

!!! warning "console_bridge API no longer reaches tesseract"
    `setLogLevel`, `getLogLevel`, `useOutputHandler`, `OutputHandler`, `log` and
    the `CONSOLE_BRIDGE_LOG_*` constants are still importable, but upstream
    tesseract no longer logs through console_bridge: they have no effect on its
    output.

## Container Types

| Type | Description |
|------|-------------|
| `TransformMap` | `dict[str, Isometry3d]` - link name to transform |
| `VectorIsometry3d` | `list[Isometry3d]` |
| `VectorVector3d` | `list[np.ndarray]` - list of 3D points |

## Auto-generated API Reference

::: tesseract_robotics.tesseract_common._tesseract_common
    options:
      show_root_heading: false
      show_source: false
      members_order: source
