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

A package is a directory that holds a `package.xml`; its name is the directory name. The
locator finds packages in the directories it is given and in their subdirectories. Without
arguments it reads `TESSERACT_RESOURCE_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`.
Both other constructors take keyword arguments only, because a positional `list[str]` could
mean either directories or environment variable names:

```python
from pathlib import Path

locator = GeneralResourceLocator(paths=[Path("~/ws/src").expanduser()])  # + the default env vars
locator = GeneralResourceLocator(environment_variables=["MY_RESOURCE_PATH"])

locator.addPath(Path("/opt/robots"))         # False if the directory does not exist
locator.loadEnvironmentVariable("MY_PATHS")  # False if the variable is unset
```

`locateResource` returns `None` when it cannot resolve the url.

### BytesResource

In-memory resource from bytes.

```python
from tesseract_robotics.tesseract_common import BytesResource

data = b"<robot name='test'></robot>"
resource = BytesResource("robot.urdf", data)
```

Every `Resource` is also a `ResourceLocator`: `resource.locateResource(url)` resolves a url
relative to the resource, through the `parent` locator it was built with (`None` without a
parent). `BytesResource` first asks the parent for the url as given, then for the file next
to its own url:

```python
urdf = BytesResource("package://my_robot/urdf/robot.urdf", data, parent=locator)
mesh = urdf.locateResource("base.stl")  # tries "base.stl", then "package://my_robot/urdf/base.stl"
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
is_allowed = acm.isCollisionAllowed("link_1", "link_2")

# Remove entry
acm.removeAllowedCollision("link_1", "link_2")

# Clear all
acm.clearAllowedCollisions()
```

An ACM converts to and from its entries, a `dict` keyed on ordered link pairs. The
constructor orders each key, so `("link_b", "link_a")` is stored as `("link_a", "link_b")`:

```python
from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix, getAllowedCollisions, makeOrderedLinkPair,
)

acm = AllowedCollisionMatrix({("link_b", "link_a"): "Adjacent links"})
entries = acm.getAllAllowedCollisions()  # {("link_a", "link_b"): "Adjacent links"}

acm.removeAllowedCollision("link_a")     # drop every entry involving link_a
acm.reserveAllowedCollisionMatrix(100)   # pre-size; entries unchanged
print(acm)                               # one "link=<a> link=<b> reason=<r>" line per entry

makeOrderedLinkPair("link_b", "link_a")  # ("link_a", "link_b")

# Links allowed to collide with any of the given links; each partner once unless
# remove_duplicates=False. The order follows the dict's, so treat it as a set.
getAllowedCollisions(["link_a"], entries)
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

`CollisionMarginPairData` holds the pair margins alone. Both types build from pair data,
shift and scale every margin, and merge another set of pair margins:

```python
from tesseract_robotics.tesseract_common import (
    CollisionMarginData,
    CollisionMarginPairData,
    CollisionMarginPairOverrideType,
)

pairs = CollisionMarginPairData({("link_a", "link_b"): 0.05, ("link_a", "link_c"): 0.02})
margins = CollisionMarginData(0.025, pairs)  # or CollisionMarginData(pairs): default 0.0

margins.incrementMargins(0.01)  # default and every pair margin +0.01 m
margins.scaleMargins(2.0)       # then x2

# MODIFY sets the given pairs and keeps the rest; REPLACE keeps only the given pairs.
margins.apply(CollisionMarginPairData({("link_b", "link_d"): 0.07}), CollisionMarginPairOverrideType.MODIFY)

margins.getMaxCollisionMargin()          # largest margin overall
margins.getMaxCollisionMargin("link_a")  # largest margin involving link_a (at least the default)
pairs.getMaxCollisionMargin("link_z")    # None: no pair margin involves link_z
```

## State

### JointState

Joint positions and velocities.

```python
from tesseract_robotics.tesseract_common import JointState

state = JointState()
state.joint_names = ["j1", "j2", "j3"]
state.position = np.array([0.0, 0.5, -0.5])
state.velocity = np.array([0.0, 0.0, 0.0])
```

### JointTrajectory

A sequence of `JointState`s with a `description` and a `uuid` (its canonical string; a
malformed string raises `ValueError`). It behaves like a list of states: `len`, indexing
(negative indices count from the end; out of range raises `IndexError`), assignment and
iteration, plus `push_back`, `pop_back`, `front`, `back`, `at`, `clear` and `empty`.

```python
from tesseract_robotics.tesseract_common import JointState, JointTrajectory

names = ["j1", "j2", "j3"]
traj = JointTrajectory([JointState(names, np.zeros(3))], "approach")
traj.push_back(JointState(names, np.array([0.0, 0.5, -0.5])))

js = traj[-1]
js.time = 1.0
traj[-1] = js  # write back: element access returns a copy
```

!!! warning "Element access returns a copy"
    `traj[i]`, `front()`, `back()`, `at(i)`, iteration and the `states` list are copies,
    so `traj[0].time = 1.0` changes nothing. Assign the edited state back with
    `traj[0] = js`, or assign a whole new list to `traj.states`. A reference into the
    underlying vector would dangle as soon as `push_back` reallocated it.

Trajectories compare by value (`==` compares `uuid`, `description` and `states`) and are
unhashable.

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

# or all at once; tcp_offset (a link name or an Isometry3d) defaults to the identity
info = ManipulatorInfo("manipulator", "base_link", "tool0")
```

`tcp_offset` holds a `str` (a link name) or an `Isometry3d`; any other type raises
`TypeError`. Reading it returns a copy, so assign a new value instead of editing it in
place.

`info.empty()` is `True` unless `manipulator`, `working_frame` and `tcp_frame` are all set.
`base.getCombined(override)` returns a copy of `base` with every non-empty field of
`override`; an overriding `tcp_frame` brings `override.tcp_offset` along with it:

```python
override = ManipulatorInfo()
override.tcp_frame = "tool1"
combined = info.getCombined(override)  # tcp_frame "tool1", tcp_offset from override
```

## Plugin Info

Plugin configuration for the four plugin loaders: `KinematicsPluginInfo`,
`ContactManagersPluginInfo`, `ProfilesPluginInfo` and `TaskComposerPluginInfo`. Each has
`search_paths`, `search_libraries`, `insert(other)`, `clear()`, `empty()` and a read-only
`CONFIG_KEY` (its key in a tesseract YAML config, e.g. `"contact_manager_plugins"`).

```python
from tesseract_robotics.tesseract_common import (
    ContactManagersPluginInfo, PluginInfo, PluginInfoContainer,
)

bullet = PluginInfo()
bullet.class_name = "BulletDiscreteBVHManagerFactory"

info = ContactManagersPluginInfo()
info.search_libraries = ["tesseract_collision_bullet_factories"]
info.discrete_plugin_infos.default_plugin = "BulletDiscreteBVHManager"
info.discrete_plugin_infos.plugins = {"BulletDiscreteBVHManager": bullet}
```

!!! note
    A `PluginInfoContainer` field (`discrete_plugin_infos`, `continuous_plugin_infos`,
    `executor_plugin_infos`, `task_plugin_infos`) is a reference: editing it in place
    persists. A `dict` field (`PluginInfoContainer.plugins`, `ProfilesPluginInfo.plugin_infos`,
    `KinematicsPluginInfo.fwd_plugin_infos`) converts to a fresh `dict` on every access:
    assign the whole field, since `info.plugin_infos["x"] = ...` edits a copy.

`ContactManagersPluginInfo`, `ProfilesPluginInfo` and `TaskComposerPluginInfo` compare by
value (`==`) and are unhashable.

## Logging

Control console_bridge logging level.

```python
from tesseract_robotics.tesseract_common import (
    getLogLevel, setLogLevel,
    CONSOLE_BRIDGE_LOG_NONE,
    CONSOLE_BRIDGE_LOG_ERROR,
    CONSOLE_BRIDGE_LOG_WARN,
    CONSOLE_BRIDGE_LOG_INFO,
    CONSOLE_BRIDGE_LOG_DEBUG,
)

# Suppress warnings
setLogLevel(CONSOLE_BRIDGE_LOG_ERROR)

# Enable debug output
setLogLevel(CONSOLE_BRIDGE_LOG_DEBUG)
```

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
