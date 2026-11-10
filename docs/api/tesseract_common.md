# tesseract_robotics.tesseract_common

Common types, utilities, and resource handling.

!!! note "Eigen arguments take arrays, lists and tuples"
    Every `Eigen::Vector3d`, `VectorXd`, `Matrix3d`, `Matrix4d` (and other dense Eigen)
    parameter accepts a NumPy array, a list or a tuple, nested for a matrix:
    `iso.translate([0.5, 0.2, 0.3])`, `Isometry3d([x, y, z], q)`, `Quaterniond(R.tolist())`.
    The shape must match exactly (`[1, 2]` for a `Vector3d` raises `TypeError`, nothing is
    padded or truncated). Integers convert to `float64`; strings, `None`, complex numbers and
    ragged nesting raise `TypeError`, and an integer parameter (mesh `faces`) refuses floats.
    Returned matrices are always NumPy arrays. The exception is a parameter a function writes
    into (see [Frame and error math](#frame-and-error-math)): it needs the array itself.

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

# Access components (read-only properties; each returns a copy)
position = pose.translation  # np.array([x, y, z])
rotation = pose.rotation     # 3x3 rotation matrix
matrix = pose.matrix         # 4x4 homogeneous matrix

# Compose
combined = pose1 * pose2

# Apply to many points at once: (N, 3) in, (N, 3) out, in one call
points = np.random.default_rng(0).normal(size=(1000, 3))
mapped = pose.apply_points(points)       # row i == pose * points[i], bitwise
rotated = pose.apply_directions(points)  # rotation only, no translation
```

`apply_points` and `apply_directions` take an `(N, 3)` array, including `N = 0` and strided views
such as `a[:, :3]`, and return a new `(N, 3)` float64 array. A single point stays `pose * p`; a
`(3,)` array passed to `apply_points` raises `TypeError`. Each row is computed exactly as
`pose * p` computes it, so the results agree bit for bit. A Python loop over `pose * p` costs about
1 µs per point; `apply_points` costs about 1 µs per call up to a few dozen points.

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

### nearest_rotation

The rotation closest to a 3x3 matrix in the Frobenius norm: `U·diag(1, 1, det(U·Vᵀ))·Vᵀ` from
the SVD `M = U·Σ·Vᵀ` (Eigen's `JacobiSVD`). Use it to project a rotation that carries arithmetic
noise (a pose stored from float32 math) back onto SO(3).

```python
from tesseract_robotics.tesseract_common import Quaterniond, nearest_rotation

R = nearest_rotation(noisy_rotation)  # (3, 3) float64, det +1
q = Quaterniond(R)                    # passes the orthonormality check
```

- The result is always a proper rotation (det +1). For a reflection (det(M) < 0) it is the
  nearest rotation, not the orthogonal factor: the smallest singular direction flips.
- Not bitwise idempotent: an exact rotation comes back within a few ulp (measured ≤ 5.5 ε over
  10 000 random rotations, osx-arm64), because `U·Vᵀ` re-rounds.
- A NaN or infinite entry raises `NonFiniteMatrixError` (a `ValueError`); a shape other than
  3x3 raises `TypeError`.
- The minimiser is unique only for rank ≥ 2 (and σ₂ > σ₃ when det(M) < 0). Otherwise one
  minimiser comes back without warning: the zero matrix, `diag(1, 0, 0)` and `diag(1, 1, -1)`
  all return the identity.

### Rigidity readouts

The checks that accept a matrix as a rotation, in Eigen: `orthonormality_residual(R)` is
`max|RᵀR − I|` of a 3x3 (0 for an exact rotation) and `determinant(R)` its determinant. The stack
forms read the rotation block of each transform of an `(M, 4, 4)` stack, and `norm_each` the norm
of each row of an `(N, 3)` array.

```python
from tesseract_robotics.tesseract_common import (
    orthonormality_residual_each,
    rotation_determinant_each,
)

residual = orthonormality_residual_each(stack)  # (M,): row i bitwise orthonormality_residual(stack[i, :3, :3])
proper = rotation_determinant_each(stack) > 0.0  # (M,): row i bitwise determinant(stack[i, :3, :3])
```

## Frame and error math

Re-base twists and jacobians, and compute pose errors, with tesseract's own math. A twist
is a `(6,)` array `[vx, vy, vz, wx, wy, wz]`; a jacobian is `(6, n)`.

```python
import numpy as np
from tesseract_robotics.tesseract_common import jacobianChangeBase, jacobianChangeRefPoint

J = joint_group.calcJacobian(q, "tool0")    # (6, n), Fortran order, in the group base frame
jacobianChangeBase(J, T_new_base)           # rotate every column into T_new_base, in place
jacobianChangeRefPoint(J, [0.0, 0.0, 0.1])  # v += ω × p, in place
```

!!! warning "In place: these functions write into the array you pass"
    `twistChangeRefPoint`, `twistChangeBase`, `jacobianChangeBase`, `jacobianChangeRefPoint`
    and `applyTolerances` return `None` and change their first argument, as in C++. They
    raise `TypeError` for anything they would have to convert first, since the result
    would be written into a temporary copy and lost: a list or tuple, `float32`, read-only
    or non-contiguous arrays, and C-order `(6, n > 1)` jacobians. `calcJacobian` returns Fortran order;
    convert others with `np.asfortranarray`.

`jacobianChangeRefPoint` takes the point in the jacobian's base frame. To match
`calcJacobian(q, link, p)`, whose `p` is in the link frame, rotate it first:
`jacobianChangeRefPoint(J, joint_group.calcFwdKin(q)[link].rotation @ p)`.

Pose errors are `[translation, angle-axis rotation]` 6-vectors:

```python
from tesseract_robotics.tesseract_common import (
    applyTolerances, calcJacobianTransformErrorDiff, calcRotationalError, calcTransformError,
)

calcRotationalError(R)               # θ·axis, θ in [-π, π]
calcTransformError(t1, t2)           # error of t1.inverse() * t2

# Finite-difference column of a pose-error jacobian. Subtracting two calcTransformError
# results jumps by ~2π near the angle-axis ±π boundary; this does not.
d = calcJacobianTransformErrorDiff(target, source, source_perturbed)
d = calcJacobianTransformErrorDiff(target, target_perturbed, source, source_perturbed)
d = calcJacobianTransformErrorDiff(target, source, source_perturbed, lower_tol, upper_tol)

v = np.array([-2.0, 0.5, 3.0])
applyTolerances(v, np.full(3, -1.0), np.full(3, 1.0))  # v is now [-1., 0., 2.]
```

The tolerance pair of `applyTolerances` and of the tolerance overloads of
`calcJacobianTransformErrorDiff` must be both empty (no tolerance) or both of the value's
size (6 for the error diff); anything else raises `ToleranceSizeMismatchError`, a
`ValueError`.

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
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.tesseract_common import GeneralResourceLocator

# tesseract's support data (the [support] extra) as a search path
locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])

# Resolve package URL
resource = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
path = resource.getFilePath()
```

`tesseract_robotics` never adds search paths itself: a locator searches what it is given and the
environment variables below.

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
is_allowed = acm.isCollisionAllowed(("link_1", "link_2"))

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
margins.setCollisionMargin("link_a", "link_b", 0.05)

default = margins.getDefaultCollisionMargin()
pair_margin = margins.getCollisionMargin("link_a", "link_b")
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
state.joint_ids = ["j1", "j2", "j3"]
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
print(f"Position min: {limits.joint_limits[:, 0]}")
print(f"Position max: {limits.joint_limits[:, 1]}")
print(f"Velocity: {limits.velocity_limits}")
print(f"Acceleration: {limits.acceleration_limits}")
print(f"Jerk: {limits.jerk_limits}")
```

Each field is an `(n, 2)` array of `[lower, upper]` rows. `limits.resize(n)` resizes all
four to `(n, 2)`; their values are unset afterwards, so assign them before use.

Three free functions check or clamp joint values against an `(n, 2)` limit array:

```python
import numpy as np
from tesseract_robotics.tesseract_common import enforceLimits, isWithinLimits, satisfiesLimits

limits = np.array([[-1.0, 1.0], [-2.0, 2.0]])
isWithinLimits(np.array([0.5, -1.5]), limits)   # True: inside or on the bounds, no tolerance
satisfiesLimits(np.array([1.0 + 1e-7, 0.0]), limits)  # True: within the 1e-6 default max_diff
enforceLimits(np.array([1.5, -3.0]), limits)    # array([ 1., -2.]): a clamped copy
```

`enforceLimits` returns a new array and leaves its input unchanged (C++ clamps in place).
`isWithinLimits` and `enforceLimits` raise `LimitsSizeMismatchError` (a `ValueError`) when
the number of values differs from the number of limit rows.

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

## Calibration Info

`CalibrationInfo` holds calibrated joint origins: for each joint name in `joints`, the
environment replaces that joint's origin (a `ChangeJointOriginCommand`). An SRDF carries one
through its `<calibration_config>` file, read into `SRDFModel.calibration_info`.

```python
from tesseract_robotics.tesseract_common import CalibrationInfo, Isometry3d

info = CalibrationInfo()
info.joints = {"joint_a1": Isometry3d.Identity()}  # assign the whole dict
other = CalibrationInfo()
other.joints = {"joint_a2": Isometry3d.Identity()}
info.insert(other)  # overwrites by joint name
assert CalibrationInfo.CONFIG_KEY == "calibration"
```

!!! note
    `joints` converts to a fresh `dict` on every access: `info.joints["j"] = t` edits a copy.
    Equality is upstream's: each transform compares with `isApprox(1e-5)`.

Every class on this page whose C++ type declares `operator==` compares by value (`==`, with
the C++ tolerances) and is unhashable, except `ResourceLocator` and `Resource`, which compare by
identity (their C++ `operator==` is always true). `KinematicLimits` of different shapes are
unequal.

## Logging

Tesseract logs through spdlog (upstream #1367). The default logger is named
`"tesseract"`; its level controls tesseract's console output.

```python
from tesseract_robotics.tesseract_common import (
    LogLevel, addLogRecordHandler, getLogger, isLogLevelEnabled, removeLogRecordHandler,
)

logger = getLogger()              # getLogger("tesseract")
logger.set_level(LogLevel.err)    # suppress warnings
logger.set_level(LogLevel.debug)  # enable debug output
assert isLogLevelEnabled(LogLevel.debug)

# Route records into Python (any thread; the handler runs with the GIL held)
records = []
handler_id = addLogRecordHandler(records.append)
...
removeLogRecordHandler(handler_id)
```

A `LogRecord` carries `level`, `message`, `logger_name`, `component_name`,
`attributes`, `timestamp`, `filename`, `line` and `function_name`. An exception
raised by a handler is reported through `sys.unraisablehook`.

A handler sees every record in addition to the logger's sinks, which still write
to stderr. To send tesseract's output to Python `logging` instead, replace the
sinks: a record then goes to each sink whose level admits it, once.

```python
import logging
from tesseract_robotics.tesseract_common import PythonLoggingSink, StderrSink, getLogger

sink = PythonLoggingSink("myapp.tesseract")  # records go to logging.getLogger("myapp.tesseract")
sink.set_pattern("%v (%s:%#)")              # spdlog pattern for the text; "%v" (the message) by default
getLogger().sinks = [sink]                    # replaces stderr; assign before other threads log

getLogger().sinks = [StderrSink()]            # back to stderr
```

Each Python `LogRecord` gets the level (spdlog `trace` maps to 5, below `DEBUG`),
the source `pathname`, `lineno` and `funcName`, and the spdlog logger's name as
the `spdlog_logger` attribute. Records the Python logger is not enabled for are
dropped at the sink. A sink's own level (`set_level`) filters before that.

Sinks are C++ objects that Python holds a share of: a logger that keeps one does
not keep a Python object alive, so the default logger, which outlives the
interpreter, holds nothing of Python's at exit.

!!! warning "Python handlers, Python sinks and C++ threads"
    A Python handler or `PythonLoggingSink` takes the GIL on whichever thread logs.
    C++ code that joins logging threads while the caller holds the GIL then
    deadlocks: drain an executor's futures (`wait()`) before dropping it while a
    handler or Python sink is installed.

!!! note "console_bridge API removed"
    `setLogLevel`, `getLogLevel`, `log`, `useOutputHandler`,
    `restorePreviousOutputHandler`, `OutputHandler` and the `CONSOLE_BRIDGE_LOG_*`
    constants are gone: upstream tesseract no longer logs through console_bridge,
    so they had no effect on its output.

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
