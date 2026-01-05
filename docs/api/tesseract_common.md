# tesseract_robotics.tesseract_common

Common types, utilities, and resource handling.

## Transforms

### Isometry3d

Rigid body transformation (rotation + translation).

```python
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Identity transform
pose = Isometry3d.Identity()

# From 4x4 matrix
mat = np.eye(4)
mat[:3, 3] = [0.5, 0.2, 0.3]
pose = Isometry3d(mat)

# Access components
position = pose.translation()  # np.array([x, y, z])
rotation = pose.rotation()     # 3x3 rotation matrix
matrix = pose.matrix()         # 4x4 homogeneous matrix

# Transform operations
pose.translate([0.1, 0, 0])    # translate in place
pose.rotate(rotation_matrix)   # rotate in place
combined = pose1 * pose2       # compose transforms
```

### Quaterniond

Quaternion rotation (w, x, y, z).

```python
from tesseract_robotics.tesseract_common import Quaterniond

q = Quaterniond(w=1.0, x=0.0, y=0.0, z=0.0)  # identity
q = Quaterniond.Identity()

# Access
print(f"w={q.w()}, x={q.x()}, y={q.y()}, z={q.z()}")
rotation_matrix = q.toRotationMatrix()
```

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
is_allowed = acm.isCollisionAllowed("link_1", "link_2")

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
state.joint_names = ["j1", "j2", "j3"]
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
