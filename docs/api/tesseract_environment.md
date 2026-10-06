# tesseract_robotics.tesseract_environment

Environment management and scene modification commands.

## Environment

Central class containing robot model, collision, and kinematics.

```python
from pathlib import Path

from tesseract_robotics.tesseract_environment import Environment

# Create empty environment
env = Environment()

# From URDF + SRDF files (+ ResourceLocator for mesh resolution)
env.init(Path(urdf_path), Path(srdf_path), locator)

# Or from raw XML strings
env.init(urdf_xml, srdf_xml, locator)

# Or use the high-level Robot helper (takes BOTH URDF and SRDF)
from tesseract_robotics.planning import Robot
robot = Robot.from_urdf(
    "package://my_robot/urdf/robot.urdf",
    "package://my_robot/urdf/robot.srdf",
)
env = robot.env
```

!!! warning "A `str` is URDF content, never a path"
    `init` binds the native overloads: `init(str, locator)` and `init(str, str, locator)` parse
    URDF/SRDF *content*; `init(Path, locator)` and `init(Path, Path, locator)` load *files*.
    A path passed as `str` is parsed as XML and `init` returns `False`. Mixing one `Path` with
    one content `str` raises `TypeError`. `initFromUrdf` / `initFromUrdfSrdf` and
    `tesseract_common.FilesystemPath` were removed (gh-165); see the CHANGELOG for the migration.

### Scene Graph Access

```python
# Get scene graph (read-only)
scene = env.getSceneGraph()
links = scene.getLinks()
joints = scene.getJoints()

# Get specific elements
link = env.getLink("base_link")
joint = env.getJoint("joint_1")
```

### State Management

```python
# Get current state
state = env.getState()
joint_positions = state.joints

# Set joint state — two overloads
env.setState({"joint_1": 0.5, "joint_2": -0.3})          # dict form
env.setState(["joint_1", "joint_2"], np.array([0.5, -0.3]))  # names + values

# Get link transform
tcp = env.getLinkTransform("tool0")

# All link transforms: current state (list), or for given joint values (dict, state unchanged)
current = env.getLinkTransforms()
at_values = env.getLinkTransforms(["joint_1", "joint_2"], np.array([0.5, -0.3]))
tcp_at_values = at_values["tool0"]

# Floating joints: current joint-origin transforms, keyed by joint name
floating = env.getCurrentFloatingJointValues()
```

`getLinkTransforms(names, values)` validates its input like `setState`: an unknown or
non-active joint name, or a length mismatch, raises `ValueError`. So does a
`floating_joints` key that is not a floating joint.

### Per-link and per-joint lookups

```python
limits = env.getJointLimits("joint_1")          # JointLimits copy: lower, upper, velocity, ...
enabled = env.getLinkCollisionEnabled("link_1")  # bool
visible = env.getLinkVisibility("link_1")        # bool
```

A name that is not in the scene graph raises `KeyError`, also for
`getCurrentFloatingJointValues(names)`.

### Kinematics

```python
# Get kinematic group (for FK/IK)
manip = env.getKinematicGroup("manipulator")

# Get joint group (FK only, no IK)
group = env.getJointGroup("manipulator")

# Available groups
groups = env.getGroupNames()
```

### Collision

```python
# Get collision managers
discrete = env.getDiscreteContactManager()
continuous = env.getContinuousContactManager()

# Allowed collision matrix
acm = env.getAllowedCollisionMatrix()
```

### Environment Info

```python
# Check initialization
if env.isInitialized():
    print(f"Root link: {env.getRootLinkName()}")
    print(f"Revision: {env.getRevision()}")
    print(f"Revision after init: {env.getInitRevision()}")

# Last change to anything / to the current state, as naive local datetime.datetime
changed = env.getTimestamp()
state_changed = env.getCurrentStateTimestamp()

# Contact-manager plugin config loaded from the SRDF
info = env.getContactManagersPluginInfo()
print(info.discrete_plugin_infos.default_plugin)
```

!!! note "Timestamps are naive local time"
    nanobind converts `std::chrono::system_clock::time_point` to a `datetime.datetime`
    without `tzinfo`, in local time. Compare against `datetime.datetime.now()`, not
    `datetime.datetime.now(datetime.timezone.utc)`.

## Commands

Modify the environment with commands. Commands are tracked for undo/redo.

### AddLinkCommand

Add a new link to the scene.

```python
from tesseract_robotics.tesseract_environment import AddLinkCommand
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType

link = Link("obstacle")
# ... configure link with visual/collision

joint = Joint("obstacle_joint")
joint.type = JointType.FIXED
joint.parent_link_name = "world"
joint.child_link_name = "obstacle"

cmd = AddLinkCommand(link, joint)
env.applyCommand(cmd)
```

### RemoveLinkCommand

```python
from tesseract_robotics.tesseract_environment import RemoveLinkCommand

cmd = RemoveLinkCommand("obstacle")
env.applyCommand(cmd)
```

### MoveLinkCommand

Move a link to a new parent.

```python
from tesseract_robotics.tesseract_environment import MoveLinkCommand

joint = Joint("new_joint")
# ... configure joint

cmd = MoveLinkCommand(joint)
env.applyCommand(cmd)
```

### ChangeJointOriginCommand

Change a joint's transform.

```python
from tesseract_robotics.tesseract_environment import ChangeJointOriginCommand
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

mat = np.eye(4)
mat[:3, 3] = [0.1, 0, 0]
new_origin = Isometry3d(mat)

cmd = ChangeJointOriginCommand("obstacle_joint", new_origin)
env.applyCommand(cmd)
```

### Joint Limit Commands

```python
from tesseract_robotics.tesseract_environment import (
    ChangeJointPositionLimitsCommand,
    ChangeJointVelocityLimitsCommand,
    ChangeJointAccelerationLimitsCommand,
)

# Position limits
cmd = ChangeJointPositionLimitsCommand("joint_1", -2.0, 2.0)

# Velocity limits
cmd = ChangeJointVelocityLimitsCommand("joint_1", 1.5)

# Acceleration limits
cmd = ChangeJointAccelerationLimitsCommand("joint_1", 5.0)
```

### ChangeLinkCollisionEnabledCommand

Enable/disable collision for a link.

```python
from tesseract_robotics.tesseract_environment import ChangeLinkCollisionEnabledCommand

cmd = ChangeLinkCollisionEnabledCommand("gripper", False)  # disable
env.applyCommand(cmd)
```

### ModifyAllowedCollisionsCommand

Update the allowed collision matrix.

```python
from tesseract_robotics.tesseract_environment import (
    ModifyAllowedCollisionsCommand, ModifyAllowedCollisionsType
)
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix

acm = AllowedCollisionMatrix()
acm.addAllowedCollision("link_a", "link_b", "custom reason")

cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
env.applyCommand(cmd)
```

| ModifyAllowedCollisionsType | Description |
|-----------------------------|-------------|
| `ADD` | Add entries to existing ACM |
| `REMOVE` | Remove entries from ACM |
| `REPLACE` | Replace entire ACM |

### ChangeCollisionMarginsCommand

Update collision margins.

```python
from tesseract_robotics.tesseract_environment import ChangeCollisionMarginsCommand

# Simplified 0.34 constructor — just pass the new default margin
cmd = ChangeCollisionMarginsCommand(0.05)
env.applyCommand(cmd)

# Or override per-pair margins via CollisionMarginPairData
from tesseract_robotics.tesseract_common import (
    CollisionMarginPairData, CollisionMarginPairOverrideType
)
pair_data = CollisionMarginPairData()
pair_data.setCollisionMargin("link_a", "link_b", 0.1)
cmd = ChangeCollisionMarginsCommand(pair_data, CollisionMarginPairOverrideType.REPLACE)
env.applyCommand(cmd)
```

### AddContactManagersPluginInfoCommand

Add contact manager plugin configuration to a live environment.

```python
from tesseract_robotics.tesseract_common import ContactManagersPluginInfo
from tesseract_robotics.tesseract_environment import AddContactManagersPluginInfoCommand

info = ContactManagersPluginInfo()
info.search_paths = ["/opt/my_plugins"]
cmd = AddContactManagersPluginInfoCommand(info)
env.applyCommand(cmd)
```

`getContactManagersPluginInfo()` returns a copy, so editing it does not change a command
already applied.

### AddTrajectoryLinkCommand

Add a link, attached to `parent_link_name`, whose collision geometry covers the robot's
links over a `JointTrajectory` (for example to keep a planned motion clear of other
moving objects). Each state names the joints it sets.

```python
import numpy as np
from tesseract_robotics.tesseract_common import JointState, JointTrajectory
from tesseract_robotics.tesseract_environment import AddTrajectoryLinkCommand

names = ["joint_1", "joint_2"]
traj = JointTrajectory([JointState(names, np.array([0.0, 0.0])),
                        JointState(names, np.array([0.5, -0.5]))])

Method = AddTrajectoryLinkCommand.Method
cmd = AddTrajectoryLinkCommand("swept_volume", "world", traj, method=Method.GLOBAL_CONVEX_HULL)
env.applyCommand(cmd)
```

| `AddTrajectoryLinkCommand.Method` | Collision geometry |
| --- | --- |
| `PER_STATE_OBJECTS` (default) | every active link's collision objects, for every state |
| `PER_STATE_CONVEX_HULL` | one convex hull per state |
| `GLOBAL_PER_LINK_CONVEX_HULL` | one convex hull per active link, over all states |
| `GLOBAL_CONVEX_HULL` | one convex hull over all active links and all states |

The table follows the enum's comments in `add_trajectory_link_command.h`; the tests check
only that each method adds the link. `getTrajectory()` returns a copy. Commands compare by
value (`==`) and are unhashable.

## Events

Subscribe to environment changes.

```python
from tesseract_robotics.tesseract_environment import (
    Events, Events_COMMAND_APPLIED, Events_SCENE_STATE_CHANGED,
    cast_CommandAppliedEvent, cast_SceneStateChangedEvent
)

def on_event(event):
    if event.type == Events_COMMAND_APPLIED:
        cmd_event = cast_CommandAppliedEvent(event)
        print(f"Command applied: revision {cmd_event.revision}")
    elif event.type == Events_SCENE_STATE_CHANGED:
        state_event = cast_SceneStateChangedEvent(event)
        print("State changed")

env.addEventCallback(Events_COMMAND_APPLIED, on_event)
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_environment._tesseract_environment
    options:
      show_root_heading: false
      show_source: false
      members_order: source
