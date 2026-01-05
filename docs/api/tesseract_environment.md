# tesseract_robotics.tesseract_environment

Environment management and scene modification commands.

## Environment

Central class containing robot model, collision, and kinematics.

```python
from tesseract_robotics.tesseract_environment import Environment

# Create empty environment
env = Environment()

# Initialize from URDF/SRDF
env.init(urdf_string, srdf_string, locator)

# Or use Robot helper
from tesseract_robotics.planning import Robot
robot = Robot.from_urdf("robot.urdf", "robot.srdf")
env = robot.env
```

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

# Set joint state
env.setState(["joint_1", "joint_2"], [0.5, -0.3])
# Or with dict
env.setState({"joint_1": 0.5, "joint_2": -0.3})

# Get link transform
tcp = env.getLinkTransform("tool0")
```

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
```

## Commands

Modify the environment with commands. Commands are tracked for undo/redo.

### AddLinkCommand

Add a new link to the scene.

```python
from tesseract_robotics.tesseract_environment import AddLinkCommand
from tesseract_robotics.tesseract_scene_graph import Link, Joint

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

new_origin = Isometry3d.Identity()
new_origin.translate([0.1, 0, 0])

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
from tesseract_robotics.tesseract_common import CollisionMarginData

margins = CollisionMarginData()
margins.setDefaultCollisionMargin(0.05)

cmd = ChangeCollisionMarginsCommand(margins)
env.applyCommand(cmd)
```

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
