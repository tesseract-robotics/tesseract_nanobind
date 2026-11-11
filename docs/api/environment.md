# tesseract_robotics.environment

<!-- snippet: ambient
from tesseract_robotics.common import Isometry3d

base_pose = Isometry3d.Identity()
start_joints = np.zeros(6)
end_joints = np.full(6, 0.1)
-->

Environment management and scene modification commands.

## Environment

Central class containing robot model, collision, and kinematics.

```python
from pathlib import Path

from tesseract_robotics import get_tesseract_support_path
from tesseract_robotics.environment import Environment

urdf_path = get_tesseract_support_path() / "urdf" / "abb_irb2400.urdf"
srdf_path = get_tesseract_support_path() / "urdf" / "abb_irb2400.srdf"

# Create empty environment
env = Environment()

# From URDF + SRDF files (+ ResourceLocator for mesh resolution)
assert env.init(Path(urdf_path), Path(srdf_path), locator)

# Or from raw XML strings
env = Environment()
urdf_xml, srdf_xml = urdf_path.read_text(), srdf_path.read_text()
assert env.init(urdf_xml, srdf_xml, locator)

# Or use the high-level Robot helper (takes BOTH URDF and SRDF)
from tesseract_robotics.planning import Robot
robot = Robot.from_urdf(
    "package://tesseract/support/urdf/abb_irb2400.urdf",
    "package://tesseract/support/urdf/abb_irb2400.srdf",
    locator,
)
env = robot.env
```

!!! warning "A `str` is URDF content, never a path"
    `init` binds the native overloads: `init(str, locator)` and `init(str, str, locator)` parse
    URDF/SRDF *content*; `init(Path, locator)` and `init(Path, Path, locator)` load *files*.
    A path passed as `str` is parsed as XML and `init` returns `False`. Mixing one `Path` with
    one content `str` raises `TypeError`. `initFromUrdf` / `initFromUrdfSrdf` and
    `common.FilesystemPath` were removed (gh-165); see the CHANGELOG for the migration.

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
# Current state
state = env.getState()
joint_positions = state.joints

# State at other joint values, without changing the environment
at_dict = env.getState({"joint_1": 0.5, "joint_2": -0.3})
at_values = env.getState(["joint_1", "joint_2"], np.array([0.5, -0.3]))

# Set joint state: dict, or names + values
env.setState({"joint_1": 0.5, "joint_2": -0.3})
env.setState(["joint_1", "joint_2"], np.array([0.5, -0.3]))

# Floating joints: every getState / setState form takes a {joint_name: Isometry3d} dict,
# either as the trailing floating_joints argument or on its own. The IRB2400 has none, so
# this adds a free-floating box with one, named "base_joint".
from tesseract_robotics.environment import AddLinkCommand
from tesseract_robotics.geometry import Box
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link

floater = Link("floater")
box = Collision()
box.geometry = Box(0.1, 0.1, 0.1)
floater.addCollision(box)
floating_joint = Joint("base_joint")
floating_joint.type = JointType.FLOATING
floating_joint.parent_link_id = "base_link"
floating_joint.child_link_id = "floater"
assert env.applyCommand(AddLinkCommand(floater, floating_joint))

env.setState({"base_joint": base_pose})
env.setState({"joint_1": 0.5}, {"base_joint": base_pose})
moved = env.getState(["joint_1"], np.array([0.5]), {"base_joint": base_pose})

# Current values of chosen joints, in the order given
q = env.getCurrentJointValues(["joint_2", "joint_1"])

# Get link transform
tcp = env.getLinkTransform("tool0")

# All link transforms: current state (list), or for given joint values (dict, state unchanged)
current = env.getLinkTransforms()
at_values = env.getLinkTransforms(["joint_1", "joint_2"], np.array([0.5, -0.3]))
tcp_at_values = at_values["tool0"]

# Floating joints: current joint-origin transforms, keyed by joint name
floating = env.getCurrentFloatingJointValues()
```

`getState`, `setState` and `getLinkTransforms` validate their input before the state solver
sees it: an unknown or non-active joint name, or a names/values length mismatch, raises
`ValueError`. So does a `floating_joints` key that is not a floating joint; a rejected
`setState` leaves the state unchanged. `getCurrentJointValues(names)` raises `KeyError` for a
name that is not an active joint.

!!! note "An empty dict"
    `{}` converts to both a joint-value dict and a floating-joint dict, so `getState({})` and
    `setState({})` take the joint-value form. Both forms do the same thing for an empty dict.

The Python-only names `getStateByMap`, `getStateByNamesAndValues`, `setStateByNamesAndValues`
and `getCurrentJointValuesByNames` are removed: call the overloads of `getState`, `setState` and
`getCurrentJointValues`.

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

# Get joint group (FK only, no IK): an SRDF group, or any joints under a name of your choice
group = env.getJointGroup("manipulator")
wrist = env.getJointGroup("wrist", ["joint_5", "joint_6"])

# Links moved by the given joints, and the links they leave fixed
moving = env.getActiveLinkIds(["joint_5", "joint_6"])
fixed = env.getStaticLinkIds(["joint_5", "joint_6"])

# Available groups
groups = env.getGroupNames()
```

These three take joints of any type and raise `ValueError` for a name that is not in the scene
graph. `getJointGroup` also needs every joint to move: a fixed joint raises `RuntimeError`.

### TCP offsets

`findTCPOffset(manip_info)` resolves `manip_info.tcp_offset`, in this order
(`environment.cpp:832–862`, tesseract 0.35.0):

1. an `Isometry3d` is returned as is;
2. a name that is a link of the scene raises `RuntimeError`: use it as `tcp_frame` instead;
3. a name in the SRDF group TCPs of `manip_info.manipulator`
   ([named TCPs](srdf.md#named-tcps-group_tcps)) returns that transform; there is no
   fallback to another group;
4. any other name goes to the find-TCP callbacks, in registration order;
5. if none returns a transform, `RuntimeError: Could not find tcp by name …`.

Every planner calls `findTCPOffset` itself and composes the result the same way
(tesseract_planning 0.35.0):

| Quantity | Composition | Source |
|---|---|---|
| TCP pose in world | `T_world_{tcp_frame}(q) * offset` | `simple/src/interpolation.cpp:108`, `core/src/utils.cpp:73` |
| Cartesian waypoint `C` | relative to `working_frame`: `T_world_{working_frame} * C` is the TCP pose | `core/src/utils.cpp:90`, `trajopt/src/trajopt_utils.cpp:83–87` |
| IK target | the `tcp_frame` pose, `C * offset.inverse()`, in `working_frame` | `ompl/src/profile/ompl_real_vector_move_profile.cpp:187`, `descartes/include/tesseract/motion_planners/descartes/impl/descartes_robot_sampler.hpp:92` |

FK (`calcFwdKin`) returns link poses and IK (`KinGroupIKInput`) takes a link pose: neither
applies the offset, so a caller doing FK/IK by hand multiplies or strips it as above. When a
move instruction carries its own `ManipulatorInfo`, it overrides the program's field by field,
but `tcp_offset` only together with a non-empty `tcp_frame`
(`ManipulatorInfo::getCombined`, `manipulator_info.cpp:54–58`): an instruction that sets only
`tcp_offset` keeps the program's offset.

```python
import numpy as np

from tesseract_robotics.common import Isometry3d, ManipulatorInfo

def tool_offsets(manip_info):
    if manip_info.tcp_offset != "nozzle":
        raise LookupError(manip_info.tcp_offset)  # "not located": the next callback is asked
    matrix = np.eye(4)
    matrix[:3, 3] = [0.0, 0.0, 0.12]
    return Isometry3d(matrix)

env.addFindTCPOffsetCallback(tool_offsets)
tcp = env.findTCPOffset(ManipulatorInfo("manipulator", "base_link", "tool0", "nozzle"))
```

!!! warning "A raising callback means \"not located\""
    `findTCPOffset` catches every exception a callback raises, including a bug in it, and asks
    the next callback. Returning anything but an `Isometry3d` (`None` included) counts the same.
    When no callback returns a transform, it raises
    `RuntimeError: Could not find tcp by name <name>'!`, and the callback's own traceback is not
    shown. Test a callback by calling it directly.

`clone()` copies the callbacks, and the environment keeps them alive, so the Python function may
go out of scope. A callback can run on any thread (task-composer workers call `findTCPOffset`)
and takes the GIL itself. It runs under the environment's lock, so it must not call into the
same environment, and it must not keep that environment alive itself (a closure over it, or
an object attribute holding it): the environment keeps the callback alive from C++, so the
garbage collector cannot see that cycle and neither is ever freed. A module-level function that
reaches the environment only as a global is fine.

Callbacks cannot be listed or removed: `getFindTCPOffsetCallbacks` is not bound, since a C++
`std::function` has nothing Python could inspect.

### Collision

```python
# Get a copy of the active collision managers
discrete = env.getDiscreteContactManager()
continuous = env.getContinuousContactManager()

# Or a copy built from any registered plugin; the active manager is unchanged
simple = env.getDiscreteContactManager("BulletDiscreteSimpleManager")

# Allowed collision matrix
acm = env.getAllowedCollisionMatrix()
```

The registered names are the keys of
`env.getContactManagersPluginInfo().discrete_plugin_infos.plugins` (and
`continuous_plugin_infos.plugins`). Any other name raises `KeyError`. A manager keeps its
`Environment` alive, because its code lives in a plugin library the environment loaded.

#### Contact allowed validator

Every contact manager the environment creates or clones gets the same
`EnvironmentContactAllowedValidator`. It allows a link pair to collide only if the scene graph's
allowed collision matrix allows it. It holds the scene graph by shared pointer, so it follows later
ACM changes (`ModifyAllowedCollisionsCommand`, `RemoveAllowedCollisionLinkCommand`) and keeps the
graph alive after the environment is gone.

```python
from tesseract_robotics.common import (
    ACMContactAllowedValidator,
    AllowedCollisionMatrix,
    CombinedContactAllowedValidator,
    CombinedContactAllowedValidatorType,
)
from tesseract_robotics.environment import EnvironmentContactAllowedValidator

validator = env.getDiscreteContactManager().getContactAllowedValidator()
assert isinstance(validator, EnvironmentContactAllowedValidator)
validator(("base_link", "link_1"))  # True: the SRDF disables this pair

# Build one for any scene graph, e.g. to combine it with an extra ACM
own = EnvironmentContactAllowedValidator(env.getSceneGraph())
extra_acm = AllowedCollisionMatrix()
extra = ACMContactAllowedValidator(extra_acm)
combined = CombinedContactAllowedValidator([own, extra], CombinedContactAllowedValidatorType.OR)
env.getDiscreteContactManager().setContactAllowedValidator(combined)
```

!!! note "A scene graph is required"
    The only constructor takes a `SceneGraph`. `EnvironmentContactAllowedValidator()` and
    `EnvironmentContactAllowedValidator(None)` raise `TypeError`: the C++ default constructor
    exists for serialization and leaves the scene graph null.

### Single-state collision checks

`checkTrajectoryState` places every active collision object of a manager at one set of link
transforms and runs a contact test; `checkTrajectorySegment` casts each object from `state0` to
`state1` (continuous manager only). Both are the C++ functions from
`tesseract/environment/utils.h`, except that C++ fills a `ContactResultMap&` out-parameter and
Python gets the map back as the return value. Each call returns a new map, so results never
carry over from an earlier call; C++ appends to the map it is given.

```python
from tesseract_robotics.collision import ContactRequest
from tesseract_robotics.environment import checkTrajectorySegment, checkTrajectoryState

names = env.getGroupJointIds("manipulator")
start = env.getState(names, start_joints).link_transforms  # dict[str, Isometry3d]
end = env.getState(names, end_joints).link_transforms

# One state, either manager
contacts = checkTrajectoryState(env.getDiscreteContactManager(), end, ContactRequest())
contacts = checkTrajectoryState(env.getContinuousContactManager(), end, ContactRequest())

# One swept segment, continuous manager
contacts = checkTrajectorySegment(env.getContinuousContactManager(), start, end, ContactRequest())

if contacts.count() > 0:
    print(contacts.getSummary())
```

The state maps must hold a transform for every name in `manager.getActiveCollisionObjects()`;
extra keys are ignored. A missing one raises `KeyError` naming every missing link, before the
manager is moved. (C++ throws `std::out_of_range` partway through, after it has already moved the
links before the missing one.) The GIL is released during the contact test.

### Environment Info

```python
# Check initialization
if env.isInitialized():
    print(f"Root link: {env.getRootLinkId()}")
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
from tesseract_robotics.environment import AddLinkCommand
from tesseract_robotics.geometry import Box
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link

link = Link("obstacle")
collision = Collision()
collision.geometry = Box(0.1, 0.1, 0.1)
link.addCollision(collision)

joint = Joint("obstacle_joint")
joint.type = JointType.FIXED
joint.parent_link_id = "base_link"
joint.child_link_id = "obstacle"

cmd = AddLinkCommand(link, joint)
assert env.applyCommand(cmd)
```

### RemoveLinkCommand

```python
from tesseract_robotics.environment import RemoveLinkCommand

cmd = RemoveLinkCommand("obstacle")
assert env.applyCommand(cmd)
```

### MoveLinkCommand

Move a link to a new parent: the joint names the link as its child and the new parent.

```python
from tesseract_robotics.environment import MoveLinkCommand

assert env.applyCommand(AddLinkCommand(link, joint))  # the obstacle is back, under base_link

moved = Joint("obstacle_joint")
moved.type = JointType.FIXED
moved.parent_link_id = "tool0"
moved.child_link_id = "obstacle"

cmd = MoveLinkCommand(moved)
assert env.applyCommand(cmd)
```

### ChangeJointOriginCommand

Change a joint's transform.

```python
from tesseract_robotics.environment import ChangeJointOriginCommand
from tesseract_robotics.common import Isometry3d
import numpy as np

mat = np.eye(4)
mat[:3, 3] = [0.1, 0, 0]
new_origin = Isometry3d(mat)

cmd = ChangeJointOriginCommand("obstacle_joint", new_origin)
env.applyCommand(cmd)
```

### Joint Limit Commands

```python
from tesseract_robotics.environment import (
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
from tesseract_robotics.environment import ChangeLinkCollisionEnabledCommand

cmd = ChangeLinkCollisionEnabledCommand("obstacle", False)  # disable
assert env.applyCommand(cmd)
```

### ModifyAllowedCollisionsCommand

Update the allowed collision matrix.

```python
from tesseract_robotics.environment import (
    ModifyAllowedCollisionsCommand, ModifyAllowedCollisionsType
)
from tesseract_robotics.common import AllowedCollisionMatrix

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
from tesseract_robotics.environment import ChangeCollisionMarginsCommand

# The constructor takes the new default margin (m)
cmd = ChangeCollisionMarginsCommand(0.05)
env.applyCommand(cmd)

# Or override per-pair margins via CollisionMarginPairData
from tesseract_robotics.common import (
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
from tesseract_robotics.common import ContactManagersPluginInfo
from tesseract_robotics.environment import AddContactManagersPluginInfoCommand

info = ContactManagersPluginInfo()
info.search_paths = ["/opt/my_plugins"]
cmd = AddContactManagersPluginInfoCommand(info)
env.applyCommand(cmd)
```

`getContactManagersPluginInfo()` returns a copy, so editing it does not change a command
already applied.

### AddTrajectoryLinkCommand

Add a link, attached to `parent_link_id`, whose collision geometry covers the robot's
links over a `JointTrajectory` (for example to keep a planned motion clear of other
moving objects). Each state names the joints it sets.

```python
import numpy as np
from tesseract_robotics.common import JointState, JointTrajectory
from tesseract_robotics.environment import AddTrajectoryLinkCommand

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
only that each method adds the link. `getTrajectory()` returns a copy.

All commands, and `Environment`, compare by value (`==`) and are unhashable. `env ==
env.clone()`; two environments built separately differ (their timestamps), and commands holding
geometry from two separate parses differ (geometry compares by uuid).

### SetActiveDiscreteContactManagerCommand / SetActiveContinuousContactManagerCommand

Switch the active contact manager plugin, as a command that the history records.

```python
from tesseract_robotics.environment import (
    SetActiveContinuousContactManagerCommand,
    SetActiveDiscreteContactManagerCommand,
)

env.applyCommand(SetActiveDiscreteContactManagerCommand("BulletDiscreteBVHManager"))
env.applyCommand(SetActiveContinuousContactManagerCommand("BulletCastBVHManager"))
```

`Environment.setActiveDiscreteContactManager(name)` (and its continuous twin) switches the
manager without recording a command, as upstream does, so `init(env.getCommandHistory())`
does not replay it. Use the command when the switch must survive a replay.

### Command history

Every command an environment applied, its own initialisation included, is in
`getCommandHistory()`. Each element comes back as its own class, and `getType()` returns a
`CommandType`. `applyCommands` applies a list in one call, and `init(commands)` rebuilds an
environment from a history.

```python
from tesseract_robotics.environment import (
    ChangeLinkVisibilityCommand, CommandType, Environment, RemoveLinkCommand
)

env.applyCommands([RemoveLinkCommand("obstacle"), ChangeLinkVisibilityCommand("link_1", False)])

history = env.getCommandHistory()
assert history[0].getType() == CommandType.ADD_SCENE_GRAPH
assert isinstance(history[-1], ChangeLinkVisibilityCommand)

replay = Environment()
assert replay.init(history)  # the list must start with an AddSceneGraphCommand
assert replay.getRevision() == env.getRevision()
```

`applyCommands` and `init(commands)` keep the Python command objects themselves, without
copying them.

## Events

Subscribe to environment changes. A callback receives each event as its own class,
`CommandAppliedEvent` or `SceneStateChangedEvent`, so it can read `revision` or `state`
directly; `event.type` says which one it is.

```python
from tesseract_robotics.environment import Events

def on_event(event):
    if event.type == Events.COMMAND_APPLIED:
        print(f"Command applied: revision {event.revision}, "
              f"last command {type(event.commands[-1]).__name__}")
    elif event.type == Events.SCENE_STATE_CHANGED:
        print(f"State changed: {len(event.state.joints)} joints")

key = env.addEventCallback(on_event)  # the key for removeEventCallback
env.setState(["joint_1"], np.array([0.1]))
env.removeEventCallback(key)
```

!!! note "An event is the callback's own copy"
    Each event holds its own copy of the command history or scene state, so a callback may
    keep it after it returns. A callback must not change the environment that fired it: the
    environment holds its lock while the callback runs.

`cast_CommandAppliedEvent(event)` and `cast_SceneStateChangedEvent(event)` remain for
existing code. They return the same object they are given, and raise `EventTypeError`
(a `TypeError`) when `event.type` is the other kind:

```python
from tesseract_robotics.environment import EventTypeError, cast_CommandAppliedEvent

def check(event):
    try:
        cast_CommandAppliedEvent(event)
    except EventTypeError as e:
        print(e)  # cast_CommandAppliedEvent: event type is Events.SCENE_STATE_CHANGED, expected Events.COMMAND_APPLIED


key = env.addEventCallback(check)
env.setState(["joint_1"], np.array([0.2]))  # a SCENE_STATE_CHANGED event
env.removeEventCallback(key)
```

## Reference

::: tesseract_robotics.environment
