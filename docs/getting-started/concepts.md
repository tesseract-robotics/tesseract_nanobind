# Core Concepts

Understanding these concepts will help you use tesseract_robotics effectively.

## Environment

The **Environment** is the central data structure containing:

- **Scene Graph**: Kinematic tree of links and joints (from URDF)
- **State Solver**: Computes forward kinematics for any joint state
- **Collision Manager**: Checks for collisions between links
- **Allowed Collision Matrix**: Defines which link pairs to ignore

```python
from tesseract_robotics.planning import Robot

robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env  # Access the Environment

# Scene graph operations
scene = env.getSceneGraph()
links = scene.getLinks()
joints = scene.getJoints()

# State operations
state = env.getState()
env.setState({"joint_1": 0.5, "joint_2": -0.3})

# Link transforms
tcp_transform = env.getLinkTransform("tool0")
```

## Kinematic Groups

A **Kinematic Group** defines a chain of joints for kinematics calculations.

Defined in the SRDF file:

```xml
<group name="manipulator">
  <chain base_link="base_link" tip_link="tool0"/>
</group>
```

Used in Python:

```python
import numpy as np
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput

# Get the kinematic group
manip = env.getKinematicGroup("manipulator")
joint_names = manip.getJointNames()

# Forward kinematics
joint_values = np.array([0.0, 0.5, -0.5, 0.0, 0.5, 0.0])
transforms = manip.calcFwdKin(joint_values)
tcp_pose = transforms["tool0"]

# Inverse kinematics (if IK solver configured): pose of tip_link in working_frame
target_pose = tcp_pose
solutions = manip.calcInvKin(KinGroupIKInput(target_pose, "base_link", "tool0"), np.zeros(6))

# Joint limits
limits = manip.getLimits()
print(f"Position limits: {limits.joint_limits}")
print(f"Velocity limits: {limits.velocity_limits}")
```

## Command Language

The **Command Language** describes motion tasks:

### Waypoints

- **StateWaypoint**: Joint-space target
- **CartesianWaypoint**: Cartesian-space target (pose)
- **JointWaypoint**: Named joint positions

### Instructions

- **MoveInstruction**: Move to a waypoint
- **CompositeInstruction**: Container for multiple instructions

```python
from tesseract_robotics.tesseract_command_language import (
    StateWaypoint, StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstruction, MoveInstructionType, MoveInstructionPoly_wrap_MoveInstruction,
    CompositeInstruction
)

# Create a joint-space waypoint
joint_wp = StateWaypointPoly_wrap_StateWaypoint(
    StateWaypoint(joint_names, joint_values)
)

# Create a Cartesian waypoint
cart_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(
    CartesianWaypoint(target_pose)
)

# Create move instructions
move1 = MoveInstruction(joint_wp, MoveInstructionType.FREESPACE, "DEFAULT")
move2 = MoveInstruction(cart_wp, MoveInstructionType.LINEAR, "DEFAULT")

# Combine into a program (instructions are wrapped in the polymorphic type, see below)
program = CompositeInstruction("DEFAULT")
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(move1))
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(move2))
```

## Planners

### Planner Types

| Planner | Strengths | Weaknesses |
|---------|-----------|------------|
| **OMPL** | Complex environments, guaranteed solutions | Slower, paths may need smoothing |
| **TrajOpt** | Smooth trajectories, collision avoidance | May get stuck in local minima |
| **TrajOptIfopt** | Real-time capable, OSQP solver | Requires good initialization |
| **Descartes** | Dense Cartesian paths | Computationally expensive |
| **Simple** | Fast interpolation | No collision checking |

### Motion Types

- **FREESPACE**: Any collision-free path (typically OMPL)
- **LINEAR**: Straight-line Cartesian motion (typically TrajOpt)
- **CIRCULAR**: Arc motion (specialized planners)

## Task Composer

The **Task Composer** orchestrates complex planning tasks:

```
┌─────────────────────────────────────────────────────────────┐
│                    TaskComposer                             │
├─────────────────────────────────────────────────────────────┤
│  Input: CompositeInstruction (program)                      │
│                                                             │
│  Pipeline:                                                  │
│    1. DescartesMotionPlannerTask (if needed)               │
│    2. OMPLMotionPlannerTask                                │
│    3. TrajOptMotionPlannerTask (smoothing)                 │
│    4. TimeParameterizationTask                             │
│    5. ContactCheckTask (validation)                        │
│                                                             │
│  Output: Optimized trajectory with timing                   │
└─────────────────────────────────────────────────────────────┘
```

## Profiles

**Profiles** configure planner behavior without code changes:

```python
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLRealVectorPlanProfile

# Profiles are named configurations
# "DEFAULT" is used if no specific profile is set

# Example: Configure OMPL profile
ompl_profile = OMPLRealVectorPlanProfile()
ompl_profile.solver_config.planning_time = 5.0  # 5 seconds max
ompl_profile.solver_config.simplify = True

# Add to profile dictionary under the planner task's namespace
profiles = ProfileDictionary()
profiles.addProfile("OMPLMotionPlannerTask", "my_profile", ompl_profile)
```

## Collision Modes

### Discrete Collision

Checks collision at a single configuration:

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactTestType,
)

robot.set_joints(dict(zip(joint_names, joint_values)))
manager = robot.env.getDiscreteContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
manager.setCollisionObjectsTransform(robot.env.getState().link_transforms)

contacts = ContactResultMap()
manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
is_safe = contacts.size() == 0
```

### Continuous Collision

Checks collision along a trajectory segment (swept volume):

```python
# Used in TrajOpt for trajectory optimization
# Ensures the path between waypoints is collision-free
```

### LVS (Longest Valid Segment)

Interpolates between waypoints and checks at discrete points:

```python
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptCollisionConfig

# Configured via the collision check config
collision_config = TrajOptCollisionConfig(margin=0.1, coeff=10.0)
collision_config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
collision_config.collision_check_config.longest_valid_segment_length = 0.05  # check every 5cm
```

## Python Binding Notes

### Nanobind Bindings

The Python bindings are generated using [nanobind](https://github.com/wjakob/nanobind), a modern
C++17 binding library. Almost the entire Tesseract API is available in Python. The bindings provide
native NumPy integration for Eigen types and automatic handling of C++ smart pointers.

### C++ Storage and Pointer Types

The Tesseract library uses several common C++ storage and pointer patterns:

- **pass-by-copy**: Objects are copied into Python and managed by Python's garbage collector.
- **shared pointers**: Automatically reference-counted. Destroyed when no references remain.
- **unique pointers**: Nanobind automatically extracts unique_ptr contents, so `result.release()`
  is not needed (unlike SWIG bindings).

### Template Containers

The Tesseract library uses C++ container templates extensively. These are wrapped by nanobind.
Typical examples include `std::vector`, `std::map`, and `std::unordered_map`. Each specific
implementation of the template is wrapped, for example `std::vector<std::string>` is `StringVector`.
Most of these templates are in `tesseract_common`, however they can be found in many modules.
These template types roughly correspond to lists and dictionaries in Python. They can normally be
implicitly created using lists and dictionaries, and when returned have roughly the same accessor
methods.

### Eigen Types

Eigen matrices and vectors are used extensively in Tesseract. These are automatically converted
to NumPy arrays by nanobind. This conversion works for `float64` types (`double` in C++) and
`int32` types (`int` in C++). The conversion uses efficient buffer protocols with minimal copying.
An Eigen argument also accepts a list or a tuple (nested for a matrix) of the exact shape;
integers convert to `float64`. A function that writes into its argument in place needs the
NumPy array itself.

Some Eigen geometry types are wrapped as classes in the `tesseract_common` module:

```python
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Identity transform
pose = Isometry3d.Identity()
pose.translate([0.5, 0.2, 0.3])

# Get components
position = pose.translation    # np.array([x, y, z])
rotation = pose.rotation       # 3x3 rotation matrix
matrix = pose.matrix           # 4x4 homogeneous matrix

# From 4x4 matrix
mat = np.eye(4)
mat[:3, 3] = [1, 2, 3]
pose_from_matrix = Isometry3d(mat)
```

### Resource Locators

Resource locators resolve `package://` URLs to file paths. The `GeneralResourceLocator` class
searches paths in `TESSERACT_RESOURCE_PATH` to find resources.

**Auto-configuration:** When you import `tesseract_robotics`, environment variables are automatically
set to point to bundled data (if not already set):

- `TESSERACT_RESOURCE_PATH` - Path for `package://` URL resolution
- `TESSERACT_SUPPORT_DIR` - Path to bundled tesseract_support directory
- `TESSERACT_TASK_COMPOSER_CONFIG_FILE` - Path to bundled task composer config

This means `package://tesseract/support/urdf/...` URLs work out of the box.

### Type Erasure Pattern

The command language uses type erasure extensively. In Python, module-level utility functions
are needed to create and cast instructions and waypoints:

```python
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)

# Wrap a waypoint in the polymorphic type
cart_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose))

# Cast back to concrete types: first instruction of the program built above
move_instr = InstructionPoly_as_MoveInstructionPoly(program[0])
state_wp = WaypointPoly_as_StateWaypointPoly(move_instr.getWaypoint())
```

`ProfileDictionary` has similar accessor functions per type:

```python
from tesseract_robotics.tesseract_motion_planners_ompl import ProfileDictionary_addOMPLProfile

ProfileDictionary_addOMPLProfile(profiles, "OMPLMotionPlannerTask", "DEFAULT", ompl_profile)
```

### Console Bridge Logging

Tesseract uses `console_bridge` for logging. Control logging level via:

```python
from tesseract_robotics.tesseract_common import (
    getLogLevel, setLogLevel,
    LogLevel,
)

setLogLevel(LogLevel.CONSOLE_BRIDGE_LOG_WARN)
```

## Next Steps

- [Environment Guide](../user-guide/environment.md) - Deep dive into environments
- [Planning Guide](../user-guide/planning.md) - Motion planning details
- [Low-Level SQP](../user-guide/low-level-sqp.md) - Real-time optimization
