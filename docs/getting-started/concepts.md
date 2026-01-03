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
# Get the kinematic group
manip = env.getKinematicGroup("manipulator")

# Forward kinematics
joint_values = np.array([0.0, 0.5, -0.5, 0.0, 0.5, 0.0])
transforms = manip.calcFwdKin(joint_values)
tcp_pose = transforms["tool0"]

# Inverse kinematics (if IK solver configured)
solutions = manip.calcInvKin(target_pose, seed_values)

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
    StateWaypointPoly, CartesianWaypointPoly,
    MoveInstruction, MoveInstructionType,
    CompositeInstruction
)

# Create a joint-space waypoint
joint_wp = StateWaypointPoly.wrap_StateWaypoint(
    StateWaypoint(joint_names, joint_values)
)

# Create a Cartesian waypoint
cart_wp = CartesianWaypointPoly.wrap_CartesianWaypoint(
    CartesianWaypoint(target_pose)
)

# Create move instructions
move1 = MoveInstruction(joint_wp, MoveInstructionType.FREESPACE, "DEFAULT")
move2 = MoveInstruction(cart_wp, MoveInstructionType.LINEAR, "DEFAULT")

# Combine into a program
program = CompositeInstruction("DEFAULT")
program.appendMoveInstruction(move1)
program.appendMoveInstruction(move2)
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
# Profiles are named configurations
# "DEFAULT" is used if no specific profile is set

# Example: Configure OMPL profile
ompl_profile = OMPLDefaultPlanProfile()
ompl_profile.planning_time = 5.0  # 5 seconds max
ompl_profile.simplify = True

# Add to profile dictionary
profiles = ProfileDictionary()
profiles.addProfile("ompl", "my_profile", ompl_profile)
```

## Collision Modes

### Discrete Collision

Checks collision at a single configuration:

```python
# Check if configuration is collision-free
is_safe = robot.check_collision(joint_values)
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
# Configured via collision_margin_buffer
collision_config = TrajOptCollisionConfig(margin=0.1, coeff=10.0)
collision_config.collision_margin_buffer = 0.10
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

Some Eigen geometry types are wrapped as classes in the `tesseract_common` module:

```python
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Identity transform
pose = Isometry3d.Identity()
pose.translate([0.5, 0.2, 0.3])

# Get components
position = pose.translation()  # np.array([x, y, z])
rotation = pose.rotation()     # 3x3 rotation matrix
matrix = pose.matrix()         # 4x4 homogeneous matrix

# From 4x4 matrix
mat = np.eye(4)
mat[:3, 3] = [1, 2, 3]
pose_from_matrix = Isometry3d(mat)
```

## Next Steps

- [Environment Guide](../user-guide/environment.md) - Deep dive into environments
- [Planning Guide](../user-guide/planning.md) - Motion planning details
- [Low-Level SQP](../user-guide/low-level-sqp.md) - Real-time optimization
