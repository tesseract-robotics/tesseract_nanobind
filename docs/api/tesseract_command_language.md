# tesseract_robotics.tesseract_command_language

Motion program instructions and waypoints.

## Overview

The command language defines motion tasks as a tree of instructions:

```
CompositeInstruction (program)
├── MoveInstruction (freespace to via point)
├── MoveInstruction (linear to target)
└── CompositeInstruction (sub-program)
    ├── MoveInstruction
    └── MoveInstruction
```

## Waypoints

### StateWaypoint

Joint-space target (specific joint values).

```python
from tesseract_robotics.tesseract_command_language import (
    StateWaypoint, StateWaypointPoly,
)
import numpy as np

joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
joint_values = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

# Create waypoint
wp = StateWaypoint(joint_names, joint_values)

# Wrap in polymorphic type (required for instructions)
wp_poly = StateWaypointPoly(wp)

# Access values
print(f"Names: {wp.getNames()}")
print(f"Position: {wp.getPosition()}")
```

### CartesianWaypoint

Cartesian-space target (pose).

```python
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint, CartesianWaypointPoly,
)
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Create target pose (build the 4x4 matrix then wrap)
mat = np.eye(4)
mat[:3, 3] = [0.5, 0.2, 0.3]
pose = Isometry3d(mat)

# Create waypoint
wp = CartesianWaypoint(pose)

# With seed (initial joint guess for IK)
wp = CartesianWaypoint(pose)
wp.setSeed(seed_state)  # StateWaypointPoly

# Wrap in polymorphic type
wp_poly = CartesianWaypointPoly(wp)
```

### JointWaypoint

Named joint positions (subset of joints).

```python
from tesseract_robotics.tesseract_command_language import (
    JointWaypoint, JointWaypointPoly,
)

names = ["joint_1", "joint_3"]
values = np.array([0.5, -0.5])

wp = JointWaypoint(names, values)
wp_poly = JointWaypointPoly(wp)
```

## Instructions

### MoveInstruction

Single motion command.

```python
from tesseract_robotics.tesseract_command_language import (
    MoveInstruction, MoveInstructionPoly,
    MoveInstructionType
)

# Create move instruction
instr = MoveInstruction(wp_poly, MoveInstructionType.FREESPACE, "DEFAULT")

# With specific profile
instr = MoveInstruction(wp_poly, MoveInstructionType.LINEAR, "my_profile")

# Wrap in polymorphic type
instr_poly = MoveInstructionPoly(instr)

# Access properties
print(f"Profile: {instr.getProfile()}")
print(f"Type: {instr.getMoveType()}")
waypoint = instr.getWaypoint()
```

### MoveInstructionType

| Type | Description | Typical Planner |
|------|-------------|-----------------|
| `FREESPACE` | Any collision-free path | OMPL |
| `LINEAR` | Straight Cartesian line | TrajOpt |
| `CIRCULAR` | Circular arc | Specialized |

### CompositeInstruction

Container for multiple instructions.

```python
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction, CompositeInstructionOrder,
)

# Create program
program = CompositeInstruction("DEFAULT")

# Set manipulator info
from tesseract_robotics.tesseract_common import ManipulatorInfo
manip_info = ManipulatorInfo()
manip_info.manipulator = "manipulator"
manip_info.tcp_frame = "tool0"
manip_info.working_frame = "base_link"
program.setManipulatorInfo(manip_info)

# Add instructions
program.push_back(move1)
program.push_back(move2)

# push_back accepts either a single Instruction or a nested CompositeInstruction
# (0.35+) so raster-style programs can be built the same way they are in C++:
program.push_back(sub_composite)

# Iterate instructions
for instr in program.getInstructions():
    pass
```

### CompositeInstructionOrder

| Order | Description |
|-------|-------------|
| `ORDERED` | Execute in sequence |
| `UNORDERED` | Can be reordered |
| `ORDERED_AND_REVERTED` | Sequence, can reverse |

## Non-Motion Instructions

Programs can also contain non-motion instructions: timed waits, IO handshakes,
analog/digital setpoint changes, and tool changes. These are typically passed
through the planning pipeline unchanged and acted on at runtime by the
trajectory executor.

Concrete instruction types (`WaitInstruction`, `TimerInstruction`,
`SetAnalogInstruction`, `SetDigitalInstruction`, `SetToolInstruction`) can be
pushed directly into a `CompositeInstruction` via `push_back` — implicit
conversion to `InstructionPoly` is registered so no explicit wrap is needed.

### WaitInstruction

Pause execution for a fixed duration or until an IO line transitions.

```python
from tesseract_robotics.tesseract_command_language import (
    WaitInstruction, WaitInstructionType,
)

# Time-based wait
program.push_back(WaitInstruction(0.5))  # wait 0.5 seconds

# IO handshake — wait until digital input 3 goes high
program.push_back(WaitInstruction(WaitInstructionType.DIGITAL_INPUT_HIGH, 3))
```

| `WaitInstructionType` | Behaviour |
|-----------------------|-----------|
| `TIME` | Wait `time` seconds |
| `DIGITAL_INPUT_HIGH` / `DIGITAL_INPUT_LOW` | Block until input `io` is high / low |
| `DIGITAL_OUTPUT_HIGH` / `DIGITAL_OUTPUT_LOW` | Block until output `io` is high / low |

### TimerInstruction

Start a timer that, on expiry, drives a digital output high or low.

```python
from tesseract_robotics.tesseract_command_language import (
    TimerInstruction, TimerInstructionType,
)

program.push_back(
    TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 4)
)
```

### SetAnalogInstruction / SetDigitalInstruction

Drive an analog or digital channel.

```python
from tesseract_robotics.tesseract_command_language import (
    SetAnalogInstruction, SetDigitalInstruction,
)

# Analog: set channel ("speed", index 1) to 0.75
program.push_back(SetAnalogInstruction("speed", 1, 0.75))

# Digital: open the gripper
program.push_back(SetDigitalInstruction("gripper", 2, True))
```

### SetToolInstruction

Activate a tool by ID.

```python
from tesseract_robotics.tesseract_command_language import SetToolInstruction

program.push_back(SetToolInstruction(5))
```

### Iteration and dispatch

When iterating a `CompositeInstruction`, each child is an `InstructionPoly`.
Use the `is*Instruction` predicates to dispatch and `as*Instruction` to obtain
the concrete value.

```python
for i in range(len(program)):
    child = program[i]
    if child.isMoveInstruction():
        mi = child.asMoveInstruction()
    elif child.isWaitInstruction():
        wi = child.asWaitInstruction()
    elif child.isTimerInstruction():
        ti = child.asTimerInstruction()
    elif child.isSetAnalogInstruction():
        si = child.asSetAnalogInstruction()
    elif child.isSetDigitalInstruction():
        si = child.asSetDigitalInstruction()
    elif child.isSetToolInstruction():
        si = child.asSetToolInstruction()
    elif child.isCompositeInstruction():
        sub = child.asCompositeInstruction()
```

`as*Instruction` raises `RuntimeError` if the underlying type does not match —
always guard with the matching `is*Instruction` predicate.

### Traversal into child composites

`flatten`, `getFirst/LastInstruction`, `getFirst/LastMoveInstruction` and the counts
descend into child composites. They return references into the program: an edit
through a returned instruction changes the program, and the returned objects keep it
alive. A filter is any callable `filter(instruction, parent) -> bool`; `moveFilter`
keeps move instructions.

References returned by `flatten`, `__getitem__`, `front` and `back` dangle after
`insert`, `__delitem__`/erase or `push_back` on the composite; take a copy
(`copy.copy`) if you need to keep one.

```python
from tesseract_robotics.tesseract_command_language import moveFilter, toJointTrajectory

moves = program.flatten(moveFilter)        # every move, at any depth
first = program.getFirstMoveInstruction()  # MoveInstructionPoly, or None
trajectory = toJointTrajectory(program)    # tesseract_common.JointTrajectory
```

`toJointTrajectory`, `getJointPosition`, `formatJointPosition`, `clampToJointLimits`
and the other functions of `tesseract/command_language/utils.h` take a `WaypointPoly`.
The in-place ones (`formatJointPosition`, `setJointPosition`, `clampToJointLimits`)
edit the `WaypointPoly` passed in, for example `move.getWaypoint()`.

### Equality, UUIDs and copies

Waypoints, instructions and their Polys compare by value (`==`) and are unhashable.
Every instruction carries a UUID and a parent UUID, read and set as canonical strings
(`getUUID`, `setUUID`, `getParentUUID`, `setParentUUID`, `regenerateUUID`; a malformed
string raises `ValueError`). `MoveInstruction` equality compares both UUIDs, so two
move instructions built separately from the same arguments are unequal until the
UUIDs are copied over. `copy.copy` and `copy.deepcopy` copy any of these types;
both give an independent copy that keeps the UUIDs.

## Profiles

### ProfileDictionary

Container for planner profiles.

```python
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLRealVectorMoveProfile

profiles = ProfileDictionary()

# addProfile(namespace, profile name, profile) takes a profile from any planner module
ompl_profile = OMPLRealVectorMoveProfile()
profiles.addProfile("OMPLMotionPlannerTask", "DEFAULT", ompl_profile)
assert profiles.hasProfile(
    OMPLRealVectorMoveProfile.createKey(), "OMPLMotionPlannerTask", "DEFAULT"
)
```

## Type Erasure Helpers

The command language uses type erasure. The Poly constructors wrap a concrete type; the
`as…()` methods unwrap it.

### Wrapping (concrete → polymorphic)

```python
# Waypoints
StateWaypointPoly(wp)
CartesianWaypointPoly(wp)
JointWaypointPoly(wp)

# Instructions
MoveInstructionPoly(instr)

# AnyPoly (for data storage; import from tesseract_task_composer)
AnyPoly_wrap_CompositeInstruction(program)
AnyPoly_wrap_ProfileDictionary(profiles)
```

`CompositeInstruction.push_back` accepts any concrete instruction type
(`MoveInstructionPoly`, `CompositeInstruction`, `WaitInstruction`,
`TimerInstruction`, `SetAnalogInstruction`, `SetDigitalInstruction`,
`SetToolInstruction`) directly — implicit conversion to `InstructionPoly`
removes the need for an explicit wrap call.

### Unwrapping (polymorphic → concrete)

```python
# Waypoints
wp_poly.asStateWaypoint()
wp_poly.asCartesianWaypoint()
wp_poly.asJointWaypoint()

# Instructions
instr_poly.asMoveInstruction()
instr_poly.asWaitInstruction()
instr_poly.asTimerInstruction()
instr_poly.asSetAnalogInstruction()
instr_poly.asSetDigitalInstruction()
instr_poly.asSetToolInstruction()

# AnyPoly (import from tesseract_task_composer)
AnyPoly_as_CompositeInstruction(any_poly)
```

`wp_poly.as…Waypoint()` returns a reference into the waypoint: an edit through it
changes the waypoint, and `copy.copy(wp_poly.asJointWaypoint())` gives an independent
copy. `instr_poly.as…Instruction()` returns a copy. Each raises `RuntimeError` when
the stored type does not match.

## Complete Example

```python
from tesseract_robotics.tesseract_command_language import (
    StateWaypoint, StateWaypointPoly,
    CartesianWaypoint, CartesianWaypointPoly,
    MoveInstruction, MoveInstructionType,
    MoveInstructionPoly,
    CompositeInstruction,
)
from tesseract_robotics.tesseract_common import ManipulatorInfo, Isometry3d
import numpy as np

# Setup
joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
start_joints = np.zeros(6)

goal_mat = np.eye(4)
goal_mat[:3, 3] = [0.5, 0.2, 0.3]
goal_pose = Isometry3d(goal_mat)

# Create waypoints
start_wp = StateWaypointPoly(
    StateWaypoint(joint_names, start_joints)
)
goal_wp = CartesianWaypointPoly(
    CartesianWaypoint(goal_pose)
)

# Create instructions (wrap in MoveInstructionPoly before appending)
start_instr = MoveInstructionPoly(
    MoveInstruction(start_wp, MoveInstructionType.FREESPACE, "DEFAULT")
)
goal_instr = MoveInstructionPoly(
    MoveInstruction(goal_wp, MoveInstructionType.LINEAR, "DEFAULT")
)

# Create program
program = CompositeInstruction("DEFAULT")

manip_info = ManipulatorInfo()
manip_info.manipulator = "manipulator"
manip_info.tcp_frame = "tool0"
manip_info.working_frame = "base_link"
program.setManipulatorInfo(manip_info)

program.push_back(start_instr)
program.push_back(goal_instr)

# Program is ready for TaskComposer
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_command_language._tesseract_command_language
    options:
      show_root_heading: false
      show_source: false
      members_order: source
