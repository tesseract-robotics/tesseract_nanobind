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
    StateWaypoint, StateWaypointPoly, StateWaypointPoly_wrap_StateWaypoint
)
import numpy as np

joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
joint_values = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

# Create waypoint
wp = StateWaypoint(joint_names, joint_values)

# Wrap in polymorphic type (required for instructions)
wp_poly = StateWaypointPoly_wrap_StateWaypoint(wp)

# Access values
print(f"Names: {wp.getNames()}")
print(f"Position: {wp.getPosition()}")
```

### CartesianWaypoint

Cartesian-space target (pose).

```python
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint, CartesianWaypointPoly, CartesianWaypointPoly_wrap_CartesianWaypoint
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
wp_poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
```

### JointWaypoint

Named joint positions (subset of joints).

```python
from tesseract_robotics.tesseract_command_language import (
    JointWaypoint, JointWaypointPoly, JointWaypointPoly_wrap_JointWaypoint
)

names = ["joint_1", "joint_3"]
values = np.array([0.5, -0.5])

wp = JointWaypoint(names, values)
wp_poly = JointWaypointPoly_wrap_JointWaypoint(wp)
```

## Instructions

### MoveInstruction

Single motion command.

```python
from tesseract_robotics.tesseract_command_language import (
    MoveInstruction, MoveInstructionPoly, MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType
)

# Create move instruction
instr = MoveInstruction(wp_poly, MoveInstructionType.FREESPACE, "DEFAULT")

# With specific profile
instr = MoveInstruction(wp_poly, MoveInstructionType.LINEAR, "my_profile")

# Wrap in polymorphic type
instr_poly = MoveInstructionPoly_wrap_MoveInstruction(instr)

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
    CompositeInstruction, CompositeInstructionOrder
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
program.appendMoveInstruction(move1)
program.appendMoveInstruction(move2)

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

## Profiles

### ProfileDictionary

Container for planner profiles.

```python
from tesseract_robotics.tesseract_command_language import (
    ProfileDictionary, ProfileDictionary_addProfile
)

profiles = ProfileDictionary()

# Add profiles (type-specific functions)
from tesseract_robotics.tesseract_motion_planners_ompl import (
    ProfileDictionary_addOMPLProfile
)
ProfileDictionary_addOMPLProfile(profiles, "DEFAULT", ompl_profile)
```

## Type Erasure Helpers

The command language uses type erasure. Use these functions to wrap/unwrap types:

### Wrapping (concrete → polymorphic)

```python
# Waypoints
StateWaypointPoly_wrap_StateWaypoint(wp)
CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
JointWaypointPoly_wrap_JointWaypoint(wp)

# Instructions
MoveInstructionPoly_wrap_MoveInstruction(instr)

# AnyPoly (for data storage)
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
WaypointPoly_as_StateWaypointPoly(wp_poly)
WaypointPoly_as_CartesianWaypointPoly(wp_poly)
WaypointPoly_as_JointWaypointPoly(wp_poly)

# Instructions
InstructionPoly_as_MoveInstructionPoly(instr_poly)
InstructionPoly_as_WaitInstruction(instr_poly)
InstructionPoly_as_TimerInstruction(instr_poly)
InstructionPoly_as_SetAnalogInstruction(instr_poly)
InstructionPoly_as_SetDigitalInstruction(instr_poly)
InstructionPoly_as_SetToolInstruction(instr_poly)

# AnyPoly
AnyPoly_as_CompositeInstruction(any_poly)
```

These free functions mirror the `as*Instruction` methods on `InstructionPoly`
and raise `RuntimeError` when the underlying type does not match.

## Complete Example

```python
from tesseract_robotics.tesseract_command_language import (
    StateWaypoint, StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstruction, MoveInstructionType,
    MoveInstructionPoly_wrap_MoveInstruction,
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
start_wp = StateWaypointPoly_wrap_StateWaypoint(
    StateWaypoint(joint_names, start_joints)
)
goal_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(
    CartesianWaypoint(goal_pose)
)

# Create instructions (wrap in MoveInstructionPoly before appending)
start_instr = MoveInstructionPoly_wrap_MoveInstruction(
    MoveInstruction(start_wp, MoveInstructionType.FREESPACE, "DEFAULT")
)
goal_instr = MoveInstructionPoly_wrap_MoveInstruction(
    MoveInstruction(goal_wp, MoveInstructionType.LINEAR, "DEFAULT")
)

# Create program
program = CompositeInstruction("DEFAULT")

manip_info = ManipulatorInfo()
manip_info.manipulator = "manipulator"
manip_info.tcp_frame = "tool0"
manip_info.working_frame = "base_link"
program.setManipulatorInfo(manip_info)

program.appendMoveInstruction(start_instr)
program.appendMoveInstruction(goal_instr)

# Program is ready for TaskComposer
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_command_language._tesseract_command_language
    options:
      show_root_heading: false
      show_source: false
      members_order: source
