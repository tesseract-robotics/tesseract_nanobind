# tesseract_robotics.tesseract_motion_planners

Base motion planner types and request/response structures.

## Overview

This module provides the base interfaces for motion planners. Specific planners
(OMPL, TrajOpt, etc.) are in separate modules.

## PlannerRequest

Input to a motion planner.

```python
from tesseract_robotics.tesseract_motion_planners import PlannerRequest

request = PlannerRequest()
request.env = env                    # Environment
request.instructions = program       # CompositeInstruction
request.profiles = profile_dict      # ProfileDictionary
request.plan_profile_remapping = {}  # Optional profile overrides
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `env` | `Environment` | Robot environment |
| `instructions` | `CompositeInstruction` | Motion program |
| `profiles` | `ProfileDictionary` | Planner profiles |
| `plan_profile_remapping` | `dict` | Profile name overrides |

## PlannerResponse

Output from a motion planner.

```python
from tesseract_robotics.tesseract_motion_planners import PlannerResponse

response = planner.solve(request)

if response.successful:
    result = response.results  # CompositeInstruction with solution
else:
    print(f"Failed: {response.message}")
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `successful` | `bool` | Planning succeeded |
| `message` | `str` | Status/error message |
| `results` | `CompositeInstruction` | Solved trajectory |

## MotionPlanner

Base class for all planners.

```python
from tesseract_robotics.tesseract_motion_planners import MotionPlanner

# MotionPlanner is abstract - use concrete implementations:
# - OMPLMotionPlanner
# - TrajOptMotionPlanner
# - DescartesMotionPlanner
# - SimpleMotionPlanner

response = planner.solve(request)
```

Every concrete planner derives from `MotionPlanner`, so `isinstance(planner, MotionPlanner)`
holds and `MotionPlanner.solve(planner, request)` dispatches to the concrete planner.

### Checking a request

`checkRequest(request)` is the C++ predicate; `checkRequestReason(request)` says why it fails.

```python
from tesseract_robotics.tesseract_motion_planners import MotionPlanner, PlannerRequest

request = PlannerRequest()
MotionPlanner.checkRequest(request)        # False
MotionPlanner.checkRequestReason(request)  # "PlannerRequest environment is nullptr"
```

`checkRequestReason` returns `""` for a valid request.

### assignSolution

Writes a joint solution into a move instruction in place, the way every planner fills its
results: a state waypoint, or with `format_result_as_input=True` the seed of a Cartesian
waypoint (the position of an unconstrained joint waypoint).

```python
MotionPlanner.assignSolution(move_instruction, joint_names, joint_values, False)
```

## Utility Functions

### assignCurrentStateAsSeed

Set current environment state as seed for planning.

```python
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed

# Modifies program in-place to use current state as seed
assignCurrentStateAsSeed(program, env)
```

### formatProgram

Reorders joint and state waypoints in place to the kinematic group's joint order.
Returns `True` if the program needed formatting.

```python
from tesseract_robotics.tesseract_motion_planners import formatProgram

changed = formatProgram(program, env)
```

### toToolpath

World-frame TCP poses of a program; the working frame in the `ManipulatorInfo` does not
change them. Takes a `CompositeInstruction`, `MoveInstructionPoly` or `InstructionPoly`;
returns a list with one `VectorIsometry3d` per sub-composite (one for a flat program).

```python
from tesseract_robotics.tesseract_motion_planners import toToolpath

toolpath = toToolpath(program, env)
first_tcp_pose = toolpath[0][0]  # Isometry3d
```

### contactCheckProgram

Collision-checks a planned program. Pass a discrete or continuous contact manager; the
C++ out-parameter comes back as the second element of the result, one `ContactResultMap`
per step.

```python
from tesseract_robotics.tesseract_collision import CollisionCheckConfig
from tesseract_robotics.tesseract_motion_planners import contactCheckProgram

results, contacts = contactCheckProgram(
    env.getDiscreteContactManager(), env.getStateSolver(), response.results, CollisionCheckConfig()
)
if results:
    print(results.condensedSummary())
```

### Robot configuration

`RobotConfig`, `getRobotConfig`, `getJointTurns` and `isWithinJointLimits`
classify a six-axis arm's joint solution by wrist, elbow and arm configuration. See
[Filtering IK solutions by configuration](../user-guide/kinematics.md#filtering-ik-solutions-by-configuration).

## Usage with TaskComposer

Motion planners are typically used via TaskComposer pipelines rather than directly:

```python
# Direct planner usage (lower level)
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLMotionPlanner

planner = OMPLMotionPlanner()
request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)

# TaskComposer usage (recommended)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory, TaskflowTaskComposerExecutor
)

# TaskComposer handles planner selection, sequencing, and post-processing
```

## Planner Modules

| Module | Planner | Description |
|--------|---------|-------------|
| `tesseract_motion_planners_ompl` | OMPL | Sampling-based (RRT, PRM, etc.) |
| `tesseract_motion_planners_trajopt` | TrajOpt | Trajectory optimization |
| `tesseract_motion_planners_descartes` | Descartes | Graph-based Cartesian |
| `tesseract_motion_planners_simple` | Simple | Joint interpolation |

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners
    options:
      show_root_heading: false
      show_source: false
      members_order: source
