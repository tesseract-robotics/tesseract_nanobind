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

## Utility Functions

### assignCurrentStateAsSeed

Set current environment state as seed for planning.

```python
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed

# Modifies program in-place to use current state as seed
assignCurrentStateAsSeed(program, env)
```

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
