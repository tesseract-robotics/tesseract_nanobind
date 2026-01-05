# tesseract_robotics.tesseract_motion_planners_simple

Simple joint-space interpolation planner.

## Overview

The Simple planner performs linear interpolation in joint space.
No collision checking - use only when path is known to be safe.

```python
from tesseract_robotics.tesseract_motion_planners_simple import (
    SimpleMotionPlanner,
    generateInterpolatedProgram,
)
```

## SimpleMotionPlanner

```python
from tesseract_robotics.tesseract_motion_planners_simple import SimpleMotionPlanner
from tesseract_robotics.tesseract_motion_planners import PlannerRequest

planner = SimpleMotionPlanner()

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)
```

## Utility Functions

### generateInterpolatedProgram

Interpolate a program to add intermediate waypoints.

```python
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

# Add waypoints between existing ones
interpolated = generateInterpolatedProgram(
    program,
    env,
    profiles,
    max_step=0.1  # max joint step between waypoints
)
```

## Use Cases

- **Initial seeding**: Generate dense trajectory for TrajOpt
- **Safe paths**: When you know the path is collision-free
- **Testing**: Quick trajectory generation for debugging
- **Time parameterization input**: Dense trajectory for velocity planning

## Example

```python
from tesseract_robotics.tesseract_motion_planners_simple import SimpleMotionPlanner
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

# Simple planner needs minimal setup
profiles = ProfileDictionary()

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

planner = SimpleMotionPlanner()
response = planner.solve(request)

if response.successful:
    # Result is linearly interpolated joint trajectory
    trajectory = response.results
```

## Limitations

- **No collision checking**: Path may collide with obstacles
- **Joint-space only**: Cartesian interpolation not supported
- **No optimization**: Just linear interpolation

## Tips

1. **Use as preprocessing** for optimization-based planners
2. **Verify collision** after planning with simple planner
3. **Good for short motions** where collision is unlikely

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners_simple._tesseract_motion_planners_simple
    options:
      show_root_heading: false
      show_source: false
      members_order: source
