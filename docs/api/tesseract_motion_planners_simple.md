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

### Interpolation

The planner's interpolation functions need no planner state:

```python
import numpy as np
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_motion_planners_simple import interpolate

poses = interpolate(Isometry3d.Identity(), target_pose, 10)   # VectorIsometry3d, 11 poses
states = interpolate(np.zeros(6), q_goal, 10)                  # 6 x 11 array, one state per column
```

`interpolate_waypoint(start, stop, steps)` does the same for two `WaypointPoly`s.
`getInterpolatedInstructions(joint_names, states, base_instruction)` (and the overload that
also takes `poses`) turns the columns after the first into child move instructions of
`base_instruction`, ending with `base_instruction` itself.

## Profiles

The seven move profiles (`SimplePlannerFixedSize*MoveProfile`, `SimplePlannerLVS*MoveProfile`)
compare by value (`==`) and are unhashable. All of them register under the key of their
base, `SimplePlannerMoveProfile.createKey()`.

`generate` runs a profile on one segment without a planner: it returns the move
instructions `SimpleMotionPlanner.solve` would emit for `base_instruction`.

```python
from tesseract_robotics.tesseract_command_language import InstructionPoly
from tesseract_robotics.tesseract_motion_planners_simple import SimplePlannerLVSMoveProfile

profile = SimplePlannerLVSMoveProfile()
moves = profile.generate(start, start, goal, InstructionPoly(), env, manip_info)
```

`from_yaml` builds a profile from the same YAML keys the C++ profile reads:

```python
profile = SimplePlannerLVSMoveProfile.from_yaml("min_steps: 4\nmax_steps: 20")
```

Malformed YAML raises `ValueError`; a value of the wrong type raises the profile's
`RuntimeError`.

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
