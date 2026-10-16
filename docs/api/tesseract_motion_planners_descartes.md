# tesseract_robotics.tesseract_motion_planners_descartes

Descartes graph-based Cartesian planner.

## Overview

Descartes solves dense Cartesian paths by building a graph of IK solutions
and finding the optimal path through it. Best for industrial toolpaths
(welding, milling) where every waypoint must be reached.

```python
from tesseract_robotics.tesseract_motion_planners_descartes import (
    DescartesMotionPlannerD,
    DescartesDefaultPlanProfileD,
    DescartesLadderGraphSolverProfileD,
)
```

## DescartesMotionPlannerD

```python
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesMotionPlannerD
from tesseract_robotics.tesseract_motion_planners import PlannerRequest

planner = DescartesMotionPlannerD("DescartesMotionPlannerTask")

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)
```

`DescartesMotionPlannerD` is a `tesseract_motion_planners.MotionPlanner`: `getName`, `solve`,
`terminate`, `clear` and `clone` are the base class's, and `solve` releases the GIL.

## Profiles

### DescartesDefaultPlanProfileD

Per-waypoint configuration.

```python
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesDefaultPlanProfileD

profile = DescartesDefaultPlanProfileD()

# Target pose sampler settings
# Configure how many IK solutions to sample per waypoint
```

### DescartesLadderGraphSolverProfileD

Solver configuration for the ladder graph.

```python
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesLadderGraphSolverProfileD

profile = DescartesLadderGraphSolverProfileD()

# Graph construction and search settings
```

Both profiles compare by value (`==` compares every field, as the C++ `operator==` does) and
are unhashable. `DescartesMoveProfileD.createKey()` and `DescartesSolverProfileD.createKey()`
return the key every profile of that kind carries, the key `ProfileDictionary` looks up:

## Inspecting the samplers

The move profile's `create*` hooks return what the planner builds for one waypoint. `str()` on
the waypoint sampler shows its working frame, TCP frame, target pose and TCP offset:

```python
sampler = DescartesDefaultMoveProfileD().createWaypointSampler(move, manip_info, env)
print(sampler)          # Working Frame: base_link, TCP Frame: tool0 ...
samples = sampler.sample()   # list[DescartesStateSampleD], one per IK solution
```

`DescartesStateSampleD(state)` builds a sample with cost 0, as `DescartesStateSampleD(state, 0.0)`.

## Tool-pose samplers

The functions behind `DescartesDefaultMoveProfileD.target_pose_sample_*` turn one tool pose into
candidate poses, returned as a `tesseract_common.VectorIsometry3d`:

| Function | Samples |
|---|---|
| `sampleFixed(tool_pose)` | the pose itself |
| `sampleToolAxis(tool_pose, axis, resolution, minimum, maximum)` | rotations of the pose about `axis` |
| `sampleToolXAxis` / `sampleToolYAxis` / `sampleToolZAxis(tool_pose, resolution, minimum, maximum)` | rotations about the tool's X / Y / Z axis |

The angles run from `minimum` in steps of `resolution` while they do not exceed `maximum`
(equal within tolerance counts), so both ends are included when the range is a whole number of
steps:

```python
import numpy as np
from tesseract_robotics.tesseract_motion_planners_descartes import sampleToolZAxis

poses = sampleToolZAxis(tool_pose, resolution=np.pi / 2, minimum=-np.pi, maximum=np.pi / 2)
len(poses)  # 4: -pi, -pi/2, 0, pi/2
```

They are useful without the planner, for example to seed IK over a tool axis the process is
indifferent to.

## How It Works

1. **Sample waypoints**: For each Cartesian waypoint, compute all IK solutions
2. **Build graph**: Create nodes for each solution, edges between consecutive waypoints
3. **Edge costs**: Based on joint motion between solutions
4. **Find path**: Dijkstra's algorithm finds minimum-cost path

```
Waypoint 1      Waypoint 2      Waypoint 3
    ○──────────────○──────────────○
    │              │              │
    ○──────────────○──────────────○
    │              │              │
    ○──────────────○──────────────○
   (IK solutions)
```

## Use Cases

- **Welding**: Dense linear paths with tool orientation constraints
- **Milling**: Toolpaths from CAM software
- **Painting**: Spray patterns with coverage requirements
- **Inspection**: Camera positioning paths

## Limitations

- **Computationally expensive**: O(n × k²) where n=waypoints, k=IK solutions
- **Requires IK solutions**: Won't work if waypoints are unreachable
- **No obstacle avoidance**: Use with collision checking post-filter

## Tips

1. **Use for dense Cartesian paths** where every point matters
2. **Combine with TrajOpt** for smoothing afterward
3. **Reduce waypoint density** if planning is too slow
4. **Check reachability** before planning - Descartes fails if any waypoint is unreachable

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners_descartes._tesseract_motion_planners_descartes
    options:
      show_root_heading: false
      show_source: false
      members_order: source
