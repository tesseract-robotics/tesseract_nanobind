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

planner = DescartesMotionPlannerD()

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)
```

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
