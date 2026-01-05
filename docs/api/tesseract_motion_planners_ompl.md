# tesseract_robotics.tesseract_motion_planners_ompl

OMPL (Open Motion Planning Library) planner bindings.

## Overview

OMPL provides sampling-based planners for complex environments where
optimization-based planners may get stuck in local minima.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLPlanProfile, OMPLRealVectorPlanProfile,
    RRTConnectConfigurator, RRTstarConfigurator, SBLConfigurator,
    ProfileDictionary_addOMPLProfile,
)
```

## OMPLMotionPlanner

The OMPL motion planner.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLMotionPlanner
from tesseract_robotics.tesseract_motion_planners import PlannerRequest

planner = OMPLMotionPlanner()

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)
```

## Profiles

### OMPLRealVectorPlanProfile

Configure OMPL planning behavior.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLRealVectorPlanProfile

profile = OMPLRealVectorPlanProfile()

# Planning time
profile.planning_time = 5.0  # seconds

# Simplification
profile.simplify = True

# State validation resolution
profile.collision_check_config.longest_valid_segment_length = 0.01

# Add planner configurator
profile.planners = [RRTConnectConfigurator()]
```

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `planning_time` | `float` | 5.0 | Max planning time (seconds) |
| `simplify` | `bool` | True | Simplify path after planning |
| `optimize` | `bool` | True | Run path optimization |
| `planners` | `list` | `[RRTConnect]` | Planner configurators |

## Planner Configurators

### RRTConnectConfigurator

Rapidly-exploring Random Tree Connect - fast, bidirectional.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import RRTConnectConfigurator

config = RRTConnectConfigurator()
config.range = 0.0  # 0 = auto

profile.planners = [config]
```

### RRTstarConfigurator

RRT* - asymptotically optimal.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import RRTstarConfigurator

config = RRTstarConfigurator()
config.range = 0.0
config.goal_bias = 0.05

profile.planners = [config]
```

### SBLConfigurator

Single-query Bidirectional Lazy PRM.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import SBLConfigurator

config = SBLConfigurator()
config.range = 0.0

profile.planners = [config]
```

## OMPLPlannerType

Available OMPL planners:

| Type | Description |
|------|-------------|
| `RRTConnect` | Bidirectional RRT (default, fast) |
| `RRT` | Basic RRT |
| `RRTstar` | Asymptotically optimal RRT |
| `PRM` | Probabilistic Roadmap |
| `PRMstar` | Optimal PRM |
| `SBL` | Single-query Bidirectional Lazy |
| `EST` | Expansive Space Trees |
| `LBTRRT` | Lower Bound Tree RRT |
| `BiTRRT` | Bidirectional Transition RRT |

## Adding Profiles

```python
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLRealVectorPlanProfile, ProfileDictionary_addOMPLProfile,
    RRTConnectConfigurator
)

# Create profile
profile = OMPLRealVectorPlanProfile()
profile.planning_time = 10.0
profile.planners = [RRTConnectConfigurator()]

# Add to dictionary
profiles = ProfileDictionary()
ProfileDictionary_addOMPLProfile(profiles, "DEFAULT", profile)

# Use with planner request
request.profiles = profiles
```

## Complete Example

```python
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner, OMPLRealVectorPlanProfile,
    RRTConnectConfigurator, ProfileDictionary_addOMPLProfile
)
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

# Configure OMPL profile
profile = OMPLRealVectorPlanProfile()
profile.planning_time = 5.0
profile.simplify = True
profile.planners = [RRTConnectConfigurator()]

profiles = ProfileDictionary()
ProfileDictionary_addOMPLProfile(profiles, "DEFAULT", profile)

# Create request
request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

# Solve
planner = OMPLMotionPlanner()
response = planner.solve(request)

if response.successful:
    trajectory = response.results
    print(f"Found path with {len(trajectory)} waypoints")
else:
    print(f"Planning failed: {response.message}")
```

## Tips

1. **RRTConnect** is usually the best starting point - fast and reliable
2. **Increase planning_time** for complex environments
3. **Use simplify=True** to reduce waypoints
4. **Lower collision resolution** (`longest_valid_segment_length`) for narrow passages
5. **OMPL finds paths, not smooth trajectories** - use TrajOpt for smoothing
