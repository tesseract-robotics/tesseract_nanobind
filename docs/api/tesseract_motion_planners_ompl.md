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

Configure OMPL planning behavior. The profile has three sub-configs:
`solver_config`, `collision_check_config`, and `contact_manager_config`.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLRealVectorPlanProfile, RRTConnectConfigurator,
)

profile = OMPLRealVectorPlanProfile()

# Solver settings
profile.solver_config.planning_time = 5.0   # seconds
profile.solver_config.simplify = True       # simplify path after planning
profile.solver_config.optimize = True       # run path optimization

# Add planner configurator(s)
profile.solver_config.addPlanner(RRTConnectConfigurator())

# State-validation resolution (for collision checking)
profile.collision_check_config.longest_valid_segment_length = 0.01
```

| Attribute | Config | Description |
|---|---|---|
| `solver_config.planning_time` | solver | Max planning time (seconds) |
| `solver_config.simplify` | solver | Simplify path after planning |
| `solver_config.optimize` | solver | Run path optimization |
| `solver_config.max_solutions` | solver | Cap on returned paths |
| `collision_check_config.longest_valid_segment_length` | collision | State-validation resolution |
| `collision_check_config.type` | collision | `CollisionEvaluatorType` (discrete vs continuous) |

## Planner Configurators

### RRTConnectConfigurator

Rapidly-exploring Random Tree Connect - fast, bidirectional.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import RRTConnectConfigurator

config = RRTConnectConfigurator()
config.range = 0.0  # 0 = auto

profile.solver_config.addPlanner(config)
```

### RRTstarConfigurator

RRT* - asymptotically optimal.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import RRTstarConfigurator

config = RRTstarConfigurator()
config.range = 0.0
config.goal_bias = 0.05

profile.solver_config.addPlanner(config)
```

### SBLConfigurator

Single-query Bidirectional Lazy PRM.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import SBLConfigurator

config = SBLConfigurator()
config.range = 0.0

profile.solver_config.addPlanner(config)
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
profile.solver_config.planning_time = 10.0
profile.solver_config.addPlanner(RRTConnectConfigurator())

# Add to dictionary (dict, namespace, profile_name, profile)
profiles = ProfileDictionary()
ProfileDictionary_addOMPLProfile(profiles, "OMPLMotionPlannerTask", "DEFAULT", profile)

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
profile.solver_config.planning_time = 5.0
profile.solver_config.simplify = True
profile.solver_config.addPlanner(RRTConnectConfigurator())

profiles = ProfileDictionary()
ProfileDictionary_addOMPLProfile(profiles, "OMPLMotionPlannerTask", "DEFAULT", profile)

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

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners_ompl._tesseract_motion_planners_ompl
    options:
      show_root_heading: false
      show_source: false
      members_order: source
