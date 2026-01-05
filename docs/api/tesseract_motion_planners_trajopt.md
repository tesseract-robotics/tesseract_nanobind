# tesseract_robotics.tesseract_motion_planners_trajopt

TrajOpt trajectory optimization planner.

## Overview

TrajOpt optimizes trajectories for smoothness while avoiding collisions.
Best for Cartesian paths and trajectory refinement.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptMotionPlanner,
    TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile,
    TrajOptPlanProfile, TrajOptCompositeProfile,
    CollisionCostConfig, CollisionConstraintConfig,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)
```

## TrajOptMotionPlanner

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptMotionPlanner
from tesseract_robotics.tesseract_motion_planners import PlannerRequest

planner = TrajOptMotionPlanner()

request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

response = planner.solve(request)
```

## Profiles

### TrajOptDefaultPlanProfile

Per-waypoint configuration.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultPlanProfile

profile = TrajOptDefaultPlanProfile()

# Cartesian waypoint config
profile.cartesian_coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0])  # xyz, rpy weights

# Joint waypoint config
profile.joint_coeff = np.ones(6)  # weight per joint
```

### TrajOptDefaultCompositeProfile

Trajectory-wide configuration.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultCompositeProfile

profile = TrajOptDefaultCompositeProfile()

# Collision avoidance
profile.collision_cost_config.enabled = True
profile.collision_cost_config.safety_margin = 0.025
profile.collision_cost_config.coeff = 20.0

profile.collision_constraint_config.enabled = False  # use cost, not constraint

# Smoothness costs
profile.smooth_velocities = True
profile.smooth_accelerations = True
profile.smooth_jerks = False

profile.velocity_coeff = np.ones(6)
profile.acceleration_coeff = np.ones(6)

# Optimization settings
profile.longest_valid_segment_length = 0.01
```

## Collision Configuration

### CollisionCostConfig

Collision as soft cost (allows slight violations).

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import CollisionCostConfig

config = CollisionCostConfig()
config.enabled = True
config.safety_margin = 0.025    # distance threshold
config.coeff = 20.0             # cost weight
config.type = CollisionEvaluatorType.DISCRETE  # or LVS_DISCRETE, CONTINUOUS
```

### CollisionConstraintConfig

Collision as hard constraint (must satisfy).

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import CollisionConstraintConfig

config = CollisionConstraintConfig()
config.enabled = True
config.safety_margin = 0.01
config.coeff = 10.0
```

### CollisionEvaluatorType

| Type | Description | Speed |
|------|-------------|-------|
| `DISCRETE` | Check at waypoints only | Fast |
| `LVS_DISCRETE` | Interpolate between waypoints | Medium |
| `LVS_CONTINUOUS` | Swept volume check | Slow |
| `CONTINUOUS` | Full continuous collision | Slowest |

## Waypoint Configurations

### TrajOptCartesianWaypointConfig

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptCartesianWaypointConfig

config = TrajOptCartesianWaypointConfig()
config.enabled = True
config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0])
config.lower_tolerance = np.zeros(6)
config.upper_tolerance = np.zeros(6)
```

### TrajOptJointWaypointConfig

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptJointWaypointConfig

config = TrajOptJointWaypointConfig()
config.enabled = True
config.coeff = np.ones(6)
config.lower_tolerance = np.zeros(6)
config.upper_tolerance = np.zeros(6)
```

## Adding Profiles

```python
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

# Plan profile (per waypoint)
plan_profile = TrajOptDefaultPlanProfile()
plan_profile.cartesian_coeff = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# Composite profile (trajectory-wide)
composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.collision_cost_config.enabled = True
composite_profile.collision_cost_config.safety_margin = 0.025
composite_profile.smooth_velocities = True

# Add to dictionary
profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "DEFAULT", composite_profile)
```

## Complete Example

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptMotionPlanner, TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile, ProfileDictionary_addTrajOptCompositeProfile,
)
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_command_language import ProfileDictionary
import numpy as np

# Configure profiles
plan_profile = TrajOptDefaultPlanProfile()
plan_profile.cartesian_coeff = np.array([10, 10, 10, 5, 5, 5])

composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.collision_cost_config.enabled = True
composite_profile.collision_cost_config.safety_margin = 0.025
composite_profile.collision_cost_config.coeff = 20.0
composite_profile.smooth_velocities = True
composite_profile.velocity_coeff = np.ones(6)

profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "DEFAULT", composite_profile)

# Create request
request = PlannerRequest()
request.env = env
request.instructions = program
request.profiles = profiles

# Solve
planner = TrajOptMotionPlanner()
response = planner.solve(request)

if response.successful:
    trajectory = response.results
else:
    print(f"TrajOpt failed: {response.message}")
```

## Logging

Control TrajOpt logging via environment variable:

```bash
export TRAJOPT_LOG_THRESH=ERROR  # FATAL, ERROR, WARN, INFO, DEBUG, TRACE
```

## Tips

1. **Use as refinement** - TrajOpt works best refining OMPL paths
2. **Start with collision cost** - easier to tune than hard constraints
3. **Increase safety_margin** if collisions occur
4. **Use LVS_DISCRETE** for better collision coverage
5. **Tune coefficients** - higher = stricter enforcement
