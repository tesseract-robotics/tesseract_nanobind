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
    TrajOptCollisionConfig,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
```

!!! note "`TrajOptCollisionConfig` lives in `trajopt_ifopt`"
    It's defined in `tesseract_robotics.trajopt_ifopt` (trajopt_common) and
    re-exported from `tesseract_motion_planners_trajopt` for convenience. Import
    from either location.

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
profile.cartesian_constraint_config.enabled = True
profile.cartesian_constraint_config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0])  # xyz, rpy weights

# Or as cost instead of constraint
profile.cartesian_cost_config.enabled = True
profile.cartesian_cost_config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0])

# Joint waypoint config (uses joint_cost_config / joint_*_config too)
# (availability depends on profile subtype; see source for current attributes)
```

### TrajOptDefaultCompositeProfile

Trajectory-wide configuration.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultCompositeProfile
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

profile = TrajOptDefaultCompositeProfile()

# Collision avoidance (0.33 API: TrajOptCollisionConfig)
profile.collision_cost_config.enabled = True
profile.collision_cost_config.collision_margin_buffer = 0.025  # was safety_margin
profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(20.0)  # was .coeff
profile.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE

profile.collision_constraint_config.enabled = False  # use cost, not constraint

# Smoothness costs
profile.smooth_velocities = True
profile.smooth_accelerations = True
profile.smooth_jerks = False

profile.velocity_coeff = np.ones(6)
profile.acceleration_coeff = np.ones(6)
```

## Collision Configuration (0.33 API)

### TrajOptCollisionConfig

Replaces the old `CollisionCostConfig` and `CollisionConstraintConfig`.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptCollisionConfig
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

# Constructor: TrajOptCollisionConfig(margin, coeff) or default
config = TrajOptCollisionConfig(0.025, 20.0)  # margin=2.5cm, coeff=20
config.enabled = True
config.collision_margin_buffer = 0.005  # additional buffer beyond margin
config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
config.collision_check_config.longest_valid_segment_length = 0.05  # for LVS modes
```

### CollisionEvaluatorType

!!! note "Module Location"
    `CollisionEvaluatorType` moved to `tesseract_collision` in 0.33 API.

| Type | Description | Speed |
|------|-------------|-------|
| `DISCRETE` | Check at waypoints only (was `SINGLE_TIMESTEP`) | Fast |
| `LVS_DISCRETE` | Interpolate between waypoints | Medium |
| `LVS_CONTINUOUS` | Swept volume check (was `DISCRETE_CONTINUOUS`) | Slow |
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
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

# Plan profile (per waypoint)
plan_profile = TrajOptDefaultPlanProfile()
plan_profile.cartesian_constraint_config.enabled = True
plan_profile.cartesian_constraint_config.coeff = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# Composite profile (trajectory-wide)
composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.collision_cost_config.enabled = True
composite_profile.collision_cost_config.collision_margin_buffer = 0.025
composite_profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(20.0)
composite_profile.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
composite_profile.smooth_velocities = True

# Add to dictionary
profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", composite_profile)
```

## Complete Example

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptMotionPlanner, TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile, ProfileDictionary_addTrajOptCompositeProfile,
)
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
import numpy as np

# Configure profiles (0.33 API)
plan_profile = TrajOptDefaultPlanProfile()
plan_profile.cartesian_constraint_config.enabled = True
plan_profile.cartesian_constraint_config.coeff = np.array([10, 10, 10, 5, 5, 5])

composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.collision_cost_config.enabled = True
composite_profile.collision_cost_config.collision_margin_buffer = 0.025
composite_profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(20.0)
composite_profile.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
composite_profile.smooth_velocities = True
composite_profile.velocity_coeff = np.ones(6)

profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", composite_profile)

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
3. **Increase collision_margin_buffer** if collisions occur
4. **Use LVS_DISCRETE** for better collision coverage
5. **Tune coefficients** - higher = stricter enforcement

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners_trajopt._tesseract_motion_planners_trajopt
    options:
      show_root_heading: false
      show_source: false
      members_order: source
