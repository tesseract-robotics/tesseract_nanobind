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

`TrajOptMotionPlanner` derives from `MotionPlanner` (`tesseract_motion_planners`).

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

# Joint waypoint config
profile.joint_constraint_config.enabled = True
profile.joint_constraint_config.coeff = np.ones(6)  # weight per joint
```

### TrajOptDefaultCompositeProfile

Trajectory-wide configuration.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultCompositeProfile
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

profile = TrajOptDefaultCompositeProfile()

# Collision avoidance — collision_*_config is a TrajOptCollisionConfig
profile.collision_cost_config.enabled = True
profile.collision_cost_config.contact_manager_config.default_margin = 0.025
profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(20.0)
profile.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE

profile.collision_constraint_config.enabled = False  # use cost, not constraint

# Smoothness costs
profile.smooth_velocities = True
profile.smooth_accelerations = True
profile.smooth_jerks = False

profile.velocity_coeff = np.ones(6)
profile.acceleration_coeff = np.ones(6)
```

The composite profile turns a collision config's longest-valid-segment
fraction and length into the step its collision terms check at. The static
`computeLongestValidSegmentLength` gives the same number from Python:
`min(fraction * extent, length)` when both are positive, the positive one
otherwise, and `0.01 * extent` when neither is, with
`extent = ||upper - lower||` over the joint limits.

```python
limits = env.getJointGroup("manipulator").getLimits().joint_limits  # n x 2
cfg = profile.collision_cost_config.collision_check_config
step = TrajOptDefaultCompositeProfile.computeLongestValidSegmentLength(
    limits, cfg.longest_valid_segment_fraction, cfg.longest_valid_segment_length
)
```

### TrajOptOSQPSolverProfile

The solver profile holds the SQP parameters (`opt_params`, a
`BasicTrustRegionSQPParameters`) and the QP solver's `settings`, an
`OSQPSettings`. A new profile holds trajopt's OSQP defaults
(`eps_abs=1e-4`, `eps_rel=1e-6`, `max_iter=8192`, polishing and adaptive rho
on); `OSQPSettings()` on its own holds OSQP's defaults.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addTrajOptSolverProfile, TrajOptOSQPSolverProfile,
)

solver = TrajOptOSQPSolverProfile()
solver.opt_params.max_iter = 200
settings = solver.settings
settings.eps_abs = 1e-5
settings.time_limit = 0.5
solver.settings = settings

ProfileDictionary_addTrajOptSolverProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", solver)
```

`createOptimizationParameters()` returns the SQP parameters the planner uses
for that profile (a copy of `opt_params`). The OSQP enum fields
`linsys_solver` and `cg_precond` are not bound.

## Collision Configuration

### TrajOptCollisionConfig

Replaces the 0.33 `CollisionCostConfig` / `CollisionConstraintConfig`.
See [changes](../changes.md) for the full migration notes.

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptCollisionConfig
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

# Constructor: TrajOptCollisionConfig(margin, coeff) or default
config = TrajOptCollisionConfig(0.025, 20.0)  # margin=2.5cm, coeff=20
config.enabled = True
config.collision_margin_buffer = 0.005  # collect contacts 5mm past the margin
config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
config.collision_check_config.longest_valid_segment_length = 0.05  # for LVS modes
```

The margin (`contact_manager_config.default_margin`, the constructor's first argument) is
the clearance the term drives toward. `collision_margin_buffer` only widens contact
collection and adds no error. A default-constructed config has no margin: the term engages
only at contact and TrajOpt settles paths on the obstacle's surface.

### CollisionEvaluatorType

!!! note "Module Location"
    `CollisionEvaluatorType` lives in
    `tesseract_robotics.tesseract_collision` (re-exported from the TrajOpt
    module for convenience).

| Type | Description | Speed |
|------|-------------|-------|
| `DISCRETE` | Check at waypoints only | Fast |
| `LVS_DISCRETE` | Interpolate between waypoints, discrete checks | Medium |
| `LVS_CONTINUOUS` | Swept volume check, LVS interpolation | Slow |
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

## Value equality

The waypoint configs, the default move/composite profiles,
`TrajOptOSQPSolverProfile`, `BasicTrustRegionSQPParameters` and
`OSQPSettings` compare by value (`==`, `!=`) and are unhashable.

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
composite_profile.collision_cost_config.contact_manager_config.default_margin = 0.025
composite_profile.collision_cost_config.collision_coeff_data.setDefaultCollisionCoeff(20.0)
composite_profile.collision_cost_config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
composite_profile.smooth_velocities = True

# Add to dictionary
profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", composite_profile)
```

`ProfileDictionary.hasProfile` / `getProfile` take a profile key. Each profile
kind has one: `TrajOptMoveProfile.createKey()`,
`TrajOptCompositeProfile.createKey()` and `TrajOptSolverProfile.createKey()`,
inherited by their concrete profiles.

```python
composite = profiles.getProfile(
    TrajOptDefaultCompositeProfile.createKey(), "TrajOptMotionPlannerTask", "DEFAULT"
)
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

# Configure profiles
plan_profile = TrajOptDefaultPlanProfile()
plan_profile.cartesian_constraint_config.enabled = True
plan_profile.cartesian_constraint_config.coeff = np.array([10, 10, 10, 5, 5, 5])

composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.collision_cost_config.enabled = True
composite_profile.collision_cost_config.contact_manager_config.default_margin = 0.025
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
3. **Increase the margin** (`contact_manager_config.default_margin`) if paths pass too close; the buffer only widens contact collection
4. **Use LVS_DISCRETE** for better collision coverage
5. **Tune coefficients** - higher = stricter enforcement

## Auto-generated API Reference

::: tesseract_robotics.tesseract_motion_planners_trajopt._tesseract_motion_planners_trajopt
    options:
      show_root_heading: false
      show_source: false
      members_order: source
