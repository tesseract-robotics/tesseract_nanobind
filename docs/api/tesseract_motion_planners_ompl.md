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

The OMPL motion planner. It derives from `MotionPlanner`
(`tesseract_motion_planners`), so it is accepted wherever a `MotionPlanner` is.

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

Each `OMPLPlannerType` has a configurator class, `<type>Configurator`. Each
one default-constructs with OMPL's defaults; `getType()` returns its
`OMPLPlannerType`. A `range` of 0 lets OMPL pick the step size.

| `OMPLPlannerType` | Configurator | Fields |
|---|---|---|
| `SBL` | `SBLConfigurator` | `range` |
| `EST` | `ESTConfigurator` | `range`, `goal_bias` |
| `LBKPIECE1` | `LBKPIECE1Configurator` | `range`, `border_fraction`, `min_valid_path_fraction` |
| `BKPIECE1` | `BKPIECE1Configurator` | `range`, `border_fraction`, `failed_expansion_score_factor`, `min_valid_path_fraction` |
| `KPIECE1` | `KPIECE1Configurator` | `range`, `goal_bias`, `border_fraction`, `failed_expansion_score_factor`, `min_valid_path_fraction` |
| `BiTRRT` | `BiTRRTConfigurator` | `range`, `temp_change_factor`, `cost_threshold`, `init_temperature`, `frontier_threshold`, `frontier_node_ratio` |
| `RRT` | `RRTConfigurator` | `range`, `goal_bias` |
| `RRTConnect` | `RRTConnectConfigurator` | `range` |
| `RRTstar` | `RRTstarConfigurator` | `range`, `goal_bias`, `delay_collision_checking` |
| `TRRT` | `TRRTConfigurator` | `range`, `goal_bias`, `temp_change_factor`, `init_temperature`, `frontier_threshold`, `frontier_node_ratio` |
| `PRM` | `PRMConfigurator` | `max_nearest_neighbors` |
| `PRMstar` | `PRMstarConfigurator` | none |
| `LazyPRMstar` | `LazyPRMstarConfigurator` | none |
| `SPARS` | `SPARSConfigurator` | `max_failures`, `dense_delta_fraction`, `sparse_delta_fraction`, `stretch_factor` |

RRTConnect is the default and usually the fastest. The PRM family and the
asymptotically optimal planners (RRTstar, PRMstar) use the whole
`planning_time`.

```python
from tesseract_robotics.tesseract_motion_planners_ompl import (
    PRMstarConfigurator, RRTstarConfigurator,
)

config = RRTstarConfigurator()
config.goal_bias = 0.1

profile.solver_config.planners = [config, PRMstarConfigurator()]
```

`planning.profiles.create_ompl_planner_configurators` builds them by type
name and sets shared fields on the configurators that have them:

```python
from tesseract_robotics.planning.profiles import create_ompl_planner_configurators

profile.solver_config.planners = create_ompl_planner_configurators(
    planners=["RRTConnect", "KPIECE1"], num_planners=2, range=0.1
)
```

`create` (which builds the OMPL planner from an `ompl::base::SpaceInformation`)
is not bound: it takes and returns OMPL types.

## OMPLSolverConfig.planners

`solver_config.planners` is the list of configurators, one parallel planner
each. The default profile holds two `RRTConnectConfigurator`. The list is
copied on access; its configurators are shared with the config.
`cfg.planners.append(x)` changes the copy, not `cfg`, but
`cfg.planners[0].range = 0.5` changes the configurator `cfg` holds. Assign a
new list, or use `addPlanner` / `clearPlanners`, which change it in place. `len(cfg.planners)` replaces the removed `getNumPlanners()`.

```python
cfg = profile.solver_config
cfg.planners = [RRTConnectConfigurator()]  # replace
cfg.addPlanner(SBLConfigurator())          # append in place
assert len(cfg.planners) == 2
```

`OMPLSolverConfig` and `OMPLRealVectorMoveProfile` compare by value and are
unhashable. Upstream compares `planners` by count only, so two configs that
hold different configurator types of the same number compare equal.
`profile.createSolverConfig()` returns the solver config the planner uses for
that profile (a copy of `solver_config`).

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

`ProfileDictionary.hasProfile` / `getProfile` take a profile key.
`OMPLMoveProfile.createKey()`, inherited by every OMPL move profile, returns
it without building a profile:

```python
assert profiles.hasProfile(
    OMPLRealVectorPlanProfile.createKey(), "OMPLMotionPlannerTask", "DEFAULT"
)
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
