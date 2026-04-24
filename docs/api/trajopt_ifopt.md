# tesseract_robotics.trajopt_ifopt

Constraints, costs, and variables for the low-level SQP solver.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## Variables (0.34+)

The `JointPosition` class from 0.33 is removed. Variables now use a three-level hierarchy.

### Var

A single variable — typically one joint value at one waypoint.

### Node

Groups multiple `Var`s per waypoint (e.g., joints + velocities at one waypoint).

### NodesVariables

Container of `Node`s. Passed to `IfoptProblem`.

### createNodesVariables

Factory helper. Builds the full hierarchy from a list of initial states.

```python
from tesseract_robotics.trajopt_ifopt import Bounds, createNodesVariables

bounds = Bounds(-3.14, 3.14)
nodes_variables = createNodesVariables(
    "trajectory", joint_names, initial_states, bounds
)

# Iterate waypoints to pull Var refs for constraints
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
```

## Constraints

| Class | Purpose |
|---|---|
| `JointPosConstraint` | Target joint values at a waypoint |
| `JointVelConstraint` | Velocity limits across consecutive waypoints |
| `JointAccelConstraint` | Acceleration limits |
| `JointJerkConstraint` | Jerk limits |
| `CartPosConstraint` | TCP pose at a waypoint |
| `CartLineConstraint` | TCP on a line segment between two poses |
| `DiscreteCollisionConstraint` | Collision at a single waypoint |
| `DiscreteCollisionNumericalConstraint` | Alternative collision jacobian (numerical) |
| `ContinuousCollisionConstraint` | Collision between two consecutive waypoints |
| `InverseKinematicsConstraint` | IK-based constraint |

0.34 constraint constructors take `Var` references directly (not
`JointPosition` lists). See [changes](../changes.md) for the migration details.

## Collision Evaluators

| Class | Pairs with |
|---|---|
| `SingleTimestepCollisionEvaluator` | `DiscreteCollisionConstraint` |
| `LVSDiscreteCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS discrete mode) |
| `LVSContinuousCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS continuous mode) |
| `DiscreteCollisionEvaluator` | Base class |
| `ContinuousCollisionEvaluator` | Base class |

`CollisionCache` was removed in 0.34 — caching is internal to each evaluator.

## Info Structs (for Cartesian / IK constraints)

| Struct | Used by |
|---|---|
| `CartLineInfo` | `CartLineConstraint` |
| `InverseKinematicsInfo` | `InverseKinematicsConstraint` |

`CartPosInfo` from 0.33 is gone — `CartPosConstraint` now takes parameters
directly (see [changes](../changes.md)).

## Config / Bounds

- `TrajOptCollisionConfig(margin, coeff)` — re-exported here and from
  `tesseract_motion_planners_trajopt`.
- `CollisionCoeffData` — per-pair collision coefficient data.
- `Bounds(lower, upper)` — single-variable bounds.
- Enums: `BoundsType`, `RangeBoundHandling`.

## Utilities

- `interpolate(start, end, steps)` — linear joint interpolation.
- `toBounds(joint_limits)` — convert a limits matrix into a `Bounds` list.

## Module API

::: tesseract_robotics.trajopt_ifopt
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3

## See also

- [Low-Level SQP API](../user-guide/low-level-sqp.md) — user-guide walkthrough
- [0.33 → 0.34 migration](../changes.md) — if you're porting from the old `ifopt` module
