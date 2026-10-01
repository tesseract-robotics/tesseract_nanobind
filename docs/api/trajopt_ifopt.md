# tesseract_robotics.trajopt_ifopt

Variables and constraint sets for the low-level SQP solver, and how each set becomes a
constraint or a cost of a `TrajOptQPProblem`.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## Variables (0.34+)

The `JointPosition` class from 0.33 is removed. Variables now use a three-level hierarchy.

### Var

A single variable — typically one joint value at one waypoint.

### Node

Groups multiple `Var`s per waypoint (e.g., joints + velocities at one waypoint).

### NodesVariables

Container of `Node`s. Passed to `TrajOptQPProblem`.

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

Every class below is a `ConstraintSet`. It goes into a `TrajOptQPProblem` either as a
constraint, with `addConstraintSet`, or as a cost, with `addCostSet(set, penalty_type)`. Every
class is accepted as a constraint. As a cost, the set's row bounds decide the penalty type:
`SQUARED` and `ABSOLUTE` need equality bounds on every row, `HINGE` one-sided bounds, and
`addCostSet` raises otherwise. There is no cost without a penalty type. The last column lists
what `addCostSet` accepts. On trajopt 0.35.0, as tesseract-robotics-nanobind 0.35.0.9 bundles it
(the first wheel that binds `TrajOptQPProblem`), the SQP models only `SQUARED` costs as it
charges them: it reads an `ABSOLUTE` or `HINGE` cost as 0 at every QP solution, and that cost's
exact value ignores its coefficient (see [the QP problem](trajopt_sqp.md#the-qp-problem)).
tesseract-robotics/trajopt#592 fixes both; every 0.35.0.x wheel bundles trajopt 0.35.0, so the
fix arrives only with a wheel built on a newer trajopt (none yet).

| Class | Purpose | Row bounds | As a cost |
|---|---|---|---|
| `JointPosConstraint` (target) | Joint values at a waypoint | equality | `SQUARED`, `ABSOLUTE` |
| `JointPosConstraint` (bounds) | Per-joint bounds at a waypoint | a range splits into two one-sided rows (the default) | `HINGE` if every row is one-sided; `SQUARED`, `ABSOLUTE` if every row is an equality |
| `JointVelConstraint` | Joint velocity toward a target (zero for smoothing) | equality | `SQUARED`, `ABSOLUTE` |
| `JointAccelConstraint` | Joint acceleration toward a target | equality | `SQUARED`, `ABSOLUTE` |
| `JointJerkConstraint` | Joint jerk toward a target | equality | `SQUARED`, `ABSOLUTE` |
| `CartPosConstraint` (pose) | TCP pose at a waypoint | equality | `SQUARED`, `ABSOLUTE` |
| `CartPosConstraint` (per-axis) | TCP pose with free axes and bands | per axis | see [per-axis Cartesian constraints](../user-guide/low-level-sqp.md#per-axis-cartesian-constraints) |
| `CartLineConstraint` | TCP on a line segment between two poses | equality | `SQUARED`, `ABSOLUTE` |
| `DiscreteCollisionConstraint` | Collision at a waypoint | one-sided (upper) | `HINGE` |
| `DiscreteCollisionNumericalConstraint` | The same, with a numerical Jacobian | one-sided (upper) | `HINGE` |
| `ContinuousCollisionConstraint` | Collision along the segment between two waypoints | one-sided (upper) | `HINGE` |
| `InverseKinematicsConstraint` | Joints toward an IK solution of a target pose | equality | `SQUARED`, `ABSOLUTE` |

A range row kept whole (`RangeBoundHandling.KEEP_AS_IS`) goes into neither: `addCostSet` raises,
and a constraint raises `Unsupported bounds type!` at the first `convexify()`. Measured on trajopt
0.35.0: each form in the table was added to a `TrajOptQPProblem` as a constraint and with each
penalty type, then set up and convexified. That measures acceptance, not how the SQP models the
cost.

0.34 constraint constructors take `Var` references directly (not
`JointPosition` lists). See [changes](../changes.md) for the migration details.

## Collision Evaluators

| Class | Pairs with | Collision check type in the config |
|---|---|---|
| `SingleTimestepCollisionEvaluator` | `DiscreteCollisionConstraint` | `DISCRETE` (the default); the constructor raises otherwise |
| `LVSDiscreteCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS discrete mode) | `LVS_DISCRETE`; the constructor raises otherwise |
| `LVSContinuousCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS continuous mode) | `LVS_CONTINUOUS` or `CONTINUOUS`; the constructor raises otherwise |
| `DiscreteCollisionEvaluator` | Base class | |
| `ContinuousCollisionEvaluator` | Base class | |

Set the type with `config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE`
(`CollisionEvaluatorType` is in `tesseract_robotics.tesseract_collision`).

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
