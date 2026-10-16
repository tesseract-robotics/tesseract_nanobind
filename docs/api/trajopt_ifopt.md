# tesseract_robotics.trajopt_ifopt

Variables and constraint sets for the low-level SQP solver, and how each set becomes a
constraint or a cost of a `TrajOptQPProblem`.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## Variables (0.34+)

The `JointPosition` class from 0.33 is removed. Variables now use a three-level hierarchy.

### Var

A single variable — typically one joint value at one waypoint. `Var(index, name, value, bounds)`
and `Var(index, identifier, names, values, bounds)` build a standalone one (no parent `Node`) at
`index` of the decision vector. It belongs to no variable set, so a constraint built on it
raises `VarNotInVariableSetError` when linked (below); a collision constraint already when built.

### Node

Groups multiple `Var`s per waypoint (e.g., joints + velocities at one waypoint).

### NodesVariables

Container of `Node`s. Passed to `TrajOptQPProblem`. `NodesVariables(name, nodes)` takes
ownership of the `Node`s: after the call the caller's `Node` objects are unusable (read the nodes
back with `getNode` / `getNodes`), and each `Var` reports its `Node` (`Var.getParent()`) and the
`Node` its set (`Node.getParent()`). A `Node` given twice, or one already in a set, raises
`NodeOwnershipError`. A `Var` or `Node` read out of a set keeps the set alive, as does a handed-over
`Node`, so a `Var` taken from a `Node` before the handover stays valid after the set is dropped.

```python
from tesseract_robotics.trajopt_ifopt import Bounds, Node, NodesVariables

nodes = []
for k, q in enumerate(initial_states):
    node = Node(f"waypoint_{k}")
    node.addVar("joints", joint_names, q, joint_bounds)
    node.addVar("time", 0.1, Bounds(0.01, 1.0))  # a second var per node
    nodes.append(node)
nodes_variables = NodesVariables("trajectory", nodes)
```

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
what `addCostSet` accepts. On trajopt 0.35.0, which every tesseract-robotics-nanobind 0.35.0.x
wheel bundles (`TrajOptQPProblem` first ships in 0.35.0.9; earlier releases bind only
`IfoptQPProblem`), the SQP models only `SQUARED` costs as it charges them: it reads an `ABSOLUTE`
or `HINGE` cost as 0 at every QP solution, and that cost's exact value ignores its coefficient
(see [the QP problem](trajopt_sqp.md#the-qp-problem)). tesseract-robotics/trajopt#592 fixes both;
the fix arrives only with a wheel built on a newer trajopt (none yet).

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
| `DiscreteCollisionConstraintD` | Collision at a waypoint, one row per contact (the TrajOpt-Ifopt planner's term) | one-sided (upper) | `HINGE` |
| `ContinuousCollisionConstraintD` | Collision along a segment, one row per contact | one-sided (upper) | `HINGE` |
| `ContinuousCollisionNumericalConstraint` | `ContinuousCollisionConstraint` with a numerical Jacobian | one-sided (upper) | `HINGE` |

The `…D` sets are dynamic: `update()` recounts their rows from the contacts it finds, and resets
their bounds to one upper bound per row.

A range row kept whole (`RangeBoundHandling.KEEP_AS_IS`) goes into neither: `addCostSet` raises,
and a constraint raises `Unsupported bounds type!` at the first `convexify()`. Measured on trajopt
0.35.0: each form in the table was added to a `TrajOptQPProblem` as a constraint and with each
penalty type, then set up and convexified. That measures acceptance, not how the SQP models the
cost.

A constraint has a Jacobian only once it is linked to the optimization variables:
`addConstraintSet` / `addCostSet` link it, or call `linkWithVariables(nodes)` yourself.
`getJacobian()` on an unlinked set raises `ConstraintNotLinkedError` (a `RuntimeError`); upstream
would dereference a null pointer. The collision constraints also need `update()` before
`getValues()` or `getJacobian()`, which `TrajOptQPProblem.setup()` calls for every set.

An exception raised in a Python `ConstraintSet` subclass reaches the caller with its own type,
also when trajopt calls the set from `TrajOptQPProblem` or `TrustRegionSQPSolver.solve()`: an
`IndexError` in `getValues()` is an `IndexError`, not trajopt's `RuntimeError`. On macOS it
used to arrive as `RuntimeError` with the Python traceback as its message
([#156](https://github.com/tesseract-robotics/tesseract_nanobind/issues/156)).

`CartLineConstraint.getJacobian()` raises `CartLineIndicesUnsupportedError` (a `ValueError`)
unless `CartLineInfo.indices` holds all 6 twist rows. Upstream sizes the Jacobian by the number
of indices but fills 6 rows, numeric and analytic alike, so fewer corrupt memory
([#237](https://github.com/tesseract-robotics/tesseract_nanobind/issues/237)). Construction,
`getValues()` and `calcValues()` work with 1 to 6 indices. The check covers direct Python calls
only: `TrajOptQPProblem` calls the C++ Jacobian, so a solve with fewer than 6 indices can still
crash. Use all 6 indices for a constraint you solve. The check goes once the pinned trajopt
includes the upstream fix (trajopt#618).

`setBounds(bounds)` replaces a set's row bounds; it raises `BoundsSizeError` (a `ValueError`)
unless there is one bound per row (trajopt checks with `assert` only).

`calcJacobianBlock(joint_vals)` on `CartPosConstraint`, `CartLineConstraint` and
`DiscreteCollisionNumericalConstraint` evaluates the Jacobian at given joint values, sized like
`getJacobian()`, so it also needs linked variables; a wrong `joint_vals` length raises
`JointValuesSizeError`. `InverseKinematicsConstraint` declares the method, but trajopt 0.35.0
never defines it, so it is not bound. Nor are the `list[Bounds]` constructors of
`JointVelConstraint`, `JointAccelConstraint` and `JointJerkConstraint`, for the same reason.

A constraint writes its Jacobian columns at its `Var`s' indices, so linking it to a variable set
(`linkWithVariables`, or `TrajOptQPProblem.addConstraintSet` / `addCostSet`) raises
`VarNotInVariableSetError` unless every `Var` it was built on is in that set; the constraint stays
unlinked. The collision constraints read the variable set's hash through `var.getParent().getParent()`.
They raise `VarNotInVariableSetError` (a `ValueError`) at construction for a `Var` that is not in
a `NodesVariables`; upstream would dereference null on the first `update()`.

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

The two bases expose what a collision constraint works from. `calcCollisions(dof_vals,
max_allowed)` (discrete) and `calcCollisionData(dof_vals0, dof_vals1, vars0_fixed, vars1_fixed,
max_allowed)` (continuous) return a `CollisionCacheData`: the `ContactResultMap` and one
`GradientResultsSet` per link pair. `getGradient(dofvals, contact)` /
`calcGradientData(dof_vals0, dof_vals1, contact)` return the `GradientResults` of one contact,
and `getJointGroup()` the evaluator's joint group.

```python
data = evaluator.calcCollisions(q, 3)
for results_set in data.gradient_results_sets:
    print(results_set.key, results_set.getMaxError())
```

## Info Structs (for Cartesian / IK constraints)

| Struct | Used by |
|---|---|
| `CartLineInfo` | `CartLineConstraint` |
| `InverseKinematicsInfo` | `InverseKinematicsConstraint` |

Both have field constructors: `CartLineInfo(manip, source_frame, target_frame,
target_frame_offset1, target_frame_offset2[, source_frame_offset[, indices]])` and
`InverseKinematicsInfo(manip, working_frame, tcp_frame[, tcp_offset])`. The bracketed arguments
default to the identity and all 6 indices. `CartLineConstraint.getInfo()`, `getLine()` and
`InverseKinematicsConstraint.getKinematicInfo()` read them back; `setTargetPose` moves the IK
target.

`CartPosInfo` from 0.33 is gone — `CartPosConstraint` now takes parameters
directly (see [changes](../changes.md)).

## Config / Bounds

- `TrajOptCollisionConfig(margin, coeff)` — re-exported here and from
  `tesseract_motion_planners_trajopt`.
- `CollisionCoeffData` — per-pair collision coefficient data. `getCollisionCoeffPairData()`
  returns `{(link_a, link_b): coeff}`, `getPairsWithZeroCoeff()` the switched-off pairs.
- `CollisionCoeffData` and `TrajOptCollisionConfig` compare by value and are unhashable.
- `Bounds(lower, upper)` — single-variable bounds. `b += s` / `b -= s` shift both limits and
  reclassify `getType()`.
- Enums: `BoundsType`, `RangeBoundHandling`.

## Utilities

- `interpolate(start, end, steps)` — linear joint interpolation.
- `toBounds(joint_limits)` — convert a limits matrix into a `Bounds` list.
- `getClosestValidPoint(x, bounds)`, `calcBoundsErrors(x, bounds)`,
  `calcBoundsViolations(x, bounds)` — clamp into bounds, the signed error and the violation per
  entry; one bound per entry or `BoundsSizeError`.
- `toTrajArray(vars)` / `toJointTrajectory(vars)` — the vars' values as a waypoints × joints
  matrix, or a `JointTrajectory` with one state per var.
- `calcNumericalConstraintGradient(variables, constraint_set, epsilon=1e-8)` and
  `calcForwardNumJac(f, x, epsilon)` — forward-difference Jacobians, to check an analytic
  `getJacobian()` (a Python `ConstraintSet` subclass's included).

```python
numerical = calcNumericalConstraintGradient(nodes_variables, my_constraint)
assert np.allclose(numerical.toarray(), my_constraint.getJacobian().toarray(), atol=1e-6)
```

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
