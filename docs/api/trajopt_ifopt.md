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

`Node.getParent()` returns the set only when Python built it (`NodesVariables(...)` or
`createNodesVariables`), it is alive and it holds that `Node`; the result keeps the set alive. A
`Node` of a set C++ owns, such as the planner's nodes a Python composite profile's `create` receives,
raises `NodeParentUnavailableError` (a `RuntimeError`), during `create` too: the planner frees that
set when its solve returns, and a `Node` holds only a raw pointer to it. For a set of a Python
subclass of `NodesVariables`, `getParent()` returns that very object. A reference cycle through a `Node` on such an object
(`s.n = s.getNode(0)`) is not collected by the garbage collector: break it by hand.

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
class is accepted as a constraint and as a cost with any penalty type: every row must be an
equality or bounded on one side, and a set may mix the two. A two-sided range row is refused by
`addConstraintSet` and `addCostSet` alike. There is no cost without a penalty type. The SQP
models every penalty type as it charges it: an `ABSOLUTE` or `HINGE` cost on its linearized rows,
weighted by its coefficients (see [the QP problem](trajopt_sqp.md#the-qp-problem)).

| Class | Purpose | Row bounds |
|---|---|---|
| `JointPosConstraint` (target) | Joint values at a waypoint | equality |
| `JointPosConstraint` (bounds) | Per-joint bounds at a waypoint | a range splits into two one-sided rows (the default) |
| `JointVelConstraint` (target) | Joint velocity toward a target (zero for smoothing) | equality |
| `JointVelConstraint` (bounds) | Per-joint velocity band, the same every step | a range splits into two one-sided rows (the default) |
| `JointAccelConstraint` (target) | Joint acceleration toward a target | equality |
| `JointAccelConstraint` (bounds) | Per-joint acceleration band | a range splits into two one-sided rows (the default) |
| `JointJerkConstraint` (target) | Joint jerk toward a target | equality |
| `JointJerkConstraint` (bounds) | Per-joint jerk band | a range splits into two one-sided rows (the default) |
| `CartPosConstraint` (pose) | TCP pose at a waypoint | equality |
| `CartPosConstraint` (per-axis) | TCP pose with free axes and bands | per axis |
| `CartLineConstraint` | TCP on a line segment between two poses | equality |
| `CartAxisConeConstraint` | A link axis within a half angle of a target axis; rotation about it free | one-sided (upper), in radians |
| `CartAxisAlignConstraint` | A link axis along a target axis; rotation about it free | equality (two rows), in radians |
| `DiscreteCollisionConstraint` | Collision at a waypoint | one-sided (upper) |
| `DiscreteCollisionNumericalConstraint` | The same, with a numerical Jacobian | one-sided (upper) |
| `ContinuousCollisionConstraint` | Collision along the segment between two waypoints | one-sided (upper) |
| `InverseKinematicsConstraint` | Joints toward an IK solution of a target pose | equality |
| `DiscreteCollisionConstraintD` | Collision at a waypoint, one row per contact (the TrajOpt-Ifopt planner's term) | one-sided (upper) |
| `ContinuousCollisionConstraintD` | Collision along a segment, one row per contact | one-sided (upper) |
| `ContinuousCollisionNumericalConstraint` | `ContinuousCollisionConstraint` with a numerical Jacobian | one-sided (upper) |

The `…D` sets are dynamic: `update()` recounts their rows from the contacts it finds, and resets
their bounds to one upper bound per row.

A range row kept whole (`RangeBoundHandling.KEEP_AS_IS`) goes into neither: both add calls raise
`rows must have equality or one-sided bounds!`. Measured on trajopt master (`a504b22`): sets with
equality, one-sided, mixed and range rows were added to a `TrajOptQPProblem` as a constraint and
with each penalty type, then set up and convexified. That measures acceptance, not how the SQP
models the cost. trajopt 0.35.0 accepted less: `SQUARED` and `ABSOLUTE` costs only with equality
rows, `HINGE` only with one-sided rows, and a range constraint failed only at the first
`convexify()`.

A constraint has a Jacobian only once it is linked to the optimization variables:
`addConstraintSet` / `addCostSet` link it, or call `linkWithVariables(nodes)` yourself.
`getJacobian()` on an unlinked set raises `ConstraintNotLinkedError` (a `RuntimeError`); upstream
would dereference a null pointer. The collision constraints also need `update()` before
`getValues()` or `getJacobian()` (and `calcValues()` / `calcJacobianBlock()` on
`DiscreteCollisionNumericalConstraint`), which `TrajOptQPProblem.setup()` calls for every set.
Before the first `update()` they raise `RuntimeError("<name>: collision data not computed, call
update() first")` (trajopt#617; trajopt 0.35.0 dereferenced a null pointer).

An exception raised in a Python `ConstraintSet` subclass reaches the caller with its own type,
also when trajopt calls the set from `TrajOptQPProblem` or `TrustRegionSQPSolver.solve()`: an
`IndexError` in `getValues()` is an `IndexError`, not trajopt's `RuntimeError`. On macOS it
used to arrive as `RuntimeError` with the Python traceback as its message
([#156](https://github.com/tesseract-robotics/tesseract_nanobind/issues/156)).

`CartLineConstraint` takes 1 to 6 `CartLineInfo.indices`, in any order: `getValues()`,
`getJacobian()` and `calcJacobianBlock()` return one row per index, the rows of the 6-index result
that the indices name. trajopt 0.35.0 filled 6 Jacobian rows whatever their number, so fewer
corrupted memory, and the binding raised `CartLineIndicesUnsupportedError` until the pinned trajopt
had the fix (trajopt#612,
[#237](https://github.com/tesseract-robotics/tesseract_nanobind/issues/237)).

`setBounds(bounds)` replaces a set's row bounds; it raises `BoundsSizeError` (a `ValueError`)
unless there is one bound per row (trajopt checks with `assert` only).

`calcJacobianBlock(joint_vals)` on `CartPosConstraint`, `CartLineConstraint`,
`DiscreteCollisionNumericalConstraint` and `InverseKinematicsConstraint` evaluates the Jacobian
at given joint values, sized like `getJacobian()`, so it also needs linked variables; a wrong
`joint_vals` length raises `JointValuesSizeError`. trajopt 0.35.0 declared the IK one without
defining it; trajopt#626 defines it.

The `list[Bounds]` constructors of `JointVelConstraint`, `JointAccelConstraint` and
`JointJerkConstraint` take one bound per joint, repeated for every step. The rows are undivided
finite differences of the positions, so a velocity limit `v` (rad/s) over a step `dt` is the band
`Bounds(-v * dt, v * dt)`. As with `JointPosConstraint`, a range splits into a lower-bound row and
an upper-bound row by default; `RangeBoundHandling.KEEP_AS_IS` keeps it as one row, which
`TrajOptQPProblem` refuses. trajopt 0.35.0 declared these constructors without defining them
(trajopt#606).

```python
band = [ti.Bounds(-0.1, 0.1)] * n_dof          # rad per step
problem.addConstraintSet(ti.JointVelConstraint(band, vars_list, np.ones(n_dof), "VelocityLimit"))
```

A constraint writes its Jacobian columns at its `Var`s' indices, so linking it to a variable set
(`linkWithVariables`, or `TrajOptQPProblem.addConstraintSet` / `addCostSet`) raises
`VarNotInVariableSetError` unless every `Var` it was built on is in that set (the set holds the
`Var`'s `Node`, and that `Node` the `Var`); the constraint stays unlinked. The collision constraints read the variable set's hash through `var.getParent().getParent()`.
They raise `VarNotInVariableSetError` (a `ValueError`) at construction for a `Var` that is not in
a `NodesVariables`; upstream would dereference null on the first `update()`. A linked collision
constraint's `update()` reads the set it is linked to. Unlinked, `update()` needs each `Var`'s set
to be one built from Python and still alive, and holds it for the call; for a `Var` kept from a
planner's set (a profile's `create`) it raises `NodeParentUnavailableError`, since the planner frees
that set when its solve returns.

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
(`CollisionEvaluatorType` is in `tesseract_robotics.collision`).

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

- `TrajOptCollisionConfig(margin, coeff, request, type, longest_valid_segment_length, check_program_mode)`
  — the four trailing parameters default as in C++.
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
