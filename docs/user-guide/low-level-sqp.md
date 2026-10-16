# Low-Level SQP API

Real-time trajectory optimization via Sequential Quadratic Programming. Use this when you need sub-10ms optimizer steps for online replanning — lower-level than `TaskComposer`, higher-control than `plan_cartesian`.

## When to reach for this

| Situation | Use |
|---|---|
| Offline, one-shot plan | `plan_freespace` / `plan_cartesian` |
| Multi-stage pipeline (sampling → optimize → time-param) | `TaskComposer` |
| **Online replanning with moving obstacle, ~100 Hz** | **Low-level SQP (this page)** |

Measured step rates for a reference 8-DOF gantry problem are printed at runtime by
[`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) — run it locally to see numbers on your machine.

## Modules

| Module | Purpose |
|---|---|
| `tesseract_robotics.trajopt_ifopt` | Variables (`Var`, `Node`, `NodesVariables`), constraints (joint, Cartesian, collision), factory helpers |
| `tesseract_robotics.trajopt_sqp` | `TrajOptQPProblem` (the QP problem), `TrustRegionSQPSolver`, `OSQPEigenSolver` |

The standalone `tesseract_robotics.ifopt` module was **removed in 0.34**.
All types merged into `tesseract_robotics.trajopt_ifopt`. See the
[0.33 → 0.34 migration guide](../changes.md) for details.

## Variable Hierarchy (0.34+)

The `JointPosition` class is gone. Variables now use a three-level hierarchy:

- **`Var`** — a single variable (e.g., joint values at one waypoint)
- **`Node`** — groups multiple `Var`s (e.g., joints + velocities at one waypoint)
- **`NodesVariables`** — container of `Node`s passed to the problem constructor

Build the full hierarchy with `createNodesVariables`:

```python
from tesseract_robotics.trajopt_ifopt import Bounds, createNodesVariables

bounds = Bounds(-3.14, 3.14)
nodes_variables = createNodesVariables(
    "trajectory", joint_names, initial_states, bounds,
)
```

Get `Var` references for passing to constraints:

```python
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
```

## Problem Setup

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:setup"
```

## Constraints and Costs

The `build_optimization_problem` function builds the problem tesseract_planning's
`online_planning_example.cpp` builds: a `TrajOptQPProblem` over the `NodesVariables`, the
start and target poses and one collision constraint per step as constraint sets, the joint
velocity as a squared cost, then `problem.setup()`:

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:problem"
```

Available constraint types in `tesseract_robotics.trajopt_ifopt`. Each goes in as a constraint
or as a cost with a penalty type its row bounds allow; the
[`trajopt_ifopt` reference](../api/trajopt_ifopt.md#constraints) lists the bounds and penalty types
per class:

| Constraint | Purpose |
|---|---|
| `JointPosConstraint` | Joint values, or per-joint bounds, at a waypoint |
| `JointVelConstraint` | Joint velocity toward a target (zero for smoothing) |
| `JointAccelConstraint` | Joint acceleration toward a target |
| `JointJerkConstraint` | Joint jerk (3rd derivative) toward a target — smoother motion |
| `CartPosConstraint` | TCP pose at a waypoint |
| `CartLineConstraint` | TCP on a line segment |
| `DiscreteCollisionConstraint` | Collision at single timesteps |
| `DiscreteCollisionNumericalConstraint` | Alt. collision jacobian |
| `ContinuousCollisionConstraint` | Collision across segments (LVS) |
| `InverseKinematicsConstraint` | IK-based optimization |

Available collision evaluators. Each needs its own collision check type in the config, and its
constructor raises otherwise:

| Evaluator | Pairs with | Check type |
|---|---|---|
| `SingleTimestepCollisionEvaluator` | `DiscreteCollisionConstraint` | `DISCRETE` (the default) |
| `LVSDiscreteCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS discrete) | `LVS_DISCRETE` |
| `LVSContinuousCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS continuous) | `LVS_CONTINUOUS` or `CONTINUOUS` |


### Per-axis Cartesian constraints

`CartPosConstraint` has two constructors. The short one is an **equality on all six
axes at unit weight** — the TCP is pinned to the target pose. That form cannot express
a process constraint, which usually needs one or both of:

* a **free axis** — a rotation the task is indifferent to (about an axis-symmetric
  tool, say), which must not be pulled back to the target;
* an **asymmetric bound** — a standoff that may open but never close.

The second constructor takes `coeffs` (per-axis weights) and `bounds`, both length 6
in `[x, y, z, rx, ry, rz]` order, ahead of `manip`. Build the band with `toBounds`:

```python
import numpy as np
from tesseract_robotics import trajopt_ifopt
from tesseract_robotics.tesseract_common import Isometry3d

#                   x      y        z       rx      ry      rz
lower = np.array([-5e-4, -5e-4, -1.5e-3, -0.087, -0.087, -np.pi])
upper = np.array([ 5e-4,  5e-4,  0.0,     0.087,  0.087,  np.pi])

constraint = trajopt_ifopt.CartPosConstraint(
    var,
    np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.0]),  # rz free
    trajopt_ifopt.toBounds(lower, upper),
    manip,
    "tool0",
    "base_link",
    Isometry3d.Identity(),
    Isometry3d.Identity(),
    "CartPos",
)
```

Two semantics are worth knowing before you rely on the row count:

| Behaviour | Effect |
|---|---|
| **Zero coefficient** | Drops that axis' row entirely — six rows become five. This is how an axis is *freed*, not merely de-weighted. |
| `RangeBoundHandling.KEEP_AS_IS` | Each range stays one row with `[lower, upper]` — the form that preserves an asymmetric band as written. `TrajOptQPProblem` takes no range row: `addConstraintSet` and `setup()` accept one, and the first `convexify()` raises `Unsupported bounds type!`. |
| `RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES` (default) | Each *range* row becomes two one-sided rows, `g(x) >= lower` and `g(x) <= upper`, so five constrained axes report ten rows. Equality and already one-sided bounds are unaffected. |

Both are prerequisites for adding the term as a **cost** rather than a hard constraint:
as an equality it only ever pulls straight back to the target pose, so a soft Cartesian
preference is unexpressible and the free axis has to be given up entirely.
`TrajOptQPProblem.addCostSet` checks every row against the penalty type:

| Term | `SQUARED` / `ABSOLUTE` | `HINGE` |
|---|---|---|
| Equality on every constrained axis | accepted | raises |
| Band, split (default): one-sided rows | raises | accepted |
| Band, `KEEP_AS_IS`: range rows | raises | raises |
| Equality and band axes mixed | raises | raises |

A band is a cost only as a hinge on its split rows. On trajopt 0.35.0, which every
tesseract-robotics-nanobind 0.35.0.x wheel bundles (`TrajOptQPProblem` first ships in 0.35.0.9;
earlier releases bind only `IfoptQPProblem`), the SQP's model reads a `HINGE` or
`ABSOLUTE` cost as 0 at every QP solution, so it predicts the whole cost away at every step (a
violation the trust box cannot cut by a quarter stalls at its seed), and the exact cost ignores
the coefficient; tesseract-robotics/trajopt#592 fixes both, unreleased (see
[the QP problem](../api/trajopt_sqp.md#the-qp-problem)). Until a wheel is built on a trajopt that
contains that fix, a band goes in as a constraint. A term that mixes equality and band axes goes
in as a constraint, or as two terms, one per kind (the band term a constraint until then),
freeing the other kind's axes with zero coefficients.

!!! note "Eigen arguments are required"
    `coeffs`, `source_frame_offset`, and `target_frame_offset` have no Python-side
    defaults. Eigen default arguments raise `std::bad_cast` in this module, so pass
    them explicitly — `Isometry3d.Identity()` where you want no offset.

### Per-joint bounds

`JointPosConstraint` has two constructors. The target one pins every joint of a waypoint to a value. The bounds one takes one `Bounds` per joint, ahead of the variable: an equality, a one-sided limit, or a range.

```python
import numpy as np
from tesseract_robotics import trajopt_ifopt

bounds = [
    trajopt_ifopt.Bounds(0.0, 0.0),      # joint 0 pinned
    trajopt_ifopt.Bounds(-np.inf, 1.2),  # joint 1 at most 1.2 rad
    trajopt_ifopt.Bounds(-0.5, 0.5),     # joint 2 within a band
]
constraint = trajopt_ifopt.JointPosConstraint(bounds, var, np.array([5.0]), "band")
```

By default a range becomes two one-sided rows (`RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES`), because the QP problems accept only equality and one-sided rows; pass `KEEP_AS_IS` to keep it as one. A length-1 `coeffs` weights every row.


## SQP Solver Loop

The solver loop: initial global `solve`, then per-tick `setVariables` + `stepSQPSolver`
for warm-started incremental optimization. `setBoxSize` resets the trust region each
tick (trust-region shrinking can drive the box to zero):

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:sqp_loop"
```

## Warm-Starting

Between solver steps, update `NodesVariables` with the previous solution:

```python
nodes_variables.setVariables(trajectory.flatten())
solver.init(problem)
solver.stepSQPSolver()
```

A built problem does not see the environment change. Each collision evaluator keeps
its own contact manager, and the pose of every link outside the manipulator is fixed in it
when the evaluator is built: `env.setState(...)` afterwards moves the obstacle in the
environment, not in the problem. Measured on the example's scene with the obstacle moved
onto the planned path (summed constraint violation, discrete and LVS continuous alike):

| Problem | Before the move | After the move |
|---|---|---|
| Held from before the move | 0.0 | 0.0 |
| Rebuilt after the move | — | 1.92 |

Perturbing the variables after the move leaves the held problem at 0.002, which is the
start-position constraint alone, so this is not the evaluator's cache.

That is why the example rebuilds the problem every tick, as upstream's
`online_planning_example.cpp` (0.35.0, lines 351–357) does. Only when nothing outside the
manipulator moves can you hold one problem and call `setVariables` + `stepSQPSolver`.

## Starting values and failure

The example uses upstream's values. They are a starting point for this scene, not tuned
guidance:

| Term | Kind | Value |
|---|---|---|
| `JointPosConstraint` on the first node | constraint | coefficient 5 per joint |
| `CartPosConstraint` on the last node | constraint | default coefficients |
| `JointVelConstraint`, zero target | `SQUARED` cost | coefficient 1 |
| Collision (`TrajOptCollisionConfig(0.1, 10)`) | constraint | 0.1 m margin, coefficient 10, `collision_margin_buffer` 0.10 m |

After `solve`, `solver.getStatus()` returns an `SQPStatus`: `NLP_CONVERGED` is the only
success. `ITERATION_LIMIT`, `PENALTY_ITERATION_LIMIT`, `OPT_TIME_LIMIT`, `QP_SOLVER_ERROR`
and `CALLBACK_STOPPED` each name why it stopped. A run can stop with constraints still
violated, so check `problem.getExactConstraintViolations()` before using the result.

## What the TrajOpt-Ifopt planner builds

`TrajOptIfoptMotionPlanner` builds a `TrajOptQPProblem` the same way, from its profiles. Each
profile's `create` hook is bound, so Python can see the terms the planner would add without
running a plan:

| Hook | Returns |
|---|---|
| `TrajOptIfoptMoveProfile.create(move_instruction, composite_manip_info, env, index)` | `TrajOptIfoptWaypointInfo`: the waypoint's `node`, its `term_infos` and whether it is `fixed` |
| `TrajOptIfoptCompositeProfile.create(composite_manip_info, env, nodes, fixed_indices)` | `TrajOptIfoptTermInfos` over every waypoint: collision, velocity, acceleration, jerk |
| `TrajOptIfoptSolverProfile.create(verbose=False)` | the `TrustRegionSQPSolver`, with `opt_params` and `callbacks` applied |

`TrajOptIfoptTermInfos` holds four lists of constraint sets. The planner adds `constraints` with
`addConstraintSet` and the others with `addCostSet` and the matching penalty type (`SQUARED`,
`ABSOLUTE`, `HINGE`).

```python
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultMoveProfile,
)

move_profile = TrajOptIfoptDefaultMoveProfile()
infos = [move_profile.create(move, manip_info, env, i) for i, move in enumerate(moves)]
nodes = [info.node for info in infos]
fixed = [i for i, info in enumerate(infos) if info.fixed]

terms = TrajOptIfoptDefaultCompositeProfile().create(manip_info, env, nodes, fixed)
len(terms.constraints), len(terms.squared_costs), len(terms.hinge_costs)
```

`TrajOptIfoptWaypointInfo` owns its `Node`, so `info.node` is read-only and keeps `info`
alive. A constraint set from `term_infos` is shared and outlives the info. The planner moves
every node into one `NodesVariables`; Python cannot, so the nodes a hook returns are for
inspection, and a problem you solve yourself is built over `createNodesVariables(...)`.

### The planner's term factories

The planner builds its terms with five factories, bound under their C++ names. A hand-built
problem that uses them gets the planner's coefficients and sparsity choices:

| Factory | Builds |
|---|---|
| `createJointPositionConstraint(joint_waypoint, var, coeffs)` | `JointPosConstraint`: the waypoint's position, or its tolerance band |
| `createCollisionConstraints(vars, env, manip_info, config, fixed_indices, fixed_sparsity)` | One collision set per unfixed step (`DISCRETE`) or per segment (`LVS_DISCRETE`, `LVS_CONTINUOUS`, `CONTINUOUS`); none for `NONE` |
| `createJointVelocityConstraint(target, vars, coeffs)` | `JointVelConstraint` |
| `createJointAccelerationConstraint(target, vars, coeffs)` | `JointAccelConstraint` |
| `createJointJerkConstraint(target, vars, coeffs)` | `JointJerkConstraint` |

```python
import numpy as np
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    createCollisionConstraints,
    createJointVelocityConstraint,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.trajopt_sqp import CostPenaltyType

vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
for constraint in createCollisionConstraints(
    vars_list, env, manip_info, TrajOptCollisionConfig(0.1, 10.0), [0], False
):
    problem.addConstraintSet(constraint)
problem.addCostSet(
    createJointVelocityConstraint(np.zeros(n_joints), vars_list, np.ones(n_joints)),
    CostPenaltyType.SQUARED,
)
```

An empty `vars` raises `ValueError`, a `None` in it `TypeError`.

## Python Subclassing

You can subclass `ConstraintSet` from Python to define custom constraints — the
trampoline landed in 0.34.1.1 (see the
[CHANGELOG](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/CHANGELOG.md)).
`scipy` is required at runtime because nanobind's sparse-matrix conversion
goes through `scipy.sparse`.

## Migrating from 0.33

If you have 0.33 SQP code, see the [full migration guide](../changes.md) —
critical items:

- `from tesseract_robotics import ifopt` → `from tesseract_robotics import trajopt_ifopt`
- `JointPosition(...)` → `createNodesVariables(...)` + `Var` refs
- `IfoptQPProblem()` → `TrajOptQPProblem(nodes_variables)`, with every constraint and cost
  set added to it; 0.34's `IfoptQPProblem(IfoptProblem(nodes_variables))` is no longer bound
  (see [Breaking API changes](../breaking-changes.md#ifoptqpproblem-and-ifoptproblem-removed))
- `CartPosInfo` struct removed — `CartPosConstraint` takes args directly
- `CollisionCache` removed — caching is internal
- `getTotalExactCost()` / `getExactCosts()` take no arguments
- `getStaticKey()` → `getKey()` (instance method on profiles)
- Call `problem.setup()` after adding all constraint/cost sets

## See also

- [`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) — full working example
- [Online Planning examples](../examples/online-planning.md)
- [0.33 → 0.34 Migration Guide](../changes.md)
