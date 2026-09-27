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

Available constraint types in `tesseract_robotics.trajopt_ifopt`:

| Constraint | Purpose |
|---|---|
| `JointPosConstraint` | Target joint values at specific waypoints |
| `JointVelConstraint` | Velocity limits across waypoints |
| `JointAccelConstraint` | Acceleration limits |
| `JointJerkConstraint` | Jerk (3rd derivative) limits — smoother motion |
| `CartPosConstraint` | TCP pose at a waypoint |
| `CartLineConstraint` | TCP on a line segment |
| `DiscreteCollisionConstraint` | Collision at single timesteps |
| `DiscreteCollisionNumericalConstraint` | Alt. collision jacobian |
| `ContinuousCollisionConstraint` | Collision across segments (LVS) |
| `InverseKinematicsConstraint` | IK-based optimization |

Available collision evaluators:

| Evaluator | Pairs with |
|---|---|
| `SingleTimestepCollisionEvaluator` | `DiscreteCollisionConstraint` |
| `LVSDiscreteCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS discrete) |
| `LVSContinuousCollisionEvaluator` | `ContinuousCollisionConstraint` (LVS continuous) |


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

A band is a cost only as a hinge on its split rows. A term that mixes equality and band
axes goes in as a constraint, or as two terms, one per kind, freeing the other kind's axes
with zero coefficients.

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

The example above rebuilds the problem each tick because the obstacle pose is baked
into collision constraints. If your scene is static, you can reuse the same problem
and only call `setVariables` + `stepSQPSolver`.

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
  (see [`trajopt_sqp`](../api/trajopt_sqp.md#the-qp-problem))
- `CartPosInfo` struct removed — `CartPosConstraint` takes args directly
- `CollisionCache` removed — caching is internal
- `getTotalExactCost()` / `getExactCosts()` take no arguments
- `getStaticKey()` → `getKey()` (instance method on profiles)
- Call `problem.setup()` after adding all constraint/cost sets

## See also

- [`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) — full working example
- [Online Planning examples](../examples/online-planning.md)
- [0.33 → 0.34 Migration Guide](../changes.md)
