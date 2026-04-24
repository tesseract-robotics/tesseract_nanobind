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
| `tesseract_robotics.trajopt_sqp` | `IfoptProblem` (NLP), `IfoptQPProblem` (QP wrapper), `TrustRegionSQPSolver`, `OSQPEigenSolver` |

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

The `build_optimization_problem` function shows the full 0.34 pattern — `NodesVariables`
factory, `IfoptProblem` (NLP) for joint/Cartesian constraints, then `IfoptQPProblem(nlp)`
wrapping for collision constraints, followed by `problem.setup()`:

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
- `IfoptQPProblem()` → `IfoptProblem(nodes_variables)` + `IfoptQPProblem(nlp)`
- `CartPosInfo` struct removed — `CartPosConstraint` takes args directly
- `CollisionCache` removed — caching is internal
- `getTotalExactCost()` / `getExactCosts()` take no arguments
- `getStaticKey()` → `getKey()` (instance method on profiles)
- Call `problem.setup()` after adding all constraint/cost sets

## See also

- [`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) — full working example
- [Online Planning examples](../examples/online-planning.md)
- [0.33 → 0.34 Migration Guide](../changes.md)
