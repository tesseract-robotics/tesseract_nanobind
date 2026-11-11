# Low-level SQP

<!-- snippet: ambient
from tests.docs.doc_assets import sqp_scene

globals().update(sqp_scene(robot))

from tests.docs.doc_assets import joint_program

_program = joint_program(robot)
manip_info = _program.getManipulatorInfo()
moves = [instruction.asMoveInstruction() for instruction in _program]
n_joints = len(joint_names)
-->

The low-level SQP API builds a trajectory optimization problem term by term (variables,
constraints, costs) and solves it with a trust-region SQP solver, one step at a time if you
want. Use it when you replan repeatedly from the previous solution, such as around a moving
obstacle, or when a constraint has to be written in Python. It sits below the task composer: no
profiles, no pipeline, every term explicit. The modules are `trajopt_ifopt` (variables and
terms) and `trajopt_sqp` (problem and solver).

## When to reach for this

| Situation | Use |
|---|---|
| Offline, one-shot plan | `plan_freespace` / `plan_cartesian` |
| Multi-stage pipeline (sampling → optimize → time-param) | `TaskComposer` |
| **Online replanning with a moving obstacle** | **Low-level SQP (this page)** |

The reference 8-DOF gantry problem prints its replan timing at runtime; run
[`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) to see numbers on your machine. The SQP loop has no latency bound: a step's time depends on the problem and the collision mode, so measure before relying on a control rate.

## Modules

| Module | Purpose |
|---|---|
| `tesseract_robotics.trajopt_ifopt` | Variables (`Var`, `Node`, `NodesVariables`), constraints (joint, Cartesian, collision), factory helpers |
| `tesseract_robotics.trajopt_sqp` | `TrajOptQPProblem` (the QP problem), `TrustRegionSQPSolver`, `OSQPEigenSolver` |

The standalone `tesseract_robotics.ifopt` module was **removed in 0.34**.
All types merged into `tesseract_robotics.trajopt_ifopt`. See the
[0.33 → 0.34 migration guide](../changes.md) for details.

## Variable hierarchy (0.34+)

The `JointPosition` class is gone. Variables now use a three-level hierarchy:

- **`Var`** — a single variable (e.g., joint values at one waypoint)
- **`Node`** — groups multiple `Var`s (e.g., joints + velocities at one waypoint)
- **`NodesVariables`** — container of `Node`s passed to the problem constructor

Build the full hierarchy with `createNodesVariables`:

```python
from tesseract_robotics.trajopt_ifopt import Bounds, createNodesVariables

bounds = [Bounds(-3.14, 3.14)] * len(joint_names)  # one Bounds per joint
nodes_variables = createNodesVariables(
    "trajectory", joint_names, initial_states, bounds,
)
```

Get `Var` references for passing to constraints:

```python
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
```

## Problem setup

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:setup"
```

## Constraints and costs

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
from tesseract_robotics.common import Isometry3d

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
| `RangeBoundHandling.KEEP_AS_IS` | Each range stays one row with `[lower, upper]` — the form that preserves an asymmetric band as written. `TrajOptQPProblem` takes no range row: `addConstraintSet` and `addCostSet` raise `rows must have equality or one-sided bounds!`. |
| `RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES` (default) | Each *range* row becomes two one-sided rows, `g(x) >= lower` and `g(x) <= upper`, so five constrained axes report ten rows. Equality and already one-sided bounds are unaffected. |

Both are prerequisites for adding the term as a **cost** rather than a hard constraint:
as an equality it only ever pulls straight back to the target pose, so a soft Cartesian
preference is unexpressible and the free axis has to be given up entirely.
`TrajOptQPProblem.addCostSet` and `addConstraintSet` check every row's bounds, whatever the
penalty type:

| Term | As a constraint, or a `SQUARED`, `ABSOLUTE` or `HINGE` cost |
|---|---|
| Equality on every constrained axis | accepted |
| Band, split (default): one-sided rows | accepted |
| Band, `KEEP_AS_IS`: range rows | raises |
| Equality and band axes mixed, split | accepted |

A band goes in as a cost with any penalty type on its split rows, and one term may mix equality
and band axes. An `ABSOLUTE` or `HINGE` cost is modelled on its linearized rows, as it is charged
(see [the QP problem](../api/trajopt_sqp.md#the-qp-problem)).

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
    *[trajopt_ifopt.Bounds(-3.14, 3.14)] * 3,  # joints 3-5 within their limits
]
constraint = trajopt_ifopt.JointPosConstraint(bounds, var, np.array([5.0]), "band")
```

By default a range becomes two one-sided rows (`RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES`), because the QP problems accept only equality and one-sided rows; pass `KEEP_AS_IS` to keep it as one. A length-1 `coeffs` weights every row.


## SQP solver loop

The solver loop: initial global `solve`, then per-tick `setVariables` + `stepSQPSolver`
for warm-started incremental optimization. `setBoxSize` resets the trust region each
tick (trust-region shrinking can drive the box to zero):

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:sqp_loop"
```

## Warm-Starting

Between solver steps, set the previous solution through the problem. `problem.setVariables`
writes the variables, then updates every component and recounts the dynamic rows;
`NodesVariables.setVariables` only writes them. A wrong length raises `VariablesSizeError`:

```python
problem.setVariables(trajectory.flatten())
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

After `solve`, `solver.getStatus()` returns an `SQPStatus`: `CONVERGED` is the only
success. `ITERATION_LIMIT`, `PENALTY_ITERATION_LIMIT`, `TIME_LIMIT`, `QP_SOLVE_FAILED`
and `STOPPED_BY_CALLBACK` each name why it stopped. A run can stop with constraints still
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
from tesseract_robotics.motion_planners_trajopt_ifopt import (
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
from tesseract_robotics.motion_planners_trajopt_ifopt import (
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

### Progress callbacks through the planner

A Python `SQPCallback` set in `TrajOptIfoptSolverProfile.callbacks` is registered on the solver
the planner builds. `execute` runs once per SQP iteration, with the GIL taken for the call:

```python
from tesseract_robotics.trajopt_sqp import SQPCallback


class Progress(SQPCallback):
    def execute(self, problem, sqp_results):
        print(sqp_results.overall_iteration, sqp_results.best_exact_merit)
        return True


from tesseract_robotics.motion_planners_trajopt_ifopt import TrajOptIfoptOSQPSolverProfile

solver_profile = TrajOptIfoptOSQPSolverProfile()
solver_profile.callbacks = [Progress()]  # assign the whole list; it is copied
```

!!! warning "`problem` is valid only during the call"
    The planner owns the problem it passes to `execute` and destroys it when the solve ends.
    Read what you need from `problem` inside the callback; do not store it, or anything
    taken from it, for use after `execute` returns.

!!! note "Returning `False` stops the solve"
    The solver ends the solve after that call with `SQPStatus.STOPPED_BY_CALLBACK`. Through
    `TrajOptIfoptMotionPlanner` the response is unsuccessful, its message ending in
    `SQP_STOPPED_BY_CALLBACK`. The 0.35.0.x wheels up to 0.35.0.8 bundle trajopt 0.35.0, whose
    solver dropped the `False` and kept stepping: there, bound a solve with
    `opt_params.max_iterations` or `opt_params.max_time`.

## Python subclassing

You can subclass `ConstraintSet` from Python to define custom constraints — the
trampoline landed in 0.34.1.1 (see the
[CHANGELOG](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/CHANGELOG.md)).
`scipy` is required at runtime because nanobind's sparse-matrix conversion
goes through `scipy.sparse`.

### `PyConstraint`: a rule from `f(q)`

A `ConstraintSet` subclass has five methods to write and a sparse Jacobian to place in the
right columns. `trajopt_ifopt.python_constraints.PyConstraint` reduces that to one: `f(q)`,
the rule's values at one waypoint's joint values, with `lower <= f(q) <= upper`.

```python
import numpy as np
from tesseract_robotics.trajopt_ifopt.python_constraints import PyConstraint


class ElbowAbove(PyConstraint):
    """link_3 stays at least `height` above link_2."""

    def __init__(self, var, manip, height):
        super().__init__("elbow_above", var, lower=[height], upper=[np.inf])
        self.manip = manip

    def f(self, q):
        tf = self.manip.calcFwdKin(q)
        return [tf["link_3"].translation[2] - tf["link_2"].translation[2]]


for var in vars_list:
    problem.addConstraintSet(ElbowAbove(var, manip, 0.2))
```

| What | How |
|---|---|
| **Jacobian** | By default a forward difference over that waypoint's joints (`calcForwardNumJac`, `n_dof + 1` calls to `f`), placed in the waypoint's columns. Override `jacobian(q)` where an analytic one is cheap. |
| **Range rows** | A row with `lower < upper`, both finite, is split into two one-sided rows, as `RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES` does, because `TrajOptQPProblem` takes no range row. |
| **Weight** | `coeff` is the merit coefficient, one value or one per row. |

The module ships rules built on it, from what Tesseract already computes:

| Rule | Value | From |
|---|---|---|
| `PointsDown` | cosine of the tool axis' angle to a direction | forward kinematics, analytic Jacobian |
| `HoldAxis` | the tool axis' two components across a direction, for a tolerance of a fraction of a degree | forward kinematics, analytic Jacobian |
| `StayDexterous` | manipulability (`calcManipulability(J).m.volume`) | the Jacobian |
| `ElbowUp` | height of one link above another | forward kinematics |
| `WatchTheTool` | cosine of the angle between a camera's optical axis and the line to the tool | forward kinematics of two links |
| `Clearance` | closest distance of some links to the scene, in a window | the discrete contact manager |
| `WorldHold` | tool position in the world, given the base's pose | forward kinematics, analytic Jacobian |

`PointsDown` and `HoldAxis` have C++ counterparts, `CartAxisConeConstraint` and
`CartAxisAlignConstraint`: use those in production, and read the Python ones as worked
examples of an analytic Jacobian. `scripts/bench_python_constraints.py` measures what a
Python rule costs in the online SQP loop against the C++ constraint.

The [Python constraints example](../examples/python-constraints.md) runs all seven in one
live loop, one solve per frame, with a gauge per rule.

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
