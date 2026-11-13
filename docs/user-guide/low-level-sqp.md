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

Use the low-level SQP API when you replan from the previous solution every tick, around a
moving obstacle, or when a constraint has to be written in Python; for a one-shot plan, use a
[pipeline](task-composer.md). Here you build the trajectory optimization problem term by term
(variables, constraints, costs) and solve it with a trust-region SQP solver over OSQP, a whole
solve or one step at a time. Nothing is implicit: no profiles, no pipeline, no contact check or
time parameterization afterwards. The modules are `trajopt_ifopt` (variables and terms) and
`trajopt_sqp` (problem and solver).

| Situation | Use |
|---|---|
| Offline, one-shot plan | `plan_freespace` / `plan_ompl` / `plan_cartesian` |
| Your own sequence of planners and checks | a [custom pipeline](task-composer.md#custom-pipelines) |
| Replanning every tick from the last solution; a Python constraint | this page |

The SQP loop has no latency bound: a step's time depends on the problem and the collision
mode. [`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py)
(an 8-DOF gantry) prints its replan timing; measure on your problem before you rely on a
control rate.

## Variables

| Type | Holds |
|---|---|
| `Var` | one block of variables, such as the joint values at one waypoint |
| `Node` | the `Var`s of one waypoint |
| `NodesVariables` | every `Node`; the problem is built over it |

`createNodesVariables` builds all three from a seed trajectory, one row of joint values (rad,
or m for a prismatic joint) per waypoint, and one `Bounds` per joint:

```python
from tesseract_robotics.trajopt_ifopt import Bounds, createNodesVariables

bounds = [Bounds(-3.14, 3.14)] * len(joint_names)  # rad
nodes_variables = createNodesVariables(
    "trajectory", joint_names, initial_states, bounds,
)
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
```

The terms take these `Var`s. `toBounds(manip.getLimits().joint_limits)` gives the bounds from
the URDF limits.

## Build and solve a problem

The example builds the problem upstream's `online_planning_example.cpp` builds, with the target
as joints rather than a pose: a `TrajOptQPProblem` over the `NodesVariables`, the start and
target joints and one collision constraint per step as constraint sets, the joint velocity as a
squared cost weighted per joint, then `problem.setup()`. Call `setup()` after the last set is
added.

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:setup"
```

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:problem"
```

The solver does one global `solve`, then per tick `init` on the new problem and a few
`stepSQPSolver` calls. `setBoxSize` resets the trust region after each step, because the
trust-region loop can shrink it to zero:

```python
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:sqp_loop"
```

`solver.getStatus()` returns an `SQPStatus`; `CONVERGED` is the only success, and the others
name why the solve stopped (see [What goes wrong](#what-goes-wrong)). A solve can stop with
constraints still violated, so check `problem.getExactConstraintViolations()` before using the
result.

### Solver settings

`solver.params` is an `SQPParameters`; set fields before `solve`. The defaults, read from
`SQPParameters()`:

| Field | Default | Meaning |
|---|---|---|
| `max_iter` | 50 | SQP iterations per solve |
| `max_time` | no limit (`float` max) | wall-clock budget of a solve, s |
| `initial_trust_box_size` | 0.1 | half-width of the trust region, in variable units (rad) |
| `min_trust_box_size` | 1e-4 | the trust-region loop ends when the box shrinks below this |
| `trust_shrink_ratio` / `trust_expand_ratio` | 0.1 / 1.5 | factors applied to the box after a rejected / accepted step |
| `improve_ratio_threshold` | 0.25 | the actual-over-predicted improvement a step needs to be accepted |
| `cnt_tolerance` | 1e-4 | the violation below which a constraint counts as satisfied |
| `initial_merit_error_coeff` | 10 | penalty weight of constraint violation in the merit function |
| `merit_coeff_increase_ratio` | 10 | factor on that weight when constraints stay violated |
| `max_merit_coeff_increases` | 5 | penalty increases before `PENALTY_ITERATION_LIMIT` |

```python
solver.params.max_iter = 100
solver.params.max_time = 0.05  # s
```

## Terms

Every term is a constraint set: added with `addConstraintSet` it is a constraint; added with
`addCostSet(term, penalty)`, `penalty` a `CostPenaltyType` (`SQUARED`, `ABSOLUTE`, `HINGE`), it is
a cost. The
[`trajopt_ifopt` reference](../api/trajopt_ifopt.md#constraints) lists each class's row bounds
and the penalty types they allow.

| Term | Value |
|---|---|
| `JointPosConstraint` | joint values, or per-joint bounds, at a waypoint |
| `JointVelConstraint` | joint velocity toward a target (zero for smoothing) |
| `JointAccelConstraint` | joint acceleration toward a target |
| `JointJerkConstraint` | joint jerk toward a target |
| `CartPosConstraint` | TCP pose error at a waypoint |
| `CartLineConstraint` | TCP distance from a line segment |
| `DiscreteCollisionConstraint` | collision at one waypoint |
| `DiscreteCollisionNumericalConstraint` | the same, with a numerical Jacobian |
| `ContinuousCollisionConstraint` | collision along the segment between two waypoints |
| `InverseKinematicsConstraint` | joint distance to an IK solution |

A collision term takes an evaluator, and each evaluator needs its check type in the config's
`collision_check_config.type`; its constructor raises otherwise. The collision margin is in
`TrajOptCollisionConfig(margin_m, coeff)`; see
[Collision configuration](planning.md#collision-configuration).

| Evaluator | Pairs with | Check type |
|---|---|---|
| `SingleTimestepCollisionEvaluator` | `DiscreteCollisionConstraint` | `DISCRETE` (the default) |
| `LVSDiscreteCollisionEvaluator` | `ContinuousCollisionConstraint` | `LVS_DISCRETE` |
| `LVSContinuousCollisionEvaluator` | `ContinuousCollisionConstraint` | `LVS_CONTINUOUS` or `CONTINUOUS` |

### Per-axis Cartesian constraints

`CartPosConstraint` has two constructors. The short one is an equality on all six axes at unit
weight: the TCP is pinned to the target. A process constraint usually needs a **free axis** (a
rotation the task does not care about, such as about an axis-symmetric tool) or an
**asymmetric band** (a standoff that may open but not close). The long constructor takes
per-axis `coeffs` and `bounds`, both in `[x, y, z, rx, ry, rz]` order (m, rad), ahead of
`manip`; `toBounds` builds the band:

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

| Behaviour | Effect |
|---|---|
| Zero coefficient | drops that axis' row: six rows become five. The axis is freed, not de-weighted. |
| `RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES` (default) | each range row becomes two one-sided rows, so five banded axes give ten rows; equality and one-sided rows are unchanged |
| `RangeBoundHandling.KEEP_AS_IS` | each range stays one row; `TrajOptQPProblem` refuses it (see below) |

`addConstraintSet` and `addCostSet` check every row's bounds, whatever the penalty type: an
equality row and a one-sided row go in, a range row raises. A split band therefore goes in as a
constraint or as a cost of any penalty type, and one term may mix equality and band axes. An
`ABSOLUTE` or `HINGE` cost is modelled on its linearized rows (see
[the QP problem](../api/trajopt_sqp.md#the-qp-problem)).

`coeffs`, `source_frame_offset` and `target_frame_offset` have no Python defaults: pass them,
with `Isometry3d.Identity()` for no offset.

### Per-joint bounds

`JointPosConstraint` has two constructors: the target one pins every joint of a waypoint; the
bounds one takes one `Bounds` per joint, ahead of the variable, each an equality, a one-sided
limit or a range (rad):

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

Ranges split into two one-sided rows by default, as above. A length-1 `coeffs` weights every row.

## Warm-starting

`problem.setVariables(x)` writes the variables, updates every term and recounts the dynamic rows;
`NodesVariables.setVariables` only writes them, and does not check the length. A wrong length
raises `VariablesSizeError` from `problem.setVariables` only:

```python
problem.setVariables(trajectory.flatten())
solver.init(problem)
solver.stepSQPSolver()
```

A built problem does not see the environment change. Each collision evaluator keeps its own
contact manager, and fixes the pose of every link outside the manipulator when it is built:
`env.setState(...)` afterwards moves the obstacle in the environment, not in the problem.
On the online planning scene (the robot's run down the cell, the person walking a lap, 12
steps, solved once with the person at tick 0), the held problem's summed constraint violation
stays at zero however the person moves, while a problem rebuilt at the worst tick of the lap
reads about 2.5 (discrete) and 4.5 (LVS continuous). The held problem does not see the move, so this
is not the evaluator's cache. The example therefore rebuilds the problem every tick, as
upstream's `online_planning_example.cpp` does (the rebuild block is lines 349-357, under a
TODO comment). Hold one problem only when nothing outside the manipulator moves.

| Term | Kind | Value |
|---|---|---|
| `JointPosConstraint` on the first node | constraint | coefficient 5 per joint |
| `JointPosConstraint` on the last node | constraint | coefficient 5 per joint (upstream: `CartPosConstraint` on the target pose) |
| `JointVelConstraint`, zero target | `SQUARED` cost | per joint: `joint_1`'s velocity limit over the joint's, 13.1 and 5.2 for the gantry, 0.33 to 1 for the arm (upstream: 1) |
| Collision, `TrajOptCollisionConfig(0.1, 10)` | constraint | 0.1 m margin, coefficient 10, `collision_margin_buffer` 0.10 m |
| Collision rows, `max_num_cnt` | constraint size | 10 per waypoint (upstream: the default 3) |

A pose target with these weights goes wrong: the 8-axis chain is redundant, and once the gantry
costs more than the arm the solver reaches for the pose with the arm to save gantry travel,
twisting it (1.5 rad of `joint_4`) and leaving a link inside the margin.

A collision constraint keeps only its `max_num_cnt` contacts as rows. Give it at least as many
as can be in contact at once: on the online planning scene, with the default 3, the solver
converged with an arm link 0.03 m inside the margin.

## What goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `SQPStatus.ITERATION_LIMIT` | `params.max_iter` reached; only `TrustRegionSQPSolver.solve()` enforces it, `stepSQPSolver()` does not check it | more iterations, or a better seed |
| `SQPStatus.PENALTY_ITERATION_LIMIT` | constraints still violated after `max_merit_coeff_increases` penalty increases: the constraints conflict, or the seed is far from feasible | check `problem.getExactConstraintViolations()` for the term; loosen it or make it a cost |
| `SQPStatus.TIME_LIMIT` | `params.max_time` reached; only `TrustRegionSQPSolver.solve()` enforces it, `stepSQPSolver()` does not check it, so a per-tick step loop bounds its own time | a larger budget, or fewer steps per tick |
| `SQPStatus.QP_SOLVE_FAILED` | OSQP failed `max_qp_solver_failures` times (default 3) | `solver.verbose = True` logs the iterations |
| `SQPStatus.STOPPED_BY_CALLBACK` | a callback returned `False` | intended |
| `rows must have equality or one-sided bounds!` | a `KEEP_AS_IS` range row went to `addConstraintSet` / `addCostSet` | use the default split |
| `VariablesSizeError` | `problem.setVariables` got the wrong length (`NodesVariables.setVariables` does not check) | one value per joint per waypoint |
| a collision evaluator's constructor raises | the config's check type does not match the evaluator | see the evaluator table |
| the plan ignores a moved obstacle | the problem was built before the move | rebuild it ([Warm-starting](#warm-starting)) |
| `ModuleNotFoundError: scipy` in a sparse Jacobian | `scipy` is missing; nanobind converts Eigen sparse matrices through `scipy.sparse` | install the package's dependencies |

## What the TrajOpt-Ifopt planner builds

`TrajOptIfoptMotionPlanner` builds a `TrajOptQPProblem` the same way, from its profiles. Each
profile's `create` hook is bound, so Python can see the terms the planner would add without
running a plan:

| Hook | Returns |
|---|---|
| `TrajOptIfoptMoveProfile.create(move_instruction, composite_manip_info, env, index)` | `TrajOptIfoptWaypointInfo`: the waypoint's `node`, its `term_infos` and whether it is `fixed` |
| `TrajOptIfoptCompositeProfile.create(composite_manip_info, env, nodes, fixed_indices)` | `TrajOptIfoptTermInfos` over every waypoint: collision, velocity, acceleration, jerk |
| `TrajOptIfoptSolverProfile.create(verbose=False)` | the `TrustRegionSQPSolver`, with `opt_params` and `callbacks` applied |

`TrajOptIfoptTermInfos` holds four lists of terms. The planner adds `constraints` with
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

`TrajOptIfoptWaypointInfo` owns its `Node`, so `info.node` is read-only and keeps `info` alive.
A term from `term_infos` is shared and outlives the info. The planner moves every node into one
`NodesVariables`; Python cannot, so the nodes a hook returns are for inspection, and a problem
you solve yourself is built over `createNodesVariables(...)`.

### The planner's term factories

The planner builds its terms with five factories, bound under their C++ names. A hand-built
problem that uses them gets the planner's coefficients and sparsity:

| Factory | Builds |
|---|---|
| `createJointPositionConstraint(joint_waypoint, var, coeffs)` | `JointPosConstraint`: the waypoint's position, or its tolerance band |
| `createCollisionConstraints(vars, env, manip_info, config, fixed_indices, fixed_sparsity)` | one collision term per unfixed step (`DISCRETE`) or per segment (`LVS_DISCRETE`, `LVS_CONTINUOUS`, `CONTINUOUS`); none for `NONE` |
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

A Python `SQPCallback` in `TrajOptIfoptSolverProfile.callbacks` is registered on the solver the
planner builds. `execute` runs once per SQP iteration, with the GIL taken for the call;
returning `False` ends the solve with `SQPStatus.STOPPED_BY_CALLBACK`, and the planner's response
is unsuccessful, its message ending in `SQP_STOPPED_BY_CALLBACK`.

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
    Read what you need from `problem` inside the callback; do not keep it, or anything taken
    from it, after `execute` returns.

## Constraints in Python

`ConstraintSet` can be subclassed in Python. `scipy` is a runtime requirement: nanobind converts
the sparse Jacobian through `scipy.sparse`.

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
| Jacobian | by default a forward difference over that waypoint's joints (`calcForwardNumJac`, `n_dof + 1` calls to `f`), placed in the waypoint's columns. Override `jacobian(q)` where an analytic one is cheap. |
| Range rows | a row with `lower < upper`, both finite, is split into two one-sided rows, as `SPLIT_TO_TWO_INEQUALITIES` does, because `TrajOptQPProblem` takes no range row. |
| Weight | `coeff` is the merit coefficient, one value or one per row. |

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
`CartAxisAlignConstraint`: `CartAxisAlignConstraint` matches `HoldAxis` only at zero tolerance
(an equality), and `CartAxisConeConstraint` works on the angle, not the cosine. Use those in
production, and read the Python ones as worked examples
of an analytic Jacobian. `scripts/bench_python_constraints.py` measures what a Python rule costs
in the online SQP loop against the C++ constraint. The
[Python constraints example](../examples/python-constraints.md) runs all seven in one live loop,
one solve per frame, with a gauge per rule.

## See also

- [Online planning examples](../examples/online-planning.md)
- [`trajopt_sqp`](../api/trajopt_sqp.md) and [`trajopt_ifopt`](../api/trajopt_ifopt.md) reference
- [Migration from 0.33](../changes.md): `ifopt` merged into `trajopt_ifopt`, `JointPosition`
  replaced by `createNodesVariables`, `IfoptQPProblem` by `TrajOptQPProblem`
