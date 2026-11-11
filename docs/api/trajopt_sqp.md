# tesseract_robotics.trajopt_sqp

<!-- snippet: ambient
from tests.docs.doc_assets import sqp_scene

globals().update(sqp_scene(robot))
-->

Sequential Quadratic Programming (SQP) solver for trajectory optimization.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## The QP problem

`TrajOptQPProblem(nodes_variables)` is the problem the solver takes: the one tesseract_planning's TrajOpt-Ifopt planner and its `online_planning_example.cpp` build. Constraint and cost sets go straight in, and `setup()` runs after the last one:

```python
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

nodes_variables = ti.createNodesVariables("trajectory", joint_names, states, bounds)
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
joint_constraint = ti.JointPosConstraint(states[0], vars_list[0], np.ones(len(joint_names)), "Joints")
collision_constraint = ti.DiscreteCollisionConstraint(evaluator, vars_list[1])
vel_cost = ti.JointVelConstraint(np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity")

problem = tsqp.TrajOptQPProblem(nodes_variables)
problem.addConstraintSet(joint_constraint)                  # joint / Cartesian constraints
problem.addConstraintSet(collision_constraint)              # collision constraints
problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)  # any penalty type
problem.setup()                                             # must call before solving
```

Every row of a cost or constraint set must be an equality or bounded on one side, with any penalty type; `addCostSet` and `addConstraintSet` raise on a two-sided range row. (trajopt 0.35.0 required equality rows for squared and absolute costs and one-sided rows for hinge costs.) Each set is linked to the variables when it is added, and dynamic sets (`isDynamic()`) are accepted. The exact merit weights each cost row by `getCoefficients()`, as the convex model does, and counts every cost term once; it charges each constraint set its violation weighted the same way (`ConstraintViolations.weighted`). So the trust-region ratio compares like with like for every penalty type. The problem keeps one merit coefficient, one violation and one name per constraint set, not per row, and counts cost sets the same way.

After `convexify()` the problem exposes the QP it built: minimise x'Hx + g'x over
`getNumQPVars()` variables (the NLP variables, then the slacks) subject to
lower ≤ A x ≤ upper over `getNumQPConstraints()` rows. `getHessian()` and
`getConstraintMatrix()` return `scipy.sparse` copies; `getGradient()`, `getBoundsLower()`,
`getBoundsUpper()` and `getConstraintMeritCoeff()` vectors.

`problem.setVariables(x)` sets the NLP variables and then updates every component, so a
dynamic set recounts its rows; `NodesVariables.setVariables` only writes the values. It raises
`VariablesSizeError` (a `ValueError`) unless `len(x) == getNumNLPVars()`.

`evaluateConvexCosts`, `evaluateTotalConvexCost` and `evaluateConvexConstraintViolations` take a
point of the QP the last `convexify()` built: `getNumQPVars()` entries, the NLP variables followed
by the slacks. They raise `QPSolutionSizeError` for any other size and `NotConvexifiedError`
before the first `convexify()`, both `ValueError`s. trajopt reads only the NLP block, so a shorter
vector would be read past its end, and before `convexify()` there is no model to read.

An `ABSOLUTE` or `HINGE` cost is modelled on its linearized rows, without the slacks: the
model charges each row its coefficient times the row's linearized violation, and the exact cost
charges the same coefficient times the exact violation. A violation larger than the trust box is
then reduced step by step, as a squared one is. #149's development builds on trajopt 0.35.0 read
such a cost as 0 at every QP solution and dropped its coefficient from the exact cost (#151);
tesseract-robotics/trajopt#592 fixed both before any wheel shipped `TrajOptQPProblem`.

!!! warning "`IfoptQPProblem` and `IfoptProblem` are no longer bound"
    `IfoptQPProblem(IfoptProblem(nodes_variables))`, the two-layer form of 0.34, is gone; trajopt
    is removing `IfoptQPProblem` (tesseract-robotics/trajopt#595). It is not a rename: see
    [Breaking API changes](../breaking-changes.md#ifoptqpproblem-and-ifoptproblem-removed) for
    the migration and the five ways the problems differ.

## Solver

### TrustRegionSQPSolver

Main SQP solver with trust-region globalization.

```python
qp_solver = tsqp.OSQPEigenSolver()
solver = tsqp.TrustRegionSQPSolver(qp_solver)
solver.verbose = False
solver.params.initial_trust_box_size = 0.01

# Solve
solver.solve(problem)
results = solver.getResults()

# Or single incremental step (for online/replanning)
solver.init(problem)
solver.stepSQPSolver()
solver.setBoxSize(0.01)          # trust region resets don't belong in the solve loop
```

Cost/constraint evaluation uses `getTotalExactCost()` / `getExactCosts()`
on the QP problem (no arguments) — the 0.33
`evaluate*` methods are gone.

### OSQPEigenSolver

`QPSolver` implementation backed by OSQP. Used by `TrustRegionSQPSolver`, or driven by hand
through the `QPSolver` interface: `init(num_vars, num_cnts)`, then `updateHessianMatrix(H)`,
`updateGradient(g)`, `updateLinearConstraintsMatrix(A)` and `updateBounds(lower, upper)`, then
`solve()` and `getSolution()`. The objective is x'Hx + g'x (the solver doubles H for OSQP's
½ x'Px). Each update raises `QPSizeError` (a `ValueError`) unless its size matches `init()`.
Before the first `solve()` only `updateBounds` stores bounds; `updateLowerBound` and
`updateUpperBound` return `False` until OSQP is set up.

```python
import scipy.sparse

qp_solver = tsqp.OSQPEigenSolver()
qp_solver.init(2, 2)
qp_solver.updateHessianMatrix(scipy.sparse.identity(2, format="csr"))
qp_solver.updateGradient([-2.0, -4.0])
qp_solver.updateLinearConstraintsMatrix(scipy.sparse.identity(2, format="csr"))
qp_solver.updateBounds([-10.0, -10.0], [0.5, 10.0])
qp_solver.solve()
qp_solver.getSolution()  # [0.5, 2.0]
```

`setWarmStart` has two forms: `setWarmStart(True)` / `setWarmStart(False)` sets the OSQP
option, and `setWarmStart(problem)` seeds the next solve from a convexified problem's variables
and slacks (a no-op while the option is off; the solver must be `init()`-ed at the problem's QP
size, else `QPSizeError`).

## Parameters and Results

- `SQPParameters` — configure iteration limits, trust region, penalty
  coefficients. Mutated via `solver.params` before `solve()`.
- `SQPResults` — the solver's iteration state. Notable attributes:
  `best_var_vals`, `best_exact_merit`, `best_costs`, `best_constraint_violations`,
  `cost_names`, `constraint_names`, plus the various iteration counters.
- Solver status: `solver.getStatus()` returns the `SQPStatus` enum; `toString(status)` gives
  trajopt's text for it (`"SQP_CONVERGED"`).
- `SQPParameters` compares by value and is unhashable.

## Enums

```python
from tesseract_robotics.trajopt_sqp import (
    SQPStatus, QPSolverStatus, CostPenaltyType,
)

SQPStatus.RUNNING
SQPStatus.CONVERGED
QPSolverStatus.UNINITIALIZED
CostPenaltyType.SQUARED
```

Python names are unchanged from 0.33 — see [changes](../changes.md).
The 0.33 `ConstraintType` enum was removed; if you referenced it, drop the usage.

## Callbacks

Override `execute(problem, results)` — return `False` to stop the solver.

```python
class MyCallback(tsqp.SQPCallback):
    def execute(self, problem, results):
        print(f"iter {results.overall_iteration}: best_merit={results.best_exact_merit}")
        return True

solver.registerCallback(MyCallback())
```

## Reference

::: tesseract_robotics.trajopt_sqp

## See also

- [`trajopt_ifopt`](trajopt_ifopt.md) — variables and constraints
- [Low-Level SQP API](../user-guide/low-level-sqp.md)
- [0.33 → 0.34 migration](../changes.md)
