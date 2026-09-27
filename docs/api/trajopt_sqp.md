# tesseract_robotics.trajopt_sqp

Sequential Quadratic Programming (SQP) solver for trajectory optimization.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## The QP problem

`TrajOptQPProblem(nodes_variables)` is the problem the solver takes: the one tesseract_planning's TrajOpt-Ifopt planner and its `online_planning_example.cpp` build. Constraint and cost sets go straight in, and `setup()` runs after the last one:

```python
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

nodes_variables = ti.createNodesVariables("trajectory", joint_names, states, bounds)

problem = tsqp.TrajOptQPProblem(nodes_variables)
problem.addConstraintSet(joint_constraint)                  # joint / Cartesian constraints
problem.addConstraintSet(collision_constraint)              # collision constraints
problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)  # equality bounds required
problem.setup()                                             # must call before solving
```

A squared or absolute cost must have equality bounds, a hinge cost inequality bounds; `addCostSet` raises otherwise. Each set is linked to the variables when it is added. The exact merit weights each squared cost row by `getCoefficients()`, as the convex model does, and counts every cost term once, so the trust-region ratio compares like with like. The problem keeps one merit coefficient per constraint set, and counts and names constraint and cost sets, not rows.

!!! warning "`IfoptQPProblem` and `IfoptProblem` are no longer bound"
    `IfoptQPProblem(IfoptProblem(nodes_variables))`, the two-layer form of 0.34, is gone; trajopt
    is removing `IfoptQPProblem`
    ([trajopt#595](https://github.com/tesseract-robotics/trajopt/issues/595)). Its trust-region
    ratio was inconsistent: in trajopt 0.35.0 it weights the model but not the merit, and counts
    the model once per cost term, so with several weighted cost terms its solver can reject every
    step and report `NLP_CONVERGED` at the seed. A cost added to the wrapped `IfoptProblem` fared
    worse: it was neither squared nor fed to the Gauss-Newton Hessian. To migrate, construct
    `TrajOptQPProblem` over the same variables and add every set to it: constraints with
    `addConstraintSet`, costs with `addCostSet(cost, penalty_type)`, including costs that went
    into the `IfoptProblem`.

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

`QPSolver` implementation backed by OSQP. Used by `TrustRegionSQPSolver`.

## Parameters and Results

- `SQPParameters` — configure iteration limits, trust region, penalty
  coefficients. Mutated via `solver.params` before `solve()`.
- `SQPResults` — the solver's iteration state. Notable attributes:
  `best_var_vals`, `best_exact_merit`, `best_costs`, `best_constraint_violations`,
  `cost_names`, `constraint_names`, plus the various iteration counters.
- Solver status: `solver.getStatus()` returns the `SQPStatus` enum.

## Enums

```python
from tesseract_robotics.trajopt_sqp import (
    SQPStatus, QPSolverStatus, CostPenaltyType,
)

SQPStatus.RUNNING
SQPStatus.NLP_CONVERGED
QPSolverStatus.UNITIALIZED
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

## Module API

::: tesseract_robotics.trajopt_sqp
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3

## See also

- [`trajopt_ifopt`](trajopt_ifopt.md) — variables and constraints
- [Low-Level SQP API](../user-guide/low-level-sqp.md)
- [0.33 → 0.34 migration](../changes.md)
