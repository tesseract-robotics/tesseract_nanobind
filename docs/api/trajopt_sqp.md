# tesseract_robotics.trajopt_sqp

Sequential Quadratic Programming (SQP) solver for trajectory optimization.
See the [Low-Level SQP guide](../user-guide/low-level-sqp.md) for the user-guide walkthrough.

## Problem hierarchy (0.34)

0.34 splits the problem into two layers:

- **`IfoptProblem(nodes_variables)`** — the non-linear program (NLP). Holds
  the variables and accepts joint/cartesian/cost sets.
- **`IfoptQPProblem(nlp)`** — the QP wrapper passed to the solver. Accepts
  collision constraints and must be `setup()` before solving.

```python
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

nodes_variables = ti.createNodesVariables("trajectory", joint_names, states, bounds)

nlp = tsqp.IfoptProblem(nodes_variables)
nlp.addConstraintSet(joint_constraint)   # joint / cartesian constraints on NLP
nlp.addCostSet(vel_cost)                 # costs on NLP

problem = tsqp.IfoptQPProblem(nlp)
problem.addConstraintSet(collision_constraint)  # collision constraints on QP
problem.setup()                                 # must call before solving
```

`IfoptQPProblem` no longer has a default constructor — see
[changes](../changes.md) for the full migration.

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
