# tesseract_robotics.trajopt_sqp

Sequential Quadratic Programming (SQP) solver for trajectory optimization.

## Overview

```python
from tesseract_robotics.trajopt_sqp import (
    # Solver
    TrustRegionSQPSolver, OSQPEigenSolver,
    # Problem types
    IfoptQPProblem, IfoptProblem, QPProblem, QPSolver,
    # Configuration
    SQPParameters, SQPResults, SQPCallback,
    # Enums
    SQPStatus, QPSolverStatus, CostPenaltyType, ConstraintType,
)
```

## Quick Start

```python
from tesseract_robotics.trajopt_sqp import (
    TrustRegionSQPSolver, OSQPEigenSolver, IfoptQPProblem, SQPParameters
)
from tesseract_robotics.trajopt_ifopt import JointPosition, JointVelConstraint
from tesseract_robotics.ifopt import Bounds
import numpy as np

# Create variables
joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
n_steps = 10
variables = []
for i in range(n_steps):
    var = JointPosition(np.zeros(6), joint_names, f"pos_{i}")
    var.SetBounds([Bounds(-3.14, 3.14)] * 6)
    variables.append(var)

# Create problem
qp_solver = OSQPEigenSolver()
nlp = IfoptQPProblem(qp_solver)

# Add variables
for var in variables:
    nlp.addVariableSet(var)

# Add constraints (e.g., velocity limits)
for i in range(n_steps - 1):
    vel_constraint = JointVelConstraint(
        np.zeros(6), [variables[i], variables[i+1]], np.ones(6), f"vel_{i}"
    )
    nlp.addConstraintSet(vel_constraint)

# Configure and solve
params = SQPParameters()
params.max_iterations = 50

solver = TrustRegionSQPSolver(qp_solver)
solver.init(nlp)
solver.setParameters(params)
solver.solve()

# Get results
results = solver.getResults()
print(f"Status: {results.status}")
print(f"Final cost: {results.overall_cost}")
```

## TrustRegionSQPSolver

Main SQP solver with trust region method.

```python
from tesseract_robotics.trajopt_sqp import TrustRegionSQPSolver, OSQPEigenSolver

qp_solver = OSQPEigenSolver()
solver = TrustRegionSQPSolver(qp_solver)

# Initialize with problem
solver.init(nlp)

# Configure
params = SQPParameters()
solver.setParameters(params)

# Register callback (optional)
solver.registerCallback(callback)

# Solve
solver.solve()

# Get results
results = solver.getResults()
```

| Method | Description |
|--------|-------------|
| `init(nlp)` | Initialize with problem |
| `setParameters(params)` | Set solver parameters |
| `registerCallback(cb)` | Register iteration callback |
| `solve()` | Run optimization |
| `stepSQPSolver()` | Single iteration step |
| `getResults()` | Get optimization results |

## OSQPEigenSolver

QP solver using OSQP (Operator Splitting QP).

```python
from tesseract_robotics.trajopt_sqp import OSQPEigenSolver

qp_solver = OSQPEigenSolver()

# Used internally by TrustRegionSQPSolver
# Solves convex QP subproblems at each SQP iteration
```

## IfoptQPProblem

Problem formulation combining ifopt interface with QP solver.

```python
from tesseract_robotics.trajopt_sqp import IfoptQPProblem, OSQPEigenSolver

qp_solver = OSQPEigenSolver()
nlp = IfoptQPProblem(qp_solver)

# Add optimization components
nlp.addVariableSet(position_var)
nlp.addConstraintSet(collision_constraint)
nlp.addCostSet(velocity_cost)

# Setup (called automatically by solver.init())
nlp.setup()
```

| Method | Description |
|--------|-------------|
| `addVariableSet(var)` | Add optimization variable |
| `addConstraintSet(constraint)` | Add constraint |
| `addCostSet(cost)` | Add cost term |
| `setup()` | Finalize problem structure |
| `GetNumberOfOptimizationVariables()` | Total variable count |
| `GetNumberOfConstraints()` | Total constraint count |

## SQPParameters

Solver configuration parameters.

```python
from tesseract_robotics.trajopt_sqp import SQPParameters

params = SQPParameters()

# Iteration limits
params.max_iterations = 50

# Convergence tolerances
params.min_approx_improve = 1e-4
params.min_trust_box_size = 1e-4

# Trust region
params.initial_trust_box_size = 0.1
params.trust_shrink_ratio = 0.1
params.trust_expand_ratio = 1.5

# Constraint handling
params.initial_constraint_penalty = 10.0
params.constraint_penalty_increase_factor = 2.0

# Logging
params.log_results = False
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iterations` | 50 | Maximum SQP iterations |
| `min_approx_improve` | 1e-4 | Min improvement to continue |
| `min_trust_box_size` | 1e-4 | Min trust region size |
| `initial_trust_box_size` | 0.1 | Initial trust region |
| `trust_shrink_ratio` | 0.1 | Trust region shrink factor |
| `trust_expand_ratio` | 1.5 | Trust region expand factor |
| `initial_constraint_penalty` | 10.0 | Initial penalty coefficient |
| `constraint_penalty_increase_factor` | 2.0 | Penalty increase rate |

## SQPResults

Optimization results.

```python
results = solver.getResults()

# Status
print(f"Status: {results.status}")  # SQPStatus enum

# Cost information
print(f"Overall cost: {results.overall_cost}")
print(f"Cost names: {results.cost_names}")
print(f"Constraint names: {results.constraint_names}")

# Solution
best_values = results.best_var_vals  # np.ndarray of all variables
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `status` | `SQPStatus` | Solver exit status |
| `overall_cost` | `float` | Final cost value |
| `best_var_vals` | `np.ndarray` | Optimal variable values |
| `cost_names` | `list[str]` | Names of cost terms |
| `constraint_names` | `list[str]` | Names of constraints |

## Enums

### SQPStatus

```python
from tesseract_robotics.trajopt_sqp import SQPStatus

SQPStatus.RUNNING           # Still iterating
SQPStatus.CONVERGED         # Converged successfully
SQPStatus.ITERATION_LIMIT   # Hit max iterations
SQPStatus.TRUST_REGION_CONVERGED  # Trust region too small
SQPStatus.PENALTY_CONVERGED      # Penalty method converged
SQPStatus.CALLBACK_STOPPED       # Stopped by callback
```

### QPSolverStatus

```python
from tesseract_robotics.trajopt_sqp import QPSolverStatus

QPSolverStatus.OPTIMAL      # QP solved optimally
QPSolverStatus.PRIMAL_INFEASIBLE  # Infeasible
QPSolverStatus.DUAL_INFEASIBLE    # Dual infeasible
```

### CostPenaltyType

```python
from tesseract_robotics.trajopt_sqp import CostPenaltyType

CostPenaltyType.SQUARED  # Quadratic penalty
CostPenaltyType.HINGE    # Hinge loss
```

### ConstraintType

```python
from tesseract_robotics.trajopt_sqp import ConstraintType

ConstraintType.EQ   # Equality constraint
ConstraintType.INEQ # Inequality constraint
```

## SQPCallback

Custom callback for monitoring optimization.

```python
from tesseract_robotics.trajopt_sqp import SQPCallback

class MyCallback(SQPCallback):
    def __call__(self, nlp, results):
        print(f"Iteration {results.iteration}: cost = {results.overall_cost}")
        return True  # Return False to stop optimization

callback = MyCallback()
solver.registerCallback(callback)
```

## Performance Tips

1. **Warm starting**: Initialize variables close to expected solution
2. **Trust region**: Start with smaller `initial_trust_box_size` for difficult problems
3. **Collision frequency**: Use discrete collision for speed, continuous for safety
4. **Variable scaling**: Ensure variables have similar magnitudes

```python
# Warm start example
from tesseract_robotics.trajopt_ifopt import interpolate

start = current_joints
goal = target_joints
initial_traj = interpolate(start, goal, n_steps)

for i, var in enumerate(variables):
    var.SetVariables(initial_traj[i])
```
