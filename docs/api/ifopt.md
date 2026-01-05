# tesseract_robotics.ifopt

Base optimization primitives from the [ifopt](https://github.com/ethz-adrl/ifopt) library.

## Overview

The `ifopt` module provides base classes for constrained nonlinear optimization:

```python
from tesseract_robotics.ifopt import Bounds, VariableSet, ConstraintSet, CostTerm
```

## Bounds

Define variable/constraint bounds.

```python
from tesseract_robotics.ifopt import Bounds, NoBound, BoundZero, BoundGreaterZero, BoundSmallerZero, inf

# Custom bounds
bounds = Bounds(-1.0, 1.0)  # lower=-1, upper=1
print(f"Lower: {bounds.lower_}, Upper: {bounds.upper_}")

# Special bounds
no_bound = NoBound()           # (-inf, inf)
zero = BoundZero()             # (0, 0) - equality constraint
positive = BoundGreaterZero()  # (0, inf)
negative = BoundSmallerZero()  # (-inf, 0)

# inf constant (1e20)
from tesseract_robotics.ifopt import inf
print(f"ifopt.inf = {inf}")  # 1e20
```

| Class | Range | Use Case |
|-------|-------|----------|
| `Bounds(lower, upper)` | `[lower, upper]` | Custom bounds |
| `NoBound()` | `(-inf, inf)` | Unbounded |
| `BoundZero()` | `[0, 0]` | Equality to zero |
| `BoundGreaterZero()` | `[0, inf)` | Non-negative |
| `BoundSmallerZero()` | `(-inf, 0]` | Non-positive |

## VariableSet

Base class for optimization variables.

```python
from tesseract_robotics.ifopt import VariableSet

# VariableSet is abstract - use JointPosition from trajopt_ifopt
from tesseract_robotics.trajopt_ifopt import JointPosition
import numpy as np

joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
values = np.zeros(6)
var = JointPosition(values, joint_names, "position_0")

# VariableSet interface
print(f"Name: {var.GetName()}")
print(f"Rows: {var.GetRows()}")
values = var.GetValues()  # np.ndarray
```

| Method | Returns | Description |
|--------|---------|-------------|
| `GetName()` | `str` | Variable set name |
| `GetRows()` | `int` | Number of variables |
| `GetValues()` | `np.ndarray` | Current values |
| `SetVariables(x)` | `None` | Set values |
| `GetBounds()` | `list[Bounds]` | Variable bounds |

## ConstraintSet

Base class for optimization constraints.

```python
from tesseract_robotics.ifopt import ConstraintSet

# ConstraintSet is abstract - use concrete types from trajopt_ifopt
from tesseract_robotics.trajopt_ifopt import JointPosConstraint

# Constraint interface
print(f"Name: {constraint.GetName()}")
print(f"Rows: {constraint.GetRows()}")
values = constraint.GetValues()  # constraint violations
bounds = constraint.GetBounds()  # constraint bounds
```

| Method | Returns | Description |
|--------|---------|-------------|
| `GetName()` | `str` | Constraint name |
| `GetRows()` | `int` | Number of constraints |
| `GetValues()` | `np.ndarray` | Constraint values |
| `GetBounds()` | `list[Bounds]` | Constraint bounds |

## CostTerm

Base class for optimization cost functions (inherits ConstraintSet).

```python
from tesseract_robotics.ifopt import CostTerm

# Costs return scalar value via GetValues()[0]
# Bounds are typically NoBound() for costs
```

## Composite

Container for variables and constraints.

```python
from tesseract_robotics.ifopt import Composite

# Composite aggregates multiple VariableSets/ConstraintSets
# Used internally by IfoptProblem
```

## Component

Base class for all ifopt components (VariableSet, ConstraintSet, CostTerm).

```python
from tesseract_robotics.ifopt import Component

# Common interface for all optimization components
# GetName(), GetRows(), etc.
```

## Auto-generated API Reference

::: tesseract_robotics.ifopt._ifopt
    options:
      show_root_heading: false
      show_source: false
      members_order: source
