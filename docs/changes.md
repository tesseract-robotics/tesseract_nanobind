# API Changes: 0.33 → 0.34

## C++ Dependency Versions

| Package | 0.33 | 0.34 |
|---------|------|------|
| tesseract | 0.33.0 | 0.34.1 |
| tesseract_planning | 0.33.0 | 0.34.0 |
| trajopt | 0.33.0 | 0.34.4 |
| ifopt | 2.1.3 | 2.1.4 |
| ruckig | 0.9.2 | 0.15.3 |

## Breaking Changes

### `ifopt` module removed

The standalone `tesseract_robotics.ifopt` module no longer exists. All types moved into `tesseract_robotics.trajopt_ifopt`.

```python
# 0.33
from tesseract_robotics import ifopt
bounds = ifopt.Bounds(-1.0, 1.0)
problem = ifopt.Problem()

# 0.34
from tesseract_robotics import trajopt_ifopt as ti
bounds = ti.Bounds(-1.0, 1.0)
# Problem is now IfoptProblem (see below)
```

### Variable system redesign: `JointPosition` → `Var` / `Node` / `NodesVariables`

The `JointPosition` class is removed. The new API uses a three-level hierarchy:

- **`Var`** — a single variable (e.g. joint values at one waypoint)
- **`Node`** — groups multiple `Var`s (e.g. joints + velocities)
- **`NodesVariables`** — container of `Node`s passed to the problem constructor

Use the `createNodesVariables` factory to build the full hierarchy in one call:

```python
# 0.33
from tesseract_robotics import ifopt
vars_list = []
for i, state in enumerate(initial_states):
    var = ti.JointPosition(state, joint_names, f"Joint_{i}")
    var.SetBounds(bounds)
    problem.AddVariableSet(var)
    vars_list.append(var)

# 0.34
nodes_variables = ti.createNodesVariables(
    "trajectory", joint_names, initial_states, bounds
)
nlp = tsqp.IfoptProblem(nodes_variables)

vars_list = []
for node in nodes_variables.getNodes():
    vars_list.append(node.getVar("joints"))
```

### Problem creation

`IfoptQPProblem` no longer has a default constructor. Create it from an `IfoptProblem`:

```python
# 0.33
problem = tsqp.IfoptQPProblem()
problem.addVariableSet(var)
problem.addConstraintSet(constraint)

# 0.34
nlp = tsqp.IfoptProblem(nodes_variables)     # NLP with variables
nlp.addConstraintSet(joint_constraint)        # joint/cartesian constraints on NLP
nlp.addCostSet(vel_cost)                      # costs on NLP
problem = tsqp.IfoptQPProblem(nlp)            # QP wrapper
problem.addConstraintSet(collision_constraint) # collision constraints on QP
problem.setup()                                # must call after adding all sets
```

### `CartPosInfo` removed

`CartPosConstraint` now takes parameters directly instead of a `CartPosInfo` struct:

```python
# 0.33
info = ti.CartPosInfo()
info.manipulator = manip
info.source_frame = "tool0"
info.target_frame = "world"
info.target = target_tf
constraint = ti.CartPosConstraint(info, var, "Target")

# 0.34
constraint = ti.CartPosConstraint(
    var,                      # Var (not JointPosition)
    manip,                    # KinematicGroup
    "tool0",                  # source_frame
    "world",                  # target_frame
    Isometry3d.Identity(),    # source_frame_offset
    target_tf,                # target_frame_offset
    "Target",                 # name
)
```

### `CollisionCache` removed

Collision evaluators no longer take a `CollisionCache` parameter — caching is now internal:

```python
# 0.33
cache = ti.CollisionCache(100)
evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, config, cache, True)

# 0.34
evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, config, True)
```

Same for `LVSDiscreteCollisionEvaluator` and `LVSContinuousCollisionEvaluator`.

### Constraint constructors take `Var` instead of `JointPosition`

All constraint constructors now accept `Var` references:

```python
# 0.33 — JointPosition (list)
ti.JointPosConstraint(target, [var], coeffs, "name")
ti.JointVelConstraint(target, [var0, var1, ...], coeffs, "name")
ti.DiscreteCollisionConstraint(evaluator, joint_pos, ...)

# 0.34 — Var (single or list)
ti.JointPosConstraint(target, var, coeffs, "name")        # single Var
ti.JointVelConstraint(target, [var0, var1, ...], coeffs, "name")  # list of Var
ti.DiscreteCollisionConstraint(evaluator, var, ...)        # single Var
```

### `getStaticKey()` removed from profiles

All profile classes (`TrajOptMoveProfile`, `OMPLMoveProfile`, `DescartesMoveProfileD`, etc.) no longer have a static `getStaticKey()` method. Use the instance method `getKey()` instead:

```python
# 0.33
key = TrajOptDefaultPlanProfile.getStaticKey()

# 0.34
profile = TrajOptDefaultPlanProfile()
key = profile.getKey()
```

### Cost/constraint evaluation methods renamed

```python
# 0.33
cost = problem.evaluateTotalExactCost(var_vals)
costs = problem.evaluateExactCosts(var_vals)

# 0.34
cost = problem.getTotalExactCost()     # no arguments
costs = problem.getExactCosts()        # no arguments
```

### Serialization: Boost → Cereal

The serialization backend changed from Boost.Serialization to Cereal. The Python API is identical — `toArchiveStringXML`, `fromArchiveStringXML`, etc. all work the same way. Only relevant if you have existing serialized files from 0.33 (they are not compatible with 0.34).

## Non-Breaking Changes

### Enum renames (Python names preserved)

`trajopt_sqp` enums were renamed in C++ to `kCamelCase` style, but the Python-facing names are unchanged:

```python
# These all still work
SQPStatus.RUNNING
SQPStatus.NLP_CONVERGED
QPSolverStatus.UNITIALIZED
CostPenaltyType.SQUARED
```

### `ConstraintType` enum removed

The `ConstraintType` enum from `trajopt_sqp` no longer exists. If you referenced it, remove the usage.

### Warm-starting SQP

Use `NodesVariables.setVariables()` to set warm-start values:

```python
# Set warm-start from previous trajectory
nodes_variables.setVariables(trajectory.flatten())
solver.init(problem)
solver.stepSQPSolver()
```

## Migration Checklist

- [ ] Replace `from tesseract_robotics import ifopt` → `from tesseract_robotics import trajopt_ifopt`
- [ ] Replace `JointPosition` with `createNodesVariables` factory + `Var` refs
- [ ] Replace `IfoptQPProblem()` → `IfoptProblem(nodes_variables)` + `IfoptQPProblem(nlp)`
- [ ] Replace `CartPosInfo` struct with direct `CartPosConstraint` parameters
- [ ] Remove `CollisionCache` from evaluator constructors
- [ ] Replace `evaluateTotalExactCost(x)` → `getTotalExactCost()`
- [ ] Replace `getStaticKey()` → `getKey()` (instance method)
- [ ] Call `problem.setup()` after adding all constraint/cost sets
