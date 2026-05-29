# API Changes: 0.34 → 0.35

## C++ Dependency Versions

| Package | 0.34 | 0.35 |
|---------|------|------|
| tesseract | 0.34.1 | 0.35.0 |
| tesseract_planning | 0.34.0 | 0.35.0 |
| trajopt | 0.34.4 | 0.35.0 |
| descartes_light | 0.4.9 | 0.4.10 |
| opw_kinematics | 0.5.2 | 0.5.3 |
| ruckig | `pantor/ruckig @ v0.15.3` | `Levi-Armstrong/ruckig @ cpack-v0.9.2` |

`ros_industrial_cmake_boilerplate` (0.7.4), `boost_plugin_loader` (0.4.3), and `ifopt` (2.1.4) were already at latest.

## Upstream Structural Changes

Both `tesseract` and `tesseract_planning` consolidated their many small CMake packages into single multi-component packages. The upstream `MIGRATION.md` files (in each repo at tag `0.35.0`) document the full mapping; below are the highlights this binding repo has to absorb.

### Include path remap

| Old | New |
|-----|-----|
| `<tesseract_common/X.h>` | `<tesseract/common/X.h>` |
| `<tesseract_collision/core/X.h>` | `<tesseract/collision/X.h>` (`core/` flattened) |
| `<tesseract_kinematics/core/X.h>` | `<tesseract/kinematics/X.h>` (`core/` flattened) |
| `<tesseract_motion_planners/core/X.h>` | `<tesseract/motion_planners/X.h>` (`core/` flattened) |
| `<tesseract_task_composer/core/X.h>` | `<tesseract/task_composer/X.h>` (`core/` flattened) |

The full set of subpackages (`geometry`, `scene_graph`, `state_solver`, `srdf`, `urdf`, `environment`, `visualization`, `support`, `command_language`, `time_parameterization`, `examples`) all move under `tesseract/`.

### C++ namespace remap

| Old | New |
|-----|-----|
| `tesseract_common::Foo` | `tesseract::common::Foo` |
| `tesseract_collision::Foo` | `tesseract::collision::Foo` |
| ... (every `tesseract_<name>::` becomes `tesseract::<name>::`) | |

### CMake target rename

`tesseract::tesseract_<name>` → `tesseract::<name>` across the board.

### find_package consolidation

```cmake
# 0.34
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core bullet)
find_package(tesseract_kinematics REQUIRED COMPONENTS core kdl)

# 0.35
find_package(tesseract REQUIRED
             COMPONENTS common collision collision_bullet kinematics kinematics_kdl)
```

Same pattern for `tesseract_planning`: one `find_package(tesseract_planning REQUIRED COMPONENTS ...)` replaces all the per-subpackage calls.

### dependencies file rename

`dependencies.rosinstall` → `dependencies.repos` (matches upstream's ros2 vcs-tool convention). File format unchanged (YAML, `vcs import` reads both extensions).

### ruckig: switch to Levi-Armstrong fork

`tesseract_planning` 0.35.0 pins ruckig at `Levi-Armstrong/ruckig.git @ cpack-v0.9.2` (a fork with a cpack patch). We had been diverging on upstream `pantor/ruckig @ v0.15.3`; this release brings us back in line with what upstream CI builds against.

## Python API Changes

Surface-level Python API is unchanged. The breaks below are infrastructure/resource-resolution, not user-facing types.

### URDF/SRDF resource URI scheme

`package://tesseract_support/` → `package://tesseract/support/` everywhere.

In 0.34, `tesseract_support` was its own ROS-style package (with its own `package.xml`). In 0.35, the support data is a subdirectory of the unified `tesseract` package, so the URI prefix changed. Affects every URDF/SRDF reference in tests/examples (25 files in this repo). Upstream's own URDFs at `ws/install/share/tesseract/support/urdf/` already use the new URI.

### Dev-workspace directory renames

Resource paths used by `tesseract_robotics/__init__.py` (editable-install fallback) and `scripts/env.sh`:

| 0.34 path | 0.35 path |
|-----------|-----------|
| `ws/src/tesseract/tesseract_support/` | `ws/src/tesseract/support/` |
| `ws/src/tesseract_planning/tesseract_task_composer/` | `ws/src/tesseract_planning/task_composer/` |

The internal wheel-data layout (`tesseract_robotics/data/tesseract_support/`) is unchanged — that's our naming, not upstream's.

### Cereal serialization registry — consumer-side workaround obsoleted

In 0.34, the `tesseract_serialization_bindings.cpp` had a hand-rolled mirror of upstream's `CEREAL_REGISTER_TYPE` / `CEREAL_REGISTER_POLYMORPHIC_RELATION` macros to populate the Windows .pyd's per-DLL cereal registry. In 0.35, `<tesseract/command_language/cereal_serialization.h>` transitively includes a new `cereal_serialization_impl.hpp` registration-only header. Every consumer TU that includes the .h now gets the registrations automatically — including Windows .pyd consumers. The consumer-side mirror is now actively harmful (triggers `redefinition of binding_name<...>` errors); we deleted it.

### namespace aliases — `tesseract_planning` umbrella is gone

`namespace tesseract_planning` (the 0.34 umbrella for command_language + motion_planners + task_composer + time_parameterization types) has no 0.35 equivalent — each subpackage has its own nested `tesseract::<X>` namespace. Binding files that used `namespace tp = tesseract_planning;` have been remapped per-file to whichever sub-namespace dominates:

- `tesseract_command_language_bindings.cpp` → `tesseract::command_language`
- `tesseract_motion_planners*_bindings.cpp` → `tesseract::motion_planners`
- `tesseract_task_composer*_bindings.cpp` → `tesseract::task_composer`
- `tesseract_time_parameterization_bindings.cpp` → `tesseract::time_parameterization`

Three files (simple/task_composer/time_param) reference `CompositeInstruction` (lives in command_language) — they additionally get a `tcl = tesseract::command_language` alias.

### Test suite status (macOS arm64)

Post-upgrade: **560 passed, 7 failed, 14 skipped** (vs pre-upgrade: 352 passed, 95 failed, 92 errors on first build, before any Python-side fixes).

The 7 remaining failures are all `tests/benchmarks/test_parallel_scaling.py` "harder problem" stress cases throwing `Planning failed: ErrorTask: Error (Abort Triggered)`. Likely an algorithmic-tolerance change in 0.35's task composer rather than a binding-layer break. Out of scope for the upgrade milestone.

## Migration Checklist

- [x] Bump versions in `dependencies.repos`
- [x] Switch to `Levi-Armstrong/ruckig` fork
- [x] Rename `dependencies.rosinstall` → `dependencies.repos`
- [x] Update build script + CI workflows to reference new filename
- [x] Apply upstream sed scripts (tesseract + tesseract_planning `MIGRATION.md`) to `src/` and `CMakeLists.txt`
- [x] Manual sweep for nanobind-specific cases sed misses (namespace aliases, redundant cereal registrations)
- [x] Rebuild C++ workspace (`pixi run build-cpp`)
- [x] Rebuild bindings — all 23 modules import clean
- [x] Update resource paths (`__init__.py`, `env.sh`, CMakeLists)
- [x] Rewrite `package://tesseract_support/` URIs across tests/examples
- [x] Run test suite; document Python-side breaks
- [ ] Triage 7 `parallel_scaling` benchmark regressions (likely tolerance change, not binding break)
- [ ] Verify Linux + Windows wheel builds
- [ ] Update CLAUDE.md re: cereal Windows-DLL workaround (now obsoleted by upstream)

---

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
