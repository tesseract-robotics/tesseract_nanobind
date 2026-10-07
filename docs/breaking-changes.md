# Breaking API changes

Changes that break existing Python code, newest first, each with what replaces it. The next
release removes `IfoptQPProblem` and `IfoptProblem`: build a `TrajOptQPProblem` instead, and
expect different results wherever a cost went in without a penalty type. Its convex evaluators
take only the QP solution vector. The [changelog](CHANGELOG.md) lists every change; the
[upgrade guide](changes.md) covers the tesseract upgrades in full.

| Release | What breaks | Use instead |
|---|---|---|
| Unreleased | `TaskComposerPluginFactory.createTaskComposerNode` / `createTaskComposerExecutor` no longer return `None` | [`except TaskComposerPluginError`](#createtaskcomposernode-and-createtaskcomposerexecutor-raise) |
| Unreleased | `margin_data_override_type`, `set{Default,Pair}CollisionMarginData`, `CollisionMarginOverrideType`, `get/setPairCollisionMargin` removed | [The tesseract names](#pre-033-collision-margin-aliases-removed) |
| Unreleased | `ContactTestType_*`, `Events_*`, `ModifyAllowedCollisionsType_*` and module-level `CONSOLE_BRIDGE_LOG_*` constants removed | [The enum members](#swig-era-enum-constants-removed) |
| Unreleased | `tesseract_environment.AnyPoly_wrap_EnvironmentConst`, `tesseract_command_language.AnyPoly_*` and `_HAS_TASK_COMPOSER` removed | [`tesseract_task_composer.AnyPoly_*`](#anypoly-helpers-only-in-tesseract_task_composer) |
| Unreleased | `FilesystemPath`, `_FilesystemPath`, `TransformMap`, `Environment.initFromUrdf` / `initFromUrdfSrdf` removed; `Environment.init(str, …)` is content | [`pathlib.Path`](#a-str-is-content-pathlibpath-is-a-file) |
| Unreleased | `evaluateConvexCosts`, `evaluateTotalConvexCost`, `evaluateConvexConstraintViolations` raise `ValueError` on a `var_vals` that is not `getNumQPVars()` long | [The QP solution vector](#the-convex-evaluators-take-the-qp-solution-vector) |
| Unreleased | `trajopt_sqp.IfoptQPProblem` and `trajopt_sqp.IfoptProblem` removed | [`TrajOptQPProblem`](#ifoptqpproblem-and-ifoptproblem-removed) |
| 0.35.0.1 | `planning.Transform` removed; `Pose` is an `Isometry3d` | [`Pose`](#transform-removed-pose-is-an-isometry3d) |
| 0.35.0.1 | tesseract 0.35: `package://tesseract_support/` resource URIs | [`package://tesseract/support/`](#resource-uris-moved) |
| 0.34.1.0 | tesseract 0.34: `ifopt` module, `JointPosition`, `CartPosInfo`, `CollisionCache`, … | [0.33 → 0.34 guide](changes.md#breaking-changes) |

## Unreleased

### `createTaskComposerNode` and `createTaskComposerExecutor` raise

Upstream's `TaskComposerPluginFactory` returns null when it cannot create a node or executor:
the name is not configured, the plugin symbol fails to load, or the plugin factory throws. It
only logs a console_bridge warning. Python got `None`, against a stub that promised the object.
Both methods now raise `TaskComposerPluginError`, a `RuntimeError` (#203). For a name that is
not configured, the message lists the configured names.

| before | after |
| --- | --- |
| `node = factory.createTaskComposerNode(name)`<br>`if node is None: …` | `try: node = factory.createTaskComposerNode(name)`<br>`except TaskComposerPluginError: …` |
| `executor = factory.createTaskComposerExecutor(name)`<br>`if executor is None: …` | `try: executor = factory.createTaskComposerExecutor(name)`<br>`except TaskComposerPluginError: …` |

`TaskComposer.plan` still returns a failed `PlanningResult` for an unknown pipeline; its message
is now the factory's error.

### Pre-0.33 collision-margin aliases removed

tesseract 0.33 renamed the collision-margin members; the bindings kept the old names as aliases
of the new ones. The eight aliases are gone (#177); each replacement takes the same arguments
and does the same thing.

| before | after |
| --- | --- |
| `config.margin_data_override_type` | `config.pair_margin_override_type` |
| `manager.setDefaultCollisionMarginData(m)` | `manager.setDefaultCollisionMargin(m)` |
| `manager.setPairCollisionMarginData(a, b, m)` | `manager.setCollisionMarginPair(a, b, m)` |
| `CollisionMarginOverrideType` | `CollisionMarginPairOverrideType` |
| `margins.getPairCollisionMargin(a, b)` | `margins.getCollisionMargin(a, b)` |
| `margins.setPairCollisionMargin(a, b, m)` | `margins.setCollisionMargin(a, b, m)` |

`manager` is a `DiscreteContactManager` or `ContinuousContactManager`, `config` a
`ContactManagerConfig`, `margins` a `CollisionMarginData`. A `ContactManagerConfig` has no
`margin_data` member (the collision API page used one; that snippet never ran): set its
`default_margin`, `pair_margin_data` and `pair_margin_override_type`, then pass it to
`manager.applyContactManagerConfig(config)`.

### SWIG-era enum constants removed

`tesseract_collision`, `tesseract_environment` and `tesseract_common` no longer export a module
constant per enum value. The 14 constants were copies kept from the SWIG bindings; C++ has
none of them, and every enum is bound as a class whose members are the same objects (#176).
Importing one now raises
`ImportError: cannot import name 'ContactTestType_ALL' from 'tesseract_robotics.tesseract_collision'`;
reading one as an attribute raises `AttributeError`.

| before | after |
| --- | --- |
| `ContactTestType_ALL` (also `_FIRST`, `_CLOSEST`, `_LIMITED`) | `ContactTestType.ALL` (`.FIRST`, `.CLOSEST`, `.LIMITED`) |
| `Events_COMMAND_APPLIED`, `Events_SCENE_STATE_CHANGED` | `Events.COMMAND_APPLIED`, `Events.SCENE_STATE_CHANGED` |
| `ModifyAllowedCollisionsType_ADD` (also `_REMOVE`, `_REPLACE`) | `ModifyAllowedCollisionsType.ADD` (`.REMOVE`, `.REPLACE`) |
| `CONSOLE_BRIDGE_LOG_DEBUG` (also `_INFO`, `_WARN`, `_ERROR`, `_NONE`) | `LogLevel.CONSOLE_BRIDGE_LOG_DEBUG` (…) |

```python
# before
from tesseract_robotics.tesseract_collision import ContactRequest, ContactTestType_ALL
from tesseract_robotics.tesseract_common import CONSOLE_BRIDGE_LOG_WARN, setLogLevel

request = ContactRequest(ContactTestType_ALL)
setLogLevel(CONSOLE_BRIDGE_LOG_WARN)

# after
from tesseract_robotics.tesseract_collision import ContactRequest, ContactTestType
from tesseract_robotics.tesseract_common import LogLevel, setLogLevel

request = ContactRequest(ContactTestType.ALL)
setLogLevel(LogLevel.CONSOLE_BRIDGE_LOG_WARN)
```

The `CONSOLE_BRIDGE_LOG_*` names keep their prefix because they are the C++ enumerator names of
console_bridge's unscoped `LogLevel`; only the module-level copies are gone.

### `AnyPoly` helpers only in `tesseract_task_composer`

`tesseract_environment` re-exported `AnyPoly_wrap_EnvironmentConst`, and
`tesseract_command_language` re-exported `AnyPoly_wrap_CompositeInstruction`,
`AnyPoly_wrap_ProfileDictionary` and `AnyPoly_as_CompositeInstruction`, from
`tesseract_task_composer` inside a `try: … except ImportError: pass`. An install whose
task-composer extension failed to load lost the names without an error, and importing either
package loaded the task composer. The re-exports and `tesseract_command_language._HAS_TASK_COMPOSER`
are removed (#192); the functions are defined in, and imported from, `tesseract_task_composer`.
Code that imports them from the old place now fails with `ImportError: cannot import name …`.

| before | after |
| --- | --- |
| `from tesseract_robotics.tesseract_environment import AnyPoly_wrap_EnvironmentConst` | `from tesseract_robotics.tesseract_task_composer import AnyPoly_wrap_EnvironmentConst` |
| `from tesseract_robotics.tesseract_command_language import AnyPoly_wrap_CompositeInstruction` (also `AnyPoly_wrap_ProfileDictionary`, `AnyPoly_as_CompositeInstruction`) | `from tesseract_robotics.tesseract_task_composer import AnyPoly_wrap_CompositeInstruction` (…) |

### A `str` is content, `pathlib.Path` is a file

`tesseract_common.FilesystemPath` was a `str` subclass, so `Environment.init` had to guess
whether a string was a path or URDF content, and `init(str, str, locator)` loaded *files* where
the C++ `std::string` overload parses *content*. `std::filesystem::path` now converts with
nanobind's caster (`str | os.PathLike` in, `pathlib.Path` out), and `Environment.init` binds the
four native overloads (#165):

| call | meaning |
| --- | --- |
| `init(urdf_xml, locator)`, `init(urdf_xml, srdf_xml, locator)` | parse URDF/SRDF content |
| `init(Path(urdf), locator)`, `init(Path(urdf), Path(srdf), locator)` | load files |
| `init(str(urdf_path), …)` | parsed as XML: returns `False` |
| `init(urdf_xml, Path(srdf), locator)` (either order) | `TypeError` |

The path overloads of `Environment.init` and the `ContactManagersPluginFactory` /
`KinematicsPluginFactory` config constructors accept `os.PathLike` only, never `str`, because
each sits beside a `str` content overload. Every other `std::filesystem::path` parameter takes
`str | os.PathLike`.

| before | after |
| --- | --- |
| `env.init(FilesystemPath(u), FilesystemPath(s), loc)` | `env.init(Path(u), Path(s), loc)` |
| `env.init(path_str, loc)` (path) | `env.init(Path(path_str), loc)` |
| `env.initFromUrdf(xml, loc)` | `env.init(xml, loc)` |
| `env.initFromUrdfSrdf(u_xml, s_xml, loc)` | `env.init(u_xml, s_xml, loc)` |
| `KinematicsPluginFactory(_FilesystemPath(p), loc)` (also `ContactManagersPluginFactory`) | `KinematicsPluginFactory(Path(p), loc)` |
| `TaskComposerPluginFactory(FilesystemPath(p), loc)` | `TaskComposerPluginFactory(str(p), loc)` |
| `TransformMap()` | `{}` |

`TaskComposerPluginFactory` still takes its config path as `str`; its path/content pair is not
bound yet.

### The convex evaluators take the QP solution vector

`QPProblem.evaluateConvexCosts`, `evaluateTotalConvexCost` and
`evaluateConvexConstraintViolations` read `var_vals` in the layout of the QP that the last
`convexify()` built: `getNumQPVars()` entries, the NLP variables followed by the slack variables
([L28][t28]). trajopt documents that size ([L59][h59], [L67][h67]) but does not check it. The
binding now raises `ValueError` for any other size, and before the first `convexify()`, while
`getNumQPVars()` is still 0 ([L99][t99], [L831][t831]):

```text
ValueError: evaluateConvexCosts: var_vals has 3 entries; it must be the QP solution vector of getNumQPVars() = 5 entries, the NLP variables followed by the slack variables
ValueError: evaluateConvexCosts: the problem has no convex model yet; call convexify() first
```

What used to work and now raises: an NLP-sized `var_vals` on a problem with constraint sets
and no hinge or absolute cost. Each constraint row adds one or two slack variables, and trajopt
0.35.0, which every tesseract-robotics-nanobind 0.35.0.x wheel bundles, reads only the NLP block
there, so the call returned the right values. Append zeros for the slack variables:

```python
problem.convexify()
n_slack = problem.getNumQPVars() - problem.getNumNLPVars()
model_costs = problem.evaluateConvexCosts(np.concatenate([x, np.zeros(n_slack)]))
```

A QP solution, such as `SQPResults.new_var_vals` in a callback, already has the right size.
`SQPResults.best_var_vals` needs it too: on trajopt 0.35.0 it is NLP-sized until the first
accepted step and QP-sized after it (always NLP-sized in a wheel built on a trajopt that contains
tesseract-robotics/trajopt#592), so take its first `getNumNLPVars()` entries and pad them as
above. A problem with no constraint sets and only squared costs has no slack variables, so its
NLP point is a QP point and nothing changes.

Why the binding checks: with a hinge or absolute cost, trajopt 0.35.0 multiplies the cost's full
QP rows, slack columns included, into `var_vals` ([L187–L188][t187]), so an NLP-sized vector was
read past its end. On a one-joint problem whose absolute cost is 0.5, a view of the first three
entries of a longer buffer read 3.5 or 0.5, depending on the two entries after the view, and a
fresh three-entry array read 14.6. Before the first `convexify()`, an empty or NLP-sized vector
segfaulted on a squared cost. The rule covers all three evaluators, although trajopt 0.35.0 reads
only the NLP block in two of them: one contract, the one trajopt documents for all three once
tesseract-robotics/trajopt#592 is in.

[t28]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L28
[t99]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L99
[t187]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L187-L188
[t831]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L831
[h59]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/include/trajopt_sqp/qp_problem.h#L59
[h67]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/include/trajopt_sqp/qp_problem.h#L67

### `IfoptQPProblem` and `IfoptProblem` removed

`tesseract_robotics.trajopt_sqp` no longer binds `IfoptQPProblem` or `IfoptProblem`. trajopt is
removing `IfoptQPProblem` (tesseract-robotics/trajopt#595), and `IfoptProblem` was bound only to
construct it. Code that uses either now fails with
`AttributeError: module 'tesseract_robotics.trajopt_sqp' has no attribute 'IfoptQPProblem'`
(or `'IfoptProblem'`). Build a `TrajOptQPProblem` over the same variables, and add every set to
it, each cost with a penalty type (#148):

```python
# before
nlp = tsqp.IfoptProblem(nodes_variables)
nlp.addConstraintSet(joint_constraint)
nlp.addCostSet(vel_cost)                        # no penalty type
problem = tsqp.IfoptQPProblem(nlp)
problem.addConstraintSet(collision_constraint)
problem.setup()

# after
problem = tsqp.TrajOptQPProblem(nodes_variables)
problem.addConstraintSet(joint_constraint)
problem.addConstraintSet(collision_constraint)
problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)
problem.setup()
```

It is not a rename. In trajopt 0.35.0, which every tesseract-robotics-nanobind 0.35.0.x wheel
bundles (`TrajOptQPProblem` first ships in 0.35.0.9; earlier releases bind `IfoptQPProblem`),
the two problems differ in five ways, and the third and fourth change results:

| | `IfoptQPProblem` (removed) | `TrajOptQPProblem` |
|---|---|---|
| **1. Per set, not per row** | One merit coefficient, name and violation per constraint row; names are `<set>_<row>` ([L93][i93], [L100][i100]). | One per constraint set ([L668][t668], [L675][t675], [L1021–L1043][t1021]). Code that indexes names, violations or merit coefficients by row breaks, and the solver inflates a whole set's merit coefficient at once. |
| **2. `addCostSet` checks bounds** | Checks nothing; rejects `HINGE` ([L50–L78][i50]). | `SQUARED` and `ABSOLUTE` need equality bounds on every row, `HINGE` one-sided bounds; anything else, a range row included, raises ([L416–L476][t416]). |
| **3. No raw-cost path** | A cost added with `IfoptProblem.addCostSet` bypasses `IfoptQPProblem::addCostSet` ([L59][i59]): it gets no gradient and no Hessian in the QP ([L190][i190], [L263][i263]), while the merit still reads its raw, signed rows ([L525–L531][i525]). | Every cost goes in with `addCostSet(set, penalty_type)`. A squared cost that was inert now shapes the steps; for `ABSOLUTE` and `HINGE` costs on trajopt 0.35.0, see the fourth difference. |
| **4. The merit matches the model for squared costs** | Weights the model but not the merit, and counts the model once per cost term, so the solver can reject every step and stop at the seed (tesseract-robotics/trajopt#595). | The merit weights and counts squared costs as the model does, so steps the old ratio rejected can be accepted. trajopt 0.35.0 has two exceptions, both fixed by tesseract-robotics/trajopt#592 (merged 2026-09-30; the fix arrives only with a wheel built on a newer trajopt, none yet). Constraint coefficients weight a set's slack in the QP ([L798][t798]) but not its violation in the merit ([L1021–L1043][t1021]). The model reads an `ABSOLUTE` or `HINGE` cost as 0 at every QP solution ([L166–L196][t166]) while its exact cost ignores its coefficient ([L1015][t1015]), so the model predicts the whole cost away and the solver rejects a step that removes less than `improve_ratio_threshold` of it; a violation the trust box cannot cut by that much stops at its seed ([the QP problem](api/trajopt_sqp.md#the-qp-problem)). |
| **5. Dynamic sets** | Throws on `isDynamic()` sets ([L42–L48][i42]). | Accepts them ([L404–L414][t404]). |

How much the third difference moves a result: this package's online SQP example added its velocity
cost with `IfoptProblem.addCostSet`, so its QP had no cost term and OSQP solved with an empty
Hessian. As a squared cost on `TrajOptQPProblem`, the global solve's summed squared joint steps
fell from 23.2 to 3.9, against 3.6 for the straight-line seed, and `getTotalExactCost()` now reports
that sum instead of the first velocity row (0.5).

!!! note "Evidence"
    The five differences are read from the trajopt 0.35.0 sources linked above. The example's
    numbers are measured on this repository's `online_planning_sqp_example.py`, and
    `tests/examples/test_online_planning_sqp_example.py` pins its problem class, its cost as the
    squared joint steps, and a global solve that reaches the target from the pinned start. How
    much results move in other code depends on its costs; nothing here measures that. The
    `ABSOLUTE`/`HINGE` exception is also measured: `TestTrajOptQPProblemPenaltyCosts` in
    `tests/trajopt_sqp/test_trajopt_sqp_bindings.py` pins it.

[i42]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L42-L48
[i50]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L50-L78
[i59]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L59
[i93]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L93
[i100]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L100
[i190]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L190
[i263]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L263
[i525]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L525-L531
[t166]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L166-L196
[t404]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L404-L414
[t416]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L416-L476
[t668]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L668
[t675]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L675
[t798]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L798
[t1015]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L1015
[t1021]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L1021-L1043

## 0.35.0.1

### `Transform` removed; `Pose` is an `Isometry3d`

`tesseract_robotics.planning.Pose` now subclasses `Isometry3d`. Its rotation math runs in Eigen,
its numpy-property facade is gone, its stored data are properties, and its factories collapse to
one name each that accepts both scalar-positional and array-like arguments. The `Transform` alias
is removed: use `Pose` (#76).

### Resource URIs moved

tesseract 0.35 folds its support data into the `tesseract` package, so
`package://tesseract_support/...` is now `package://tesseract/support/...` in every URDF, SRDF and
`locateResource` call. The rest of that upgrade leaves the Python API unchanged; see the
[0.34 → 0.35 guide](changes.md#urdfsrdf-resource-uri-scheme).

## 0.34.1.0

The tesseract 0.34 upgrade removed the `ifopt` module (its types moved to `trajopt_ifopt`) and
replaced `JointPosition` with `Var` / `Node` / `NodesVariables`. It also removed `CartPosInfo` and
`CollisionCache`, dropped `getStaticKey()` from profiles, and renamed the cost and constraint
evaluation methods. The [0.33 → 0.34 guide](changes.md#breaking-changes) has each change with its
replacement.
