# Breaking API changes

The migration table for Python code, newest first: each breaking change with what replaces it. 0.36 is
a breaking release and removes names outright, with no aliases and no deprecation warnings. The
[changelog](CHANGELOG.md) is the release log, [0.35 → 0.36](changes-0.36.md) the narrative
migration with before/after examples, and [0.33 → 0.35](changes.md) the earlier tesseract upgrades.

| Release | What breaks | Use instead |
|---|---|---|
| 0.36.0 | Python modules lose the `tesseract_` prefix (`tesseract_robotics.tesseract_collision` → `tesseract_robotics.collision`, 20 in all); the old names are gone and raise `ModuleNotFoundError` | [The new names](#036-module-names) |
| 0.36.0 | `TaskComposer.plan` raises `ProfileTypeMismatchError` when a Descartes node's namespace holds a profile only as the other scalar type (`D` for a Cartesian or raster pipeline); `create_descartes_default_profiles(move_profile=…)` refuses a Python subclass of `DescartesMoveProfileD` / `F` | [Register the type the node reads](#descartes-profiles-match-the-nodes-scalar-type) |
| 0.36.0 | Python 3.9, 3.10 and 3.11 are no longer supported; Rhino 8 loses wheels | [Python 3.12 or later, or 0.35.0.8](#python-312-is-the-minimum) |
| 0.36.0 | `planning.program.Target` and `emitters.core.events.Event` are `type` aliases, which `isinstance` rejects | [Their `__value__`, or the member classes](#target-and-event-are-type-aliases) |
| 0.36.0 | console_bridge logging API removed (`setLogLevel`, `useOutputHandler`, `OutputHandler`, `CONSOLE_BRIDGE_LOG_*`, …): upstream logs through spdlog | [`getLogger()`, `addLogRecordHandler()`](api/common.md#logging) |
| 0.36.0 | `CollisionCoeffData.setPairCollisionCoeff` / `getPairCollisionCoeff` removed | [The C++ names](#setpaircollisioncoeff-and-getpaircollisioncoeff-removed) |
| 0.36.0 | `CollisionCoeffData`, `TrajOptCollisionConfig`, `SQPParameters` compare by value and are unhashable | [A stable key, or `id()`](#trajopt-value-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | A collision constraint on a `Var` outside a `NodesVariables` raises `VarNotInVariableSetError` (a `ValueError`) at construction instead of segfaulting on its first `update()` | [Hand the `Node` over first](#collision-constraints-need-a-var-in-a-variable-set) |
| 0.36.0 | `Node.getParent` raises `NodeParentUnavailableError` for planner-owned nodes, including inside `create` overrides | [Keep the planner's `Node`s, not their set](#nodegetparent-raises-for-planner-owned-nodes) |
| 0.36.0 | `OSQPEigenSolver.update*` raise `QPSizeError` on a wrong-length vector | [Size by `init()`](#osqpeigensolver-updates-check-sizes) |
| 0.36.0 | 16 command-language waypoint, instruction and Poly types compare by value and are unhashable | [A stable key, or `id()`](#command-language-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | `TOTGCompositeProfile`, `ISPCompositeProfile`, `ISPMoveProfile`, `ConstantTCPSpeedCompositeProfile` compare by value and are unhashable | [A stable key, or `id()`](#time-parameterization-profiles-compare-by-value-and-are-unhashable) |
| 0.36.0 | The seven simple-planner move profiles compare by value and are unhashable | [A stable key, or `id()`](#simple-planner-move-profiles-compare-by-value-and-are-unhashable) |
| 0.36.0 | `OMPLSolverConfig.getNumPlanners()` removed | [`len(cfg.planners)`](#omplsolverconfiggetnumplanners-removed) |
| 0.36.0 | OMPL, TrajOpt and task-composer planning value types compare by value and are unhashable | [A stable key, or `id()`](#planner-value-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | `create_ompl_planner_configurators` raises `ValueError` on a keyword none of the requested planners has | [Drop the keyword](#create_ompl_planner_configurators-rejects-unused-keywords) |
| 0.36.0 | 7 TrajOpt-Ifopt and Descartes profiles and waypoint configs compare by value and are unhashable | [A stable key, or `id()`](#trajopt-ifopt-and-descartes-profiles-compare-by-value-and-are-unhashable) |
| 0.36.0 | Overriding a hook (`generate`, `create`, `create*`) on a Python subclass of a concrete profile raises `ProfileOverrideNotDispatchedError` at class definition; the override was silently never called | [Subclass the abstract base and hold the concrete profile](#overriding-a-hook-on-a-concrete-profile-raises) |
| 0.36.0 | `TaskComposerPluginFactory(str, locator)` and `TaskComposerServer.loadConfig(str, locator)` read the `str` as YAML text | [A `pathlib.Path`](#taskcomposerpluginfactory-a-str-is-yaml-text) |
| 0.36.0 | `createTaskComposerPluginFactory`, `createTaskComposerDataStorage`, `TaskComposerKeys.addKeys`, `TaskComposerDataStorage.getAllData`, `TaskComposerNodeInfoContainer.getAllInfos` / `getAbortingNodeInfo` and the container overloads of `getDotgraph` / `saveDotgraph` removed | [The C++ names](#task-composer-compatibility-helpers-removed) |
| 0.36.0 | `TaskComposerNode.saveDotgraph` raises `OSError` instead of returning `False`, and returns `None` | [Catch `OSError`](#taskcomposernodesavedotgraph-raises-oserror) |
| 0.36.0 | `TaskComposerKeys.get` of a list port returns `list[str]`; `TaskComposerNodeInfo` fields are writable | [Task composer value types](#task-composer-value-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | `AnyPoly`, `TaskComposerKeys`, `TaskComposerDataStorage`, `TaskComposerNodeInfo`, `TaskComposerNodeInfoContainer`, `TaskComposerContext`, `TaskComposerNodePorts`, `TaskComposerLog` compare by value and are unhashable | [A stable key, or `id()`](#task-composer-value-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | `command_language.MoveInstructionType_*` constants removed | [The `MoveInstructionType` members](#swig-era-moveinstructiontype_-constants-removed) |
| 0.36.0 | `MoveInstruction(Poly).assign{Cartesian,Joint,State}Waypoint`, `ProfileDictionary_addProfile` removed | [`setWaypoint`, `addProfile`](#assignwaypoint-and-profiledictionary_addprofile-removed) |
| 0.36.0 | `*_wrap_*`, `InstructionPoly_as_*`, `WaypointPoly_as_*`, `CompositeInstruction.appendMoveInstruction` removed; `wp.as…Waypoint()` is a reference | [Constructors and `as…()` methods](#swig-era-poly-helpers-and-appendmoveinstruction-removed) |
| 0.36.0 | `TOTGCompositeProfile`, `ISPCompositeProfile`, `ISPMoveProfile`, `ConstantTCPSpeedCompositeProfile` renamed | [The C++ names](#time-parameterization-profiles-renamed-to-their-c-names) |
| 0.36.0 | `SQPStatus.NLP_CONVERGED`, `OPT_TIME_LIMIT`, `QP_SOLVER_ERROR`, `CALLBACK_STOPPED`, `QPSolverStatus.UNITIALIZED`, `QP_ERROR` renamed | [The C++ names](#six-trajopt_sqp-enumerators-renamed-to-their-c-names) |
| 0.36.0 | `ProfileDictionary_add*`, `*_as_ProfileConstPtr`, `cast_Descartes*`, the `…PlanProfile` aliases and `motion_planners_trajopt.TrajOptCollisionConfig` / `CollisionEvaluatorType` removed | [`addProfile` and the `…MoveProfile` names](#planner-module-compatibility-names-removed) |
| 0.36.0 | `create_time_optimal_parameterization(path_tolerance, min_angle_change)`, `create_iterative_spline_parameterization(add_points)`: the ignored arguments are removed | [Set the composite profile's fields](#time-parameterization-factories-take-no-arguments) |
| 0.36.0 | `tesseract_robotics.ompl_base` and `tesseract_reeds_shepp_example` removed | [`ompl_base` removed](#ompl_base-removed) |
| 0.36.0 | `ChangeCollisionMarginsCommand(pair_margin_data)` merges the pair margins (`MODIFY`) instead of replacing them | [Pass `REPLACE`](#changecollisionmarginscommandpair_margin_data-defaults-to-modify) |
| 0.36.0 | `MotionProgram` / `get_manipulator_info` require `tcp_frame` and `working_frame`; `fk` / `ik` require `tip_link`; `to_composite_instruction` has no `tcp_frame` | [Name the frames](#the-planning-api-no-longer-picks-a-tcp-or-working-frame) |
| 0.36.0 | `geometry.PointCloudPoint` renamed | [`PointCloud.Point`](#pointcloudpoint-renamed-to-pointcloudpoint) |
| 0.36.0 | `scene_graph.JointType_*` constants removed | [The `JointType` members](#swig-era-jointtype_-constants-removed) |
| 0.36.0 | 37 collision, common and environment value types compare by value and are unhashable: `hash()`, `set` members and `dict` keys raise `TypeError` | [A stable key, or `id()`](#value-types-compare-by-value-and-are-unhashable) |
| 0.36.0 | `TaskComposerPluginFactory.createTaskComposerNode` / `createTaskComposerExecutor` no longer return `None` | [`except TaskComposerPluginError`](#createtaskcomposernode-and-createtaskcomposerexecutor-raise) |
| 0.36.0 | `margin_data_override_type`, `set{Default,Pair}CollisionMarginData`, `CollisionMarginOverrideType`, `get/setPairCollisionMargin` removed | [The tesseract names](#pre-033-collision-margin-aliases-removed) |
| 0.36.0 | `ContactTestType_*`, `Events_*`, `ModifyAllowedCollisionsType_*` and module-level `CONSOLE_BRIDGE_LOG_*` constants removed | [The enum members](#swig-era-enum-constants-removed) |
| 0.36.0 | `environment.AnyPoly_wrap_EnvironmentConst`, `command_language.AnyPoly_*` and `_HAS_TASK_COMPOSER` removed | [`task_composer.AnyPoly_*`](#anypoly-helpers-only-in-task_composer) |
| 0.36.0 | `FilesystemPath`, `_FilesystemPath`, `TransformMap`, `Environment.initFromUrdf` / `initFromUrdfSrdf` removed; `Environment.init(str, …)` is content | [`pathlib.Path`](#a-str-is-content-pathlibpath-is-a-file) |
| 0.36.0 | `evaluateConvexCosts`, `evaluateTotalConvexCost`, `evaluateConvexConstraintViolations` raise `QPSolutionSizeError` (a `ValueError`) on a `var_vals` that is not `getNumQPVars()` long | [The QP solution vector](#the-convex-evaluators-take-the-qp-solution-vector) |
| 0.36.0 | `trajopt_sqp.IfoptQPProblem` and `trajopt_sqp.IfoptProblem` removed | [`TrajOptQPProblem`](#ifoptqpproblem-and-ifoptproblem-removed) |
| 0.36.0 | `Quaterniond(w, x, y, z)` and `Quaterniond(coeffs=)` removed | [`from_xyzw`, `Quaterniond([x, y, z, w])`](#quaterniond-takes-xyzw-only) |
| 0.36.0 | `environment.EventCallbackFn` removed; `addEventCallback(fn)` returns the key; `removeEventCallback(hash=)` is `key=`; `getJointGroup(name=)` is `group_name=` | [Environment events and keywords](#environment-events-and-keywords) |
| 0.36.0 | `Environment.getStateByMap`, `getStateByNamesAndValues`, `setStateByNamesAndValues`, `getCurrentJointValuesByNames`, `StateSolver.setStateByMap` / `setStateByNamesAndValues`, `JointGroup.calcJacobianWithPoint`, `KinematicGroup.calcInvKinMultiple` removed | [The native overloads](#overload-copies-removed-from-environment-statesolver-and-kinematics) |
| 0.36.0 | `common.VectorVector3d` and `VectorIsometry3d` removed | [Plain lists](#vectorvector3d-and-vectorisometry3d-removed) |
| 0.36.0 | Missing ids raise `KeyError`; `as*Instruction()` return views and a wrong cast raises `PolyTypeError`; `applyCommand` is one overload | [Errors and views](#environment-and-poly-lookups-raise-and-casts-are-views) |
| 0.36.0 | `Isometry3d(matrix)` takes rotations only (`NotRotationError`); a zero quaternion or axis raises `DegenerateRotationError`; `Hyperplane3d.Through` follows Eigen | [Geometry inputs](#geometry-inputs-are-validated) |
| 0.36.0 | `serialization.*_to_file` return `None` and raise `OSError`; binary archives are `bytes`; `urdf.writeURDFFile` returns the path and raises `URDFWriteError` | [Files and bytes](#serialization-and-urdf-writes-raise-and-bytes-are-bytes) |
| 0.36.0 | A plugin-factory config path the locator cannot resolve raises `FileNotFoundError`; `trajopt_ifopt` is a package; Python constraint and Descartes profile errors are named | [Named errors](#named-errors-in-the-bindings) |
| 0.36.0 | `Robot.ik` raises `IKNoSolutionError`; `Robot` mutators return `None` and raise `EnvironmentCommandError`; planning failures are named exceptions | [Planning errors](#the-planning-api-raises-instead-of-returning-flags) |
| 0.36.0 | `MotionProgram.start_at`, `create_descartes_pipeline_profiles`, `TRAJOPT_PROFILE_NAMES`, `CartesianTarget(position=, quaternion=)`, `TaskComposer(config_path=)` removed; `_create_trajopt_profiles` is `create_trajopt_profile_pair` | [Planning names](#planning-names-removed-or-renamed) |
| 0.36.0 | `TaskComposer.plan` raises for programming and configuration errors; `plan_ompl` defaults to `FreespacePipeline`; a pipeline without default profiles raises `NoDefaultProfilesError` | [Plan errors](#taskcomposerplan-raises-and-plan_ompl-uses-freespacepipeline) |
| 0.36.0 | `get_tesseract_resource_path` has one source (the support wheel); `__version__` has no placeholder; `from scene_graph import *` omits `SceneState`; `planning`, `common`, `scene_graph` have no `__all__` | [Package surface](#package-surface) |
| 0.36.0 | Examples' `main()` raise `PlanningFailedError` when a plan fails | [Examples](#examples-raise-planningfailederror) |
| 0.36.0 | Emitters take frames from the `ManipulatorInfo` (`frames=`, `UnboundFrameError`); `emit_rapid` returns an `EmittedProgram` and needs a posture solver for confdata; JBI needs a `JbiPulseCalibration`; `SetToolInstruction` raises; names are validated | [Emitters](#emitters-take-frames-from-the-manipulatorinfo) |
| 0.35.0.1 | `planning.Transform` removed; `Pose` is an `Isometry3d` | [`Pose`](#transform-removed-pose-is-an-isometry3d) |
| 0.35.0.1 | tesseract 0.35: `package://tesseract_support/` resource URIs | [`package://tesseract/support/`](#resource-uris-moved) |
| 0.34.1.0 | tesseract 0.34: `ifopt` module, `JointPosition`, `CartPosInfo`, `CollisionCache`, … | [0.33 → 0.34 guide](changes.md#breaking-changes) |

## 0.36.0

### 0.36 module names

The `tesseract_` prefix is gone from 20 Python module names (issue #42). It repeated the package name
(`tesseract_robotics.tesseract_collision`).

| Old (deprecated) | New |
|---|---|
| `tesseract_robotics.tesseract_collision` | `tesseract_robotics.collision` |
| `tesseract_robotics.tesseract_command_language` | `tesseract_robotics.command_language` |
| `tesseract_robotics.tesseract_common` | `tesseract_robotics.common` |
| `tesseract_robotics.tesseract_environment` | `tesseract_robotics.environment` |
| `tesseract_robotics.tesseract_geometry` | `tesseract_robotics.geometry` |
| `tesseract_robotics.tesseract_kinematics` | `tesseract_robotics.kinematics` |
| `tesseract_robotics.tesseract_motion_planners` | `tesseract_robotics.motion_planners` |
| `tesseract_robotics.tesseract_motion_planners_descartes` | `tesseract_robotics.motion_planners_descartes` |
| `tesseract_robotics.tesseract_motion_planners_ompl` | `tesseract_robotics.motion_planners_ompl` |
| `tesseract_robotics.tesseract_motion_planners_simple` | `tesseract_robotics.motion_planners_simple` |
| `tesseract_robotics.tesseract_motion_planners_trajopt` | `tesseract_robotics.motion_planners_trajopt` |
| `tesseract_robotics.tesseract_motion_planners_trajopt_ifopt` | `tesseract_robotics.motion_planners_trajopt_ifopt` |
| `tesseract_robotics.tesseract_scene_graph` | `tesseract_robotics.scene_graph` |
| `tesseract_robotics.tesseract_serialization` | `tesseract_robotics.serialization` |
| `tesseract_robotics.tesseract_srdf` | `tesseract_robotics.srdf` |
| `tesseract_robotics.tesseract_state_solver` | `tesseract_robotics.state_solver` |
| `tesseract_robotics.tesseract_task_composer` | `tesseract_robotics.task_composer` |
| `tesseract_robotics.tesseract_task_composer_planning` | `tesseract_robotics.task_composer_planning` |
| `tesseract_robotics.tesseract_time_parameterization` | `tesseract_robotics.time_parameterization` |
| `tesseract_robotics.tesseract_urdf` | `tesseract_robotics.urdf` |

`trajopt_ifopt`, `trajopt_sqp`, `planning`, `viewer` and `emitters` keep their names.

```python
# before
from tesseract_robotics.tesseract_collision import ContactRequest
from tesseract_robotics.tesseract_common import Isometry3d

# after
from tesseract_robotics.collision import ContactRequest
from tesseract_robotics.common import Isometry3d
```

**No aliases.** 0.36 is a breaking release: the old names are gone, and importing one raises
`ModuleNotFoundError`. Every class, enum and function keeps its name; only the module path changes.

**Layout.** Sixteen modules (14 of the renamed ones, plus `trajopt_ifopt` and `trajopt_sqp`) are the compiled extension itself, installed as
`tesseract_robotics/<name>.abi3.so` with a `<name>.pyi` stub and no `__init__.py` wrapper. The other
six keep a Python package around a compiled `_<name>` module because they hold Python code:
`common` (axis constants), `scene_graph` (lazy `SceneState` re-export), and `motion_planners`,
`motion_planners_simple`, `motion_planners_descartes` and `motion_planners_trajopt_ifopt` (profile-override
guards). Both layouts import the same way. The C++ binding sources moved from
`src/tesseract_<name>/tesseract_<name>_bindings.cpp` to `src/bindings/<name>.cpp`; build
scripts and forks that name the old paths need the new ones. Upstream C++ names (the
`tesseract_collision::` namespaces, `find_package(tesseract_collision)`, the headers) did not change.

### Descartes profiles match the node's scalar type

A Descartes node reads only the profiles of its own scalar type, and runs the library defaults
when it finds none (#350):

| Pipeline | Descartes node | Reads |
|---|---|---|
| `CartesianPipeline`, `DescartesFPipeline`, `DescartesFNPCPipeline`, every `Raster*` pipeline | `DescartesFMotionPlannerTaskFactory` | `DescartesMoveProfileF`, `DescartesSolverProfileF` |
| `DescartesDPipeline`, `DescartesDNPCPipeline` (the `plan_cartesian` default) | `DescartesDMotionPlannerTaskFactory` | `DescartesMoveProfileD`, `DescartesSolverProfileD` |

Only the `D` types were bound, so a profile never reached a Cartesian or raster task: the
pipeline planned with the defaults (target pose fixed, no vertex margin) and reported success.
`TaskComposer.plan` now raises `ProfileTypeMismatchError` (a `TypeError`) before it runs when a
profile name in a Descartes node's namespace holds the other type's move or solver profile but
not the node's own. The profile helpers register both types, and a namespace may hold both.

`create_descartes_default_profiles(move_profile=…)` registers the profile with its twin of the
other type, copied field by field. A Python subclass of `DescartesMoveProfileD` or `...F` has
hooks no twin can call, so the helper refuses it with `ProfileOverrideNotDispatchedError`:

| before | after |
| --- | --- |
| `profiles.addProfile(ns, name, DescartesDefaultMoveProfileD())`, planned with `CartesianPipeline` | `create_descartes_default_profiles(move_profile=profile)`, or add a `DescartesDefaultMoveProfileF` too |
| `create_descartes_default_profiles(move_profile=MyMoveProfile())`, `MyMoveProfile(DescartesMoveProfileD)` | `profiles.addProfile("DescartesMotionPlannerTask", name, MyMoveProfile())` for a `D` pipeline; subclass `DescartesMoveProfileF` for a Cartesian or raster one |

### Python 3.12 is the minimum

`requires-python` is `>=3.12`. One `cp312-abi3` wheel per platform installs on CPython 3.12, 3.13
and 3.14, so Rhino 9 (CPython 3.13) is covered. Rhino 8 embeds CPython 3.9 and has no newer wheels:

```bash
pip install "tesseract-robotics-nanobind==0.35.0.8"  # Rhino 8, Python 3.9 (macOS, Windows); Python 3.10-3.11
```

pip picks that release by itself on those Pythons, because later ones declare the floor.

### `Target` and `Event` are `type` aliases

`planning.program.Target` and `emitters.core.events.Event` use the `type` statement. Annotations are unaffected; `isinstance(x, Target)` raises `TypeError`.

```python
isinstance(x, (CartesianTarget, JointTarget, StateTarget))
```

`Target.__value__` is the union, for code that introspects it.

### `setPairCollisionCoeff` and `getPairCollisionCoeff` removed

`trajopt_ifopt.CollisionCoeffData` bound two copies of its setter and getter under names C++ does
not have (#298). Call the C++ names:

| before | after |
| --- | --- |
| `data.setPairCollisionCoeff(a, b, c)` | `data.setCollisionCoeff(a, b, c)` |
| `data.getPairCollisionCoeff(a, b)` | `data.getCollisionCoeff(a, b)` |

### trajopt value types compare by value and are unhashable

`trajopt_ifopt.CollisionCoeffData`, `trajopt_ifopt.TrajOptCollisionConfig` and
`trajopt_sqp.SQPParameters` bind `__eq__`/`__ne__` from their C++ `operator==` and set
`__hash__ = None` (#261), as the collision, common and environment types do.
`CollisionCoeffData() == CollisionCoeffData()` was `False`. `hash(obj)`, `{obj}` and `d[obj] = …`
now raise `TypeError: unhashable type`:

| before | after |
| --- | --- |
| `seen = {config}` | `seen = {id(config)}` |
| `cache[params] = result` | key on the fields you vary, e.g. `cache[params.max_iter] = result` |

### Collision constraints need a `Var` in a variable set

trajopt's collision constraints key their contact cache on the variable set's hash, which they
read through `var.getParent().getParent()` without a check: a `Var` outside a `NodesVariables`
dereferenced null on the first `update()`. `DiscreteCollisionConstraint`,
`DiscreteCollisionConstraintD`, `DiscreteCollisionNumericalConstraint`,
`ContinuousCollisionConstraint`, `ContinuousCollisionConstraintD` and
`ContinuousCollisionNumericalConstraint` now raise `VarNotInVariableSetError` (a `ValueError`) at
construction (#297). A `Var` from `createNodesVariables` always qualifies; one from `Node.addVar`
qualifies once its `Node` is handed to `NodesVariables`:

| before | after |
| --- | --- |
| `var = node.addVar(...)`; `DiscreteCollisionConstraint(evaluator, var)`; `NodesVariables("t", [node])` | `var = node.addVar(...)`; `NodesVariables("t", [node])`; `DiscreteCollisionConstraint(evaluator, var)` |

### `Node.getParent` raises for planner-owned nodes

A Python `TrajOptIfoptCompositeProfile.create` receives the planner's `Node`s, whose variable set
the planner builds in C++ and frees when its solve returns. `Node.getParent()` on such a `Node`
returned a view of that set, which read freed memory once the solve was over (#297). It now raises
`NodeParentUnavailableError` (a `RuntimeError`) for every `Node` of a set C++ owns, during `create`
too. `getParent()` still returns the set for `Node`s of a set built from Python:

| before | after |
| --- | --- |
| `nodes[0].getParent().getRows()` in `create` | read what `create` needs from the `Node`s themselves: `sum(node.size() for node in nodes)` |

### `OSQPEigenSolver` updates check sizes

`updateGradient`, `updateLowerBound`, `updateUpperBound` and `updateBounds` now sit on `QPSolver`
and raise `QPSizeError` (a `ValueError`) unless the vector has `init()`'s `num_vars` (gradient)
or `num_cnts` (bounds) entries (#283). trajopt checked with `assert` only and read past a short
vector. Call `init(num_vars, num_cnts)` with the QP's size before the updates.
### Command-language types compare by value and are unhashable

The command-language waypoints, instructions and Polys compared by identity: two
`JointWaypoint(["a"], [1.0])` were unequal. They now bind `__eq__`/`__ne__` from the C++
operators and set `__hash__ = None`, as the collision, common and environment types do
(#261): `CartesianWaypoint`, `JointWaypoint`, `StateWaypoint`, `WaypointPoly`,
`CartesianWaypointPoly`, `JointWaypointPoly`, `StateWaypointPoly`, `InstructionPoly`,
`MoveInstructionPoly`, `MoveInstruction`, `CompositeInstruction`, `WaitInstruction`,
`TimerInstruction`, `SetAnalogInstruction`, `SetDigitalInstruction`, `SetToolInstruction`.

`hash(obj)`, `{obj}` and `d[obj] = …` now raise `TypeError: unhashable type`. Every instruction
has a UUID, which is the natural key; `MoveInstruction` equality also compares it, so two moves
built separately are unequal until the UUIDs match:

| before | after |
| --- | --- |
| `seen = {move}` | `seen = {move.getUUID()}` |
| `cache[waypoint] = result` | `cache[id(waypoint)] = result` (keep `waypoint` alive) |
| `move in done` meant identity | `any(m is move for m in done)`; `in` now compares values |

### Time-parameterization profiles compare by value and are unhashable

The four time-parameterization profiles were bound without their C++ `operator==`, so `==`
compared identity: `TOTGCompositeProfile() == TOTGCompositeProfile()` was `False`. They now bind
`__eq__`/`__ne__` and set `__hash__ = None` (#261), as the collision, common and environment value
types do. The new `RuckigTrajectorySmoothingCompositeProfile` behaves the same way.

| before | after |
| --- | --- |
| `{TOTGCompositeProfile(): …}`, `set_of_profiles.add(p)` | key by name, e.g. `{"DEFAULT": p}`, or by `id(p)` |
| `p == q` (identity) | `p is q` |

### Simple-planner move profiles compare by value and are unhashable

`SimplePlannerFixedSize{,Assign,AssignNoIK}MoveProfile` and
`SimplePlannerLVS{,NoIK,Assign,AssignNoIK}MoveProfile` bind the C++ `operator==` (#261): two
profiles with the same fields are `==`, and `hash()`, `set` members and `dict` keys raise
`TypeError`, as for the [collision, common and environment value types](#value-types-compare-by-value-and-are-unhashable).

| before | after |
| --- | --- |
| `{profile: name}` | `{id(profile): name}`, or key by the profile name |
| `profile in seen_set` (identity) | `any(p is profile for p in seen)` |

### `OMPLSolverConfig.getNumPlanners()` removed

`OMPLSolverConfig.planners` is bound (#302), so the count is `len(cfg.planners)`. The list is
copied on access; its configurators are shared with the config. `cfg.planners.append(x)`
changes the copy, but `cfg.planners[0].range = 0.5` edits the config. Assign a new list, or
use `addPlanner` / `clearPlanners`, which change it in place.

| before | after |
| --- | --- |
| `cfg.getNumPlanners()` | `len(cfg.planners)` |

### Planner value types compare by value and are unhashable

As for collision, common and environment, these classes bind the C++ `operator==`
and set `__hash__ = None` (#261). `hash()`, `set` members and `dict` keys raise `TypeError`:

| module | classes |
| --- | --- |
| `motion_planners_ompl` | `OMPLSolverConfig`, `OMPLRealVectorMoveProfile` |
| `motion_planners_trajopt` | `TrajOptCartesianWaypointConfig`, `TrajOptJointWaypointConfig`, `TrajOptDefaultMoveProfile`, `TrajOptDefaultCompositeProfile`, `TrajOptOSQPSolverProfile`, `BasicTrustRegionSQPParameters` (and the new `OSQPSettings`) |
| `task_composer_planning` | `ContactCheckProfile`, `FixStateBoundsProfile`, `FixStateCollisionProfile`, `KinematicLimitsCheckProfile`, `MinLengthProfile`, `ProfileSwitchProfile`, `UpsampleTrajectoryProfile` |

Key on a field, or on `id(obj)` where identity is meant, as in
[the value-types entry](#value-types-compare-by-value-and-are-unhashable).
`OMPLSolverConfig` compares `planners` by count only (upstream `ompl_solver_config.cpp`), so two
configs holding different configurator types of the same number are equal.

### `create_ompl_planner_configurators` rejects unused keywords

The helper accepts every `OMPLPlannerType` name (#301) and sets each keyword on the requested
configurators that have that field. It used to ignore a keyword no configurator used; now it
raises `ValueError` naming the keyword. A keyword that fits at least one requested planner is
still accepted (`goal_bias` with `["RRTConnect", "RRTstar"]` sets it on the RRTstar ones).

| before | after |
| --- | --- |
| `create_ompl_planner_configurators(["RRTConnect"], goal_bias=0.1)` | `create_ompl_planner_configurators(["RRTConnect"])` |

### TrajOpt-Ifopt and Descartes profiles compare by value and are unhashable

These classes bind `__eq__`/`__ne__` from their C++ `operator==` and set `__hash__ = None`, as the
[collision, common and environment value types](#value-types-compare-by-value-and-are-unhashable) do (#261):

| `motion_planners_trajopt_ifopt` | `TrajOptIfoptCartesianWaypointConfig`, `TrajOptIfoptJointWaypointConfig`, `TrajOptIfoptDefaultMoveProfile`, `TrajOptIfoptDefaultCompositeProfile`, `TrajOptIfoptOSQPSolverProfile` |
| `motion_planners_descartes` | `DescartesDefaultMoveProfileD`, `DescartesLadderGraphSolverProfileD` |

`hash(profile)`, `{profile}` and `d[profile] = …` raise `TypeError: unhashable type`. Key on the
profile's name in the `ProfileDictionary`, or on `id(profile)` where identity is what you mean:

| `seen = {profile}` | `seen = {profile_name}` |
| `cache[profile] = result` | `cache[id(profile)] = result` (keep `profile` alive) |

### Overriding a hook on a concrete profile raises

Only the abstract profile bases forward the planner's calls to Python. A Python subclass of a
concrete profile (the seven simple-planner move profiles, `TrajOptIfoptDefaultMoveProfile`,
`TrajOptIfoptDefaultCompositeProfile`, `TrajOptIfoptOSQPSolverProfile`,
`DescartesDefaultMoveProfileD`, `DescartesLadderGraphSolverProfileD`) that defined `generate`,
`create` or a `create*` hook planned with the C++ hook and never ran the override. Such a class
statement now raises `ProfileOverrideNotDispatchedError`, a `TypeError` (#340):

| before | after |
| --- | --- |
| `class P(SimplePlannerLVSMoveProfile): def generate(...)` | `class P(SimplePlannerMoveProfile)`, holding a `SimplePlannerLVSMoveProfile` and calling its `generate` |
| `class P(TrajOptIfoptDefaultCompositeProfile): def create(...)` | `class P(TrajOptIfoptCompositeProfile)`, holding a `TrajOptIfoptDefaultCompositeProfile` and calling its `create` |
| `class P(TrajOptIfoptOSQPSolverProfile): def create(...)` | configure the solver through the profile's fields and `callbacks` |

A subclass that defines no hook (to add fields or helpers) is unaffected.

### `TaskComposerPluginFactory`: a `str` is YAML text

The factory bound one constructor, `(config: str, locator)`, and passed the string to the C++
*path* constructor, while `KinematicsPluginFactory`, `ContactManagersPluginFactory` and
`Environment.init` read a `str` as content. It now binds both C++ constructors the same way: an
`os.PathLike` (`pathlib.Path`) is a config file, a `str` is YAML text. A path passed as `str`
parses as a YAML scalar and raises `TaskComposerConfigError` (a `ValueError`) naming the text. `loadConfig` and the new
`TaskComposerServer.loadConfig` take the same pair (#337, #323).

| before | after |
| --- | --- |
| `TaskComposerPluginFactory(str(p), loc)` | `TaskComposerPluginFactory(Path(p), loc)` |
| `createTaskComposerPluginFactory(str(p), loc)` | `TaskComposerPluginFactory(Path(p), loc)` |

### Task-composer compatibility helpers removed

Python-only names that duplicated a constructor, or stood in for C++ API that is now bound
under its own name (#313, #314, #336, #338):

| before | after |
| --- | --- |
| `createTaskComposerDataStorage()` | `TaskComposerDataStorage()` |
| `createTaskComposerPluginFactory(p, loc)` | `TaskComposerPluginFactory(Path(p), loc)` |
| `keys.addKeys(port, ["a", "b"])` | `keys.add(port, ["a", "b"])` |
| `storage.getAllData()` | `storage.getData()` |
| `infos.getAllInfos()` | `list(infos.getInfoMap().values())` |
| `infos.getAbortingNodeInfo()` | `infos.getInfo(infos.getAbortingNode())` (`None` when nothing aborted) |
| `node.getDotgraph(infos)` | `node.getDotgraph(infos.getInfoMap())` |
| `node.saveDotgraph(path, infos)` | `node.saveDotgraph(path, infos.getInfoMap())` |
| `task_composer.__all__` | the module's attributes (`from … import *` still works) |

### `TaskComposerNode.saveDotgraph` raises `OSError`

`saveDotgraph` returned `False` when the file could not be opened or written, which a caller
that ignored the result never saw. It now raises `OSError` (`FileNotFoundError` for a missing
directory) and returns `None`, as `TaskComposerPluginFactory.saveConfig` does, and takes a `str`
or an `os.PathLike` (#316).

| before | after |
| --- | --- |
| `if not node.saveDotgraph(path): ...` | `try: node.saveDotgraph(path)` / `except OSError: ...` |

### Task-composer value types compare by value and are unhashable

As for the collision, common and environment types: `AnyPoly`, `TaskComposerKeys`,
`TaskComposerDataStorage`, `TaskComposerNodeInfo`, `TaskComposerNodeInfoContainer`,
`TaskComposerContext`, `TaskComposerNodePorts` and `TaskComposerLog` bind C++ `operator==`, so
`==` compares content, and `hash()`, `set` members and `dict` keys raise `TypeError` (#261).
Two `AnyPoly` holding separately built instructions are unequal: an instruction's uuid takes
part in its equality.

`TaskComposerKeys.get(port)` returns what the port holds, `str` or `list[str]`; it raised
`RuntimeError` for a list port before (#313). `TaskComposerNodeInfo`'s fields are read-write
(#319).

### SWIG-era `MoveInstructionType_*` constants removed

`command_language` no longer exports a module constant per `MoveInstructionType` value.
The three constants were copies kept from the SWIG bindings; C++ has none of them, and
`MoveInstructionType` is bound as an enum whose members are the same objects (#275, as #176 and
#214 did for the other modules). Importing one now raises
`ImportError: cannot import name 'MoveInstructionType_LINEAR' from 'tesseract_robotics.command_language'`;
reading one as an attribute raises `AttributeError`.

The package's `__all__` list is gone as well: `from tesseract_robotics.command_language import *`
now exports every public name of the extension instead of a hand-kept subset.

| before | after |
| --- | --- |
| `MoveInstructionType_LINEAR` (also `_FREESPACE`, `_CIRCULAR`) | `MoveInstructionType.LINEAR` (`.FREESPACE`, `.CIRCULAR`) |
| `from tesseract_robotics.command_language import MoveInstructionType_LINEAR` | `from tesseract_robotics.command_language import MoveInstructionType` |

### `assign*Waypoint` and `ProfileDictionary_addProfile` removed

Seven Python-only names duplicated a bound method (#276). `assignCartesianWaypoint`,
`assignJointWaypoint` and `assignStateWaypoint` on `MoveInstruction` and `MoveInstructionPoly`
did what `setWaypoint` does; C++ 0.33 removed them. `ProfileDictionary_addProfile` worked around
a cross-module limitation that no longer exists: `ProfileDictionary.addProfile` accepts a profile
from any planner module, including a Python subclass of a trampolined profile.

| before | after |
| --- | --- |
| `mi.assignCartesianWaypoint(wp)` (also `assignJointWaypoint`, `assignStateWaypoint`) | `mi.setWaypoint(wp)` |
| `ProfileDictionary_addProfile(d, ns, name, profile)` | `d.addProfile(ns, name, profile)` |

### SWIG-era Poly helpers and `appendMoveInstruction` removed

`command_language` kept 15 Python-only names from the SWIG bindings (#328). Each
duplicated a constructor or method; C++ has none of them. Importing one raises `ImportError`,
calling `appendMoveInstruction` raises `AttributeError`.

| before | after |
| --- | --- |
| `CartesianWaypointPoly_wrap_CartesianWaypoint(wp)` | `CartesianWaypointPoly(wp)` |
| `JointWaypointPoly_wrap_JointWaypoint(wp)` | `JointWaypointPoly(wp)` |
| `StateWaypointPoly_wrap_StateWaypoint(wp)` | `StateWaypointPoly(wp)` |
| `MoveInstructionPoly_wrap_MoveInstruction(mi)` | `MoveInstructionPoly(mi)` |
| `InstructionPoly_as_MoveInstructionPoly(ip)` | `ip.asMoveInstruction()` |
| `InstructionPoly_as_CompositeInstruction(ip)` | `ip.asCompositeInstruction()` |
| `InstructionPoly_as_WaitInstruction(ip)` (also `Timer`, `SetAnalog`, `SetDigital`, `SetTool`) | `ip.asWaitInstruction()` (`asTimerInstruction()`, …) |
| `WaypointPoly_as_CartesianWaypointPoly(wp)` | `wp.asCartesianWaypoint()` |
| `WaypointPoly_as_JointWaypointPoly(wp)` | `wp.asJointWaypoint()` |
| `WaypointPoly_as_StateWaypointPoly(wp)` | `wp.asStateWaypoint()` |
| `program.appendMoveInstruction(mi)` | `program.push_back(mi)` |

**Reference, not copy.** `WaypointPoly_as_*` returned a copy of the stored waypoint;
`wp.as…Waypoint()` returns a reference into it. An edit through the result now changes the
waypoint, and so the program that holds it:

```python
mi.getWaypoint().asCartesianWaypoint().setTransform(target)  # edits mi
snapshot = copy.copy(mi.getWaypoint().asCartesianWaypoint())  # an independent copy
```

`InstructionPoly.as…()` returns a copy, as the helpers did.

### Time-parameterization profiles renamed to their C++ names

The four time-parameterization profiles were bound under abbreviated names. They now carry
their C++ names, as every other planner profile does (#329, following #187). There is no
alias: importing an old name raises `ImportError`. The `time_parameterization`
package's `__all__` list, which named the old classes, is gone as well.

| before | after |
| --- | --- |
| `TOTGCompositeProfile` | `TimeOptimalTrajectoryGenerationCompositeProfile` |
| `ISPCompositeProfile` | `IterativeSplineParameterizationCompositeProfile` |
| `ISPMoveProfile` | `IterativeSplineParameterizationMoveProfile` |
| `ConstantTCPSpeedCompositeProfile` | `ConstantTCPSpeedParameterizationCompositeProfile` |

The earlier entries for these profiles (value equality, `createKey()`,
`minimum_time_delta`) apply under the new names.

### Six `trajopt_sqp` enumerators renamed to their C++ names

trajopt 0.34 renamed its enumerators to `kCamelCase`; the bindings kept the old Python names.
For most of them the old name already was the `UPPER_SNAKE` form of the C++ name
(`kRunning` → `RUNNING`). For six it was not, so they now follow the same rule (#330). There
are no aliases. Code that reads an old member raises `AttributeError`; code that compares
`status.name` to an old string (`status.name == "NLP_CONVERGED"`) no longer matches and
raises nothing, so search for the strings as well as the attributes.

| before | after |
| --- | --- |
| `SQPStatus.NLP_CONVERGED` | `SQPStatus.CONVERGED` |
| `SQPStatus.OPT_TIME_LIMIT` | `SQPStatus.TIME_LIMIT` |
| `SQPStatus.QP_SOLVER_ERROR` | `SQPStatus.QP_SOLVE_FAILED` |
| `SQPStatus.CALLBACK_STOPPED` | `SQPStatus.STOPPED_BY_CALLBACK` |
| `QPSolverStatus.UNITIALIZED` | `QPSolverStatus.UNINITIALIZED` |
| `QPSolverStatus.QP_ERROR` | `QPSolverStatus.FAILED` |

### Planner-module compatibility names removed

The four planner modules kept names from older APIs that C++ does not have (#334, #335).
They are gone with no deprecation cycle, as in #176 and #214. Importing one raises
`ImportError`.

**Profile helpers (#334).** Each one either put a profile in a `ProfileDictionary` or upcast it
to `Profile` so it could go in. `ProfileDictionary.addProfile` takes every planner's profiles
directly, including a Python subclass of `DescartesMoveProfileD`:

| before | after |
| --- | --- |
| `ProfileDictionary_addOMPLProfile(pd, ns, name, p)`, `ProfileDictionary_addOMPLMoveProfile(…)` | `pd.addProfile(ns, name, p)` |
| `ProfileDictionary_addTrajOpt{Move,Plan,Composite,Solver}Profile(pd, ns, name, p)` | `pd.addProfile(ns, name, p)` |
| `ProfileDictionary_addTrajOptIfopt{Move,Plan,Composite,Solver}Profile(pd, ns, name, p)` | `pd.addProfile(ns, name, p)` |
| `OMPLMoveProfile_as_ProfileConstPtr(p)`, `OMPLPlanProfile_as_ProfileConstPtr(p)` | `p` |
| `cast_DescartesMoveProfileD(p)`, `cast_DescartesPlanProfileD(p)`, `cast_DescartesSolverProfileD(p)` | `p` |

**0.28 profile names (#335).** Tesseract 0.33 renamed the planner move profiles; the old names
were kept as aliases:

| before | after |
| --- | --- |
| `OMPLPlanProfile` | `OMPLMoveProfile` |
| `OMPLRealVectorPlanProfile` | `OMPLRealVectorMoveProfile` |
| `TrajOptPlanProfile` | `TrajOptMoveProfile` |
| `TrajOptDefaultPlanProfile` | `TrajOptDefaultMoveProfile` |
| `TrajOptIfoptPlanProfile` | `TrajOptIfoptMoveProfile` |
| `TrajOptIfoptDefaultPlanProfile` | `TrajOptIfoptDefaultMoveProfile` |
| `DescartesPlanProfileD` | `DescartesMoveProfileD` |
| `DescartesDefaultPlanProfileD` | `DescartesDefaultMoveProfileD` |

**trajopt's 0.28 re-exports (#335).** `motion_planners_trajopt` no longer re-exports
`TrajOptCollisionConfig` or `CollisionEvaluatorType`. Import them from their own modules,
`tesseract_robotics.trajopt_ifopt` and `tesseract_robotics.collision`.
`motion_planners_trajopt_ifopt` keeps its re-exports (`TrajOptCollisionConfig`,
`CollisionEvaluatorType`, `SQPParameters`), so its profile fields are still configured from one
import.

**`__all__` (#335).** The six `motion_planners*` packages lose their hand-kept `__all__`.
`from … import *` now exports every public name of the extension. The OMPL list had named only
the legacy aliases, so a star import gave `OMPLRealVectorPlanProfile` but not
`OMPLRealVectorMoveProfile`.

### `ompl_base` removed

`tesseract_robotics.ompl_base` wrapped five OMPL headers directly and nothing in the package used it. The module, its stub, `reeds_shepp_example.py` and the `tesseract_reeds_shepp_example` script are gone. OMPL itself stays: the OMPL planner loads it in C++.

For `State`, `RealVectorBounds`, `SE2StateSpace`, `ReedsSheppStateSpace` or `DubinsStateSpace`, use OMPL's own Python bindings.

### Time-parameterization factories take no arguments

`create_time_optimal_parameterization` accepted `path_tolerance` and `min_angle_change`, and
`create_iterative_spline_parameterization` accepted `add_points`, but neither used them: since
0.33 these settings are fields of the composite profiles that `compute` looks up in the
`ProfileDictionary`, and the algorithm objects the factories return have none. The arguments are
gone, so a call that passes one raises `TypeError` instead of silently doing nothing.

| before | after |
| --- | --- |
| `create_time_optimal_parameterization(path_tolerance=0.05)` | `profile = TimeOptimalTrajectoryGenerationCompositeProfile()`, `profile.path_tolerance = 0.05` |
| `create_time_optimal_parameterization(min_angle_change=0.01)` | `profile.min_angle_change = 0.01` on the same profile |
| `create_iterative_spline_parameterization(add_points=False)` | `profile = IterativeSplineParameterizationCompositeProfile()`, `profile.add_points = False` |

Add the profile with `profiles.addProfile(algorithm.getName(), "DEFAULT", profile)`.

### `ChangeCollisionMarginsCommand(pair_margin_data)` defaults to `MODIFY`

The binding defaulted `override_type` to `REPLACE`, so leaving it out replaced every pair margin
in the environment, where the same C++ call merges the given pairs into the existing ones (#247).
Both overloads that take `pair_margin_data` now default to `MODIFY`, as upstream. Code that left
`override_type` out and relied on the replace passes it explicitly:

| before | after |
| --- | --- |
| `ChangeCollisionMarginsCommand(pairs)` | `ChangeCollisionMarginsCommand(pairs, CollisionMarginPairOverrideType.REPLACE)` |

`CollisionMarginPairOverrideType` is in `common`. The new
`ChangeCollisionMarginsCommand(default_margin, pairs, override_type)` overload defaults to
`MODIFY` too.

### The planning API no longer picks a TCP or working frame

`tesseract_robotics.planning` used to fill in frames it could not know (#30): the TCP as the last
of `getActiveLinkNames()`, which is only the tip for a serial chain, and the working frame as the
literal `"base_link"`. `get_manipulator_info` also cached the guess and returned one shared,
mutable `ManipulatorInfo` per key. Now the caller names every frame: a missing one raises
`TypeError` at the call.

| before | after |
| --- | --- |
| `MotionProgram("manipulator")` | `MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")` |
| `MotionProgram("manipulator", tcp_frame="tool0")` | `MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")` |
| `robot.get_manipulator_info("manipulator")` | `robot.get_manipulator_info("manipulator", "tool0", "base_link")` |
| `robot.fk("manipulator", q)` | `robot.fk("manipulator", q, "tool0")` |
| `robot.ik("manipulator", pose, seed=q)` | `robot.ik("manipulator", pose, "tool0", seed=q)` |
| `robot.ik("manipulator", pose, q, "tool0")` (positional `seed`, `tip_link`) | `robot.ik("manipulator", pose, "tool0", q)` |
| `program.to_composite_instruction(joint_names, "tool0")` | `program.to_composite_instruction(joint_names)` |

For the bundled `abb_irb2400` and `lbr_iiwa_14_r820` the old guesses were `"tool0"` and
`"base_link"`, so the right-hand column behaves exactly as before. For another robot, pass the
link your tool is mounted on and the frame your Cartesian targets are expressed in.

New: `MotionProgram` and `get_manipulator_info` take a keyword-only `tcp_offset`, a named TCP
(SRDF `group_tcps`, #133) or an `Isometry3d`, and the planner puts `tcp_frame * tcp_offset` on
each Cartesian target. See [Frames and the TCP](user-guide/planning.md#frames-and-the-tcp).

### `PointCloudPoint` renamed to `PointCloud.Point`

The point type of `PointCloud` is nested in C++ (`tesseract::geometry::PointCloud::Point`) and is
now bound nested (#209). The module-level `PointCloudPoint` is gone: importing it raises
`ImportError`, reading it as an attribute raises `AttributeError`. `addPoint(x, y, z)` is
unchanged.

| before | after |
| --- | --- |
| `PointCloudPoint()`, `PointCloudPoint(x, y, z)` | `PointCloud.Point()`, `PointCloud.Point(x, y, z)` |
| `from tesseract_robotics.geometry import PointCloudPoint` | `from tesseract_robotics.geometry import PointCloud`, then `PointCloud.Point` |

### SWIG-era `JointType_*` constants removed

`scene_graph` no longer exports a module constant per `JointType` value. The seven
constants were copies kept from the SWIG bindings; C++ has none of them, and `JointType` is bound
as an enum whose members are the same objects (#214, finishing #176). Importing one now raises
`ImportError: cannot import name 'JointType_FIXED' from 'tesseract_robotics.scene_graph'`;
reading one as an attribute raises `AttributeError`.

| before | after |
| --- | --- |
| `JointType_FIXED` (also `_UNKNOWN`, `_REVOLUTE`, `_CONTINUOUS`, `_PRISMATIC`, `_FLOATING`, `_PLANAR`) | `JointType.FIXED` (`.UNKNOWN`, `.REVOLUTE`, …) |
| `from tesseract_robotics.scene_graph import JointType_FIXED` | `from tesseract_robotics.scene_graph import JointType` |

### Value types compare by value and are unhashable

Every collision, common and environment class whose C++ header declares `operator==` was bound
without it, so `==` compared identity: `ContactRequest() == ContactRequest()` was `False`, and so
was `env == env.clone()`. These classes now bind `__eq__`/`__ne__` from the C++ operators and set
`__hash__ = None` (#169):

| module | classes |
| --- | --- |
| `collision` | `ContactResult`, `ContactResultMap`, `ContactRequest`, `ContactManagerConfig`, `CollisionCheckConfig` |
| `common` | `AllowedCollisionMatrix`, `BytesResource`, `CollisionMarginData`, `CollisionMarginPairData`, `GeneralResourceLocator`, `JointState`, `KinematicLimits`, `KinematicsPluginInfo`, `ManipulatorInfo`, `PluginInfo`, `PluginInfoContainer`, `SimpleLocatedResource` |
| `environment` | `Environment`, `Command`, `AddKinematicsInformationCommand`, `AddLinkCommand`, `AddSceneGraphCommand`, `ChangeCollisionMarginsCommand`, `ChangeJointAccelerationLimitsCommand`, `ChangeJointOriginCommand`, `ChangeJointPositionLimitsCommand`, `ChangeJointVelocityLimitsCommand`, `ChangeLinkCollisionEnabledCommand`, `ChangeLinkOriginCommand`, `ChangeLinkVisibilityCommand`, `ModifyAllowedCollisionsCommand`, `MoveJointCommand`, `MoveLinkCommand`, `RemoveAllowedCollisionLinkCommand`, `RemoveJointCommand`, `RemoveLinkCommand`, `ReplaceJointCommand` |

They join the classes that already compared by value (`JointTrajectory`, the plugin-info structs,
`AddTrajectoryLinkCommand`, `AddContactManagersPluginInfoCommand` and the two
`SetActive*ContactManagerCommand`s).

What breaks is hashing. These objects are mutable, and an object that compares by value must not
hash by identity, or two equal objects land in different `set` buckets. So `hash(obj)`, `{obj}`
and `d[obj] = …` now raise `TypeError: unhashable type`. Key on a field that identifies the
object, or on `id(obj)` where identity is what you mean:

| before | after |
| --- | --- |
| `seen = {cmd}` | `seen = {(cmd.getType(), cmd.getLinkName())}` (whatever names the command) |
| `cache[env] = result` | `cache[env.getName()] = result`, or `cache[id(env)]` (keep `env` alive) |
| `cmd in history` meant identity | `any(c is cmd for c in history)`; `in` now compares values |

The equality is the C++ one, unchanged, so it inherits upstream's choices:

- Floating-point fields compare within a tolerance, not exactly: `JointState` vectors with
  `isApprox(1e-5)` after an exact size check ([joint_state.cpp][js35]), `ContactResult` and
  `CollisionCheckConfig` with `almostEqualRelativeAndAbs`, the command origins with
  `isApprox(1e-5)`.
- `ContactRequest.is_valid` compares the validator object: two requests are equal only while they
  hold the same validator ([types.cpp][ty90]).
- `BytesResource` and `SimpleLocatedResource` compare their parent locators only for presence:
  `ResourceLocator::operator==` returns `true` ([resource_locator.cpp][rl46]).
- A geometry compares by type and its per-instance uuid ([geometry.cpp][ge40]), so commands
  holding geometry are equal only when they share it: the same URDF parsed twice gives unequal
  `AddSceneGraphCommand`s.
- `Environment` compares its revisions, the types of its commands, its current state and both
  timestamps ([environment.cpp][en412]). `env == env.clone()` holds, because `clone()` copies the
  timestamps ([L442–L444][en442]); two environments initialised separately from the same URDF are
  unequal. Its `__eq__` releases the GIL, because the C++ operator takes the environment's lock.

Two classes keep identity equality: `ResourceLocator` and `Resource`. Their C++ `operator==`
returns `true` for any pair ([L46][rl46], [L206][rl206]), so binding it would make every two
Python locators equal. A Python locator subclass stays hashable.

`KinematicLimits` compares the shapes of its four limit matrices before calling the C++ operator,
which calls `isApprox` without checking them and reads out of bounds when they differ
([kinematic_limits.cpp][kl37]). Limits of different shapes are unequal.

[js35]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/common/src/joint_state.cpp#L35-L45
[ty90]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/collision/core/src/types.cpp#L90-L99
[rl46]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/common/src/resource_locator.cpp#L46
[rl206]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/common/src/resource_locator.cpp#L206
[ge40]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/geometry/src/geometry.cpp#L40
[en412]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/environment/src/environment.cpp#L393-L422
[en442]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/environment/src/environment.cpp#L442-L444
[kl37]: https://github.com/tesseract-robotics/tesseract/blob/0.35.0/common/src/kinematic_limits.cpp#L37-L45

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

`TaskComposer.plan` raises `UnknownPipelineError` for an unknown pipeline; only a planner that
ran and found no solution returns a failed `PlanningResult`.

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

`collision`, `environment` and `common` no longer export a module
constant per enum value. The 14 constants were copies kept from the SWIG bindings; C++ has
none of them, and every enum is bound as a class whose members are the same objects (#176).
Importing one now raises
`ImportError: cannot import name 'ContactTestType_ALL' from 'tesseract_robotics.collision'`;
reading one as an attribute raises `AttributeError`.

| before | after |
| --- | --- |
| `ContactTestType_ALL` (also `_FIRST`, `_CLOSEST`, `_LIMITED`) | `ContactTestType.ALL` (`.FIRST`, `.CLOSEST`, `.LIMITED`) |
| `Events_COMMAND_APPLIED`, `Events_SCENE_STATE_CHANGED` | `Events.COMMAND_APPLIED`, `Events.SCENE_STATE_CHANGED` |
| `ModifyAllowedCollisionsType_ADD` (also `_REMOVE`, `_REPLACE`) | `ModifyAllowedCollisionsType.ADD` (`.REMOVE`, `.REPLACE`) |
| `CONSOLE_BRIDGE_LOG_DEBUG` (also `_INFO`, `_WARN`, `_ERROR`, `_NONE`) | `LogLevel.CONSOLE_BRIDGE_LOG_DEBUG` (…) |

```python
# before
from tesseract_robotics.collision import ContactRequest, ContactTestType_ALL
from tesseract_robotics.common import CONSOLE_BRIDGE_LOG_WARN, setLogLevel

request = ContactRequest(ContactTestType_ALL)
setLogLevel(CONSOLE_BRIDGE_LOG_WARN)

# after
from tesseract_robotics.collision import ContactRequest, ContactTestType
from tesseract_robotics.common import LogLevel, setLogLevel

request = ContactRequest(ContactTestType.ALL)
setLogLevel(LogLevel.CONSOLE_BRIDGE_LOG_WARN)
```

The `CONSOLE_BRIDGE_LOG_*` names keep their prefix because they are the C++ enumerator names of
console_bridge's unscoped `LogLevel`; only the module-level copies are gone.

### `AnyPoly` helpers only in `task_composer`

`environment` re-exported `AnyPoly_wrap_EnvironmentConst`, and
`command_language` re-exported `AnyPoly_wrap_CompositeInstruction`,
`AnyPoly_wrap_ProfileDictionary` and `AnyPoly_as_CompositeInstruction`, from
`task_composer` inside a `try: … except ImportError: pass`. An install whose
task-composer extension failed to load lost the names without an error, and importing either
package loaded the task composer. The re-exports and `command_language._HAS_TASK_COMPOSER`
are removed (#192); the functions are defined in, and imported from, `task_composer`.
Code that imports them from the old place now fails with `ImportError: cannot import name …`.

| before | after |
| --- | --- |
| `from tesseract_robotics.environment import AnyPoly_wrap_EnvironmentConst` | `from tesseract_robotics.task_composer import AnyPoly_wrap_EnvironmentConst` |
| `from tesseract_robotics.command_language import AnyPoly_wrap_CompositeInstruction` (also `AnyPoly_wrap_ProfileDictionary`, `AnyPoly_as_CompositeInstruction`) | `from tesseract_robotics.task_composer import AnyPoly_wrap_CompositeInstruction` (…) |

### A `str` is content, `pathlib.Path` is a file

`common.FilesystemPath` was a `str` subclass, so `Environment.init` had to guess
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

`TaskComposerPluginFactory` kept its config path as `str` in this release; the next one binds
its path/content pair the same way (#337).

### The convex evaluators take the QP solution vector

`QPProblem.evaluateConvexCosts`, `evaluateTotalConvexCost` and
`evaluateConvexConstraintViolations` read `var_vals` in the layout of the QP that the last
`convexify()` built: `getNumQPVars()` entries, the NLP variables followed by the slack variables
([L28][t28]). trajopt documents that size ([L59][h59], [L67][h67]) but does not check it. The
binding now raises `QPSolutionSizeError` for any other size, and `NotConvexifiedError` before
the first `convexify()`, while `getNumQPVars()` is still 0 ([L99][t99], [L831][t831]). Both are
`ValueError`s:

```text
QPSolutionSizeError: evaluateConvexCosts: var_vals has 3 entries; it must be the QP solution vector of getNumQPVars() = 5 entries, the NLP variables followed by the slack variables
NotConvexifiedError: evaluateConvexCosts: the problem has no convex model yet; call convexify() first
```

What used to work and now raises: an NLP-sized `var_vals` on a problem with constraint sets
and no hinge or absolute cost. Each constraint row adds one or two slack variables, and trajopt
0.35.0, which 0.35.0.8 and earlier bundle, reads only the NLP block there, so the call returned
the right values. Append zeros for the slack variables:

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

It is not a rename. `IfoptQPProblem` as 0.35.0.8 and earlier bind it, on trajopt 0.35.0, and
`TrajOptQPProblem` on the pinned trajopt `a504b22` differ in five ways, and the third and fourth
change results:

| | `IfoptQPProblem` (removed) | `TrajOptQPProblem` |
|---|---|---|
| **1. Per set, not per row** | One merit coefficient, name and violation per constraint row; names are `<set>_<row>` ([L93][i93], [L100][i100]). | One per constraint set ([L738–L760][p738], [L833–L835][p833], [L1163–L1186][p1163]). Code that indexes names, violations or merit coefficients by row breaks, and the solver inflates a whole set's merit coefficient at once. |
| **2. `addCostSet` checks bounds** | Checks nothing; rejects `HINGE` ([L50–L78][i50]). | Every row must be an equality or one-sided, with any penalty type; a range row raises, from `addConstraintSet` too ([L115–L127][t115]). |
| **3. No raw-cost path** | A cost added with `IfoptProblem.addCostSet` bypasses `IfoptQPProblem::addCostSet` ([L59][i59]): it gets no gradient and no Hessian in the QP ([L190][i190], [L263][i263]), while the merit still reads its raw, signed rows ([L525–L531][i525]). | Every cost goes in with `addCostSet(set, penalty_type)`. A cost that was inert now shapes the steps. |
| **4. The merit matches the model** | Weights the model but not the merit, and counts the model once per cost term, so the solver can reject every step and stop at the seed (tesseract-robotics/trajopt#595). | The merit weights and counts every cost as the model does ([L1118–L1162][p1118]), and weighs each constraint set's violation by its coefficients, as the QP prices its slack ([L887–L903][p887]), so steps the old ratio rejected can be accepted. An `ABSOLUTE` or `HINGE` cost is modelled on its linearized rows, without the slacks ([L233–L295][p233]; tesseract-robotics/trajopt#592), so a violation larger than the trust box is reduced step by step ([the QP problem](api/trajopt_sqp.md#the-qp-problem)). |
| **5. Dynamic sets** | Throws on `isDynamic()` sets ([L42–L48][i42]). | Accepts them ([L488–L540][p488]). |

How much the third difference moves a result: this package's online SQP example added its velocity
cost with `IfoptProblem.addCostSet`, so its QP had no cost term and OSQP solved with an empty
Hessian. As a squared cost on `TrajOptQPProblem`, the global solve's summed squared joint steps
fell from 23.2 to 3.9, against 3.6 for the straight-line seed, and `getTotalExactCost()` now reports
that sum instead of the first velocity row (0.5).

!!! note "Evidence"
    The five differences are read from the sources linked above: trajopt 0.35.0 for
    `IfoptQPProblem`, the pinned trajopt `a504b22` for `TrajOptQPProblem`. The example's
    numbers are measured on this repository's `online_planning_sqp_example.py`, and
    `tests/examples/test_online_planning_sqp_example.py` pins its problem class, its cost as the
    squared joint steps, and a global solve that reaches the target from the pinned start. How
    much results move in other code depends on its costs; nothing here measures that. The
    `ABSOLUTE`/`HINGE` model is also measured: `TestTrajOptQPProblemPenaltyCosts` in
    `tests/trajopt_sqp/test_trajopt_sqp_bindings.py` pins it.

[i42]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L42-L48
[i50]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L50-L78
[i59]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L59
[i93]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L93
[i100]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L100
[i190]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L190
[i263]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L263
[i525]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L525-L531
[t115]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L115-L127
[p233]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L233-L295
[p488]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L488-L540
[p738]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L738-L760
[p833]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L833-L835
[p887]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L887-L903
[p1118]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L1118-L1162
[p1163]: https://github.com/tesseract-robotics/trajopt/blob/a504b2208d/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L1163-L1186

### `Quaterniond` takes xyzw only

The four-scalar constructor was scalar-first (`w, x, y, z`), the opposite of every other quaternion
surface in the package. It is gone; `Quaterniond(w, x, y, z)` raises `TypeError`.

| before | after |
| --- | --- |
| `Quaterniond(w, x, y, z)` | `Quaterniond.from_xyzw(x, y, z, w)` or `Quaterniond([x, y, z, w])` |
| `Quaterniond(coeffs=[x, y, z, w])` | `Quaterniond(xyzw=[x, y, z, w])` |

### Environment events and keywords

`EventCallbackFn` wrapped a callable only to hand it to `addEventCallback`. The method takes the
callable and returns the key.

| before | after |
| --- | --- |
| `env.addEventCallback(key, EventCallbackFn(fn))` | `key = env.addEventCallback(fn)` |
| `env.removeEventCallback(hash=key)` | `env.removeEventCallback(key)` |
| `env.getJointGroup(name=..., joint_ids=...)` | `env.getJointGroup(group_name=..., joint_ids=...)` |

### Overload copies removed from `Environment`, `StateSolver` and kinematics

Each removed name duplicated a native overload under another name.

| before | after |
| --- | --- |
| `Environment.getStateByMap`, `getStateByNamesAndValues` | `Environment.getState` |
| `Environment.setStateByNamesAndValues` | `Environment.setState(joint_ids, joint_values)` |
| `Environment.getCurrentJointValuesByNames` | `Environment.getCurrentJointValues(joint_ids)` |
| `StateSolver.setStateByMap`, `setStateByNamesAndValues` | `StateSolver.setState` |
| `JointGroup.calcJacobianWithPoint(q, link, p)` | `JointGroup.calcJacobian(q, link, p)` |
| `KinematicGroup.calcInvKinMultiple(list, seed)` | `KinematicGroup.calcInvKin(list, seed)` |

### `VectorVector3d` and `VectorIsometry3d` removed

| before | after |
| --- | --- |
| `common.VectorVector3d([...])` | `[np.array(...), ...]` |
| `common.VectorIsometry3d([...])` | `[Isometry3d(...), ...]` |

Functions that returned these containers return lists.

### Environment and Poly lookups raise, and casts are views

| before | after |
| --- | --- |
| `Environment.getLink` / `getJoint` of a missing id: `RuntimeError` | `KeyError` |
| `getJointGroup` / `getKinematicGroup` / `getGroupJointIds` / `getLinkTransform`, contact managers by name, unknown id: `RuntimeError` | `KeyError` |
| `getLink` / `getJoint` returned a reference that dangled after a removal | the shared `Link` / `Joint`, valid after a removal |
| `ip.asMoveInstruction()` returned a copy | a view: edits write through; `copy.copy(...)` copies |
| a wrong `as*()` cast raised `RuntimeError` | `PolyTypeError` (a `TypeError`), also for the `WaypointPoly` casts |
| 20 per-type `applyCommand` overloads | `applyCommand(command)`, one overload |
| `CompositeInstruction.push_back` / `insert` / `del` / `clear` while Python holds an element | `InstructionHeldError`; drop the reference first |

### Geometry inputs are validated

| before | after |
| --- | --- |
| `Isometry3d(matrix)` accepted any 4x4 | `NotRotationError` unless the upper 3x3 is a rotation; the same for the rotation inputs of `AngleAxisd` |
| a zero quaternion or axis | `DegenerateRotationError`; a non-unit one is normalized |
| `Hyperplane3d.Through(p0, p1, p2)`: normal (p1 - p0) x (p2 - p0) | normal (p2 - p0) x (p1 - p0), as Eigen; swap `p1` and `p2` for the old plane |
| `Hyperplane3d` / `ParametrizedLine3d` degenerate: `ValueError` for small inputs | `DegenerateGeometryError` (a `ValueError`), only for exactly-zero vectors |

### Serialization and URDF writes raise, and bytes are `bytes`

```python
# 0.35
ok = serialization.environment_to_file(env, "env.xml")   # False on failure
blob = serialization.environment_to_binary(env)          # list[int]
urdf.writeURDFFile(env, package_path, "robot")           # None, failures silent

# 0.36
serialization.environment_to_file(env, Path("env.xml"))  # None; OSError on failure
blob = serialization.environment_to_binary(env)          # bytes
path = urdf.writeURDFFile(env, package_path, "robot")    # package_path/urdf/robot.urdf
```

Files are written and read exactly at `path` (`str` or `os.PathLike`, no suffix appended).
`writeURDFFile` raises `URDFWriteError` with the nested cause; `urdf_name` is a stem.

### Named errors in the bindings

| before | after |
| --- | --- |
| a plugin-factory config path the locator cannot resolve: a crash | `FileNotFoundError` (`ContactManagersPluginFactory`, `KinematicsPluginFactory`, `TaskComposerPluginFactory`, `TaskComposerServer.loadConfig`) |
| `CartAxis` constraint with a zero axis: `RuntimeError` | `DegenerateAxisError` (a `ValueError`) |
| `python_constraints`: bare `ValueError` | `BoundsShapeError`, `InvertedBoundsError`, `ClearanceMarginError` |
| a Descartes Python profile whose `create*` returns the wrong type: `AttributeError` on a worker | `ProfileReturnTypeError` (a `TypeError`) at creation |
| `ContactResultVector[-1]` | works |
| `ContactResultMap.filter` passed a reference | passes a copy, copied back into the map |
| `OSQPEigenSolver` without `setAdaptiveRhoInterval` or getters | both OSQP classes have `set*` / `get*` for every setting |
| `tesseract_robotics/trajopt_ifopt.pyi`, a flat module | the package `tesseract_robotics.trajopt_ifopt`; imports are unchanged |

### The planning API raises instead of returning flags

```python
# 0.35
q = robot.ik("tool0", pose)
if q is None:
    ...
if not robot.add_link(link, joint):
    ...

# 0.36
try:
    q = robot.ik("tool0", pose)           # ndarray
except IKNoSolutionError:
    ...
robot.add_link(link, joint)               # None; EnvironmentCommandError on refusal
```

`remove_link`, `move_link`, `add_allowed_collision`, `set_collision_margin` and `create_obstacle`
behave like `add_link`. Planning failures are named exceptions in `tesseract_robotics.planning.errors`,
each a subclass of the builtin it replaces. `Robot.build(env)` validates the environment and
`RobotState` checks its name and position counts.

### Planning names removed or renamed

| before | after |
| --- | --- |
| `MotionProgram.start_at(...)` | `add_target(...)` |
| `create_descartes_pipeline_profiles` | `create_descartes_default_profiles` |
| `CartesianTarget(position=p, quaternion=q)` | `CartesianTarget.from_xyz_quat(p, q)`; `CartesianTarget(pose)` |
| `TRAJOPT_PROFILE_NAMES` | `STANDARD_PROFILE_NAMES` |
| `planning.profiles._create_trajopt_profiles` | `planning.profiles.create_trajopt_profile_pair` (returns composite, move) |
| `TaskComposer(config_path=...)` | removed; the config comes from `get_task_composer_config_path()` |

`planning.profiles` is a package (`trajopt`, `ompl`, `descartes`, `time_param`, `pipelines`,
`names`); `STANDARD_PROFILE_NAMES` and the other name lists are tuples. `num_threads` is not capped
to the core count and must be at least 1.

### `TaskComposer.plan` raises, and `plan_ompl` uses `FreespacePipeline`

| before | after |
| --- | --- |
| an unknown pipeline, a malformed program or a missing pipeline port: `successful=False` | `UnknownPipelineError` or the matching named error; `successful=False` only when a planner ran and found no solution |
| a pipeline outside the default-profile table planned with an empty dictionary | `NoDefaultProfilesError` |
| `plan_ompl(...)` ran `OMPLPipeline` | runs `FreespacePipeline`; pass `pipeline="OMPLPipeline"` for raw OMPL |
| `warmup(["Bad"])` skipped a pipeline that did not load | `PipelineWarmupError` |
| `plan` seeded the caller's `CompositeInstruction` in place | seeds a copy |
| a target without a profile defaulted to `DEFAULT` over `MotionProgram(profile=...)` | inherits the program's profile |

### Package surface

| before | after |
| --- | --- |
| `get_tesseract_resource_path()` fell back to an environment's `share/` | the support distribution only; `SupportDataNotFoundError` otherwise |
| `__version__` fell back to a placeholder | the installed distribution's version |
| `from tesseract_robotics.scene_graph import *` included `SceneState` | `from tesseract_robotics.scene_graph import SceneState` |
| `planning`, `common`, `scene_graph` defined `__all__` | no `__all__`; `import *` exports every public name |
| trajectory points always carried `time` | `time` only where a time parameterization set velocities |
| `mesh_from_file` on a missing file | `ResourceNotFoundError`; `AmbiguousMeshError` for several meshes |

### Examples raise `PlanningFailedError`

The examples' `main()` returned a failed result after printing. They raise `PlanningFailedError`,
and `online_planning_sqp_example` raises when the initial solve does not converge or a replanning
QP step fails. Catch the error where you called `run()` or `main()`.

### Emitters take frames from the `ManipulatorInfo`

Each move now carries its working frame, TCP frame and TCP offset from its `ManipulatorInfo`.
Every dialect resolves them through a required frame binding and raises `UnboundFrameError` for
a frame it does not know.

| before | after |
| --- | --- |
| `emit_krl(program)` | `emit_krl(program, frames=krl_frames(...))` |
| `emit_ls(program)` | `emit_ls(program, frames=ls_frames(...), config=resolver)` |
| `emit_jbi(program)`, `JbiProfile(tool=...)` | `emit_jbi(program, frames=jbi_frames(...), calibration=JbiPulseCalibration(...))`; no `tool` field |
| `emit_urscript(program)` | `emit_urscript(program, frames=ur_frames(...))` |
| `emit_rapid(program, profiles)` returned a `str`; fixed `[1, 0, 0, 0]` confdata | returns an `EmittedProgram`; `frames=rapid_frames(...)`, `posture=OpwPostureSolver(...)`; `cf1/cf4/cf6` from the seed joints |
| JBI joint degrees under `///POSTYPE PULSE` | integer encoder pulses from the calibration; a Cartesian move raises |
| `SetToolInstruction` emitted a tool command | `UnsupportedInstructionError` in every dialect |
| names rewritten to fit the dialect | `IdentifierError` |
| more than six joint values: extra axes dropped | `ExternalAxisError` on KRL, LS, JBI and URScript |
| KRL FREESPACE Cartesian move | `MissingConfigurationError` (a PTP to a frame needs Status and Turn) |
| `RapidEmitterError`, `Config.neutral_for`, `rapid.utils`, `core.naming.safe_identifier`, `jbi.backend.MissingSeedError`, `UnknownKinematicTopologyError`, `ExternalAxisUnitError` | removed; the errors are named per failure mode (`InvalidProfileError`, `InvalidLayoutError`, `InvalidDeclarationError`, `ProgramStructureError`, ...) |
| `Trajectory.from_composite(program)` of an untimed program | `UntimedTrajectoryError` unless `untimed_step_s=` is given (also `TesseractViewer.update_trajectory`) |

The RAPID DSL declares typed objects with a `.name` (`Robtarget`, `Jointtarget`, `Tooldata`,
`Workobject`, `Speed`, `Zone`, `Pos`, `RapidBool`); `Tooldata` and `Workobject` take `Pose`s in
metres, `Speed` and `Zone` take SI values, and statements run inside `with RapidWriter() as w:`.

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
