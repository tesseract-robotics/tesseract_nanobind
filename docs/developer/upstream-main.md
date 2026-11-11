# Building against upstream main

The `upstream-main` branch tracks the `master` branches of tesseract, trajopt and
tesseract_planning ahead of the next release (0.36), so the bindings catch up
while upstream moves rather than after it tags. See issue #141.

!!! note "History"
    This page records the 0.36 catch-up. The feedstock builds, the `packaged` and `upstream` pixi
    envs and `scripts/build_upstream.sh` were replaced by one native build shared with CI, see
    [The native stack build](native-stack.md); commands below that name them no longer exist.

## Status

The main catch-up is done. The branch is rebased onto `origin/main` at `4e3982f0`,
every change main made since the previous rebase is ported to the 0.36 API, and main's
tests run against it: on osx-arm64 the full suite passed, 3087 tests, in the `default`
env against the `20260929` / `20260930` dev-label packages. At the current pins, the
tests #363 changes pass in the `default` env on osx-arm64; the full suite is left to
CI. The user-facing changes are in
[API Changes: 0.35 → 0.36](../changes-0.36.md).

- **The root env is on the 0.36 dev label.** `[tool.pixi.dependencies]` pins
  `tesseract-robotics ==0.36.0.dev20261103`, `tesseract-robotics-planning
  ==0.36.0.dev20261014` and `trajopt ==0.36.0.dev20261013` from
  `tesseract-robotics/label/dev`, which the workspace channel list puts at priority
  10, so the `default` env and the `py3xx` CI envs build the 0.36 bindings with no
  source build. The label carries the stack packages and forks of three conda-forge
  packages: `pcl` without VTK, Qt or OpenGL, and `bullet-cpp` and `ompl` without a Python
  pin. Everything else comes from conda-forge as before.
- **Feedstock CI publishes the dev stack.** The `dev` branches of the
  boost-plugin-loader, pcl, bullet, ompl, tesseract-robotics, trajopt and
  tesseract-robotics-planning feedstocks build boost-plugin-loader 0.5.0 and the three
  forks, then tesseract, trajopt and planning, on
  linux-64, linux-aarch64, osx-arm64 and win-64, and upload each package to
  `tesseract-robotics/label/dev`. Each feedstock reads that label as a
  `channel_source`, so a stage links the packages the stage before it uploaded; a
  per-repo CI cache could not feed the next feedstock. The `upstream` (source) and
  `packaged` envs stay as the inner loop and as a cross-check of the published
  packages.
- **The stack includes PIQP through the trajopt feedstock.** The trajopt recipe builds
  PIQP 0.6.4 from a hash-pinned tarball, static on Windows, instead of depending on
  conda-forge's `piqp`: its win-64 build ships a `.lib` and no DLL, and its builds pin a
  Python version, which the stack packages must not. So `piqp` is a pixi dependency only
  in the `upstream` env, which compiles trajopt itself.

## Binding status

At tesseract `2b79e91`, trajopt `a504b22`, tesseract_planning `a648382` every
binding module compiles (`pixi run -e upstream build-upstream` exits 0) and the test
suite passes in the `upstream` env. The user-facing changes are
in [API Changes: 0.35 → 0.36](../changes-0.36.md).

| Module | Was | Resolution |
|---|---|---|
| tesseract_common | `JointState::joint_names` → `joint_ids` | `LinkId` / `JointId` / `LinkIdPair` bound once (`bindNameId<Tag>`), imported by every id-using module; `str` converts implicitly; ACM / margin pair overloads; spdlog logging API |
| tesseract_scene_graph | `parent_link_name` / `child_link_name`, `get*Names` removed | `*_id` attributes, `get*Ids` |
| tesseract_state_solver | name → id API | id getters; binding-only `setStateByMap` etc. keep their names, kwargs now `joint_ids` |
| tesseract_collision, tesseract_kinematics, tesseract_environment | links/joints by id | `ContactResult.link_ids`, `KinGroupIKInput.tip_link_id`, `getRootLinkId`, `getGroupJointIds`, id-keyed `dict[LinkId, Isometry3d]` results |
| tesseract_srdf | group maps keyed by id | `group_tcps` inner key `LinkId` |
| tesseract_command_language | waypoint constructors | `getJointIds` / `setJointIds`, constructors take `list[JointId]` |
| tesseract_geometry | `sdf_mesh.h` removed | `SignedDistanceField` (cherry-picked #129) |
| tesseract_task_composer | `task_composer_keys.h` removed | `TaskComposerPortMap`, `get{Input,Output}PortMappings`, `setPortMappings`; plugin-pin warning via `TESSERACT_LOG_WARN` (upstream no longer links console_bridge) |
| trajopt_ifopt | two-id `getCollisionCoeff` removed; evaluator gradients moved to `trajopt_common::getGradient` | `getCollisionCoeff(pair)`; free `getGradient` (both overloads); `getPairsWithZeroCoeff` returns `set[LinkIdPair]` (`nanobind/stl/unordered_set.h`) |
| trajopt_sqp | violations became a struct | `ConstraintViolations {raw, weighted}` |

The `tesseract_robotics.planning` layer keeps `str` in its public signatures and
converts ids with `.name()` at its boundary
(`test_names_cross_the_planning_boundary_as_str`).

## Gotchas found

- **cereal "Trying to save an unregistered polymorphic type" on macOS.** Since
  tesseract#1305 the polymorphic registrations live in the compiled
  `libtesseract_*` dylibs, not in the headers. conda's clang activation puts
  `-fvisibility-inlines-hidden` in `CXXFLAGS`; it hides cereal's registry
  singleton (`nm -m` shows `StaticObject<…>::create()::t` as *non-external (was a
  private external)*), and under Mach-O's two-level namespace every image then
  keeps its own empty registry. Both sides need the flag gone:
  `scripts/build_upstream.sh` strips it on darwin (as upstream's own conda recipe
  does), and `CMakeLists.txt` strips it from `CMAKE_CXX_FLAGS` on Apple —
  `VISIBILITY_INLINES_HIDDEN OFF` only stops CMake *adding* the flag, it never
  removed the one from the environment. CMake reads `CXXFLAGS` at first configure
  only: an existing `upstream/build/<name>` needs
  `cmake -DCMAKE_CXX_FLAGS="…" upstream/build/<name>` once. The tesseract feedstock
  strips it too (`recipe/build.sh`, build `_1`). Without the strip, all 1121
  `create()::t` singletons in the packaged `libtesseract_{common,geometry,collision,environment}`
  were non-external and `TestEnvironmentSerialization` failed 4/9; with it, all
  1121 are external and the suite passes. tesseract_planning needs no strip: on
  macOS it registers its types in headers (`command_language/CMakeLists.txt`),
  so its dylibs carry no cereal symbols and the singletons live in the binding
  modules, which `CMakeLists.txt` already builds without the flag.
- **Which profile types are registered depends on the platform.** On Linux a
  planner's types are registered once its `.so` is in the process; on macOS and
  Windows only in a TU that includes its `cereal_serialization.h`. Without help,
  saving raises on macOS and Windows, and loading on Linux fails in a process
  without that planner. `src/bindings/serialization_planner_cereal_registration.cpp`
  includes every planner's header and the module links the planners, which makes
  all three the same; a registration in a planner's own binding module would not
  reach this module on Windows, where the registry is per DLL.
- **Implicit id conversions are invisible to stubgen.** `scripts/generate_stubs.sh`
  runs `scripts/widen_implicit_id_stubs.py`, which widens parameter-position
  `LinkId` / `JointId` to `| str` and `LinkIdPair` to `| tuple[str, str]`. Mapping
  keys stay exact (invariance), so a literal `str` lookup on an id-keyed dict
  type-checks red while working at runtime.
- **`generate_stubs.sh` swallowed stubgen failures** (`|| true`) and hardcoded the
  `default` env, so on this branch it produced nothing and exited 0. It now uses the
  env it runs in and aborts on the first failure:
  `pixi run -e upstream bash scripts/generate_stubs.sh`.
- **`createNodesVariables` still takes `list[str]`**: upstream `Node::addVar` keeps
  `std::vector<std::string>` child names. Pass `[j.name() for j in group.getJointIds()]`.

## trajopt workarounds (#146, #151)

Checked against the pinned trajopt `a504b22` (`0.36.0.dev20261013`); trajopt#592 merged as
`de9e941`:

| Item | Upstream fix | In `a504b22`? | Status |
|---|---|---|---|
| #151 ABSOLUTE / HINGE penalty costs mis-modelled | trajopt#592 | yes | `test_penalty_only_cost_beyond_the_trust_box_is_reduced` and `test_penalty_exact_cost_is_weighted_by_the_coefficient` assert the fixed behaviour. The docs state the fixed model: no published wheel shipped `TrajOptQPProblem` on trajopt 0.35.0 (0.35.0.8 binds only `IfoptQPProblem`) |
| #146.1 `JointPosConstraint` bounds ctor broadcasts `coeffs` | trajopt#592 | yes | upstream's split now indexes `coeffs_` and the ctor broadcasts length 0 / 1 itself; binding broadcast removed, `test_joint_pos_constraint_bounds.py` passes against trajopt's own ctor |
| #146.2 `TrajOptQPProblemBinding` subclass | trajopt#598 (PR #615: move operations defined out of line) | yes | removed; `TrajOptQPProblem` is bound directly. No bound API returns a C++-built problem, so Python sees no difference |
| #146.3 trampoline `non_zeros_ = 0` | trajopt#599 (open, no PR) | no | kept: `component.cpp:91` asserts `non_zeros_ >= 0`, so a non-negative hint is the subclass's job; release builds skip the assert and `trajopt_qp_problem.cpp:944` reserves the raw sum |
| #146.5 `validate_qp_solution` | trajopt#592 reads only the NLP block | partly | kept. Re-measured with the check off, each case in a child interpreter: before `convexify()` a squared cost segfaults and a penalty cost or a constraint reads 0; after it, a vector shorter than `getNumNLPVars()` is read past its end. The QP size stays the rule: trajopt documents it (`qp_problem.h:61, :69, :92`) |
| #146.6 `ConstraintViolations` | trajopt#592 | yes | bound (`bd2650a9`) |

## Logging

Upstream replaced console_bridge with spdlog (#1367). The console_bridge Python API
no longer reached tesseract's output (`test_log_level_silences_tesseract` was red
through it) and is removed; the binding no longer links console_bridge, and the
spdlog enum took its `LogLevel` name. The spdlog-backed API mirrors upstream: `getLogger(name)` → `Logger.set_level`,
`isLogLevelEnabled`, `addLogRecordHandler` / `removeLogRecordHandler` with a
`LogRecord` copy. Handler exceptions go to `sys.unraisablehook` (upstream swallows
them), `removeLogRecordHandler` releases the GIL while upstream waits for in-flight
calls, and handlers still registered at interpreter exit are removed by an `atexit`
hook. Upstream ships no migration note for #1367; a draft issue asking for one is
pending.

Two narrow deadlocks remain by construction, both from the handler taking the GIL on
the logging thread: C++ that joins logging threads while the caller holds the GIL
(dropping a `TaskflowTaskComposerExecutor` with a run still in flight; the
`planning` layer always `wait()`s), and a handler added by an `atexit` function that
runs after ours, then called by a C++ worker during finalization.

## Ids at the Python boundary

- Id-typed data members (`Joint.parent_link_id`, `ManipulatorInfo.tcp_frame`, …)
  are bound with `nb::rv_policy::copy`. `def_rw` defaults to `reference_internal`,
  so a read id would alias C++ storage and a later assignment would rewrite a dict
  key in place.
- `LinkIdPair.__eq__` takes its argument with `noconvert()`: the implicit
  `tuple → LinkIdPair` conversion would make `("b", "a") == pair` true while the
  hashes differ.
- `planning` and `viewer` convert ids with `.name()` where they leave tesseract
  (signatures, glTF/JSON payloads).
