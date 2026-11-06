# Building against upstream main

The `upstream-main` branch tracks the `master` branches of tesseract, trajopt and
tesseract_planning ahead of the next release (0.36), so the bindings catch up
while upstream moves rather than after it tags. See issue #141.

Two loops share the same pinned upstream SHAs:

| Loop | Command | What it builds | Use it for |
|---|---|---|---|
| inner | `pixi run -e upstream build-upstream` | upstream C++ from git checkouts, incrementally, then the bindings | day-to-day binding work |
| outer | `pixi run build-feedstock` | conda packages, through the tesseract-robotics-packaging feedstocks | checking the release packaging; CI |

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

### Merging `pixi.lock` on a rebase

`pixi.lock` conflicts on every rebase onto main: main re-locks its own envs, and this
branch adds envs main does not have (`upstream`, `packaged`) or re-locks one both
have (`default` and the `py3xx` envs, once they moved to the dev label). Text merges
of the lock are never right; merge it per environment.

1. **Start from main's lock.** Take the rebase target's side (stage 2,
   `git show :2:pixi.lock`) as the base and add every environment that exists only in
   the replayed commit (stage 3), with its platforms and the package records it
   references (a small YAML script; `merge_pixi_lock.py` in the catch-up did this).
   Main's envs keep main's solve.
2. **Replace re-locked envs.** For an env both sides have but the branch re-locked,
   replace main's entry with the branch's (stage 3) and add the package records it
   needs. Main's solve of that env is the wrong one: it pins the 0.35 packages.
3. **Canonicalise.** Run `pixi lock`. It rewrites the file in its own order, prunes
   records nothing references and re-solves only what the manifest no longer
   satisfies.
4. **Check per env.** Compare each environment against main's lock (a script that
   loads both files and prints `same` / `DIFF` / `only-new` / `only-ref` per env;
   `check_lock_envs.py` in the catch-up). Every env the branch did not change must be
   `same`; every `DIFF` must be one the branch meant to re-lock.

Then `git add pixi.lock` and continue the rebase. Never resolve the lock with
`git checkout --ours` / `--theirs`: either side alone drops the other's envs.

## Layout

```text
upstream/                         # upstream sources (submodules, pinned SHAs)
├── boost_plugin_loader           # 0.4.5 — tesseract main needs it, the channel has 0.4.3
├── tesseract
├── trajopt
├── tesseract_planning
└── build/                        # persistent ninja build dirs (gitignored)
packaging/                        # feedstocks (submodules, `dev` branches)
├── boost-plugin-loader-feedstock
├── tesseract-robotics-feedstock
├── trajopt-feedstock
├── tesseract-robotics-planning-feedstock
├── descartes-light-feedstock     # not in the chain, see below
├── output/                       # local conda channel the outer loop writes (gitignored)
└── logs/                         # build logs (gitignored)
```

The feedstock submodules point at forks under `jf---`; their `dev` branches go
upstream to tesseract-robotics-packaging as PRs.

descartes_light and opw_kinematics come from the `tesseract-robotics` channel
in both loops: neither has build-relevant changes past the packaged versions
(descartes_light 0.4.10, opw_kinematics 0.5.3).

## Inner loop

```bash
git submodule update --init upstream packaging
pixi run -e upstream build-upstream-cpp   # C++ only
pixi run -e upstream build-upstream       # C++, then the bindings
```

`scripts/build_upstream.sh` builds boost_plugin_loader → tesseract → trajopt
(`trajopt_common`, `trajopt_sco`, `trajopt_ifopt`, `trajopt`, `trajopt_sqp`) →
tesseract_planning and installs each into the `upstream` env's prefix, where the
bindings find them exactly as they find the conda packages on `main`. It mirrors
the feedstocks' `recipe/build.sh`:

- same compilers: `clang_osx-arm64` / `clangxx_osx-arm64` 19 on macOS,
  `gcc_linux-64` / `gxx_linux-64` 14 on Linux, as pinned in the feedstocks'
  `.ci_support`. These activation packages export `CXX`, `CXXFLAGS` and
  `CMAKE_ARGS`; the `c-compiler` 2.x metapackage installs bare clang without
  them.
- same `${CMAKE_ARGS}` and `-D` flags. Outside conda-build `CMAKE_ARGS` carries
  no install prefix, so the script adds `CMAKE_INSTALL_PREFIX=$CONDA_PREFIX`.
- third-party deps: the `upstream` pixi feature lists the union of the
  feedstock recipes' `host` sections, and no tesseract packages.
- on macOS, `CXXFLAGS` minus `-fvisibility-inlines-hidden`, as upstream's own
  conda recipe (see [Gotchas found](#gotchas-found)).

Each package keeps a ninja build dir under `upstream/build/<name>` and compiles
through ccache; ninja uses every core. Measured on an M1 Max (10 cores):

| Build | Time |
|---|---|
| no-op | 1–2 s |
| one tesseract `.cpp` touched | 2 s (1 translation unit + relink) |

To reconfigure a package (new CMake options, or after a big SHA jump), remove
its build dir: `upstream/build/<name>`. To change one cache option in place, pass
the source dir explicitly — `cmake -B` alone takes the cwd as the source and
refuses:

```bash
pixi run -e upstream cmake -DOPTION=VALUE -S upstream/tesseract -B upstream/build/tesseract
```

The script configures with `CMAKE_BUILD_WITH_INSTALL_RPATH=ON`. Without it, every
repeat install prints `install_name_tool: no LC_RPATH load command` errors on
macOS: CMake's install script re-runs `-delete_rpath <build tree>` on files it
reports up to date, and the first install already removed that rpath. Nothing
runs from the build tree (tests are off), so the build-tree rpath has no use.

## Outer loop

```bash
pixi run build-feedstock
```

Runs each feedstock's `build-locally.py` — the same `.scripts/run_osx_build.sh`
/ `run_docker_build.sh` its CI runs — for the host platform's config
(`FEEDSTOCK_CONFIG`: `osx_arm64_` or `linux_64_`), in dependency order:
boost-plugin-loader → tesseract → trajopt → planning. Every build writes to
`packaging/output`, and rattler-build also reads that directory as a channel,
so each stage links the packages built before it rather than the 0.35 packages
on the published channel:

```text
boost-plugin-loader  0.5.0               output
tesseract-robotics   0.36.0.dev20261103  output
trajopt              0.36.0.dev20261013  output
```

Each package runs its recipe tests. Every run provisions a fresh build env and
compiles from scratch, which is why this is the outer loop, not the dev loop.

The `dev` recipes source a GitHub archive of the pinned SHA and version it
`0.36.0.dev<commit date>`: conda orders `.dev` below the release, so the 0.36.0
package supersedes it.

!!! warning "Apple SDK licence"
    macOS builds need `OSX_SDK_DIR`; conda-smithy downloads the macOS SDK there.
    Setting it means accepting Apple's SDK licence terms. The task sets it to
    `packaging/SDKs`.

### Gotchas

- `OSX_SDK_DIR` must exist before the build starts: `run_osx_build.sh` probes it
  with `mktemp` and aborts with "not writeable" otherwise. The task creates it.
- conda-forge's build setup appends `CONDA_BUILD_SYSROOT` to
  `.ci_support/<config>.yaml` in place. CI checkouts are thrown away; locally a
  second run fails with `Duplicate key "CONDA_BUILD_SYSROOT"`. Every task
  restores `.ci_support` with `git checkout` before building.
- tesseract main fails to compile against boost_plugin_loader 0.4.3
  (`no member named 'acquireLibraryLifetimeTokens'`); hence the 0.4.5 feedstock
  at the head of the chain.
- The `.ci_support` files still pin `console_bridge`, which the `dev` recipes no
  longer use (upstream switched to spdlog). Harmless; a conda-smithy rerender
  refreshes them.
- **rattler-build's index is unreadable to pixi 0.70.1.** rattler-build indexes
  `packaging/output` with rattler-index ≥ 0.31, which writes a
  `repodata_revisions` map; pixi 0.70.1 (the version CI pins) fails with
  "invalid type: map, expected a sequence". pixi 0.81 reads it. Rather than move
  the pixi pin, `build-feedstock` ends with `index-feedstock-channel`, which
  re-indexes with rattler-index 0.30.11, the last format 0.70.1 reads.
  anaconda.org serves its own classic index, so this is local-only.
- **A re-solve needs a readable channel before any task runs.** `pixi run`
  re-locks first when `pyproject.toml` changed, and that reads
  `packaging/output`. If the channel is missing or carries rattler-build's index,
  bootstrap the index outside `pixi run`:
  `pixi exec --spec 'rattler-index==0.30.11' rattler-index fs --force packaging/output`.
  With an up-to-date lock, `pixi run` doesn't read the channel.
- **Local channels need a `./` prefix.** A bare `chan` is an anaconda.org
  channel name. `pixi.lock` records the channel and every package from it as an
  absolute `file://` path, so the `packaged` lock entries are machine-specific
  until the channel becomes `tesseract-robotics/label/dev`.
- **A rebuild that keeps the filename keeps the old lock entry.** Bump
  `build.number` in a recipe whose content changes (tesseract went `_0` → `_1`
  for the visibility strip), so the artifact gets a new identity and
  `pixi update -e packaged` picks it up.

## Packaged stack (#162 step 0)

`pixi run -e packaged test-packaged` builds the bindings against the
`0.36.0.dev` packages and runs the suite. The `packaged` feature now reads
`tesseract-robotics/label/dev`, the packages feedstock CI uploads; for a local
feedstock build, point its channel at `./packaging/output` and run
`pixi run index-feedstock-channel` first. It was the gate before anything went to the
channel.

**Result (2026-10-04, osx-arm64, local feedstock builds): 997 passed, 3 skipped, same
as the source-built `upstream` env**, against tesseract `0.36.0.dev20260929 _1`,
trajopt `0.36.0.dev20260930 _0`, tesseract-robotics-planning
`0.36.0.dev20260929 _1` and boost-plugin-loader `0.4.5`. The 3 skips are the
benchmark modules (`pytest_benchmark` isn't in the env).

| Run | tesseract package | Result |
|---|---|---|
| 1 | `_0`, built with `-fvisibility-inlines-hidden` | 993 passed, 4 failed: `TestEnvironmentSerialization`, "Trying to save an unregistered polymorphic type (tesseract::common::GeneralResourceLocator)" |
| 2 | `_1`, flag stripped in `recipe/build.sh` | 997 passed |

Run 1 used the packages as built, so the failure is attributable to them. The
root cause is the cereal visibility gotcha below; the feedstock fix is the
same strip upstream's own recipe carries.

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
  without that planner. `src/tesseract_serialization/planner_cereal_registration.cpp`
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

Checked against trajopt `de9e941`:

| Item | Upstream fix | In `de9e941`? | Status |
|---|---|---|---|
| #151 ABSOLUTE / HINGE penalty costs mis-modelled | trajopt#592 (`4634678`…`de9e941`) | yes | the two 0.35.0 characterization tests failed exactly as their docstrings predicted; replaced by `test_penalty_only_cost_beyond_the_trust_box_is_reduced` and `test_penalty_exact_cost_is_weighted_by_the_coefficient` (the assertions those docstrings named). The 0.35.0 caveats in `docs/api/trajopt_sqp.md`, `trajopt_ifopt.md` and `user-guide/low-level-sqp.md` stay: they describe the released wheels |
| #146.1 `JointPosConstraint` bounds ctor broadcasts `coeffs` | trajopt#592 | yes | upstream's split now indexes `coeffs_` and the ctor broadcasts length 0 / 1 itself; binding broadcast removed, `test_joint_pos_constraint_bounds.py` passes against trajopt's own ctor |
| #146.2 `TrajOptQPProblemBinding` subclass | trajopt#598 | no (move ctor still `= default` in the header) | keep |
| #146.3 trampoline `non_zeros_ = 0` | trajopt#599 | no (raw `getNonZeros()` sum still reserves) | keep |

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

## Default environment

The `default` env pins the 0.36 dev packages from `tesseract-robotics/label/dev`
(see [Status](#status)), so `pixi run build` and `pixi run test` build and test this
branch with no source build. The bindings do not build against the 0.35.0 packages,
by design: `pixi run build` against 0.35.0 fails in at least 9 binding modules,
because the id API (`tesseract::common::LinkId`, `JointId`, `getJointIds`, …),
`tesseract/common/logging.h` and `tesseract/geometry/impl/signed_distance_field.h`
do not exist in 0.35. Released wheels (0.35.0.x) are built from `main`'s 0.35
history. The `CMakeLists.txt` visibility change applies to every env (harmless
against 0.35, which registers cereal types in the headers).
