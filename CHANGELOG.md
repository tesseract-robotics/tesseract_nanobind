# Changelog

## [Unreleased]

- **`contactTest` releases the GIL — parallel collision checking across cores** — `DiscreteContactManager.contactTest` and `ContinuousContactManager.contactTest` now bind with `nb::call_guard<nb::gil_scoped_release>()`, mirroring the motion planners' `solve()` guards. Without it a Python worker thread sweeping a trajectory for collisions only time-shared the GIL with the main thread (concurrency, not parallelism — the sweep didn't get faster and contended with the UI); with it, sweeping on per-thread cloned managers scales near-linearly — measured **1.95×** on 2 threads (10-core machine) against a **0.99×** pure-Python control that rules out a measurement artifact. Enables off-main-thread, sharded collision scans of a whole program ([#122]).
- **Plugin-backed objects keep their Environment alive — fixes the gh-72 SIGSEGV lottery** — `getKinematicGroup` / `getJointGroup` / `getDiscreteContactManager` / `getContinuousContactManager` now bind with `nb::keep_alive<0, 1>`. These objects are plugin-created (OPW/KDL/UR kinematics, bullet/fcl collision factories) and their vtables live in dylibs owned by the Environment's plugin loader; when Python destroyed the Environment first (interpreter-teardown dict order, pytest GC), the loader dlclosed the plugin dylibs and the survivors' destructors virtual-called into unmapped pages. Whether a given script crashed was pure destruction-order luck — the reported "`calcJacobian` segfaults under pytest but works in plain Python" was this lottery, not the jacobian path (lldb forensics: dangling vptr pinned inside unloaded `libtesseract_kinematics_opw.dylib`) ([#72]).
- **Linux wheels: libstdc++ no longer bundled — fixes dual-copy heap corruption** — the ldd-closure bundling swept the conda env's gcc-14 `libstdc++.so.6` into every wheel; numpy/scipy load the system copy, the loader unifies `STB_GNU_UNIQUE` locale statics across the two, and `std::regex` inside `PropertyTree::rebuildAutoValidators` freed a stack address during `Environment.init` (SIGSEGV/`double free or corruption` — detection-dependent UB, so the pytest suite corrupted silently). The gcc runtime (`libstdc++`, `libgcc_s`) is now excluded from bundling, the GH [#35] `RTLD_GLOBAL` preload is removed (its "two copies coexist safely" premise was falsified), and a test-wheel CI guard asserts ships-no-copy / one-copy-mapped / numpy-first `env.init` clean. Wheels now require a gcc-13+ era system or conda libstdc++ (ubuntu 24.04+, debian 13+); older distros fail loudly at import. Full forensics: `docs/developer/linux-wheels.md` ([#119]).
- **Basic Descartes example** — `descartes_raster_c_api_example.py`, low-level port of upstream `motion_planners/examples/raster_example.cpp`: a 3-pass raster program solved by `DescartesMotionPlannerD` directly (no TaskComposer) with TrajOpt refinement chained on the result; `CompositeInstruction` gains the full C++ constructor (`profile, manipulator_info, order`) so UNORDERED transition composites are constructible from Python ([#87]).
- **Descartes woven into the high-level planning API** — new `create_descartes_pipeline_profiles()` for the standalone `DescartesF/D(NPC)Pipeline` family, and `create_descartes_default_profiles()` grows the 0.35 knob surface: tool-axis rotation sampling for axis-symmetric processes (`sample_axis`/`sample_resolution`/`sample_min`/`sample_max` → `target_pose_fixed=False`), `ik_solver` override, `use_redundant_joint_solutions`, and `move_profile=` injection of custom (Python-subclassable, [#83]) `DescartesMoveProfileD` instances — mutually exclusive with the knobs, fails loud ([#85]).
- **Lowlevel planning example: TrajOpt profiles reach the solver** — `tesseract_planning_lowlevel_c_api_example.py` added its TrajOpt plan/composite profiles to the OMPL dictionary while handing the solver an empty `trajopt_profiles`; TrajOpt silently planned on fallback defaults ([#115]).
- **`get_task_composer_config_path()` resolves like the runtime** — returns the `@PLUGIN_PATH@`-patched config that `_configure_environment()` publishes via `TESSERACT_TASK_COMPOSER_CONFIG_FILE` (env var → bundled data → conda share) instead of unconditionally returning the bundled-only path, which doesn't exist in dev envs and was unpatched in wheels; raises the new `TaskComposerConfigNotFoundError` when nothing resolves; the hand-rolled fallback chains in the task composer tests, `TaskComposer.from_config`, and `scripts/generate_pipeline_dotgraphs.py` now delegate to it ([#110]).
- **ruff enforces the py39 floor** — `FA102` added to lint select so PEP 604 unions without `from __future__ import annotations` fail the hooks; with `target-version = "py39"` (rejects 3.10+ syntax) this turns 3.9 compatibility from convention into a gate ([#92]).

## [0.35.0.4] - 2026-06-08 — conda-forge C++ deps + OSQP tunability + Linux libstdc++ preload + CI hardening

- **OSQP solver settings exposed** — `OSQPEigenSolver` gains seven tunable setters (`setPolish`, `setWarmStart`, `setAdaptiveRho`, `setMaxIteration`, `setAbsoluteTolerance`, `setRelativeTolerance`, `setVerbosity`) and `TrajOptIfoptOSQPSolverProfile` gains `setAdaptiveRhoInterval`, enabling iteration-based (deterministic) rho adaptation for reproducible SQP solves; fixes dangling-pointer segfault in `getJacobian()` via `rv_policy::reference_internal` on `getVar`/`getNode`/`getNodes` ([0f40048], [c984044], [c083f5e], [#103]).
- **OMPL RNG determinism** — `RNG_setSeed`/`RNG_getSeed` bound in `tesseract_motion_planners_ompl`; `conftest.py` auto-reseeds per test for xdist-safe reproducibility; single-planner + `optimize=False` recipe documented as the only guaranteeable determinism contract (parallel-plan best-of-N races even with seeded samplers); Windows determinism test dropped — conda-forge `ompl.lib` embeds a separate `RNGSeedGenerator` copy per DLL so seeding the extension can't reach the planner ([a0f49c6], [5f4776e], [31c7541], [#103]).
- **conda-forge C++ deps** — C++ stack migrated from colcon workspace build to conda-forge prebuilts; `pixi install && pixi run build` replaces `pixi run build-cpp` / colcon; `pixi.lock` v7 with eigen-abi-tagged `orocos-kdl _2` ending heap corruption in all KDL paths; `locked:true` + pinned `pixi-version` in CI so stale-lock drift fails loud; build-dir cache keyed on `pixi.lock` hash; Python 3.9 feature env added for Rhino 8 ([b6a14c4], [3c80a25], [6210a99], [0455e34]).
- **Linux libstdc++ preload** — bundled gcc-14 `libstdc++` loaded `RTLD_GLOBAL` as the first act of `__init__` wins the SONAME race before numpy/scipy can pull in the system copy on Ubuntu 22.04 and older; gated on `CONDA_PREFIX` absence to avoid double-loading inside pixi/conda envs ([#35], [ee83dc8], [7715422]).
- **`Environment.setState` input validation** — unknown or non-active (fixed/mimic) joint names now raise `ValueError` with a clear message instead of silently dereferencing unmapped entries and crashing with `SIGSEGV`; all three entry points covered (dict, names+values, `setStateByNamesAndValues`) ([#43], [ca620d3]).
- **`SceneGraph` type registration in URDF module** — `_tesseract_urdf` now imports `_tesseract_scene_graph` at module load so the return type of `parseURDFFile` is always registered; was masked by full-suite imports, exposed by the new minimal-import ABI canary ([f944293]).
- **CI hardening** — Windows test-wheel now runs the full suite (kinematics + examples, benchmarks-only exclusion); macOS nanobind build directory cached to halve warm builds; ABI canary (install wheel into pixi env → kinematics smoke-test) on Linux and macOS build jobs catches dep-ABI rot in ~30 s instead of as scattered segfaults in test-wheel; duplicate CI env block merged ([e60f272], [b29c37a], [04a812f], [6210a99], [607b4b9]).
- **Dotgraph polish** — `TaskComposer` pipeline diagrams woven into docs; `clean_dot_labels` strips dump-debug fields for legibility; contract test + type annotations ([4aa5efb], [57537da], [1bfb4c3]).

## [0.35.0.3] - 2026-06-05 — complete wheel matrix for the 0.35.0.2 content

Reissue of [0.35.0.2] with the full wheel set actually published. In 0.35.0.2 a dirty CI build tree made `setuptools_scm` stamp a PEP 440 local segment (`+d<date>`) onto some wheels; PyPI rejects those with HTTP 400, and the non-idempotent publish step aborted mid-upload — so macOS (all) and Linux x86_64 (`cp39`/`cp311`) wheels never landed. 0.35.0.2 is yanked. Identical bindings to 0.35.0.2 — no API changes ([49f1aa9]).

- **Release pipeline hardened** — `setuptools_scm` `local_scheme = "no-local-version"` so a dirty tag build can't produce a PyPI-rejected local version; `skip-existing: true` on all three publish workflows so re-runs fill gaps instead of 400-aborting; macOS `fail-fast: false` so one flaky build can't cancel sibling wheels; plus a 3× retry on the macOS `upload-artifact` step working around GitHub's long-standing arm64-runner `CreateArtifact` timeout (`actions/upload-artifact#569`, still open) that otherwise skips the entire macOS publish ~80% of the time ([49f1aa9], [804d5b8]).
- **`cp312-abi3` support verified on all platforms** — macOS and Windows `test-wheel` now install the `cp312-abi3` wheel on Python 3.13/3.14 (mirroring Linux; `wheel != python`), so the published 3.9–3.14 support matrix is verified in CI, not just asserted ([14a4cea]).

## [0.35.0.2] - 2026-06-04 — aarch64 / Jetson Linux wheels + collision manager bindings

Extends Linux wheel coverage to `aarch64` (including NVIDIA Jetson), completes the contact-manager binding surface, and hardens the new ROS RPY decomposition against gimbal-lock edge cases ([#97], [669f2da], [f30014b]).

- **`aarch64` / NVIDIA Jetson Linux wheels** — new GitHub Actions matrix entry plus Jetson build patches, built for Python 3.12+ on `aarch64`; Linux wheel CI standardised on `pixi run build-cpp` / `build-wheel` ([#52], [dc3f1df], [605dc4d], [af7bc4e], [#91], [ba0510f], [c897251], [@Joelkang]).
- **Collision manager bindings completed** — remaining discrete/continuous contact-manager surface plus Descartes `MoveProfile` contact-manager config ([#94], [9bfd6b1], [7a3b0e3], [@Joelkang]).

## [0.35.0.1] - 2026-05-29 — tesseract 0.35.0 upgrade + Eigen geometry suite + RAPID emitter

Major dependency bump to **tesseract 0.35.0**, a complete Eigen geometry binding surface with scalar-last quaternions as the project canon, a `Pose` that now *is* an `Isometry3d`, and a new ABB RAPID code emitter. (First wheel release of the 0.35 series; the project's `setuptools_scm` scheme requires four-component tags, so it is `0.35.0.1`, not `0.35.0`.)

- **Complete Eigen geometry binding suite** — fleshed-out `Vector3d` / `Vector4d` / `Matrix3d` / `Quaterniond` / `AngleAxisd` / `Isometry3d` surface, with scalar-last `[qx, qy, qz, qw]` `Quaterniond.from_xyzw` and `Quaterniond.from_rpy` / `to_rpy` ZYX ROS RPY interop; float64 default precision centralised via Eigen `NumTraits` ([#76], [492ab7a], [544f7dd], [1d6f5c7], [ef26fc9]).
- **ABB RAPID code emitter** for `tesseract_command_language` (`tesseract_robotics.emitters.rapid`) ([#75], [37f21b2]).
- **`Octree` geometry bindings** (plus `octomap::OcTree`, `OctreeSubType`, `PointCloud`, `createOctree`) and the remaining `tesseract_geometry` utilities (`isIdentical`, `extractVertices`, `toTriangleMesh`) ([#67], [4575aa9], [@Joelkang]).
- **Python-subclassable Descartes evaluators/sampler/profile** for `tesseract_motion_planners_descartes` — new bindings for `DescartesStateD`, `DescartesStateSampleD`, `DescartesEdgeEvaluatorD`, `DescartesWaypointSamplerD`, `DescartesStateEvaluatorD`, and a now-subclassable `DescartesMoveProfileD` let users override `createWaypointSampler` / `createEdgeEvaluator` / `createStateEvaluator` from Python ([#83], [620b228], [@Joelkang]).
- **Upgraded to tesseract 0.35.0** — deps bumped (`rosinstall` → `repos`), `MIGRATION.md` sed applied, namespace aliases added, resource URIs and dev paths updated for the 0.35 layout, wheel data dir `tesseract_support` → `tesseract/support`, `kinematics_core_factories` → `kinematics_factories`, descartes `tp::MoveInstructionPoly` → `tcl::MoveInstructionPoly` ([#86], [6710ccf], [422c5ea], [4e08a36]).
- **`Pose` now subclasses `Isometry3d`** (`tesseract_robotics.planning`) — rotation math via Eigen, numpy-property facade dropped, factories collapsed to one name accepting both scalar-positional and arraylike call styles, `Transform` alias removed in favour of the single canonical `Pose`, stored-data accessors exposed as properties ([#76], [a202a45], [caa3b19]).
- **Scalar-last quaternions declared project canon** — `[qx, qy, qz, qw]`; `Quaterniond(w, x, y, z)` literals migrated to `Quaterniond.from_xyzw` ([0bd408d], [f7b2fea]).
- README gains an examples section and a refreshed platform-support matrix ([d7bde99], [8eed899]).
- `KinematicsInformation.group_tcps` exposed to Python (`tesseract_srdf`) ([#79], [52fdfba]).
- `TaskComposerNodeInfoContainer` returns `TaskComposerNodeInfo` ([#81], [106d162], [@Joelkang]).
- `getSeed()` exposed on `CartesianWaypoint` ([#80], [0c57074], [@Joelkang]).
- Viewer light color ([#73], [3b4c545], [@johnwason]).

## [0.34.1.7] - 2026-05-09 — command_language I/O instructions + TaskComposer introspection

Python API surface expansion: `tesseract_command_language` gains its remaining I/O and pacing instruction types (`Wait`, `Timer`, `SetAnalog`, `SetDigital`, `SetTool`) — closing the gap that previously forced raster and pick-and-place pipelines through C++ or YAML — alongside TaskComposer DAG visualisation via GraphViz DOT output and `TaskComposerDataStorage` introspection helpers. Welcomes [@seonixx] (Simon White) as a new contributor.

- **Remaining `tesseract_command_language` instruction types** bound: `WaitInstruction`, `TimerInstruction`, `SetAnalogInstruction`, `SetDigitalInstruction`, `SetToolInstruction`, with their `WaitInstructionType` / `TimerInstructionType` enums and `InstructionPoly_as_*` cast helpers. Closes the I/O-and-pacing gap that previously forced raster/pick-and-place pipelines to be authored in C++ or YAML. New `tests/tesseract_command_language/test_command_language.py` covers construction, casting, and `InstructionPoly` round-tripping ([#70], [a4c26df], [@Joelkang]).
- **TaskComposer DAG visualisation** — `getDotgraph()` / `saveDotgraph()` overloads on `TaskComposerNode`, with an optional `TaskComposerNodeInfoContainer` argument to highlight the executed path on a previously-run DAG. Emits GraphViz DOT for debugging otherwise-opaque planning pipelines ([#63], [e7c3a92], [@Joelkang]).
- **`TaskComposerDataStorage` AnyPoly cast/wrap helpers** — `AnyPoly_wrap_TaskComposerDataStorage` and `AnyPoly_as_TaskComposerDataStorage`, so DAG storage state can be inspected after each node from Python instead of only via C++ ([#66], [b30ed46], [@Joelkang]).
- **`TaskComposerDataStorage.getAllData()`** no-arg overload binding returning the full data map ([#66], [990705f], [@seonixx]).
- **`DiscreteContactCheckTask` exposes `contact_results`** through `task_infos` `data_storage`, with unit and integration test coverage ([#66], [4fe07bf], [06178f0], [@seonixx]).
- **SimplePlanner and time-parameterization profiles** now reachable via the public `tesseract_motion_planners_simple` and `tesseract_time_parameterization` package paths instead of only the underscore-prefixed C++ extension modules. C++ binding-side registrations retired in favour of `__init__.py` re-exports; `test_constant_tcp.py` consolidated into `test_time_parameterization.py` ([#62], [d73e227], [30c8e9f], [@Joelkang]).
- `InstructionPoly_as_CompositeInstruction` re-exported from `tesseract_command_language` package init — was bound but not surfaced ([#70], [110e1e0], [@Joelkang]).

## [0.34.1.6] - 2026-04-27 — PyPI README refresh

Same wheels as 0.34.1.5; recut so the published wheel's README metadata includes the Windows build badge alongside Linux and macOS on PyPI's project page.

- README badge row includes a Windows build badge ([6656815]).

## [0.34.1.5] - 2026-04-27 — Windows x64 wheels debut + Python profile bindings

First complete cross-platform PyPI release: Linux, macOS, **and** Windows wheels for Python 3.9–3.12. Closes the long-running [#40] Windows wheels initiative ([#58]). Also ships a batch of profile-binding contributions from [@Joelkang] that had been queued in Unreleased.

- **Windows x64 wheels** published to PyPI via the new `wheels-windows.yml` workflow. Mirrors `wheels-{linux,macos}.yml`; sources C++ deps from conda-forge via pixi; bundles plugin DLLs via `delvewheel repair --analyze-existing` ([f9c122d], [3e3a2e4]).
- `__init__.py` prepends the package dir and `tesseract_robotics_nanobind.libs/` to `PATH` on Windows — required because `boost::dll`'s plain `LoadLibrary` ignores `os.add_dll_directory` ([006ce38]).
- Cereal polymorphic types re-registered in the `tesseract_serialization` binding TU — MSVC instantiates the cereal registry per-DLL, so upstream's registrations don't reach the consumer `.pyd` without re-registration ([af88515]).
- New developer doc `docs/developer/windows-wheels.md` capturing four Windows wheel packaging gotchas surfaced during the gh-40 diagnosis ([fe5ad75]).
- `CompositeInstruction.push_back` now accepts a `CompositeInstruction` in addition to a plain `Instruction`, enabling nested composites (e.g. raster programs) to be built from Python the same way they are in C++ ([#51], [6d17314], [@Joelkang]).
- **SimplePlanner profiles** configurable from Python — `SimplePlannerCompositeProfile` plus seven concrete move profiles (`FixedSize`, `FixedSizeAssign`, `FixedSizeAssignNoIK`, `LVS`, `LVSNoIK`, `LVSAssign`, `LVSAssignNoIK`), with `ProfileDictionary_addSimplePlannerMoveProfile` / `…CompositeProfile` helpers for registration ([#54], [e2bf837], [@Joelkang]).
- **task_composer planning-node profiles** exposed through a new `tesseract_task_composer_planning` module: `ContactCheckProfile`, `FixStateBoundsProfile`, `FixStateCollisionProfile`, `KinematicLimitsCheckProfile`, `MinLengthProfile`, `ProfileSwitchProfile`, and `UpsampleTrajectoryProfile`. Heavy TrajOpt-typed members on `FixStateCollisionProfile` (OSQP settings, SQP params, coeff data) are intentionally skipped — configure those via YAML or C++ ([#56], [62cd2cd], [@Joelkang]).
- **`ConstantTCPSpeedParameterization`** (KDL-based, constant TCP-speed time parameterization) bound alongside its `ConstantTCPSpeedCompositeProfile` ([#55], [f4f981d], [@Joelkang]).
- vcpkg-based `wheels-windows.yml` variant (workflow_dispatch-only, never green) replaced by the conda-forge / pixi pipeline of the same name ([3e3a2e4]).

## [0.34.1.4] - 2026-04-27 [YANKED] — partial cross-platform release

**Yanked.** Linux and macOS wheels published successfully; Windows wheels built and tested but the publish job hit a PyPI Trusted Publisher configuration mismatch and didn't upload. Superseded by 0.34.1.5.

## [0.34.1.2] - 2026-04-23 — macOS arm64 stability

Follow-up to the macOS arm64 debut in `0.34.1.1`, resolving a garbage-collection crash that surfaced in fresh virtual environments during motion planning.

- Dedupe versioned dylibs in the wheel — the actual root cause of the macOS GC crash ([#48], [7f53185]).
- Pin plugin dylibs on macOS to prevent a garbage-collection crash during motion planning ([#48], [590343f]).
- Restore the CI macOS test scope that had been narrowed as a workaround for [#48] ([#49], [07f8f9c]).

## [0.34.1.1] - 2026-04-23 — macOS arm64 wheels debut

First PyPI-published macOS arm64 wheels, shipping via a dedicated `wheels-macos.yml` workflow parallel to the existing Linux x86_64 pipeline. Also introduces a Python trampoline for `ConstraintSet`, Python 3.9 compatibility for example modules, and installable example scripts.

- **macOS arm64 wheels** published to PyPI via a split, per-platform workflow ([d80e689]).
- Python trampoline for `ifopt.ConstraintSet` so users can subclass it from Python for custom SQP constraints ([361c60e]).
- Examples installable as console scripts (e.g. `tesseract_basic_cartesian_example`) via `[project.scripts]` ([2c62952], [@marip8]).
- `pixi run build-wheel` task for local wheel builds ([e26df2a]).
- Non-benchmark tests fail loud instead of being silently skipped ([a1d7607]).
- Python 3.9 compatibility for example modules via `from __future__ import annotations` ([87ce68e]).

[Unreleased]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.35.0.4...HEAD
[0.35.0.4]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.35.0.3...0.35.0.4
[0.35.0.3]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.35.0.2...0.35.0.3
[0.35.0.2]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.35.0.1...0.35.0.2
[0.35.0.1]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.7...0.35.0.1
[0.34.1.7]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.6...0.34.1.7
[0.34.1.6]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.5...0.34.1.6
[0.34.1.5]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.4...0.34.1.5
[0.34.1.4]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.2...0.34.1.4
[0.34.1.2]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.1...0.34.1.2
[0.34.1.1]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.0...0.34.1.1
[#35]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/35
[#40]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/40
[#43]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/43
[#48]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/48
[#49]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/49
[#51]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/51
[#54]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/54
[#55]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/55
[#56]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/56
[#58]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/58
[#62]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/62
[#63]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/63
[#66]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/66
[#70]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/70
[#52]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/52
[#67]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/67
[#73]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/73
[#75]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/75
[#76]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/76
[#79]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/79
[#80]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/80
[#81]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/81
[#83]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/83
[#86]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/86
[#91]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/91
[#94]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/94
[#97]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/97
[#103]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/103
[#122]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/122
[07f8f9c]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/07f8f9c8c54ab13c3d10ceca00181091d0126336
[2c62952]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/2c62952fded6cb1253cb45441d7cd6f9b0423593
[361c60e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/361c60e263f0768e1b23d3a9919f700d06b15993
[590343f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/590343f93dc4b82fae92f56d038cdb5ba41a511d
[62cd2cd]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/62cd2cdd02f9c711d78726d1993d7585a8d54141
[6d17314]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6d17314a0b5aa1db45ffb9d9b18a8b3e048d3d06
[7f53185]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/7f531851ce70a2564251761f560020d768e3e940
[87ce68e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/87ce68ed5d015ae596a9047d629ed26f8595e3b0
[a1d7607]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/a1d7607afa7d03cd27e845059cb455eb74a4d7ec
[d80e689]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/d80e689f559f560aeb5e5b28cb332e942d15ecef
[e26df2a]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/e26df2a10d37b9d4ffb689a384e6bad490b2fb7d
[e2bf837]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/e2bf837c61b307157eb8cc3313e34f772c7d1b27
[f4f981d]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/f4f981d161a59d7d4367139e5eb79a4c3a6b1eef
[006ce38]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/006ce38
[3e3a2e4]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/3e3a2e4
[6656815]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6656815
[af88515]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/af88515
[f9c122d]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/f9c122d
[fe5ad75]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/fe5ad75
[06178f0]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/06178f0
[110e1e0]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/110e1e0
[30c8e9f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/30c8e9f
[4fe07bf]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/4fe07bf
[990705f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/990705f
[a4c26df]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/a4c26df
[b30ed46]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/b30ed46
[d73e227]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/d73e227
[e7c3a92]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/e7c3a92
[492ab7a]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/492ab7a
[544f7dd]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/544f7dd
[1d6f5c7]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/1d6f5c7
[ef26fc9]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/ef26fc9
[37f21b2]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/37f21b2
[4575aa9]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/4575aa9
[620b228]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/620b228
[6710ccf]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6710ccf
[422c5ea]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/422c5ea
[4e08a36]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/4e08a36
[a202a45]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/a202a45
[caa3b19]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/caa3b19
[0bd408d]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/0bd408d
[f7b2fea]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/f7b2fea
[52fdfba]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/52fdfba
[106d162]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/106d162
[0c57074]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/0c57074
[3b4c545]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/3b4c545
[d7bde99]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/d7bde99
[8eed899]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/8eed899
[9bfd6b1]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/9bfd6b1
[dc3f1df]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/dc3f1df
[605dc4d]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/605dc4d
[af7bc4e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/af7bc4e
[ba0510f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/ba0510f
[c897251]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/c897251
[49f1aa9]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/49f1aa9
[804d5b8]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/804d5b8
[14a4cea]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/14a4cea
[669f2da]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/669f2da
[f30014b]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/f30014b
[7a3b0e3]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/7a3b0e3
[04a812f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/04a812f
[0f40048]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/0f40048
[0455e34]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/0455e34
[1bfb4c3]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/1bfb4c3
[31c7541]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/31c7541
[3c80a25]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/3c80a25
[4aa5efb]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/4aa5efb
[57537da]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/57537da
[5f4776e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/5f4776e
[607b4b9]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/607b4b9
[6210a99]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6210a99
[7715422]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/7715422
[a0f49c6]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/a0f49c6
[b29c37a]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/b29c37a
[b6a14c4]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/b6a14c4
[c083f5e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/c083f5e
[c984044]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/c984044
[ca620d3]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/ca620d3
[e60f272]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/e60f272
[ee83dc8]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/ee83dc8
[f944293]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/f944293
[@Joelkang]: https://github.com/Joelkang
[@johnwason]: https://github.com/johnwason
[@marip8]: https://github.com/marip8
[@seonixx]: https://github.com/seonixx
