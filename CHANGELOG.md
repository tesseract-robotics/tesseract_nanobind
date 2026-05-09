# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [0.34.1.7] - 2026-05-09 — command_language I/O instructions + TaskComposer introspection

Python API surface expansion: `tesseract_command_language` gains its remaining I/O and pacing instruction types (`Wait`, `Timer`, `SetAnalog`, `SetDigital`, `SetTool`) — closing the gap that previously forced raster and pick-and-place pipelines through C++ or YAML — alongside TaskComposer DAG visualisation via GraphViz DOT output and `TaskComposerDataStorage` introspection helpers. Welcomes [@seonixx] (Simon White) as a new contributor.

### Added

- contributed by [@Joelkang]

  - **Remaining `tesseract_command_language` instruction types** bound: `WaitInstruction`, `TimerInstruction`, `SetAnalogInstruction`, `SetDigitalInstruction`, `SetToolInstruction`, with their `WaitInstructionType` / `TimerInstructionType` enums and `InstructionPoly_as_*` cast helpers. Closes the I/O-and-pacing gap that previously forced raster/pick-and-place pipelines to be authored in C++ or YAML. New `tests/tesseract_command_language/test_command_language.py` covers construction, casting, and `InstructionPoly` round-tripping ([#70], [a4c26df]).
  - **TaskComposer DAG visualisation** — `getDotgraph()` / `saveDotgraph()` overloads on `TaskComposerNode`, with an optional `TaskComposerNodeInfoContainer` argument to highlight the executed path on a previously-run DAG. Emits GraphViz DOT for debugging otherwise-opaque planning pipelines ([#63], [e7c3a92]).
  - **`TaskComposerDataStorage` AnyPoly cast/wrap helpers** — `AnyPoly_wrap_TaskComposerDataStorage` and `AnyPoly_as_TaskComposerDataStorage`, so DAG storage state can be inspected after each node from Python instead of only via C++ ([#66], [b30ed46]).

- contributed by [@seonixx]

  - **`TaskComposerDataStorage.getAllData()`** no-arg overload binding returning the full data map ([#66], [990705f]).
  - **`DiscreteContactCheckTask` exposes `contact_results`** through `task_infos` `data_storage`, with unit and integration test coverage ([#66], [4fe07bf], [06178f0]).

### Fixed

- contributed by [@Joelkang]

  - **SimplePlanner and time-parameterization profiles** now reachable via the public `tesseract_motion_planners_simple` and `tesseract_time_parameterization` package paths instead of only the underscore-prefixed C++ extension modules. C++ binding-side registrations retired in favour of `__init__.py` re-exports; `test_constant_tcp.py` consolidated into `test_time_parameterization.py` ([#62], [d73e227], [30c8e9f]).
  - `InstructionPoly_as_CompositeInstruction` re-exported from `tesseract_command_language` package init — was bound but not surfaced ([#70], [110e1e0]).

## [0.34.1.6] - 2026-04-27 — PyPI README refresh

Same wheels as 0.34.1.5; recut so the published wheel's README metadata includes the Windows build badge alongside Linux and macOS on PyPI's project page.

### Changed

- README badge row includes a Windows build badge ([6656815]).

## [0.34.1.5] - 2026-04-27 — Windows x64 wheels debut + Python profile bindings

First complete cross-platform PyPI release: Linux, macOS, **and** Windows wheels for Python 3.9–3.12. Closes the long-running [#40] Windows wheels initiative ([#58]). Also ships a batch of profile-binding contributions from [@Joelkang] that had been queued in Unreleased.

### Added

- **Windows x64 wheels** published to PyPI via the new `wheels-windows.yml` workflow. Mirrors `wheels-{linux,macos}.yml`; sources C++ deps from conda-forge via pixi; bundles plugin DLLs via `delvewheel repair --analyze-existing` ([f9c122d], [3e3a2e4]).
- `__init__.py` prepends the package dir and `tesseract_robotics_nanobind.libs/` to `PATH` on Windows — required because `boost::dll`'s plain `LoadLibrary` ignores `os.add_dll_directory` ([006ce38]).
- Cereal polymorphic types re-registered in the `tesseract_serialization` binding TU — MSVC instantiates the cereal registry per-DLL, so upstream's registrations don't reach the consumer `.pyd` without re-registration ([af88515]).
- New developer doc `docs/developer/windows-wheels.md` capturing four Windows wheel packaging gotchas surfaced during the gh-40 diagnosis ([fe5ad75]).


- contributed by [@Joelkang]

  - `CompositeInstruction.push_back` now accepts a `CompositeInstruction` in addition to a plain `Instruction`, enabling nested composites (e.g. raster programs) to be built from Python the same way they are in C++ ([#51], [6d17314]).
  - **SimplePlanner profiles** configurable from Python — `SimplePlannerCompositeProfile` plus seven concrete move profiles (`FixedSize`, `FixedSizeAssign`, `FixedSizeAssignNoIK`, `LVS`, `LVSNoIK`, `LVSAssign`, `LVSAssignNoIK`), with `ProfileDictionary_addSimplePlannerMoveProfile` / `…CompositeProfile` helpers for registration ([#54], [e2bf837]).

  - **task_composer planning-node profiles** exposed through a new `tesseract_task_composer_planning` module: `ContactCheckProfile`, `FixStateBoundsProfile`, `FixStateCollisionProfile`, `KinematicLimitsCheckProfile`, `MinLengthProfile`, `ProfileSwitchProfile`, and `UpsampleTrajectoryProfile`. Heavy TrajOpt-typed members on `FixStateCollisionProfile` (OSQP settings, SQP params, coeff data) are intentionally skipped — configure those via YAML or C++ ([#56], [62cd2cd]).

  - **`ConstantTCPSpeedParameterization`** (KDL-based, constant TCP-speed time parameterization) bound alongside its `ConstantTCPSpeedCompositeProfile` ([#55], [f4f981d]).


### Removed

- vcpkg-based `wheels-windows.yml` variant (workflow_dispatch-only, never green) replaced by the conda-forge / pixi pipeline of the same name ([3e3a2e4]).

## [0.34.1.4] - 2026-04-27 [YANKED] — partial cross-platform release

**Yanked.** Linux and macOS wheels published successfully; Windows wheels built and tested but the publish job hit a PyPI Trusted Publisher configuration mismatch and didn't upload. Superseded by 0.34.1.5.

## [0.34.1.2] - 2026-04-23 — macOS arm64 stability

Follow-up to the macOS arm64 debut in `0.34.1.1`, resolving a garbage-collection crash that surfaced in fresh virtual environments during motion planning.

### Fixed

- Dedupe versioned dylibs in the wheel — the actual root cause of the macOS GC crash ([#48], [7f53185]).
- Pin plugin dylibs on macOS to prevent a garbage-collection crash during motion planning ([#48], [590343f]).

### Changed

- Restore the CI macOS test scope that had been narrowed as a workaround for [#48] ([#49], [07f8f9c]).

## [0.34.1.1] - 2026-04-23 — macOS arm64 wheels debut

First PyPI-published macOS arm64 wheels, shipping via a dedicated `wheels-macos.yml` workflow parallel to the existing Linux x86_64 pipeline. Also introduces a Python trampoline for `ConstraintSet`, Python 3.9 compatibility for example modules, and installable example scripts.

### Added

- **macOS arm64 wheels** published to PyPI via a split, per-platform workflow ([d80e689]).
- Python trampoline for `ifopt.ConstraintSet` so users can subclass it from Python for custom SQP constraints ([361c60e]).
- Examples installable as console scripts (e.g. `tesseract_basic_cartesian_example`) via `[project.scripts]` ([2c62952]).
  - contributed by [@marip8]
- `pixi run build-wheel` task for local wheel builds ([e26df2a]).

### Changed

- Non-benchmark tests fail loud instead of being silently skipped ([a1d7607]).

### Fixed

- Python 3.9 compatibility for example modules via `from __future__ import annotations` ([87ce68e]).

[Unreleased]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.7...HEAD
[0.34.1.7]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.6...0.34.1.7
[0.34.1.6]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.5...0.34.1.6
[0.34.1.5]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.4...0.34.1.5
[0.34.1.4]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.2...0.34.1.4
[0.34.1.2]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.1...0.34.1.2
[0.34.1.1]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.0...0.34.1.1
[tesseract-planning]: https://github.com/tesseract-robotics/tesseract_planning
[#40]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/40
[#48]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/48
[#49]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/49
[#50]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/50
[#51]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/51
[#54]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/54
[#55]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/55
[#56]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/56
[#58]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/58
[#62]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/62
[#63]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/63
[#66]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/66
[#70]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/70
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
[@Joelkang]: https://github.com/Joelkang
[@marip8]: https://github.com/marip8
[@seonixx]: https://github.com/seonixx
