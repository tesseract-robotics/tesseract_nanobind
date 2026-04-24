# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased] — targeting 0.35.0.0

### Added

- `CompositeInstruction.push_back` now accepts a `CompositeInstruction` in addition to a plain `Instruction`, enabling nested composites (e.g. raster programs) to be built from Python the same way they are in C++ ([#51], [6d17314]).
  - contributed by [@Joelkang]

### In progress

- **Windows x64 wheels** — targeted for this release, pending the next upstream [tesseract-planning][tesseract-planning] release with the Windows-compatibility changes we depend on. A working CI pipeline is being prepared in [`wheels-windows.yml`][#50]; tracked by [#40].
  - **Call for contributors:** if you have Windows build experience (MSVC, delocate-style DLL bundling, or CI troubleshooting), please comment on [#40] — help is actively welcome.

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

[Unreleased]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.2...HEAD
[0.34.1.2]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.1...0.34.1.2
[0.34.1.1]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/0.34.1.0...0.34.1.1
[tesseract-planning]: https://github.com/tesseract-robotics/tesseract_planning
[#40]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/40
[#48]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/48
[#49]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/49
[#50]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/50
[#51]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/51
[07f8f9c]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/07f8f9c8c54ab13c3d10ceca00181091d0126336
[2c62952]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/2c62952fded6cb1253cb45441d7cd6f9b0423593
[361c60e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/361c60e263f0768e1b23d3a9919f700d06b15993
[590343f]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/590343f93dc4b82fae92f56d038cdb5ba41a511d
[6d17314]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/6d17314a0b5aa1db45ffb9d9b18a8b3e048d3d06
[7f53185]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/7f531851ce70a2564251761f560020d768e3e940
[87ce68e]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/87ce68ed5d015ae596a9047d629ed26f8595e3b0
[a1d7607]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/a1d7607afa7d03cd27e845059cb455eb74a4d7ec
[d80e689]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/d80e689f559f560aeb5e5b28cb332e942d15ecef
[e26df2a]: https://github.com/tesseract-robotics/tesseract_nanobind/commit/e26df2a10d37b9d4ffb689a384e6bad490b2fb7d
[@Joelkang]: https://github.com/Joelkang
[@marip8]: https://github.com/marip8
