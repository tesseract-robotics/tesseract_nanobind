# Windows wheels to PyPI — Design

**Date:** 2026-04-23
**Status:** Approved for implementation
**Owner:** Jelle Feringa
**Issue:** [#40 — Windows build (pypi)](https://github.com/tesseract-robotics/tesseract_nanobind/issues/40)
**Branch:** `gh-40-windows-build`

## Context

`main` now has two per-platform workflow files: `wheels-linux.yml` and `wheels-macos.yml`. Each is self-contained (own `build`, own `test-wheel`, own `publish`), triggered by tag push, and downloads its own platform-specific artifacts. The split landed when the macOS path re-enabled, and it replaced a previously monolithic `wheels.yml`.

This work adds Windows x64 wheels to the same PyPI release by introducing a third sibling: `wheels-windows.yml`. Each platform's workflow publishes independently to PyPI on tag; wheel filenames are platform-tagged so there is no upload collision.

## Goal

Publish Windows x64 wheels to PyPI on tagged releases for Python 3.9–3.12, via a self-contained `wheels-windows.yml` that mirrors the structural pattern established by `wheels-linux.yml` and `wheels-macos.yml`. Full feature parity with Linux/macOS: `trajopt_ifopt` and `trajopt_sqp` enabled.

## Scope

**In scope:**
- New file `.github/workflows/wheels-windows.yml` containing `build`, `test-wheel`, `publish` (mirrors per-platform structure).
  - `build` — `windows-2022`, Python 3.9–3.12 matrix, x64.
  - `test-wheel` — `windows-2022`, Python 3.9 + 3.12 bookends.
  - `publish` — `ubuntu-latest` (pypa action is Linux-only), downloads `python-*-windows-x64` artifacts.
- Minor `paths-ignore` additions to `wheels-linux.yml` and `wheels-macos.yml` to exclude edits to `wheels-windows.yml` from triggering unrelated platform CI.
- Windows branch in `src/tesseract_robotics/__init__.py` for runtime DLL resolution.

**Out of scope:**
- **Local Windows developer experience** (pixi-on-Windows, Windows build scripts, IDE setup). Separate follow-up. A Windows contributor cloning the repo today gets the same "build via CI or on Linux/macOS" expectation — unchanged by this PR.
- **Windows ARM64 / Windows-on-ARM wheels.** Hosted ARM Windows runners are in preview; demand is low.
- **Python 3.13 support.** Requires cross-platform bump; separate PR across all three platforms.
- **Documentation updates.** Deferred to a follow-up PR after Windows CI has been green on `main` for a stable period — premature docs create broken promises.

## Design

### Architecture after this PR

```
.github/workflows/
├── wheels-linux.yml    (existing — build + test-wheel + publish for linux-64)
├── wheels-macos.yml    (existing — build + test-wheel + publish for macos-arm64)
└── wheels-windows.yml  (NEW     — build + test-wheel + publish for windows-x64)
```

Each workflow:
- Triggers on push to `main`, version tags (`X.Y.Z` or `X.Y.Z.N`), PRs, and `workflow_dispatch`.
- `paths-ignore` excludes edits to the *other* platform workflows.
- Has `concurrency.group = ${{ github.workflow }}-${{ github.ref }}` with `cancel-in-progress: true`.
- Publishes its own platform wheels on tag; filenames don't collide, so parallel uploads to PyPI work.

### Hybrid C++ dependency sourcing on Windows

Two package managers, each chosen for what it handles best on win-64:

- **vcpkg** (via `johnwason/vcpkg-action@v7`, triplet `x64-windows-release`) supplies the bulk: `boost-*`, `eigen3`, `tinyxml2`, `console-bridge`, `assimp`, `urdfdom`, `octomap`, `orocos-kdl`, `pcl`, `flann`, `jsoncpp`, `yaml-cpp`, `openblas`, `fcl`, `ompl`, `taskflow`, `bullet3[multithreading,double-precision,rtti]`, `ccd[double-precision]`, `gperftools`. Exact mirror of upstream `tesseract_python`'s Windows build — proven and well-tuned.
- **conda-forge** (via `conda-incubator/setup-miniconda@v3`) supplies `osqp` only. OSQP is the solver backend for both `trajopt_sqp` (direct dep) and `ifopt` (built from source via our rosinstall, same as Linux/macOS). conda-forge's `osqp` is a clean C-API binary with small ABI surface; vcpkg's port has known CMake-config friction we avoid by sourcing elsewhere.

`CMAKE_PREFIX_PATH` includes both `${VCPKG}/installed/x64-windows-release` and `%CONDA_PREFIX%\Library` — the `\Library` suffix is non-negotiable on conda-for-Windows (CMake configs are under `Library/lib/cmake`, not the env root).

`ifopt` builds from source via `dependencies.rosinstall` — consistent with Linux/macOS, no change to the rosinstall.

### Toolchain

- **Runner:** `windows-2022` (VS 2022 Enterprise pre-installed).
- **Compiler:** `cl` from VS 2022 (v143 platform toolset), activated once per job via `ilammy/msvc-dev-cmd@v1` — env persists across subsequent steps regardless of shell.
- **Generator:** `Ninja` (pre-installed on `windows-2022`; not via choco).
- **Workspace build:** direct `colcon build` invocation (not `tesseract-robotics/colcon-action`, which assumes a colcon-buildable target — our nanobind repo builds via `pip wheel`, not `colcon build`). Upstream deps (tesseract, tesseract_planning, trajopt, ifopt, descartes, etc.) build into `ws/install`; the nanobind wheel is then built against that via `pip wheel` with `CMAKE_PREFIX_PATH` pointing at `ws/install`. Same two-stage pattern as Linux/macOS.
- **vcpkg triplet:** `x64-windows-v143-release` (vendored overlay in `.github/workflows/vcpkg_triplets/`, copied from tesseract core). Release-only build (`VCPKG_BUILD_TYPE release`) — skips debug variants of boost et al, cutting vcpkg time roughly in half.
- **Mandatory flags:** `-DEIGEN_DONT_ALIGN=1 -DEIGEN_DONT_VECTORIZE=1` in `CXXFLAGS`. Without these, Eigen alignment assumptions cross MSVC ABI boundaries and segfault at runtime. Non-negotiable Windows landmine.
- **Other flags matching Linux/macOS:** `-DCMAKE_CXX_VISIBILITY_PRESET=default`, `-DCMAKE_VISIBILITY_INLINES_HIDDEN=OFF` (RTTI typeinfo across DLL boundaries), `-DCMAKE_BUILD_TYPE=Release`, `-DBUILD_SHARED_LIBS=ON`, `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`, `-DTESSERACT_BUILD_TRAJOPT_IFOPT=ON`, `-DVCPKG_APPLOCAL_DEPS=OFF`, `-DBUILD_IPOPT=OFF`, `-DBUILD_SNOPT=OFF`, `-DNO_OPENCL=ON`, `-DTESSERACT_ENABLE_TESTING=OFF`, `-DTESSERACT_ENABLE_EXAMPLES=OFF`.
- **Shell mix:** `bash` (Git-for-Windows) for most steps — MSVC env is already active globally. `cmd` only where `%` variable expansion is genuinely cleaner than `$`.

### Wheel packaging

1. `pip wheel . -w dist\ --no-build-isolation` with `CMAKE_PREFIX_PATH` pointing at `ws/install`, vcpkg, and the conda env (in that order of precedence).
2. **Bundle plugin-factory DLLs before `delvewheel`**, not after. Plugin factories are `dlopen`-loaded at runtime, invisible to `delvewheel`'s static dep scan. Dropping them into the wheel's package dir first lets `delvewheel` resolve their transitive DLL deps in the same repair pass. Mirrors the Linux "bundle plugins → auditwheel" order.
3. `delvewheel repair --add-path "<ws/install/bin>;<vcpkg/bin>;<conda/Library/bin>" -w wheelhouse dist\*.whl` — pulls transitive DLL deps into `tesseract_robotics.libs/` (sibling dir to the package) and injects the `_delvewheel_init_patch.py` hook that Python needs for `os.add_dll_directory()`.
4. Patch `data/task_composer_config/*.yaml` — replace `/usr/local/lib` with `@PLUGIN_PATH@` so `__init__.py` can resolve at runtime. Same sed expression as Linux/macOS.
5. Strip `search_paths:` from `data/tesseract_support/*.yaml` — forces env-var-driven plugin search so bundled plugins are discovered correctly at runtime.

### Plugin-factory DLL naming

On Windows, CMake emits `foo.dll` (no `lib` prefix) for `MODULE`/`SHARED` libraries — unlike `libfoo.so` on Linux or `libfoo.dylib` on macOS. The plugin-bundling loop references bare names: `tesseract_collision_bullet_factories.dll`, `tesseract_kinematics_kdl_factories.dll`, etc.

### Runtime DLL loading (`__init__.py` Windows branch)

Python 3.8+ on Windows requires `os.add_dll_directory()` for DLL resolution outside `sys.prefix\Library\bin` — the old `PATH`-based mechanism was removed for security. `src/tesseract_robotics/__init__.py` gets a new `sys.platform == 'win32'` branch that:

1. Resolves `@PLUGIN_PATH@` in the runtime YAML rewrite to `<package_parent>\tesseract_robotics.libs` (delvewheel's output dir), NOT `.libs` or `.dylibs`.
2. Calls `os.add_dll_directory()` on both:
   - `<package_parent>\tesseract_robotics.libs` — for delvewheel-bundled deps.
   - The package dir itself — for plugin-factory DLLs bundled alongside the Python extensions.
3. **Runs BEFORE any extension import.** The current `__init__.py` imports `_tesseract_common` early; the Windows branch must come first.
4. Uses `os.pathsep` for any `TESSERACT_*_PLUGIN_DIRECTORIES` env var assembly (Windows uses `;`, POSIX uses `:`).

Feature-detect `os.add_dll_directory` rather than branching on `sys.platform` — the attribute only exists on Windows, so the `getattr` probe doubles as a platform check and avoids pyright platform-narrowing noise.

### Publish job

Mirrors `wheels-macos.yml`'s pattern: `needs: [build, test-wheel]`, `runs-on: ubuntu-latest` (pypa action is Linux-only), downloads `python-*-windows-x64`, `pypa/gh-action-pypi-publish@release/v1`.

## Files changed

| File | Change | Approx. size |
|------|--------|--------------|
| `.github/workflows/wheels-windows.yml` | **NEW** — full workflow with `build`, `test-wheel`, `publish` jobs mirroring `wheels-macos.yml` structure, adapted for Windows toolchain. | ~200 lines |
| `.github/workflows/vcpkg_triplets/x64-windows-v143-release.cmake` | **NEW** — vendored overlay triplet from tesseract core. Release-only build, v143 toolset. | ~7 lines |
| `.github/workflows/wheels-linux.yml` | Add `wheels-windows.yml` to `paths-ignore`. | +2 lines |
| `.github/workflows/wheels-macos.yml` | Add `wheels-windows.yml` to `paths-ignore`. | +2 lines |
| `src/tesseract_robotics/__init__.py` | Add `os.add_dll_directory` block (feature-detected, runs before extension imports). Extend `_configure_environment` with Windows bundled-plugin detection. | ~20 added lines |

### Files deliberately NOT changed

- **`pyproject.toml`** — `[tool.pixi.workspace].platforms` stays `["osx-arm64", "linux-64"]`. Adding `win-64` would half-enable local Windows dev (pixi install succeeds but bash build scripts fail), which is out of scope.
- **`scripts/build_tesseract_cpp.sh`**, **`scripts/build_wheel.sh`** — bash-only, no `.bat` / `.ps1` equivalent for this PR.
- **`dependencies.rosinstall`** — no Windows-specific branches; same sources everywhere.
- **`docs/`, `README.md`** — deferred to follow-up PR.

## Validation plan

No local Windows machine required. All verification via CI iteration on `gh-40-windows-build`.

1. **Build iteration** — push branch, watch the new `wheels-windows.yml`, fix forward. Expected early failures (ranked by likelihood):
   - CMake can't find OSQP → fix `CMAKE_PREFIX_PATH` to include `%CONDA_PREFIX%\Library`.
   - vcpkg cold-cache 20–30 min build; warm (cached) seconds.
   - Eigen alignment SIGSEGV → verify `EIGEN_DONT_ALIGN=1` is propagated (inspect compile log).
   - Plugin factory DLL naming mismatch → adjust loop if CMake emits unexpected names.
2. **Wheel inspection** — on first successful build, unzip artifact. Confirm `tesseract_robotics.libs/` contains OSQP, ifopt, bullet, fcl, ompl DLLs; confirm `_delvewheel_init_patch.py` is present; confirm plugin factory DLLs are in the package dir.
3. **`test-wheel` gate** — clean `windows-2022` runner, fresh venv, wheel-only install. This is the packaging-bug surface; build-env DLL leakage does not reach here. Same pytest exclusions as Linux/macOS: `tesseract_time_parameterization`, `examples`, `tesseract_kinematics`, `benchmarks`.
4. **`trajopt_ifopt` import smoke test** — explicitly added to `test-wheel`. This is the canary for the hybrid vcpkg+conda-forge integration. If this import works end-to-end, the OSQP-from-conda-forge sourcing succeeded.
5. **`workflow_dispatch`** — already present in the workflow, supports re-runs without dummy commits during iteration.
6. **Publish gate (human discipline)** — do not push a release tag until Windows CI has had 5+ consecutive green runs on `main`. No YAML enforces this; it's a release-process norm.

## Risks

| # | Risk | Severity | Mitigation |
|---|------|---------|------------|
| 1 | Plugin-factory DLL naming differs from Linux/macOS (`foo.dll` vs `libfoo.so`/`.dylib`). | Low | Bundling loop uses bare names. Verify at first build with `dir ws\install\bin\*_factories.dll`. |
| 2 | `__init__.py` has no Windows path today; delvewheel output lives in sibling `tesseract_robotics.libs/`; Python 3.8+ needs explicit `os.add_dll_directory()`. | Medium | New feature-detected `os.add_dll_directory` block runs **before** any extension import. |
| 3 | Repacking the wheel after delvewheel breaks its init-patch hook. | Medium | Bundle plugin DLLs **before** delvewheel, not after — mirrors Linux's "bundle → auditwheel" order. |
| 4 | MSVC ABI / CRT mismatch between vcpkg Release and conda-forge win-64. | Low | Both use Release + MD runtime. conda-forge's `osqp` is a small C API with no `std::` on the boundary. Escalation: pass `/D_ITERATOR_DEBUG_LEVEL=0`. |
| 5 | Path separator for `TESSERACT_*_PLUGIN_DIRECTORIES` env vars. | Low | Use `os.pathsep` in the Windows branch of `__init__.py`. |
| 6 | IfOpt CMake config can't find OSQP from conda-forge. | Medium | `CMAKE_PREFIX_PATH` includes `%CONDA_PREFIX%\Library` (not `%CONDA_PREFIX%` alone). |
| 7 | Long-path limit (260 chars) hit by `ws/src/tesseract_planning/...`. | Low | `git config --system core.longpaths true` as early step; `actions/checkout@v4` handles this on recent versions. |
| 8 | `boost-stacktrace` linking `dbghelp` on Windows. | Very low | vcpkg port handles transitively. Fallback: explicit `dbghelp.lib` link. |
| 9 | delvewheel can't trace plugins' transitive DLL deps. | Addressed | Plugins are present during delvewheel's graph walk (see risk #3 mitigation). |
| 10 | Unicode in CI prints crashes `cmd` with cp1252. | Low | Keep all CI prints ASCII. |
| 11 | First-build flakiness on a release tag. | Medium | Merge to `main` early; observe CI stability for ≥1 week before the next release. Never publish relying on first-green Windows CI. |

## Rollback

Single-file revert restores pre-Windows behavior: delete `wheels-windows.yml`, revert the two-line `paths-ignore` additions in sibling workflows, revert the `__init__.py` Windows block. If a broken Windows wheel reaches PyPI:

1. Yank the broken release on PyPI (Manage → Release → Yank). Yanking hides the version from `pip install` resolution without deleting it.
2. Delete `wheels-windows.yml` (or revert the commit).
3. Cut a new patch version.

No database migrations, no external system state. Pure CI change.

## Success criteria

- `build` (in `wheels-windows.yml`) green on all 4 Python versions (3.9, 3.10, 3.11, 3.12).
- `test-wheel` (in `wheels-windows.yml`) green on 3.12 (3.9 runs with `continue-on-error`, matching Linux/macOS policy).
- `from tesseract_robotics import trajopt_ifopt` succeeds from the wheel in a clean venv — proves full feature parity.
- `publish` uploads 4 `*-win_amd64.whl` wheels alongside Linux and macOS wheels on the next tagged release.
- No regression in `wheels-linux.yml` / `wheels-macos.yml` jobs.

## Follow-up: align with upstream tesseract-org Windows CI pattern

The current workflow hand-rolls the vcpkg+colcon orchestration. Upstream `tesseract/windows.yml` and `tesseract_planning/windows.yml` instead compose `johnwason/vcpkg-action@v7` + `ilammy/msvc-dev-cmd@v1` + `tesseract-robotics/colcon-action@v14`. Our first two are already aligned; the third is not.

**Why not now:** changing structure mid-iteration (while ifopt/bigobj/etc. patches are still being discovered) would force another cold vcpkg (~60 min) on every structural edit, slowing the hunt for real failures. Refactoring on a green baseline is cheaper.

**Planned refactor (post-green, separate PR):**

- Replace the hand-rolled colcon invocation in the C++ workspace build step with `tesseract-robotics/colcon-action@v14`, passing our patches step as `before-script`, our CXXFLAGS + cmake flags as `target-args`, and `rosdep-enabled: false` / `add-ros-ppa: false` to skip ROS-isms.
- Our nanobind repo stays out of the colcon build via `--packages-ignore tesseract_robotics` in `target-args`. The subsequent `pip wheel` step builds it.
- Keep our wheel-build/delvewheel/test-wheel/publish stages verbatim — no upstream analog, no reason to change.
- Divergence we knowingly retain: `-DTESSERACT_BUILD_TRAJOPT_IFOPT=ON` plus the ifopt include patches. Upstream `tesseract_planning/windows.yml` disables trajopt_ifopt on Windows; we cannot (feature parity requirement from this PR). Document the divergence inline in the workflow.

**Expected reduction:** ~40–60 lines removed from `wheels-windows.yml`. No behaviour change intended.
