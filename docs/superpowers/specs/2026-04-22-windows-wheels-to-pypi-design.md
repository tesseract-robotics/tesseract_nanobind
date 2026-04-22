# Windows wheels to PyPI — Design

**Date:** 2026-04-22
**Status:** Approved for implementation
**Owner:** Jelle Feringa
**Issue:** [#40 — Windows build (pypi)](https://github.com/tesseract-robotics/tesseract_nanobind/issues/40)
**Branch:** `gh-40-windows-build` (stacked on `jf/macos-wheels-to-pypi`)

## Context

`wheels.yml` currently builds Linux x64 and macOS arm64 via a shared `build` job (both driven by pixi), gates on a shared `test-wheel`, and publishes on tag via `pattern: python-*`. The macOS path was re-enabled by PR `jf/macos-wheels-to-pypi`, which also dropped the former `linux-only` artifact filter. No Windows path exists.

This work adds Windows x64 wheels to the same PyPI release alongside Linux and macOS.

**Why this is stacked on the macOS PR:** both PRs modify the `publish` job's `needs:` list. Stacking avoids a merge conflict on that line and lets the Windows PR inherit the already-corrected `pattern: python-*` filter for free. If macOS takes longer than expected, we rebase to `main` and reinstate the filter change in this PR.

## Goal

Publish Windows x64 wheels to PyPI on tagged releases for Python 3.9–3.12, gated by the same build → test-wheel → publish flow as Linux/macOS. Full feature parity with Linux/macOS: `trajopt_ifopt` and `trajopt_sqp` enabled.

## Scope

**In scope:**
- New `build-win` job on `windows-2022`, Python 3.9–3.12 matrix, x64.
- New `test-wheel-win` job on `windows-2022`, Python 3.9 + 3.12 (bookends, matching Linux/macOS test coverage).
- Extend `publish.needs` from `[build, test-wheel]` to `[build, build-win, test-wheel, test-wheel-win]`.
- Windows branch in `src/tesseract_robotics/__init__.py` for runtime DLL resolution.

**Out of scope:**
- **Local Windows developer experience** (pixi-on-Windows, Windows build scripts, IDE setup). Separate follow-up effort. A Windows contributor cloning the repo today gets the same "build via CI or on Linux/macOS" expectation — unchanged by this PR.
- **Windows ARM64 / Windows-on-ARM wheels.** Hosted ARM Windows runners are in preview; demand is low.
- **Python 3.13 support.** Requires cross-platform bump; separate PR across all three platforms.
- **Matrix-based Windows integration** (adding Windows as a matrix entry on the existing `build` job). Rejected: Windows toolchain is fundamentally different from Linux/macOS (vcpkg + conda-forge + MSVC + VsDevCmd.bat + cmd/bash shell mix vs. pixi-everywhere on Linux/macOS). A matrix split would require `if: matrix.config.os == 'windows'` guards on every step and a mixed-shell setup that's harder to read than two parallel jobs.
- **Documentation updates.** Deferred to a follow-up PR after Windows CI has been green on `main` for a stable period — premature docs create broken promises.

## Design

### Architecture after this PR

```
.github/workflows/wheels.yml
├── build              (existing, Linux + macOS matrix, pixi-driven)
├── build-win          (NEW, Windows x64, vcpkg + conda-forge hybrid)
├── test-wheel         (existing, Linux + macOS matrix)
├── test-wheel-win     (NEW, Windows bookend matrix)
└── publish            (existing — only `needs:` list extended)
```

### Hybrid C++ dependency sourcing on Windows

Two package managers, each chosen for what it handles best on win-64:

- **vcpkg** (via `johnwason/vcpkg-action@v7`, triplet `x64-windows-release`) supplies the bulk: `boost-*`, `eigen3`, `tinyxml2`, `console-bridge`, `assimp`, `urdfdom`, `octomap`, `orocos-kdl`, `pcl`, `flann`, `jsoncpp`, `yaml-cpp`, `openblas`, `fcl`, `ompl`, `taskflow`, `bullet3[multithreading,double-precision,rtti]`, `ccd[double-precision]`, `gperftools`. Exact mirror of upstream `tesseract_python`'s Windows build — proven and well-tuned.
- **conda-forge** (via `conda-incubator/setup-miniconda@v3`) supplies `osqp` only. OSQP is the solver backend for both `trajopt_sqp` (direct dep) and `ifopt` (built from source via our rosinstall, same as Linux/macOS). conda-forge's `osqp` is a clean C-API binary with small ABI surface; vcpkg's port has known CMake-config friction we avoid by sourcing elsewhere.

`CMAKE_PREFIX_PATH` includes both `${VCPKG}/installed/x64-windows-release` and `%CONDA_PREFIX%\Library` — the `\Library` suffix is non-negotiable on conda-for-Windows (CMake configs are under `Library/lib/cmake`, not the env root).

`ifopt` builds from source via `dependencies.rosinstall` — consistent with Linux/macOS, no change to the rosinstall.

### Toolchain

- **Runner:** `windows-2022` (VS 2022 Enterprise pre-installed).
- **Compiler:** `cl` from VS 2022, activated via `VsDevCmd.bat -arch=amd64 -host_arch=amd64` in each `cmd` step that invokes the compiler.
- **Generator:** `Ninja` (installed via `choco install ninja`).
- **Mandatory flags:** `-DEIGEN_DONT_ALIGN=1 -DEIGEN_DONT_VECTORIZE=1` in `CXXFLAGS`. Without these, Eigen alignment assumptions cross MSVC ABI boundaries and segfault at runtime. Non-negotiable Windows landmine.
- **Other flags matching Linux/macOS:** `-DCMAKE_CXX_VISIBILITY_PRESET=default`, `-DCMAKE_VISIBILITY_INLINES_HIDDEN=OFF` (RTTI typeinfo across DLL boundaries), `-DCMAKE_BUILD_TYPE=Release`, `-DBUILD_SHARED_LIBS=ON`, `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`, `-DTESSERACT_BUILD_TRAJOPT_IFOPT=ON`, `-DVCPKG_APPLOCAL_DEPS=OFF`, `-DBUILD_IPOPT=OFF`, `-DBUILD_SNOPT=OFF`, `-DINSTALL_OMPL=OFF`, `-DTESSERACT_ENABLE_TESTING=OFF`, `-DTESSERACT_ENABLE_EXAMPLES=OFF`.
- **Shell mix:** `cmd` for any step that invokes `VsDevCmd.bat` or the colcon/cmake build (so the VS env stays active); `bash` (Git-for-Windows, pre-installed) for sed/zip/mv-style wheel-repack steps where POSIX utilities are cleaner.

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

### Publish job change

```yaml
publish:
  needs: [build, build-win, test-wheel, test-wheel-win]   # was: [build, test-wheel]
  # pattern: python-* stays unchanged (already set by macOS PR)
  # everything else unchanged
```

One-line change. The artifact filter is already correct.

## Files changed

| File | Change | Approx. size |
|------|--------|--------------|
| `.github/workflows/wheels.yml` | Add `build-win` job (4-version matrix). Add `test-wheel-win` job (bookend 2-version matrix: 3.9, 3.12). Extend `publish.needs` to include `build-win` and `test-wheel-win`. | ~180 added lines |
| `src/tesseract_robotics/__init__.py` | Add `sys.platform == 'win32'` branch per above. | ~15 added lines |

### Files deliberately NOT changed

- **`pyproject.toml`** — `[tool.pixi.workspace].platforms` stays `["osx-arm64", "linux-64"]`. Adding `win-64` would half-enable local Windows dev (pixi install succeeds but bash build scripts fail), which is out of scope.
- **`scripts/build_tesseract_cpp.sh`**, **`scripts/build_wheel.sh`** — bash-only, no `.bat` / `.ps1` equivalent for this PR.
- **`dependencies.rosinstall`** — no Windows-specific branches; same sources everywhere.
- **`docs/`, `README.md`** — deferred to follow-up PR.

## Validation plan

No local Windows machine required. All verification via CI iteration on `gh-40-windows-build`.

1. **Build iteration** — push branch, watch `build-win`, fix forward. Expected early failures (ranked by likelihood):
   - CMake can't find OSQP → fix `CMAKE_PREFIX_PATH` to include `%CONDA_PREFIX%\Library`.
   - vcpkg cold-cache 20–30 min build; warm (cached) seconds.
   - Eigen alignment SIGSEGV → verify `EIGEN_DONT_ALIGN=1` is propagated (inspect compile log).
   - Plugin factory DLL naming mismatch → adjust loop if CMake emits unexpected names.
2. **Wheel inspection** — on first successful build, unzip artifact. Confirm `tesseract_robotics.libs/` contains OSQP, ifopt, bullet, fcl, ompl DLLs; confirm `_delvewheel_init_patch.py` is present; confirm plugin factory DLLs are in the package dir.
3. **`test-wheel-win` gate** — clean `windows-2022` runner, fresh venv, wheel-only install. This is the packaging-bug surface; build-env DLL leakage does not reach here. Same pytest exclusions as Linux/macOS: `tesseract_time_parameterization`, `examples`, `tesseract_kinematics`, `benchmarks`.
4. **`trajopt_ifopt` import smoke test** — explicitly added to `test-wheel-win`. This is the canary for the hybrid vcpkg+conda-forge integration. If this import works end-to-end, the OSQP-from-conda-forge sourcing succeeded.
5. **`workflow_dispatch`** — already present in the workflow, supports re-runs without dummy commits during iteration.
6. **Publish gate (human discipline)** — do not push a release tag until Windows CI has had 5+ consecutive green runs on `main`. No YAML enforces this; it's a release-process norm.

## Risks

| # | Risk | Severity | Mitigation |
|---|------|---------|------------|
| 1 | Plugin-factory DLL naming differs from Linux/macOS (`foo.dll` vs `libfoo.so`/`.dylib`). | Low | Bundling loop uses bare names. Verify at first build with `dir ws\install\bin\*_factories.dll`. |
| 2 | `__init__.py` has no Windows path today; delvewheel output lives in sibling `tesseract_robotics.libs/`; Python 3.8+ needs explicit `os.add_dll_directory()`. | Medium | New `sys.platform == 'win32'` branch runs **before** any extension import. |
| 3 | Repacking the wheel after delvewheel breaks its init-patch hook. | Medium | Bundle plugin DLLs **before** delvewheel, not after — mirrors Linux's "bundle → auditwheel" order. |
| 4 | MSVC ABI / CRT mismatch between vcpkg Release and conda-forge win-64. | Low | Both use Release + MD runtime. conda-forge's `osqp` is a small C API with no `std::` on the boundary. Escalation: pass `/D_ITERATOR_DEBUG_LEVEL=0`. |
| 5 | Path separator for `TESSERACT_*_PLUGIN_DIRECTORIES` env vars. | Low | Use `os.pathsep` in the Windows branch of `__init__.py`. |
| 6 | IfOpt CMake config can't find OSQP from conda-forge. | Medium | `CMAKE_PREFIX_PATH` includes `%CONDA_PREFIX%\Library` (not `%CONDA_PREFIX%` alone). |
| 7 | Long-path limit (260 chars) hit by `ws/src/tesseract_planning/...`. | Low | `git config --system core.longpaths true` as early step; `actions/checkout@v4` handles this on recent versions. |
| 8 | `boost-stacktrace` linking `dbghelp` on Windows. | Very low | vcpkg port handles transitively. Fallback: explicit `dbghelp.lib` link. |
| 9 | delvewheel can't trace plugins' transitive DLL deps. | Addressed | Plugins are present during delvewheel's graph walk (see risk #3 mitigation). |
| 10 | Unicode in CI prints crashes `cmd` with cp1252. | Low | Keep all CI prints ASCII. |
| 11 | First-build flakiness on a release tag. | Medium | Merge to `main` early; observe CI stability for ≥1 week before the next release. Never publish relying on first-green Windows CI. |
| 12 | macOS PR doesn't merge and we rebase onto `main` — need to also restore the `python-*` artifact filter change. | Low | On rebase, pick up the artifact filter change from the macOS PR into this one. Trivial one-line. |

## Rollback

Single-commit revert restores pre-Windows behavior (Linux + macOS publish only). If a broken Windows wheel reaches PyPI:

1. Yank the broken release on PyPI (Manage → Release → Yank). Yanking hides the version from `pip install` resolution without deleting it.
2. Remove `build-win` and `test-wheel-win` from `publish.needs` (or revert the commit).
3. Cut a new patch version.

No database migrations, no external system state. Pure CI change.

## Success criteria

- `build-win` green on all 4 Python versions (3.9, 3.10, 3.11, 3.12).
- `test-wheel-win` green on 3.12 (3.9 runs with `continue-on-error`, matching Linux/macOS policy).
- `from tesseract_robotics import trajopt_ifopt` succeeds from the wheel in a clean venv — proves full feature parity.
- `publish` uploads 4 `*-win_amd64.whl` wheels alongside Linux and macOS wheels on the next tagged release.
- No regression in Linux / macOS `build` / `test-wheel` / `publish` jobs.
