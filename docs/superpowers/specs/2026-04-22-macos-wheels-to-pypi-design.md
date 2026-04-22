# macOS wheels to PyPI (via split workflow)

**Date:** 2026-04-22 (rewritten after initial implementation attempt)
**Status:** Approved for implementation
**Owner:** Jelle Feringa

## Context

`wheels.yml` historically carried Linux x64 wheels. It contained a commented-out macOS arm64 matrix entry and full macOS build/delocate/plugin-bundle/YAML-patch steps, all disabled because macOS wheels exceeded PyPI's 100 MB per-file ceiling. PyPI has now granted a 250 MB limit, removing the size blocker.

A first attempt enabled macOS in-place on `wheels.yml` (three surgical edits). It surfaced problems that are all symptoms of a single root cause: **`wheels.yml`'s colcon invocation had drifted out of sync with `scripts/build_tesseract_cpp.sh` (the local C++ build script)** and `wheels.yml`'s wheel-build inline shell had drifted out of sync with `scripts/build_wheel.sh` (the local wheel-build script). The local scripts build clean on macOS; the CI inline duplication does not. Every macOS CI failure in the first attempt (`sed -i` portability, `DYLD_LIBRARY_PATH` poisoning of Apple `git`, `osqp_eigen` falling through to the pre-release `csc_set_data` code path because `try_compile` behaved differently) traced back to some divergence between those two codepaths.

Maintaining two codepaths — one in CI YAML, one in local shell scripts — is a maintenance trap. We've already paid for it. The fix is structural, not a one-line patch.

## Goal

Publish macOS arm64 wheels to PyPI on tagged releases, alongside existing Linux x64 wheels, using **a single source of truth for the build commands**: the scripts under `scripts/`. CI becomes a thin wrapper that invokes the same commands you run locally.

## Scope

**In scope:**
- Split `.github/workflows/wheels.yml` into two workflow files: `wheels.yml` (Linux, existing flow preserved) and `wheels-macos.yml` (macOS arm64).
- `wheels-macos.yml` invokes the build via `pixi run build-cpp` (already calls `scripts/build_tesseract_cpp.sh`) and `pixi run build-wheel` (new pixi task wrapping `scripts/build_wheel.sh`).
- Refactor `scripts/build_wheel.sh` to drop the `conda create --clone tesseract_nb` block. The "portable" (non-`--dev`) path runs in the currently-active env (pixi locally, pixi-activated in CI).
- Add a `build-wheel` pixi task in `pyproject.toml` that runs `zsh scripts/build_wheel.sh`.
- Each workflow has its own `publish` job gated on `refs/tags/*`. Two Trusted Publisher uploads per release (one linux, one macos); PyPI accepts incremental uploads to the same version.
- macOS matrix: Python 3.9, 3.10, 3.11, 3.12 for build; 3.9, 3.12 for test-wheel (matches Linux test coverage).

**Out of scope:**
- macOS x86_64 / Intel runners. Shrinking user base; `macos-13` Intel runners on deprecation path.
- `universal2` fat wheels.
- Wheel size reduction.
- Migration to `macos-15` runners.
- Test.pypi.org dry-run plumbing.
- Refactoring Linux build out of `wheels.yml` into its own script. Current Linux inline shell works; out of scope for this change.

## Design

### File structure after this change

```
.github/workflows/
  wheels.yml          # Linux x64 only, existing flow preserved
  wheels-macos.yml    # NEW — macOS arm64, thin wrapper around pixi tasks
scripts/
  build_tesseract_cpp.sh  # UNCHANGED — already portable, called via `pixi run build-cpp`
  build_wheel.sh          # REFACTORED — portable path runs in active env (no conda clone)
pyproject.toml        # NEW pixi task: `build-wheel` → `zsh scripts/build_wheel.sh`
```

### `scripts/build_wheel.sh` refactor

Current state: the non-`--dev` path does `eval "$(conda shell.zsh hook)"` then `conda create -n tesseract_wheel_build_$$  --clone tesseract_nb -y`, activates the cloned env, runs the wheel build + delocate + plugin bundle + YAML patch, then destroys the clone.

Problems:
- CI has no conda (uses pixi). `conda shell.zsh hook` fails.
- The clone source `tesseract_nb` is a locally-named conda env that doesn't exist in CI.
- The isolation is defensive — a historical artifact. In the pixi/conda era, the active env is already isolated; cloning is overkill.

Refactor:
- Remove the `eval "$(conda shell.zsh hook)"` + `conda create --clone` + `conda activate` + `conda deactivate` + `conda env remove` block entirely.
- The "portable wheel" path becomes: install `delocate setuptools-scm` via `pip`, run the wheel build in the active env, delocate, bundle plugins, patch YAMLs, repack.
- `--dev` path unchanged.
- Keep the `DYLD_LIBRARY_PATH` export the script already sets (local macOS needs workspace lib on the dyld path for plugin factory `dlopen` during delocate's symbol resolution). CI does NOT need it — pixi sets `CONDA_PREFIX` and our libs are rpath-resolved. Script's existing line is safe in both envs.

### `pyproject.toml` — new pixi task

```toml
[tool.pixi.tasks]
# ... existing tasks ...
build-wheel = { cmd = "zsh scripts/build_wheel.sh", description = "Build portable wheel (macOS: delocate + plugins + YAML patch)" }
```

### `wheels-macos.yml` (new file)

Structure:

```yaml
name: Build & Test (macOS)
on:
  push:
    branches: [main]
    tags:
      - '[0-9]+.[0-9]+.[0-9]+'
      - '[0-9]+.[0-9]+.[0-9]+.[0-9]+'
    paths-ignore: [**.md, docs/**, .gitignore, LICENSE]
  pull_request:
    paths-ignore: [same]
  workflow_dispatch:

jobs:
  build:
    runs-on: macos-14
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.10', '3.11', '3.12']
    defaults:
      run:
        shell: bash -el {0}
    steps:
      - actions/checkout (into ws/src/tesseract_nanobind)
      - brew install automake autoconf libtool
      - prefix-dev/setup-pixi with PIXI_ENV=py{python-version}
      - pixi run typecheck
      - cache vcs repos
      - vcs import --input dependencies.rosinstall --skip-existing
      - cache colcon build (key includes os=macos, arch=arm64)
      - pixi run build-cpp    # <- calls scripts/build_tesseract_cpp.sh
      - pixi run build-wheel  # <- NEW task, calls scripts/build_wheel.sh
      - upload-artifact python-${python}-macos-arm64

  test-wheel:
    needs: build
    runs-on: macos-14
    continue-on-error: ${{ matrix.python == '3.9' }}
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.12']
    steps:
      - actions/checkout
      - actions/setup-python
      - download-artifact python-${python}-macos-arm64
      - install wheel in virgin venv
      - smoke test (import + version)
      - run full tests (same pytest ignores as Linux)

  publish:
    needs: [build, test-wheel]
    if: startsWith(github.ref, 'refs/tags/')
    runs-on: macos-14
    environment: pypi
    permissions: {id-token: write}
    steps:
      - download-artifact python-*-macos-arm64
      - pypa/gh-action-pypi-publish
```

The macOS publish job runs on `macos-14` (not `ubuntu-latest` like the Linux one) only so we don't need to serialize Linux and macOS publishes. They run in parallel on tag.

### `wheels.yml` cleanup

Remove all macOS-conditional branches — the workflow becomes Linux-only:
- Strip `brew (macOS)` step (entirely Linux now).
- Remove `- os: macos` matrix entry (already wasn't there on the pre-PR baseline).
- Remove `colcon build C++ deps (macOS)`, `build nanobind wheel (macOS)`, `delocate (macOS)`, `bundle plugins (macOS)` steps.
- `colcon build C++ deps (Linux)` becomes just `colcon build C++ deps`.
- `build nanobind wheel (Linux)` becomes `build nanobind wheel`.
- Similar for `bundle plugins`, `bundle deps`.
- Drop `matrix.config.os` and `matrix.config.arch` since there's only one config. Matrix becomes `python: ['3.9', '3.10', '3.11', '3.12']` with fixed `runner: ubuntu-22.04`.
- Artifact name simplifies to `python-${python}-linux-x64` (stays the same as today, no change).
- `publish` download pattern is `python-*-linux-*` (reverted from the initial PR attempt's `python-*`, because the macOS publish is now in the separate workflow).

### Unchanged

- Trusted Publishing config (OIDC `id-token: write`, `environment: pypi`, `pypa/gh-action-pypi-publish@release/v1`) — used verbatim in both workflows.
- Linux `debug libs` step, Linux bundle logic, Linux pytest ignores.
- Tag trigger patterns.
- Pre-commit hooks, pytest-testmon, pyright.

## Validation plan

1. Open PR with:
   - Refactored `scripts/build_wheel.sh`
   - New `build-wheel` pixi task in `pyproject.toml`
   - New `.github/workflows/wheels-macos.yml`
   - Stripped `.github/workflows/wheels.yml`
2. PR CI runs:
   - `wheels.yml` → Linux build + test-wheel (should stay green, same flow as before)
   - `wheels-macos.yml` → macOS build + test-wheel (new — this is the test)
3. Inspect the macOS build artifact (`python-3.12-macos-arm64`):
   - Wheel size < 250 MB.
   - `.dylibs/` contains the 9 plugin factories.
   - `data/task_composer_config/*.yaml` contains `@PLUGIN_PATH@`, not `/usr/local/lib`.
4. Inspect macOS `test-wheel` logs: smoke test passes, full pytest green.
5. After merge: `workflow_dispatch` on each workflow on `main` re-confirms green on non-PR context.
6. Cut tag (`0.34.1.1`). Both workflows' `publish` jobs upload to PyPI. Verify both wheel sets appear on `https://pypi.org/project/tesseract-robotics-nanobind/0.34.1.1/`.
7. Clean-env install on macOS arm64: `pip install tesseract-robotics-nanobind==0.34.1.1`, run `tesseract_nanobind_selftest`.

## Risks

| Risk | Severity | Mitigation |
|---|---|---|
| `build_wheel.sh` refactor breaks local macOS dev workflow | Low | The `--dev` path is untouched. The "portable" path now runs in the active env (which is what local users were already in — they don't `conda create --clone` manually, they rely on the script doing it). Test locally in `pixi shell` before merge. |
| CI macOS build still fails for a reason the local script doesn't hit | Low-Medium | Possible pixi-vs-conda env differences in lib search, `setup-pixi` activation specifics. `pixi run build-cpp` and `pixi run build-wheel` using the *active* env guarantees we're running what pixi has set up. If it fails, the local script should surface the same failure (since you'd run the identical pixi tasks locally). Much easier to debug than inline YAML. |
| macOS publish runs on macos-14 (more expensive runner than Linux) | Very low | Publish step is fast (artifact download + `twine upload`, seconds). Cost of one macOS minute per release is negligible. |
| Two publish jobs per tag might race or one could succeed + the other fail | Low | PyPI Trusted Publishing is idempotent-friendly — incremental uploads to the same version are allowed as long as filenames differ. Linux wheels and macOS wheels have distinct filenames (`manylinux` vs `macosx`) so no collision. If one publish fails, re-run it independently. |

## Rollback

Two-commit revert restores Linux-only publishing:
1. Delete `wheels-macos.yml`.
2. `git revert` the `wheels.yml` cleanup and `build_wheel.sh` refactor commits.
3. Yank the broken macOS release via PyPI web UI if it reached PyPI.

No database migrations, no external system state.

## Lessons from the first attempt (for posterity)

The initial approach made three surgical YAML edits to enable macOS in `wheels.yml`. In chronological order, CI surfaced:

1. `sed -i '...' file` in the "patch upstream missing includes" step — portable only on GNU sed; BSD sed (macOS) interprets the first arg after `-i` as the backup extension. Fix was platform-guard the step (only Linux needs the patch anyway).
2. `DYLD_LIBRARY_PATH=$CONDA_PREFIX/lib:...` — shadowed Apple system `git`'s library resolution, causing CMake's `FindGit` to return empty. Fix was swap to `DYLD_FALLBACK_LIBRARY_PATH`.
3. `osqp_eigen` `try_compile` fell through to the pre-release `csc_set_data` code path — a C++ compile error. Root cause never fully isolated in the first attempt; discovering this was the trigger to pivot to the split approach, because the local `scripts/build_tesseract_cpp.sh` successfully compiles `osqp_eigen` on the same machine.

Each issue was a distinct divergence between CI inline shell and the local scripts. The split approach eliminates the class of bug, not one instance.
