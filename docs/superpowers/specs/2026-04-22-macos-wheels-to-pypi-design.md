# macOS wheels to PyPI

**Date:** 2026-04-22
**Status:** Approved for implementation
**Owner:** Jelle Feringa

## Context

`wheels.yml` has full macOS build/delocate/plugin-bundle/YAML-patch steps (lines 93–213), but the matrix entry is commented out (lines 34–37) with `# macOS disabled: wheels exceed PyPI size limit`. The `publish` job's download filter `pattern: python-*-linux-*` has a TODO to revert to `python-*` once the size limit is raised.

PyPI has now granted a 250 MB per-file limit for this project, so the size blocker is gone. Linux wheels currently ship at 99.8–99.9 MB; a delocated macOS arm64 wheel is expected to land around 100–180 MB, well under the new ceiling.

## Goal

Publish macOS arm64 wheels to PyPI on tagged releases, alongside the existing Linux x64 wheels, with the same gating (build → test-wheel → publish on tag).

## Scope

**In scope:**
- Re-enable macOS arm64 in the `build` matrix for Python 3.9, 3.10, 3.11, 3.12 (matches Linux).
- Add macOS arm64 to the `test-wheel` matrix for Python 3.9, 3.12 (matches Linux test coverage).
- Flip `publish` download pattern from `python-*-linux-*` to `python-*`.
- Remove the now-stale `TODO: change back to python-* after PyPI size limit increase` comment.

**Out of scope:**
- macOS x86_64 / Intel runners. Shrinking user base, `macos-13` Intel runners are on GitHub's deprecation path. Intel users can build from source.
- `universal2` fat wheels. Roughly doubles wheel size, tight against 250 MB ceiling, and defeats the point of trimming x86_64.
- Wheel size reduction (strip symbols, LTO, plugin trimming). Separate concern; current size fits.
- Migration to `macos-15` runners. Worth a separate plan; `macos-14` is current and supported.
- Test.pypi.org dry-run plumbing. `workflow_dispatch` already lets us re-run the build without tagging, which is sufficient validation.

## Design

### File changed: `.github/workflows/wheels.yml`

#### 1. `build` matrix

Uncomment the macOS entry:

```yaml
matrix:
  python: ['3.9', '3.10', '3.11', '3.12']
  config:
    - os: linux
      arch: x64
      runner: ubuntu-22.04
    - os: macos
      arch: arm64
      runner: macos-14
```

No other changes in the `build` job — the `if: matrix.config.os == 'macos'` guards on existing steps (brew, macOS colcon, pip wheel, delocate, plugin-bundle+YAML-patch) already route correctly. Colcon cache key already includes `${{ matrix.config.os }}-${{ matrix.config.arch }}`, so macOS gets its own cache automatically.

#### 2. `test-wheel` matrix

Add macOS arm64:

```yaml
matrix:
  python: ['3.9', '3.12']
  config:
    - os: linux
      arch: x64
      runner: ubuntu-24.04
    - os: macos
      arch: arm64
      runner: macos-14
```

The test steps (install wheel in virgin venv → import check → `pytest tests -n auto` with ignores for `tesseract_time_parameterization`, `examples`, `tesseract_kinematics`, `benchmarks`) are shell-portable and run on both platforms without changes. If macOS surfaces an additional pytest subdirectory that can't run in the virgin venv, add a platform-conditional `--ignore=` as the minimal fix.

#### 3. `publish` download filter

```yaml
- uses: actions/download-artifact@v4
  with:
    pattern: python-*
    merge-multiple: true
    path: dist/
```

Removes the `python-*-linux-*` restriction and the adjacent TODO comment.

### Unchanged

- Trusted Publishing: `environment: pypi`, OIDC `id-token: write`, `pypa/gh-action-pypi-publish@release/v1`. Platform-agnostic.
- Tag trigger patterns (`[0-9]+.[0-9]+.[0-9]+` and `[0-9]+.[0-9]+.[0-9]+.[0-9]+`).
- `workflow_dispatch` entry point.
- `paths-ignore` rules.
- `scripts/build_wheel.sh` (local macOS build script — same pipeline, independent of CI change).

## Validation plan

1. Open PR with the three changes above.
2. CI on the PR runs `build` for both platforms and `test-wheel` for both. The `publish` job is gated by `if: startsWith(github.ref, 'refs/tags/')` — it will not fire on the PR.
3. Inspect the macOS build artifact (`python-3.12-macos-arm64`): verify `.whl` size < 250 MB, extract and confirm `.dylibs/` contains the 9 plugin factories, confirm YAMLs in `data/task_composer_config/` contain `@PLUGIN_PATH@`.
4. Inspect the macOS test-wheel logs: smoke-test passes, full pytest green.
5. After merge, a `workflow_dispatch` run on `main` re-confirms green on a non-PR context.
6. Cut a patch tag (`0.34.1.1` or similar). Confirm `publish` job uploads both Linux and macOS wheels to PyPI. Install on a clean macOS arm64 machine via `pip install tesseract-robotics-nanobind`, run `tesseract_nanobind_selftest`.

## Risks

| Risk | Severity | Mitigation |
|---|---|---|
| macOS build steps have bitrotted during the disable period | Medium | PR CI is the test; fixes likely localised to `delocate-path -L` or `install_name_tool` rpath invocation. The local `scripts/build_wheel.sh` exercises the same flow successfully, reducing bitrot risk. |
| Wheel exceeds 250 MB after bundling | Low | Current Linux wheel is 99.9 MB with the same plugin set; macOS adds delocate-bundled dylibs but should not 2.5× the size. If it does, fallback is strip debug symbols in the delocate step. |
| macOS plugin dlopen fails in virgin venv (DYLD path, rpath) | Medium | `test-wheel` gate catches this before publish. Historical fixes are documented in `.claude/CLAUDE.md` (`@PLUGIN_PATH@` resolution, RTTI visibility). |
| Trusted Publishing not configured for macOS artifact uploads | Very low | Trusted Publishing is platform-agnostic — the uploader doesn't care what OS produced the `.whl`. |

## Rollback

Single commit revert restores current behaviour (linux-only publish). If a broken macOS wheel reaches PyPI:

1. Yank the broken release via the PyPI web UI (Manage → Release → Yank). Yanking hides the version from `pip install` resolution without deleting it.
2. Re-comment the macOS matrix entry.
3. Restore `pattern: python-*-linux-*`.
4. Cut a new patch version.

No database migrations, no external system state. Pure CI change.
