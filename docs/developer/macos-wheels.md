# macOS wheels: arm64 CI

Notes captured during the 0.35.0.x release-pipeline hardening
([PR #98](https://github.com/tesseract-robotics/tesseract_nanobind/pull/98)),
recorded so future maintainers don't re-diagnose them.

The macOS builds run on `macos-15` (arm64) and `macos-15-intel` (x86_64), and
have sharp edges
the Linux build doesn't. The dangerous part: each one fragments a *release*
rather than failing a *build*, so it surfaces as a gappy wheel set on PyPI
**after** the tag is cut — not as CI red you'd catch in review.

## upload-artifact times out ~1/3 of the time on arm64 runners

!!! danger "`actions/upload-artifact` `CreateArtifact: Request timeout` is a known, unfixed arm64-runner bug"
    Up to 0.35.0.x, on `macos-14` (and any arm64 macOS runner), `actions/upload-artifact@v4`'s `archive wheel` step failed
    roughly **one upload in three** with
    `Failed to CreateArtifact: Failed to make request after 5 attempts: Request timeout`.
    The wheel built fine — the failure is purely the upload handshake to GitHub's
    artifact service. It is a long-standing open defect
    ([actions/upload-artifact#569](https://github.com/actions/upload-artifact/issues/569),
    [#527](https://github.com/actions/upload-artifact/issues/527)); the Actions
    team acknowledged it in Dec 2024 and it is still open. The action's own 5
    internal retries exhaust within the same bad network window, so they don't
    help — a *fresh outer re-invocation* usually succeeds.

    Why it fragmented releases: in the 0.35 per-platform macOS workflow `publish` had
    `needs: [build]`, so **one** failed upload marked the whole `build` matrix failed
    and the entire macOS publish was skipped — zero macOS wheels for that release. At
    ~1/3 per job across 4 Python builds, P(all 4 upload) = (2/3)⁴ ≈ **20%**, so ~80% of
    macOS releases would otherwise have needed a manual job re-run. In the current
    single `wheels.yml`, `publish` needs every build job, so a failed macOS upload
    blocks the release on every platform.

    Fix (`wheels.yml`): retry the `archive wheels` step up to 3× with
    `overwrite: true` on the retries (clears any half-created artifact). The
    final attempt has **no** `continue-on-error`, so a genuine outage still
    fails loud — never a silently-missing wheel.

## The runner label is an architecture, not just an OS version

!!! warning "Lowering `runs-on: macos-15` silently flips the wheel from arm64 to x86_64"
    GitHub's macOS runner labels split by **architecture**, not just OS age:

    | label | arch | notes |
    |---|---|---|
    | `macos-12` | Intel x86_64 | removed Dec 2024 |
    | `macos-13` | Intel x86_64 | deprecation path |
    | `macos-14` | **arm64** | what we built on up to 0.35.0.x |
    | `macos-15` | **arm64** | what we build arm64 on |
    | `macos-15-intel` | Intel x86_64 | what we build x86_64 on (schedule, tags, manual runs) |

    We ship `macosx_15_0_arm64` (up to 0.35.0.x, `macosx_14_0_arm64`). "Lowering the macOS version" to `macos-13`/`-12`
    to dodge a runner problem (e.g. the upload flakiness above) does **not** keep
    the same wheel — it switches the build to **x86_64**, orphaning every
    Apple-Silicon user. And it wouldn't even help: the upload bug reproduces on
    `macos-13-xl-arm64` too, so it is arm64-runner-inherent, not OS-version
    specific.

    The legitimate "lower the macOS version" lever is
    **`MACOSX_DEPLOYMENT_TARGET`** (a build-env var, independent of the runner) —
    it sets the wheel's *minimum* macOS (`macosx_13_0_arm64`, …) for broader
    install compatibility without touching the runner or the architecture. Use
    that, not the runner label. It only lowers the tag as far as every bundled
    library allows: the repair takes the highest minimum OS of the native closure,
    and the current Homebrew `libomp` holds it at 15.0 (see below).

## STABLE_ABI: the wheel under test is not the Python under test

!!! note "3.13–3.15 are verified by installing the `cp312-abi3` wheel, not by building a 3.13–3.15 wheel"
    nanobind's `STABLE_ABI` build emits a single `cp312-abi3` wheel that pip
    installs on CPython 3.12, 3.13, 3.14+ — on every platform. So 3.13–3.15
    *support* is real on macOS and Windows even though no 3.13–3.15 wheel is
    built. To **verify** it in CI, each build job installs the wheel it just built
    into a fresh venv per interpreter through the `test_python_ver` action
    (`.github/actions/test_python_ver`), once per Python:

    ```yaml
    - name: test Python 3.13
      uses: ./tesseract_nanobind/.github/actions/test_python_ver
      with:
        python_version: "3.13"
        wheel_dir: ${{ github.workspace }}/tesseract_nanobind/test-wheels
    ```

    The build jobs run it for 3.13, 3.14 and 3.15 on pushes, tags and scheduled
    runs, and `test-artifacts` runs it for 3.12 on a fresh runner, the only suite
    run a pull request gets. Every leg
    blocks. Before PR #98 only Linux did this; macOS and
    Windows tested 3.9 + 3.12 only, leaving the cross-platform abi3 claim
    *asserted* but not *verified*. Now all three platforms verify it.

## Sharing OpenMP with conda and pixi consumers

`scripts/repair_macos_wheel.py` bundles all dependencies into `.dylibs/` and
leaves OpenMP to the dynamic linker: OpenMP load commands become
`@rpath/libomp.dylib`, with the installed Python environment's `lib/` first,
then the bundled `.dylibs/` directory. Changed Mach-O images are re-signed, and
the script rewrites the wheel's RECORD metadata.

`scripts/check_wheel_openmp.py` checks both import orders and the actual dyld
image paths. Plain consumers must load exactly one bundled runtime; the
`test-wheel-openmp` CI job must load exactly the environment runtime and pass
Descartes planning tests. Both this job and fresh-runner artifact tests gate
publishing. In the `--lib-dir` mode the wheel workflow uses, the repair leaves
the plugin YAML unchanged.

The current Homebrew runtime raises both architectures' minimum macOS to
15.0. A lower deployment target on the bindings cannot lower this requirement.

!!! note "Why the bundled-only wheel fails under conda (#136)"
    Wheels up to 0.35.0.8 were repaired by `delocate-wheel`, which bundled `libomp` and
    pointed everything at that copy.

    Install that wheel into a conda or pixi environment and the process gets
    two. MEASURED on macOS 14 arm64, binding 0.35.0.7, in an environment whose
    `numpy` comes from conda-forge:

    | Image | Provenance | Loaded by |
    |---|---|---|
    | `<env>/lib/libomp.dylib` | conda-forge `llvm-openmp` | `numpy`, at import — and it *initialises* the runtime |
    | `<env>/lib/python3.13/site-packages/tesseract_robotics/.dylibs/libomp.dylib` | the delocated wheel | `import tesseract_robotics`, mapped but idle |

    Different builds, different SHA-256. They coexist quietly, because the
    wheel's copy is only *mapped* — nothing initialises it until some plugin
    opens a parallel region. **Descartes' ladder solver is the first thing that
    does**, which is why an entire motion test suite can pass while this is
    sitting there:

    - no flag: `OMP: Error #15: Initializing libomp.dylib, but found
      libomp.dylib already initialized`, and the process aborts before a single
      waypoint is planned;
    - `KMP_DUPLICATE_LIB_OK=TRUE`: survives `num_threads=1`, and **segfaults at
      `num_threads=2` and `3`** (exit 139, 84-waypoint contour on
      `abb_irb2400`). The flag lets the second runtime load; it does not make
      running on both safe, exactly as its own warning says.

    The control confirms the cause. Point the wheel's `.dylibs/libomp.dylib`
    at the environment's copy, so that `dyld` loads one runtime, and the same
    solve runs at 1, 2, 4 and 8 threads **without** the flag.

    The fix doesn't, on its own, make Descartes faster: with vertex
    collision on, a profile puts ~97 % of a solve in serial setup — one
    contact-manager clone per waypoint, see the Descartes section of the
    [planning guide](../user-guide/planning.md) — so the threads it restores
    mostly wait.

## A Python exception crossing two extension modules lost its type

!!! note "Every module that defines a trampoline calls `register_python_error_translator()`"
    A Python override that raises reaches C++ as `nanobind::python_error`. The
    extensions are built `NB_STATIC`, so each `.so` holds its own strong
    `typeinfo for nanobind::python_error` (`nm -m` lists one per module), and
    macOS libc++ compares default-visibility typeinfo by pointer. When the
    trampoline is in one module and the bound function that called it in
    another (`ConstraintSet` in `_trajopt_ifopt`, called by
    `TrajOptQPProblem` / `TrustRegionSQPSolver` in `_trajopt_sqp`), the
    caller's `catch (python_error &)` misses, and nanobind's default
    translator turns the error into `RuntimeError(what())`: an `IndexError`
    arrived as `RuntimeError` with the traceback as its message
    ([#156](https://github.com/tesseract-robotics/tesseract_nanobind/issues/156)).
    A call that stays inside one module kept its type.

    `register_python_error_translator()` (`src/tesseract_nb.h`) registers a
    translator that runs the defining module's code, so its catch tests that
    module's typeinfo and restores the Python exception. It must be `static`:
    as an `inline` function its lambda was a weak symbol, dyld coalesced it
    across images, and every module registered the first-loaded module's
    copy, which fixed nothing. Linux (libstdc++) and Windows (MSVC) compare
    typeinfo by name, so they should not lose the type (not measured; the
    regression test `tests/trajopt_sqp/test_python_set_exception_type.py`
    runs on all three). The alternative, one shared `libnanobind` instead of
    `NB_STATIC`, would give the process a single typeinfo, at the cost of
    another shared library in all three wheel layouts.
