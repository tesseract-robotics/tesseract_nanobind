# macOS Wheels: arm64 CI Gotchas

Notes captured during the 0.35.0.x release-pipeline hardening
([PR #98](https://github.com/tesseract-robotics/tesseract_nanobind/pull/98)),
recorded so future maintainers don't re-diagnose them.

The macOS build runs on `macos-14` (Apple Silicon, arm64) and has sharp edges
the Linux build doesn't. The dangerous part: each one fragments a *release*
rather than failing a *build*, so it surfaces as a gappy wheel set on PyPI
**after** the tag is cut — not as CI red you'd catch in review.

## upload-artifact times out ~1/3 of the time on arm64 runners

!!! danger "`actions/upload-artifact@v4` `CreateArtifact: Request timeout` is a known, unfixed arm64-runner bug"
    On `macos-14` (and any arm64 macOS runner) the `archive wheel` step fails
    roughly **one upload in three** with
    `Failed to CreateArtifact: Failed to make request after 5 attempts: Request timeout`.
    The wheel built fine — the failure is purely the upload handshake to GitHub's
    artifact service. It is a long-standing open defect
    ([actions/upload-artifact#569](https://github.com/actions/upload-artifact/issues/569),
    [#527](https://github.com/actions/upload-artifact/issues/527)); the Actions
    team acknowledged it in Dec 2024 and it is still open. The action's own 5
    internal retries exhaust within the same bad network window, so they don't
    help — a *fresh outer re-invocation* usually succeeds.

    Why it fragments releases: `publish` has `needs: [build]`, so **one** failed
    upload marks the whole `build` matrix failed and the entire macOS publish is
    skipped — zero macOS wheels for that release. At ~1/3 per job across 4 Python
    builds, P(all 4 upload) = (2/3)⁴ ≈ **20%**, so ~80% of macOS releases would
    otherwise need a manual job re-run.

    Fix (`wheels-macos.yml`): retry the `archive wheel` step up to 3× with
    `overwrite: true` on the retries (clears any half-created artifact). The
    final attempt has **no** `continue-on-error`, so a genuine outage still
    fails loud — never a silently-missing wheel.

## The runner label is an architecture, not just an OS version

!!! warning "Lowering `runs-on: macos-14` silently flips the wheel from arm64 to x86_64"
    GitHub's macOS runner labels split by **architecture**, not just OS age:

    | label | arch | notes |
    |---|---|---|
    | `macos-12` | Intel x86_64 | removed Dec 2024 |
    | `macos-13` | Intel x86_64 | deprecation path |
    | `macos-14` | **arm64** | what we build on |
    | `macos-15` | arm64 | newer; raises min-OS tag to 15 |

    We ship `macosx_14_0_arm64`. "Lowering the macOS version" to `macos-13`/`-12`
    to dodge a runner problem (e.g. the upload flakiness above) does **not** keep
    the same wheel — it switches the build to **x86_64**, orphaning every
    Apple-Silicon user. And it wouldn't even help: the upload bug reproduces on
    `macos-13-xl-arm64` too, so it is arm64-runner-inherent, not OS-version
    specific.

    The legitimate "lower the macOS version" lever is
    **`MACOSX_DEPLOYMENT_TARGET`** (a build-env var, independent of the runner) —
    it sets the wheel's *minimum* macOS (`macosx_13_0_arm64`, …) for broader
    install compatibility without touching the runner or the architecture. Use
    that, not the runner label.

## colcon cache key over-invalidates on unrelated pyproject edits

!!! warning "The colcon cache key hashes all of `pyproject.toml`, so a one-line edit forces a ~15-min C++ rebuild"
    The C++ dependency cache key is
    `colcon-macos-v2-py${python}-${hashFiles('dependencies.repos', 'pyproject.toml')}`.
    Because it hashes the **entire** `pyproject.toml`, any edit — even one that
    touches no C++ dependency (a `setuptools_scm` setting, a ruff rule, a trove
    classifier) — changes the hash, misses the cache, and rebuilds all of
    tesseract from scratch (~15–18 min per Python). There is no `restore-keys`
    fallback, so the miss is total.

    This degrades *speed*, never *correctness* (a fresh build is still a correct
    build), but it makes otherwise-trivial PRs unexpectedly slow. Worth narrowing
    to a deps-only hash if cold-build time becomes a problem. It is **not** a
    cause of any wheel-fragmentation failure — those are the upload and publish
    issues above; the cache only affects how long the green path takes.

## STABLE_ABI: the wheel under test is not the Python under test

!!! note "3.13/3.14 are verified by installing the `cp312-abi3` wheel, not by building a 3.13/3.14 wheel"
    nanobind's `STABLE_ABI` build emits a single `cp312-abi3` wheel that pip
    installs on CPython 3.12, 3.13, 3.14+ — on every platform. So 3.13/3.14
    *support* is real on macOS and Windows even though no 3.13/3.14 wheel is
    built. To **verify** it in CI, the `test-wheel` matrix decouples the test
    interpreter from the wheel artifact with a `wheel` field:

    ```yaml
    include:
      - { python: '3.13', wheel: '3.12', continue_on_error: false }
      - { python: '3.14', wheel: '3.12', continue_on_error: true }
    ```

    The download step then pulls `python-${{ matrix.wheel }}-macos-arm64` (the
    `cp312-abi3` artifact) and installs it on a 3.13/3.14 interpreter
    (`allow-prereleases: true`). Before PR #98 only Linux did this; macOS and
    Windows tested 3.9 + 3.12 only, leaving the cross-platform abi3 claim
    *asserted* but not *verified*. Now all three platforms verify it.

## delocate bundles the very `libomp` the build was told to share

!!! danger "The wheel carries its own `libomp.dylib`, so a conda consumer runs two OpenMP runtimes — and Descartes dies above one thread"
    The build already takes the right side of this: it links pixi's
    `llvm-openmp` rather than Homebrew's, precisely to avoid duplicate-runtime
    crashes (`AGENTS.md`, `scripts/build_tesseract_cpp.sh`, the wheels
    workflows). Packaging then undoes it. `scripts/build_macos_wheel.sh:81`
    runs

    ```bash
    delocate-wheel -w wheelhouse -v dist/tesseract*.whl
    ```

    with no exclusions, and delocate's whole job is to copy every non-system
    dylib into the wheel — so the shipped wheel contains
    `tesseract_robotics/.dylibs/libomp.dylib`, a *copy* of the one the build
    was pointed at.

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
    solve runs at 1, 2, 4 and 8 threads **without** the flag. So on this
    platform a conda consumer cannot use `num_threads > 1` at all until the
    wheel stops carrying its own runtime.

    Fixing it will not, on its own, make Descartes faster: with vertex
    collision on, a profile puts ~97 % of a solve in serial setup — one
    contact-manager clone per waypoint, see the Descartes section of the
    [planning guide](../user-guide/planning.md) — so the threads it restores
    mostly wait.

!!! warning "CI tests threaded Descartes — in the one environment where it works"
    The suite does solve through Descartes multi-threaded: `TestDescartesPipeline`
    plans via `create_descartes_pipeline_profiles()`, whose `num_threads`
    defaults to the CPU count, and `wheels-macos.yml` runs it on `macos-14`.
    It passes because the `test-wheel` job installs the wheel into a plain
    `python -m venv`, where only the wheel's own `libomp` is loaded (inferred
    from CI passing, not measured). The configuration that fails — the
    delocated wheel inside a conda or pixi environment whose `numpy` has
    already initialised conda's `libomp` — is the one no CI job builds, and it
    is how pixi-managed consumers install the wheel. Closing the gap means
    testing the wheel in that environment, not adding a threaded test.

!!! tip "The fix is a packaging decision, and it has a real trade-off"
    `delocate-wheel --exclude libomp` leaves the wheel resolving the host
    environment's OpenMP — one runtime, threading restored, and consistent with
    the policy the build already follows. The cost is that the wheel then
    *requires* the host to provide `libomp`, which a conda or pixi environment
    always does and a bare `python -m venv` does not. Given the installation
    guide already tells consumers to run inside pixi, excluding is the
    defensible default; bundling is only right for a wheel meant to stand alone,
    and then the duplication has to be documented rather than discovered.

    Until it is decided, a consumer hitting this runs Descartes single-threaded
    under `KMP_DUPLICATE_LIB_OK=TRUE` and treats any wall-clock comparison as
    provisional.
