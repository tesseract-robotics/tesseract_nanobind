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
