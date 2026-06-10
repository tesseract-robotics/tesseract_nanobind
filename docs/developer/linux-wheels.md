# Linux Wheel Gotchas

## libstdc++ is never bundled — and must never be

The single most important rule for the Linux wheels: **the gcc runtime
(`libstdc++.so.6`, `libgcc_s.so.1`) must come from the host or the active
conda/pixi env, never from the wheel.** `scripts/build_linux_wheel.sh` skips it
explicitly in the ldd-closure bundling loop, and the test-wheel CI has a
regression guard asserting the wheel ships no copy and exactly one copy is
mapped at runtime.

### Why (gh-119 forensics, 2026-06-10)

Wheels ≤ 0.35.0.4 bundled the conda env's gcc-14 `libstdc++.so.6` (the
ldd-closure copied everything outside `/usr/lib`, and the conda prefix carries
its own gcc runtime). Our libraries resolved it via `$ORIGIN` rpath while
numpy/scipy — manylinux wheels that never vendor libstdc++ — loaded the system
copy. Result: **two libstdc++ copies in every numpy-importing process.**

That dual-copy state corrupts the heap: gcc emits template statics (the
`std::locale` facet registries that `std::regex` depends on) as
`STB_GNU_UNIQUE` symbols, which the dynamic loader unifies *across* the two
copies — so code inlined against one copy operates on the other copy's
internal state. Concretely, `Environment.init → PropertyTree::
rebuildAutoValidators → isSequenceType → std::regex` compile ends up calling
`free()` **on a stack address** (valgrind shows the wrong-vtable-slot callee
`codecvt::do_unshift` inside `_BracketMatcher::_M_ready()`).

Key experimental facts (seven rounds of no-build CI experiments against
released PyPI wheels; results emitted as check-run annotations):

- Every released Linux wheel since at least 0.35.0.3 crashes, both arches —
  the bug predates the conda-deps migration and the GH #35 preload.
- Import order is irrelevant (numpy-first and tesseract-first both crash).
- Any **single** active copy is clean: `LD_PRELOAD`ing *either* the system or
  the bundled copy fixes it, as does not importing numpy, as does deleting the
  bundled copy so the RPATH falls through to the system one.
- glibc's `double free or corruption (top)` abort is *detection*, not
  occurrence: `free(stack-address)` is UB that glibc only catches when the
  fake chunk trips its heuristics. In allocation-rich processes (the pytest
  suite) the same free corrupts **silently** — the suite was never protected,
  only undetected.
- With the bundled copy removed, `env.init` + kinematics run clean and the
  process then hits the *second* latent bug: the gh-72 teardown crash
  (Environment destroyed before plugin-backed objects → plugin `.so`s
  dlclosed → destructors virtual-call unmapped pages), fixed by
  `nb::keep_alive<0, 1>` on the Environment getters.

### The GH #35 preload is gone

`__init__.py` used to `RTLD_GLOBAL`-preload the bundled copy so it would win
the SONAME race against numpy on old distros. Its premise — "two copies
coexist safely, the newer one wins" — is exactly what the forensics falsified,
and it could never help numpy-first processes anyway (it ran too late). Do not
reintroduce it.

### Supported libstdc++ floor

The conda-forge-built tesseract libs need a gcc-13+ era libstdc++
(`GLIBCXX_3.4.32`-ish): ubuntu 24.04+, debian 13+, fedora 38+, or any
conda/pixi env. On older distros the wheel imports fail loudly with a
missing-`GLIBCXX`/`CXXABI` error — by design; that is strictly better than the
silent corruption it replaces. The `manylinux_2_35` platform tag encodes only
the glibc floor, not libstdc++, so pip will install on e.g. ubuntu 22.04 and
fail at import.

**Wheel inside a conda/pixi env on an old distro:** the env carries a suitable
libstdc++ at `$CONDA_PREFIX/lib`, but env activation does not put it on the
loader path for foreign (pip-installed) wheels — export
`LD_LIBRARY_PATH="$CONDA_PREFIX/lib"` (the ABI canary in `wheels-linux.yml`
does exactly this on its 22.04 build host).

## Plugin bundling

Plugin factory libraries are dlopen'd by boost_plugin_loader, so the linker
never sees them — `build_linux_wheel.sh` copies them into the package root
explicitly and they seed the ldd closure. Objects created by those plugins
carry vtables that live in the plugin `.so`s; binding-level
`nb::keep_alive<0, 1>` on every Environment getter returning plugin-backed
objects keeps the Environment (and so the loader and the mapped plugins) alive
as long as any such object exists.
