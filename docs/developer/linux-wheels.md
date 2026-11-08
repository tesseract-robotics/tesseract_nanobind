# Linux Wheel Gotchas

## libstdc++ is never bundled — and must never be

The single most important rule for the Linux wheels: **the gcc runtime
(`libstdc++.so.6`, `libgcc_s.so.1`) must come from the host or the active
conda/pixi env, never from the wheel.** auditwheel never bundles it (manylinux
policy), and the `test-wheel` job in `wheels.yml` asserts the installed wheel
ships no copy.

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

The wheels are built on Ubuntu 24.04 with its gcc 13 and tagged
`manylinux_2_39`, so pip installs them only where glibc is 2.39 or newer
(Ubuntu 24.04+, Debian 13+), which also carry a gcc-13 era libstdc++.

## Plugin bundling

Plugin factory libraries are dlopen'd by boost_plugin_loader, so the linker
would never see them. The collision, kinematics and task composer modules link
them anyway and register them through their anchors (see
[binaries](binaries.md#plugin-anchors)), so auditwheel bundles them with the
rest of the closure. Objects created by those plugins carry vtables that live
in the plugin `.so`s; binding-level `nb::keep_alive<0, 1>` on every
Environment getter returning plugin-backed objects keeps the Environment (and
so the loader and the mapped plugins) alive as long as any such object exists.
