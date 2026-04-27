# Windows Wheels: Packaging Gotchas

Notes captured during the [GH-40](https://github.com/tesseract-robotics/tesseract_nanobind/issues/40)
Windows-wheels-to-PyPI work. Recorded here so future maintainers don't re-walk
the same multi-day diagnosis paths.

The Windows build differs from Linux/macOS in three ways that interact badly
with anything resembling a plugin architecture (and tesseract is a plugin
architecture top-to-bottom). Each one fails silently at packaging time and
surfaces only at runtime, with an error message that points away from the
real cause.

## Plugin DLL transitive dependencies

!!! warning "delvewheel needs `--analyze-existing` to patch plugin DLLs"
    `delvewheel repair` walks the import tables of `.pyd` extension modules
    by default — nothing else. Plugin factory DLLs (`tesseract_*_factories.dll`,
    `tesseract_kinematics_opw_factory.dll`, …) that the build pipeline copies
    into the wheel's package dir **before** the repair step *do* get included
    in the wheel, but their import tables stay unpatched and any deps they
    pull in beyond what the `.pyd` files already need go **un-bundled**.
    The fix is `delvewheel repair --analyze-existing …` — that flag tells
    delvewheel to scan and rewrite every DLL in the wheel, not just `.pyd`.

    Without it, the wheel builds, installs, smoke-imports, and then fails at
    first plugin instantiation. The misleading part: every test that touches
    a plugin (kinematics, collision, task-composer pipelines) fails with
    "Failed to load symbol", which sends the diagnosis chasing
    `__declspec(dllexport)` and `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS`. Those
    aren't the bug.

## Runtime DLL search path

!!! warning "`os.add_dll_directory` does not cover plain `LoadLibrary` from C++"
    `os.add_dll_directory()` (which delvewheel auto-injects into `__init__.py`
    for the `<project-name>.libs/` directory) only affects `LoadLibrary` calls
    that pass `LOAD_LIBRARY_SEARCH_USER_DIRS`. CPython's `.pyd` loader passes
    that flag, so `add_dll_directory` "works" for module imports — which
    builds false confidence that it covers the whole process.

    Once a `.pyd` is loaded and its C++ code calls `LoadLibrary("plugin.dll")`
    itself — which is exactly what `boost::dll::shared_library` does, and
    therefore what `boost_plugin_loader` does — that C++ call uses the
    **legacy** Windows DLL search order, which respects `PATH` but ignores
    `add_dll_directory`. The fix is to prepend the package dir and
    `<project-name>.libs/` to `PATH` in `__init__.py` before any submodule
    import. See `src/tesseract_robotics/__init__.py` for the canonical block.

    Note the directory naming asymmetry that bites here: delvewheel uses the
    project's *metadata* name (`tesseract-robotics-nanobind`) for the libs
    dir, not the Python *package* name (`tesseract_robotics`). It is named
    `tesseract_robotics_nanobind.libs/` on disk.

## Diagnosing plugin-load failures

!!! danger "`boost_plugin_loader`'s 'Failed to load symbol' is two errors in a trench coat"
    `boost::dll`-backed plugin loaders log
    `Failed to load symbol 'X' in library: Y` in two distinct cases that look
    identical from Python:

    1. `LoadLibrary` succeeded but `GetProcAddress` returned `NULL` — the
       symbol genuinely isn't in the DLL's export table.
    2. `LoadLibrary` returned `NULL` because a transitive dependency couldn't
       be resolved — the loader catches the boost exception and re-reports
       it as "Failed to load symbol".

    On Windows, case 2 is far more common than case 1 and points the
    investigation in a completely different direction (DLL search path,
    bundled deps, `delvewheel --analyze-existing`) than case 1 would
    (`__declspec(dllexport)`, symbol visibility flags). Always confirm with
    `dumpbin /EXPORTS plugin.dll` whether the symbol is actually present
    *before* assuming an export issue. The CI workflow's
    `verify plugin factory exports` step runs this check automatically and
    fails fast if the export table is the problem.

## Cereal polymorphic registration

!!! warning "Cereal's polymorphic registry is per-DLL on Windows MSVC — consumers must re-register types in their own TU"
    cereal stores its polymorphic-type registry in class-template statics
    (`StaticObject<InputBindingMap<Archive>>`,
    `StaticObject<OutputBindingMap<Archive>>`, ...). Linux and macOS linkers
    dedupe these template instantiations across shared libraries, giving the
    process one shared registry. **MSVC does not** — each DLL gets its own
    copy of every template static, with no automatic dllexport/dllimport
    plumbing in cereal headers to share them.

    Consequence: an upstream library (e.g. `tesseract_command_language.dll`)
    that calls `CEREAL_REGISTER_TYPE` populates **its own** registry only.
    A downstream `.pyd` that does serialization reads from **its own** empty
    registry and fails at runtime with
    `RuntimeError: Trying to save an unregistered polymorphic type`. The
    type really is registered upstream — just not in the registry the
    consumer actually consults.

    `CEREAL_FORCE_DYNAMIC_INIT` is **not enough** on its own. It anchors
    upstream's registration TU to keep the linker from stripping it, but
    doesn't bridge the registry gap on the consumer side.

    The actual Windows fix is to **mirror upstream's registration block in
    the consumer TU**. For tesseract_command_language types
    (`MoveInstructionPoly`, `CompositeInstruction`, etc.) that means copying
    every `CEREAL_REGISTER_TYPE` and `CEREAL_REGISTER_POLYMORPHIC_RELATION`
    line from upstream
    `tesseract_command_language/src/cereal_serialization.cpp` into our
    binding. Re-registration is a no-op on Linux/macOS (cereal tolerates
    duplicate registrations of the same `{type, archive}` pair) and
    populates the consumer's per-DLL registry on Windows. See
    `src/tesseract_serialization/tesseract_serialization_bindings.cpp` for
    the canonical placement; keep `CEREAL_FORCE_DYNAMIC_INIT(LibName)` as
    belt-and-suspenders on the producer side.
