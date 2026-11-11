"""Contract tests of the module -> translation units table (scripts/audit_ownership.py).

An extension module is its primary `<module>.cpp` plus the parts in `MODULE_PARTS`; every
source in src/bindings/ is a primary, a part or a helper. No libclang: these run in the
default env too.
"""

import re
from pathlib import Path

import pytest

from ._free import load_free

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

own = load_free("audit_ownership")

PRIMARY = "#include <nanobind/nanobind.h>\nNB_MODULE(_{name}, m) {{ bind_part(m); }}\n"
PART = '#include "{name}_bindings.h"\nvoid bind_part(nb::module_& m) {{}}\n'
HELPER = "void pin() {}\n"


def _write(directory: Path, **sources: str) -> Path:
    for stem, text in sources.items():
        (directory / f"{stem}.cpp").write_text(text)
    return directory


@pytest.fixture
def layout(tmp_path):
    return _write(
        tmp_path,
        alpha=PRIMARY.format(name="alpha"),
        alpha_extra=PART.format(name="alpha"),
        alpha_more=PART.format(name="alpha"),
        beta=PRIMARY.format(name="beta"),
        beta_pinning=HELPER,
    )


PARTS = {"alpha": ("alpha_extra", "alpha_more")}
HELPERS = frozenset({"beta_pinning"})


def test_a_module_is_its_primary_then_its_parts_in_order(layout):
    sources = own.module_sources(layout, PARTS, HELPERS)
    assert sources == {
        "alpha": (layout / "alpha.cpp", layout / "alpha_extra.cpp", layout / "alpha_more.cpp"),
        "beta": (layout / "beta.cpp",),
    }


def test_a_stray_source_raises(layout):
    _write(layout, alpha_forgotten=PART.format(name="alpha"))
    with pytest.raises(own.StrayBindingSourceError, match="alpha_forgotten"):
        own.module_sources(layout, PARTS, HELPERS)


def test_a_declared_part_without_a_file_raises(layout):
    with pytest.raises(own.MissingBindingSourceError, match="alpha_gone"):
        own.module_sources(layout, {"alpha": ("alpha_extra", "alpha_more", "alpha_gone")}, HELPERS)


def test_a_part_that_holds_nb_module_raises(layout):
    with pytest.raises(own.DeclaredPrimaryError, match="beta"):
        own.module_sources(
            layout, {**PARTS, "alpha": ("alpha_extra", "alpha_more", "beta")}, HELPERS
        )


def test_a_part_declared_twice_raises(layout):
    with pytest.raises(own.DuplicateSourceError, match="alpha_extra"):
        own.module_sources(layout, {**PARTS, "beta": ("alpha_extra",)}, HELPERS)


def test_a_part_of_no_module_raises(layout):
    with pytest.raises(own.UnknownPartOwnerError, match="gamma"):
        own.module_sources(layout, {**PARTS, "gamma": ()}, HELPERS)


def test_a_helper_that_binds_raises(layout):
    _write(layout, beta_pinning=PART.format(name="beta"))
    with pytest.raises(own.BindingHelperError, match="beta_pinning"):
        own.module_sources(layout, PARTS, HELPERS)


def test_a_primary_named_after_another_module_raises(layout):
    _write(layout, beta=PRIMARY.format(name="alpha"))
    with pytest.raises(own.ModuleEntryMismatchError, match="beta"):
        own.module_sources(layout, PARTS, HELPERS)


def test_a_commented_nb_module_is_no_entry(layout):
    _write(
        layout, alpha_extra="// NB_MODULE(_alpha, m) is in alpha.cpp\n" + PART.format(name="alpha")
    )
    assert own.module_sources(layout, PARTS, HELPERS)["alpha"][1] == layout / "alpha_extra.cpp"


def test_binding_modules_exclude_parts_and_helpers():
    modules = own.binding_modules()
    parts = {s for stems in own.MODULE_PARTS.values() for s in stems}
    assert "common" in modules and "trajopt_ifopt" in modules
    assert not (parts | own.NON_MODULE_SOURCES) & set(modules)


def test_binding_sources_list_the_primary_first():
    assert own.binding_sources("common") == (
        own.BINDINGS_DIR / "common.cpp",
        own.BINDINGS_DIR / "common_eigen.cpp",
        own.BINDINGS_DIR / "common_logging.cpp",
    )
    assert own.primary_source("collision") == own.BINDINGS_DIR / "collision.cpp"
    assert own.binding_sources("collision") == (own.BINDINGS_DIR / "collision.cpp",)


def test_binding_sources_refuse_a_part_name():
    with pytest.raises(own.UnknownModuleError, match="common_eigen"):
        own.binding_sources("common_eigen")


# `target_sources(_common PRIVATE src/bindings/common_eigen.cpp ...)`: the sources CMake links
# into an extension target beside its primary.
TARGET_SOURCES = re.compile(r"target_sources\(\s*(\w+)\s+PRIVATE\s+([^)]*)\)")
BINDING_SOURCE = re.compile(r"src/bindings/(\w+)\.cpp")


def test_parts_and_helpers_are_the_sources_cmake_links_into_their_module():
    """`MODULE_PARTS` is the build's own layout: each part is linked into its module's target,
    and every source linked beside a primary is a part of that module or a helper."""
    cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    linked: dict[str, set[str]] = {}
    for target, sources in TARGET_SOURCES.findall(cmake):
        linked.setdefault(target.removeprefix("_"), set()).update(BINDING_SOURCE.findall(sources))
    declared = {m: set(ps) for m, ps in own.MODULE_PARTS.items()}
    assert {
        m: s - own.NON_MODULE_SOURCES for m, s in linked.items() if s - own.NON_MODULE_SOURCES
    } == declared
    assert own.NON_MODULE_SOURCES <= set().union(*linked.values())
