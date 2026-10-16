"""Contract tests of scripts/audit_ownership.py: module tables and header ownership (#256).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURE_INCLUDE = Path(__file__).resolve().parent / "fixtures" / "include"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


_load("audit_types")
own = _load("audit_ownership")
audit = _load("audit_bindings")

NESTED = {"fixture": ("tesseract/fixture/",), "fixture_sub": ("tesseract/fixture/sub/",)}


def test_longest_prefix_owns_nested_header():
    assert own.header_owner("tesseract/fixture/sub/nested.h", NESTED) == "fixture_sub"
    assert own.header_owner("tesseract/fixture/widget.h", NESTED) == "fixture"


def test_owning_prefix_names_module_and_prefix():
    assert own.owning_prefix("tesseract/fixture/sub/nested.h", NESTED) == (
        "fixture_sub",
        "tesseract/fixture/sub/",
    )
    assert own.owning_prefix("tesseract/elsewhere/x.h", NESTED) is None


def test_parent_module_walk_skips_nested_prefix():
    owned = audit.prefix_headers("fixture", NESTED, (FIXTURE_INCLUDE,))
    assert "tesseract/fixture/sub/nested.h" not in owned.values()
    assert "tesseract/fixture/widget.h" in owned.values()


def test_nested_module_walk_owns_nested_header():
    owned = audit.prefix_headers("fixture_sub", NESTED, (FIXTURE_INCLUDE,))
    assert set(owned.values()) == {"tesseract/fixture/sub/nested.h"}


def test_module_owns_several_prefixes():
    prefixes = {"fixture": ("tesseract/fixture/", "tesseract/typed/")}
    owned = audit.prefix_headers("fixture", prefixes, (FIXTURE_INCLUDE,))
    assert "tesseract/typed/typed.h" in owned.values()


def test_unaudited_pattern_matches_below_the_longest_prefix():
    """`test_suite/*` matches below `tesseract/fixture/sub/` too, not only the parent prefix."""
    assert own.is_unaudited("tesseract/fixture/sub/test_suite/x.h", NESTED["fixture_sub"])
    assert own.is_unaudited(
        "tesseract/fixture/sub/test_suite/x.h", (*NESTED["fixture"], *NESTED["fixture_sub"])
    )


def test_duplicate_prefix_raises(monkeypatch):
    monkeypatch.setitem(own.AUDITED_HEADER_PREFIX, "tesseract_srdf", ("tesseract/common/",))
    with pytest.raises(own.DuplicatePrefixError, match="tesseract/common/"):
        own.owner_prefixes()


def test_core_prefixes_are_one_element_tuples():
    """The 10 core modules own the same headers as before #256: one prefix each."""
    assert all(isinstance(p, tuple) and len(p) == 1 for p in own.AUDITED_HEADER_PREFIX.values())


def test_every_binding_module_is_classified():
    own.check_coverage(own.binding_modules())


def test_unclassified_module_raises():
    with pytest.raises(own.UnclassifiedModuleError, match="tesseract_new"):
        own.check_coverage([*own.binding_modules(), "tesseract_new"])


def test_module_in_both_tables_raises(monkeypatch):
    monkeypatch.setitem(own.UNAUDITED_MODULES, "tesseract_srdf", ("x", ("tesseract/srdf/",)))
    with pytest.raises(own.ConflictingCoverageError, match="tesseract_srdf"):
        own.check_coverage(own.binding_modules())


def test_table_entry_that_is_not_a_binding_raises():
    modules = [m for m in own.binding_modules() if m != "ompl_base"]
    with pytest.raises(own.UnknownModuleError, match="ompl_base"):
        own.check_coverage(modules)


def test_unaudited_module_raises_with_its_reason():
    with pytest.raises(own.UnauditedModuleError, match="#258"):
        audit.resolve_module("ompl_base")


def test_main_checks_coverage_before_parsing(monkeypatch):
    real = audit.binding_modules
    monkeypatch.setattr(audit, "binding_modules", lambda: [*real(), "tesseract_new"])
    monkeypatch.setattr(audit, "parse_tu", lambda *a, **k: pytest.fail("parsed before coverage"))
    with pytest.raises(own.UnclassifiedModuleError):
        audit.main([])


def test_owner_prefixes_come_from_the_tables_only():
    """No `tesseract/<x>/` guess: every binding module's prefixes are stated in a table."""
    prefixes = own.owner_prefixes()
    assert prefixes["tesseract_motion_planners_simple"] == ("tesseract/motion_planners/simple/",)
    assert prefixes["trajopt_ifopt"] == ("trajopt_ifopt/", "trajopt_common/")
    assert prefixes["ompl_base"] == ()
    assert set(prefixes) == set(own.binding_modules())


NESTED_BINDINGS = FIXTURE_INCLUDE.parent / "nested_bindings.cpp"


def test_parent_never_audits_a_directly_included_nested_header():
    """Review Focus 2: `nested_bindings.cpp` #includes `sub/nested.h` itself; the header is
    still `fixture_sub`'s, never an audited header of `fixture`."""
    tu = audit.parse_tu(NESTED_BINDINGS, (FIXTURE_INCLUDE,))
    dirs = (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS)
    parent = audit.audited_headers(tu, "fixture", NESTED, dirs)
    nested = audit.audited_headers(tu, "fixture_sub", NESTED, dirs)
    assert "nested.h" not in {h.name for h in parent}
    assert {h.name for h in nested} == {"nested.h"}
    assert "Nested" not in audit.cpp_api(tu, parent)


def test_prefix_without_trailing_slash_raises(monkeypatch):
    """`tesseract/motion_planners` would also own `tesseract/motion_planners_x/`."""
    monkeypatch.setitem(own.AUDITED_HEADER_PREFIX, "tesseract_srdf", ("tesseract/srdf",))
    with pytest.raises(own.MalformedPrefixError, match="tesseract/srdf"):
        own.owner_prefixes()


def test_real_prefixes_end_with_a_slash():
    own.check_prefixes()


def test_owner_prefixes_refuses_a_module_in_both_tables(monkeypatch):
    """`owner_prefixes` does not rely on `check_coverage` having run first."""
    monkeypatch.setitem(own.UNAUDITED_MODULES, "tesseract_srdf", ("x", ("tesseract/srdf/",)))
    with pytest.raises(own.ConflictingCoverageError, match="tesseract_srdf"):
        own.owner_prefixes()


def test_coverage_counts_the_binding_modules():
    assert own.coverage().total == len(own.binding_modules())


def test_unaudited_rows_ordered_by_group_then_name():
    """The reasons carry the group (`#256 group N`); `ompl_base`'s removal reason sorts last."""
    rows = list(own.coverage().unaudited.items())
    assert rows == sorted(rows, key=lambda mr: (mr[1], mr[0]))
    assert [m for m, _ in rows][:2] == [
        "tesseract_command_language",
        "tesseract_time_parameterization",
    ]
    assert rows[-1][0] == "ompl_base"


CLASH = {
    (FIXTURE_INCLUDE / "tesseract" / "clash" / name).resolve(): f"tesseract/clash/{name}"
    for name in ("clash_a.h", "clash_b.h")
}


def test_headers_that_conflict_only_together_raise():
    """Each clash header parses alone; together they do not. No clean rows: the error stands."""
    for h, s in CLASH.items():
        audit.unincluded_header_gaps(NESTED_BINDINGS, {h: s}, (FIXTURE_INCLUDE,))
    with pytest.raises(audit.HeaderParseError, match="redefinition"):
        audit.unincluded_header_gaps(NESTED_BINDINGS, CLASH, (FIXTURE_INCLUDE,))


def test_parseable_headers_must_still_parse_together_beside_an_unparseable_one():
    """A broken header beside a clashing pair: the pair's conflict is not hidden by the row."""
    broken = FIXTURE_INCLUDE / "tesseract" / "fixture" / "broken_header.h"
    headers = {**CLASH, broken.resolve(): "tesseract/fixture/broken_header.h"}
    with pytest.raises(audit.HeaderParseError, match="redefinition"):
        audit.unincluded_header_gaps(NESTED_BINDINGS, headers, (FIXTURE_INCLUDE,))


def test_audit_tu_refuses_a_prefix_that_disagrees_with_prefixes(monkeypatch):
    """`prefix` and `prefixes[module]` name the same thing; a mismatch is a caller bug."""
    monkeypatch.setattr(audit, "parse_tu", lambda *a, **k: pytest.fail("parsed before the check"))
    fixtures = FIXTURE_INCLUDE.parent
    with pytest.raises(audit.PrefixMismatchError, match="tesseract/other/"):
        audit.audit_tu(
            "fixture",
            fixtures / "fixture_bindings.cpp",
            fixtures / "_fixture.pyi",
            fixtures / "package_init.py",
            ("tesseract/fixture/",),
            (FIXTURE_INCLUDE,),
            prefixes={"fixture": ("tesseract/other/",)},
        )
