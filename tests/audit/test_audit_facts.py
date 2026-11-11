"""Contract tests of the cached TU facts (`scripts/audit_facts.py`): a cached fact equals the
fresh extraction, a hit does not parse, and an edit to any header the TU reads re-parses."""

import shutil
from pathlib import Path

import pytest

from ._load import audit, load

facts_ = load("audit_facts")
gil = load("audit_gil")
FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
IDENTITY = FIXTURES / "identity_bindings.cpp"
GIL_FIXTURE = FIXTURES / "gil_bindings.cpp"


@pytest.fixture
def cache(monkeypatch, tmp_path):
    """A fresh fact cache: the shared one may hold entries from an earlier run."""
    fresh = audit.FactCache(tmp_path / "cache")
    monkeypatch.setattr(facts_, "FACT_CACHE", fresh)
    return fresh


def _no_parse(*a, **k):
    pytest.fail("a hit parsed")


def test_bound_facts_equal_the_fresh_extraction_and_a_hit_does_not_parse(cache):
    tu = audit.parse_tu(IDENTITY, (FIXTURE_INCLUDE,), bodies=True)
    miss = audit.bound_facts(IDENTITY, (FIXTURE_INCLUDE,), audit.parse_tu)
    hit = audit.bound_facts(IDENTITY, (FIXTURE_INCLUDE,), _no_parse)
    assert miss == hit
    assert dict(hit.classes) == audit.bound_classes(tu)
    assert hit.exceptions == frozenset(audit.bound_exceptions(tu))


def test_decl_facts_equal_the_fresh_extraction_and_a_hit_does_not_walk(cache):
    tu = audit.parse_tu(IDENTITY, (FIXTURE_INCLUDE,))
    miss = audit.decl_facts(IDENTITY, (FIXTURE_INCLUDE,), lambda: tu)
    hit = audit.decl_facts(IDENTITY, (FIXTURE_INCLUDE,), _no_parse)
    assert miss == hit
    assert hit.names == audit.decl_names(tu)
    assert dict(hit.ancestors) == audit.class_ancestors(tu)
    assert dict(hit.bases) == audit.public_bases(tu)


def test_gil_releases_equal_the_fresh_scan_and_a_hit_does_not_parse(cache):
    args = [*audit.PARSE_ARGS, *(f"-I{d}" for d in audit.INCLUDE_DIRS)]

    def scan(tu):
        return gil.Scanner(tu, "fixture").scan()

    fresh = scan(gil.parse(GIL_FIXTURE, args))
    assert fresh  # the fixture releases the GIL: an empty scan would prove nothing
    assert facts_.gil_releases(GIL_FIXTURE, args, scan, gil.parse) == fresh
    assert facts_.gil_releases(GIL_FIXTURE, args, scan, _no_parse) == fresh


def test_an_edit_to_an_included_header_reparses(cache, tmp_path):
    """The dependency list is the parse's own include list: a header edit is a miss."""
    work = tmp_path / "work"
    shutil.copytree(FIXTURES, work)
    cpp, include = work / IDENTITY.name, work / "include"
    audit.bound_facts(cpp, (include,), audit.parse_tu)
    tu = audit.parse_tu(cpp, (include,), bodies=True)  # held: includes point into it
    headers = [
        Path(i.include.name)
        for i in tu.get_includes()
        if Path(i.include.name).resolve().is_relative_to(include.resolve())
    ]
    assert headers, "the fixture includes a header under its include dir"
    headers[0].write_text(headers[0].read_text() + "\n// edited\n")
    parsed = []

    def parse(*a, **k):
        parsed.append(a[0])
        return audit.parse_tu(*a, **k)

    audit.bound_facts(cpp, (include,), parse)
    assert parsed == [cpp]


def test_another_extractor_source_is_another_key():
    key = facts_.fact_key("bound", IDENTITY, ["-x", "c++"], 0, facts_.BOUND_EXTRACTORS)
    other = facts_.fact_key("bound", IDENTITY, ["-x", "c++"], 0, facts_.DECL_EXTRACTORS)
    assert key.digest() != other.digest()


def test_every_include_dir_in_the_repo_is_listed():
    key = facts_.fact_key("decl", IDENTITY, facts_.audit_args((FIXTURE_INCLUDE,)), 0, ())
    listed = {d for d, _ in key.toolchain.repo_dirs}
    assert {str(FIXTURE_INCLUDE), str(FIXTURES), str(audit.SRC)} <= listed
    assert not any(Path(d).is_relative_to(audit.CONDA_PREFIX) for d in listed)


def test_a_native_root_holding_the_repo_leaves_the_repo_listed(monkeypatch):
    """CI: AUDIT_NATIVE_ROOT is the workspace, a parent of the repo checkout."""
    workspace = audit.REPO_ROOT.parent
    monkeypatch.setattr(facts_, "NATIVE_DIRS", audit.native_stack_dirs(workspace))
    assert facts_._in_repo(audit.SRC)


def test_a_native_stack_inside_the_repo_is_not_listed(monkeypatch):
    """Local: `.native/<triplet>` under the repo is covered by header hashes, not listings."""
    root = audit.REPO_ROOT / ".native" / "x64-linux-dynamic-release"
    monkeypatch.setattr(facts_, "NATIVE_DIRS", audit.native_stack_dirs(root))
    assert not facts_._in_repo(root / "vcpkg" / "installed" / "x64-linux-dynamic-release")
    assert not facts_._in_repo(root / "ws" / "install" / "include")
    assert facts_._in_repo(audit.SRC)


# #397 stage 3: the include structure and the extracted API of a TU, also a synthetic one.

match_ = load("audit_match")
parse_ = load("audit_parse")
WIDGETS = (FIXTURE_INCLUDE / "tesseract" / "identity" / "widgets.h").resolve()
WIDGETS_SPELLING = "tesseract/identity/widgets.h"
CLASH = {
    (FIXTURE_INCLUDE / "tesseract" / "clash" / name).resolve(): f"tesseract/clash/{name}"
    for name in ("clash_a.h", "clash_b.h")
}


@pytest.fixture
def parses(monkeypatch):
    """The main files `audit_match` parsed, in order."""
    parsed = []
    real = match_.parse_tu

    def parse(cpp, *a, **k):
        parsed.append(Path(cpp))
        return real(cpp, *a, **k)

    monkeypatch.setattr(match_, "parse_tu", parse)
    return parsed


def test_include_facts_equal_the_fresh_extraction_and_a_hit_does_not_parse(cache):
    tu = audit.parse_tu(IDENTITY, (FIXTURE_INCLUDE,))
    miss = audit.include_facts(IDENTITY, (FIXTURE_INCLUDE,), lambda: tu)
    hit = audit.include_facts(IDENTITY, (FIXTURE_INCLUDE,), _no_parse)
    assert miss == hit
    assert hit.direct == audit.direct_includes(tu)
    assert hit.inclusions == audit.inclusions(tu)
    assert WIDGETS in hit.direct
    assert WIDGETS in hit.included


def test_direct_includes_reach_through_the_headers_beside_the_tu(tmp_path):
    """A module's TUs share declarations through a header beside them
    (`trajopt_ifopt_bindings.h`): what that header #includes is the TU's own direct include,
    or the split TUs would audit none of it and hand the headers to another module."""
    (tmp_path / "alpha.cpp").write_text('#include "alpha_bindings.h"\n')
    (tmp_path / "alpha_bindings.h").write_text(f"#pragma once\n#include <{WIDGETS_SPELLING}>\n")
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    # A quoted header found through -I, not beside the TU, holds no direct include.
    (elsewhere / "shared.h").write_text(f"#pragma once\n#include <{WIDGETS_SPELLING}>\n")
    (tmp_path / "beta.cpp").write_text('#include "shared.h"\n')
    alpha = audit.parse_tu(tmp_path / "alpha.cpp", (FIXTURE_INCLUDE,))
    assert WIDGETS in audit.direct_includes(alpha)
    beta = audit.parse_tu(tmp_path / "beta.cpp", (FIXTURE_INCLUDE, elsewhere))
    assert audit.direct_includes(beta) == ((elsewhere / "shared.h").resolve(),)


def test_api_facts_equal_the_fresh_extraction_in_order_and_a_hit_does_not_parse(cache):
    tu = audit.parse_tu(IDENTITY, (FIXTURE_INCLUDE,))
    headers = frozenset({WIDGETS})
    selection = audit.api_selection(headers, ())
    fresh = audit.extract_api(tu, headers, frozenset())
    assert fresh  # the fixture header declares classes: an empty API would prove nothing
    miss = audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), selection, lambda: tu)
    hit = audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), selection, _no_parse)
    assert miss == hit == fresh
    assert list(hit) == list(fresh)
    assert audit.resolve_bound(hit, audit.no_bound_class) == audit.cpp_api(tu, headers)


def test_another_selection_and_a_patched_root_are_other_entries(cache, monkeypatch):
    """Review Focus 1: an audit under a patched table must not write, or read, the entry of
    the unpatched one."""
    tu = audit.parse_tu(IDENTITY, (FIXTURE_INCLUDE,))
    parsed = []

    def parse():
        parsed.append(1)
        return tu

    headers = frozenset({WIDGETS})
    plain = audit.api_selection(headers, ())
    audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), plain, parse)
    audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), audit.api_selection((), ()), parse)
    audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), audit.api_selection(headers, headers), parse)
    with monkeypatch.context() as patch:  # not `monkeypatch.undo()`: `cache` patched with it
        patch.setattr(parse_, "UNBOUND_DEPENDENCY_ROOTS", ("tests/audit/fixtures/include/",))
        patched = audit.api_selection(headers, ())
        assert patched != plain
        audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), patched, parse)
    assert len(parsed) == 4
    audit.api_facts(IDENTITY, (FIXTURE_INCLUDE,), audit.api_selection(headers, ()), _no_parse)


def test_a_synthetic_tu_is_stored_and_reparses_on_an_edit(cache, parses, tmp_path):
    """Review Focus 2 and 3: the main file is not on disk, the binding it includes is a
    dependency, and so is every header."""
    work = tmp_path / "work"
    shutil.copytree(FIXTURES, work)
    cpp, include = work / IDENTITY.name, work / "include"
    header = (include / WIDGETS_SPELLING).resolve()
    headers = {header: WIDGETS_SPELLING}
    first = audit.unincluded_header_gaps(cpp, headers, (include,))
    assert len(parses) == 1
    assert not parses[0].exists()
    assert audit.unincluded_header_gaps(cpp, headers, (include,)) == first
    assert len(parses) == 1
    header.write_text(header.read_text() + "\n// edited\n")
    audit.unincluded_header_gaps(cpp, headers, (include,))
    assert len(parses) == 2
    cpp.write_text(cpp.read_text() + "\n// edited\n")
    audit.unincluded_header_gaps(cpp, headers, (include,))
    assert len(parses) == 3


def test_another_header_set_is_another_synthetic_tu(cache, parses):
    one = dict(list(CLASH.items())[:1])
    two = dict(list(CLASH.items())[1:])
    nested = FIXTURES / "nested_bindings.cpp"
    audit.unincluded_header_gaps(nested, one, (FIXTURE_INCLUDE,))
    audit.unincluded_header_gaps(nested, two, (FIXTURE_INCLUDE,))
    audit.unincluded_header_gaps(nested, one, (FIXTURE_INCLUDE,))
    assert len(parses) == 2


def test_a_failed_parse_stores_nothing(cache, parses):
    for _ in range(2):
        with pytest.raises(audit.HeaderParseError, match="redefinition"):
            audit._header_gaps(FIXTURES / "nested_bindings.cpp", CLASH, (FIXTURE_INCLUDE,))
    assert len(parses) == 2
    assert not list(cache.root.glob("*/*.json"))


def test_a_second_audit_of_a_tu_parses_nothing_and_reports_the_same(cache, parses):
    def report():
        return audit.audit_tu(
            "identity",
            IDENTITY,
            FIXTURES / "_fixture.pyi",
            FIXTURES / "package_init.py",
            ("tesseract/identity/",),
            (FIXTURE_INCLUDE,),
            stub_roots=(FIXTURES,),
            bound=lambda m: {},
        )

    first = report()
    assert parses
    del parses[:]
    assert report() == first
    assert parses == []
