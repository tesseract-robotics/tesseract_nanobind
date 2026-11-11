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
