"""Stub base classes vs the bound C++ bases (#256 T5).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
BASES = FIXTURES / "bases"  # own stub root: its class names stay out of other fixtures
BASES_CPP = BASES / "bases_bindings.cpp"
# What `bound_classes` reads from bases_bindings.cpp, for tests that inject a `bound` lookup.
OWN_INDEX = {"bases": {"PlannerD": "tesseract::bases::Planner"}}


def bases_report_with(**kw):
    """The bases fixture's `audit_tu`, with `kw` (an injected `bound` lookup) forwarded."""
    return audit.audit_tu(
        "bases",
        BASES_CPP,
        BASES / "_bases.pyi",
        BASES / "bases_init.py",
        ("tesseract/bases/",),
        (FIXTURE_INCLUDE,),
        stub_roots=(BASES,),
        **kw,
    )


@pytest.fixture(scope="module")
def bases_report():
    return bases_report_with()


def _base_rows(report):
    gaps = {g.symbol for g in report.gaps if g.kind is audit.Kind.BASE}
    deviations = {d.name for d in report.deviations if d.kind is audit.Kind.BASE}
    return gaps, deviations


def test_public_bases_lists_direct_public_bases_only():
    """Qualified class -> its direct public bases' qualified classes (T3 identity)."""
    bases = audit.public_bases(audit.parse_tu(BASES_CPP, (FIXTURE_INCLUDE,)))
    ns = "tesseract::bases::"
    assert bases[f"{ns}SlowJogger"] == (f"{ns}Jogger",)
    assert bases[f"{ns}DeepJogger"] == (f"{ns}Middle",)  # direct only: Jogger through Middle
    assert bases[f"{ns}Hidden"] == ()  # private base
    assert bases[f"{ns}Loner"] == ()
    assert bases[f"{ns}Planner"] == (f"{ns}Jogger",)  # the class template's base
    assert bases[f"{ns}SubPlanner"] == (f"{ns}Planner",)  # a template instance base


def test_base_rows_exact(bases_report):
    """T5-1 and the renamed template instance are gaps, T5-3 a deviation; nothing else.

    `SubPlanner(PlannerD)` raises no row: the bare stub base resolves through the own index
    (`PlannerD` -> `tesseract::bases::Planner`) to the C++ base `Planner<double>`.
    """
    assert _base_rows(bases_report) == (
        {"SlowJogger(Jogger)", "PlannerD(Jogger)"},
        {"Loner(Jogger)"},
    )


def test_missing_bound_base_is_a_gap_at_the_cpp_class(bases_report):
    """T5-1: SlowJogger's C++ base Jogger is bound; the stub omits it."""
    row = next(g for g in bases_report.gaps if g.symbol == "SlowJogger(Jogger)")
    assert row.location == "tests/audit/fixtures/include/tesseract/bases/runners.h:12"


def test_stub_base_that_is_no_cpp_ancestor_is_a_deviation_at_the_stub(bases_report):
    """T5-3: Loner claims Jogger; C++ Loner has no base."""
    row = next(d for d in bases_report.deviations if d.name == "Loner(Jogger)")
    assert row.location == "tests/audit/fixtures/bases/_bases.pyi:14"


def test_nearest_bound_ancestor_through_an_unbound_intermediate_is_enough(bases_report):
    """T5-2: DeepJogger : Middle : Jogger, stub says DeepJogger(Jogger)."""
    gaps, deviations = _base_rows(bases_report)
    assert not {r for r in gaps | deviations if r.startswith(("DeepJogger(", "Tagged(", "Hidden("))}


def test_renamed_class_is_checked_under_its_python_name(bases_report):
    """`Planner<double>` is bound as `PlannerD` (T3): the base pass reaches it by that name."""
    assert "PlannerD(Jogger)" in _base_rows(bases_report)[0]


def _rows_of(report, cls):
    rows = {g.symbol for g in report.gaps} | {d.name for d in report.deviations}
    return {r for r in rows if r.startswith(f"{cls}(")}


def test_cross_module_base_without_an_index_entry_compares_by_leaf(bases_report):
    """`bases_remote` has no binding TU: positive evidence only, leaf `Jogger` = C++ `Jogger`."""
    assert not _rows_of(bases_report, "RemoteJogger")


def test_cross_module_base_with_the_same_leaf_but_another_class_is_a_gap():
    """Identity, not leaf: a stub base of another class under the same leaf is a gap.

    `bases_remote` binds `Jogger` as `other::Jogger`, so the stub base
    `bases_remote._bases_remote.Jogger` is not the C++ base `tesseract::bases::Jogger`.
    """
    index = {**OWN_INDEX, "bases_remote": {"Jogger": "other::Jogger"}}
    report = bases_report_with(bound=lambda m: index.get(m, {}))
    assert ("RemoteJogger(Jogger)", audit.Kind.BASE) in {(g.symbol, g.kind) for g in report.gaps}
    assert ("RemoteJogger(Jogger)", audit.Kind.BASE) in {
        (d.name, d.kind) for d in report.deviations
    }


def test_cross_module_base_with_the_same_class_raises_no_row():
    index = {**OWN_INDEX, "bases_remote": {"Jogger": "tesseract::bases::Jogger"}}
    report = bases_report_with(bound=lambda m: index.get(m, {}))
    assert not _rows_of(report, "RemoteJogger")
