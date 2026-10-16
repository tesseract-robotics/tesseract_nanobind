"""Stub base classes vs the bound C++ bases (#256 T5).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
BASES_CPP = FIXTURES / "bases_bindings.cpp"


@pytest.fixture(scope="module")
def bases_report():
    return audit.audit_tu(
        "bases",
        BASES_CPP,
        FIXTURES / "_bases.pyi",
        FIXTURES / "package_init.py",
        ("tesseract/bases/",),
        (FIXTURE_INCLUDE,),
        stub_roots=(FIXTURES,),
    )


def _base_rows(report):
    gaps = {g.symbol for g in report.gaps if g.kind is audit.Kind.BASE}
    deviations = {d.name for d in report.deviations if d.kind is audit.Kind.BASE}
    return gaps, deviations


def test_public_bases_lists_direct_public_bases_only():
    bases = audit.public_bases(audit.parse_tu(BASES_CPP, (FIXTURE_INCLUDE,)))
    assert bases["SlowRunner"] == ("Runner",)
    assert bases["DeepRunner"] == ("Middle",)  # direct only: Runner comes through Middle
    assert bases["Hidden"] == ()  # private base
    assert bases["Loner"] == ()
    assert bases["Planner"] == ("Runner",)  # the class template's base


def test_base_rows_exact(bases_report):
    """T5-1 and the renamed template instance are gaps, T5-3 a deviation; nothing else."""
    assert _base_rows(bases_report) == (
        {"SlowRunner(Runner)", "PlannerD(Runner)"},
        {"Loner(Runner)"},
    )


def test_missing_bound_base_is_a_gap_at_the_cpp_class(bases_report):
    """T5-1: SlowRunner's C++ base Runner is bound; the stub omits it."""
    row = next(g for g in bases_report.gaps if g.symbol == "SlowRunner(Runner)")
    assert row.location.startswith("tests/audit/fixtures/include/tesseract/bases/runners.h:")


def test_stub_base_that_is_no_cpp_ancestor_is_a_deviation_at_the_stub(bases_report):
    """T5-3: Loner claims Runner; C++ Loner has no base."""
    row = next(d for d in bases_report.deviations if d.name == "Loner(Runner)")
    assert row.location.startswith("tests/audit/fixtures/_bases.pyi:")


def test_nearest_bound_ancestor_through_an_unbound_intermediate_is_enough(bases_report):
    """T5-2: DeepRunner : Middle : Runner, stub says DeepRunner(Runner)."""
    gaps, deviations = _base_rows(bases_report)
    assert not {r for r in gaps | deviations if r.startswith(("DeepRunner(", "Tagged(", "Hidden("))}


def test_renamed_class_is_checked_under_its_python_name(bases_report):
    """`Planner<double>` is bound as `PlannerD` (T3): the base pass reaches it by that name."""
    assert "PlannerD(Runner)" in _base_rows(bases_report)[0]
