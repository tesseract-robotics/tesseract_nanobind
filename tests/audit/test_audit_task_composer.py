"""Contract tests of the audit's task-composer mechanisms (#256 group 4).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
HOLDER_PREFIX = ("tesseract/holder/",)
HOLDER_HEADER = "holder.h"


@pytest.fixture(scope="module")
def holder_report():
    return audit.audit_tu(
        "holder",
        FIXTURES / "holder_bindings.cpp",
        FIXTURES / "_holder.pyi",
        FIXTURES / "holder_init.py",
        HOLDER_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(FIXTURES,),
    )


@pytest.fixture(scope="module")
def holder_ctor_templates():
    """`Holder`'s constructor templates, keyed by parameter count."""
    tu = audit.parse_tu(FIXTURES / "holder_bindings.cpp", (FIXTURE_INCLUDE,))
    holder = next(
        c
        for c in tu.cursor.walk_preorder()
        if c.spelling == "Holder"
        and c.is_definition()
        and c.location.file is not None
        and Path(c.location.file.name).name == HOLDER_HEADER
    )
    return {
        len([p for p in m.get_children() if p.kind == audit.ci.CursorKind.PARM_DECL]): m
        for m in holder.get_children()
        if m.kind == audit.ci.CursorKind.FUNCTION_TEMPLATE
    }


def test_forwarding_ctor_template_accepted(holder_report):
    """`template <T> Holder(T&&)` has one Python function per instance: template-instance-name."""
    rules = {a.rule for a in holder_report.accepted if a.symbol == "Holder.__init__"}
    assert "template-instance-name" in rules


def test_ctor_templates_and_plain_ctors_that_are_not_forwarding_stay_rows(holder_report):
    arities = {g.arity for g in holder_report.gaps if g.symbol == "Holder.__init__"}
    assert arities == {"1", "2"}  # Holder(const HolderInterface&), Holder(const T&, int)


def test_holder_gaps_exact(holder_report):
    assert sorted((g.symbol, g.kind, g.arity) for g in holder_report.gaps) == [
        ("Holder.__init__", audit.Kind.CONSTRUCTOR, "1"),
        ("Holder.__init__", audit.Kind.CONSTRUCTOR, "2"),
        ("HolderInterface", audit.Kind.CLASS, audit.NOT_AN_OVERLOAD),
    ]
    assert holder_report.deviations == []


def test_is_forwarding_ctor_template_unit(holder_ctor_templates):
    assert audit._is_forwarding_ctor_template(holder_ctor_templates[1])
    assert not audit._is_forwarding_ctor_template(holder_ctor_templates[2])
