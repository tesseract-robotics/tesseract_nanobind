"""Contract tests of the audit's task-composer mechanisms (#256 group 4).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
# Its own stub root, so no other fixture stub's class reads as bound elsewhere.
HOLDER = FIXTURES / "holder"
HOLDER_PREFIX = ("tesseract/holder/",)
HOLDER_HEADER = "tests/audit/fixtures/include/tesseract/holder/holder.h"
# holder.h lines of AnyHolderInterface and the AnyHolder constructors.
INTERFACE_LINE = 9  # struct AnyHolderInterface
PLAIN_CTOR_LINE = 19  # AnyHolder(const AnyHolderInterface&)
FORWARDING_LINE = 28  # template <T, enable_if> AnyHolder(T&&)
CONST_REF_LINE = 34  # template <T, enable_if> AnyHolder(const T&)
TWO_PARAM_LINE = 37  # template <T> AnyHolder(const T&, int)
# holder.h lines of AnyBox<T> and its ctor templates (review negatives).
BOX_LINE = 45  # template <T> class AnyBox
CLASS_PARAM_LINE = 49  # template <U> AnyBox(const T&): the class's T
POINTER_LINE = 52  # template <U> AnyBox(U*)


@pytest.fixture(scope="module")
def holder_report():
    return audit.audit_tu(
        "holder",
        HOLDER / "holder_bindings.cpp",
        HOLDER / "_holder.pyi",
        HOLDER / "holder_init.py",
        HOLDER_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(HOLDER,),
    )


@pytest.fixture(scope="module")
def holder_ctors():
    """`AnyHolder`'s and `AnyBox`'s constructors and constructor templates, by holder.h line."""
    tu = audit.parse_tu(HOLDER / "holder_bindings.cpp", (FIXTURE_INCLUDE,))
    records = {"AnyHolder", "AnyBox"}
    kinds = {audit.ci.CursorKind.CONSTRUCTOR, audit.ci.CursorKind.FUNCTION_TEMPLATE}
    return {
        m.location.line: m
        for c in tu.cursor.walk_preorder()
        if c.spelling in records
        and c.is_definition()
        and c.location.file is not None
        and c.location.file.name.endswith(HOLDER_HEADER)
        for m in c.get_children()
        if m.kind in kinds
    }


def test_forwarding_ctor_templates_accepted(holder_report):
    """Both constrained `AnyHolder(T&&)` and `AnyHolder(const T&)`: template-instance-name."""
    assert sorted(
        (a.rule, a.location) for a in holder_report.accepted if a.symbol == "AnyHolder.__init__"
    ) == [
        ("template-instance-name", f"{HOLDER_HEADER}:{FORWARDING_LINE}"),
        ("template-instance-name", f"{HOLDER_HEADER}:{CONST_REF_LINE}"),
    ]


def test_holder_gaps_exact(holder_report):
    """The plain ctor and the non-forwarding template stay rows; nothing else does."""
    assert sorted((g.symbol, g.kind, g.arity, g.location) for g in holder_report.gaps) == [
        ("AnyBox", audit.Kind.CLASS, audit.NOT_AN_OVERLOAD, f"{HOLDER_HEADER}:{BOX_LINE}"),
        ("AnyHolder.__init__", audit.Kind.CONSTRUCTOR, "1", f"{HOLDER_HEADER}:{PLAIN_CTOR_LINE}"),
        ("AnyHolder.__init__", audit.Kind.CONSTRUCTOR, "2", f"{HOLDER_HEADER}:{TWO_PARAM_LINE}"),
        (
            "AnyHolderInterface",
            audit.Kind.CLASS,
            audit.NOT_AN_OVERLOAD,
            f"{HOLDER_HEADER}:{INTERFACE_LINE}",
        ),
    ]
    assert holder_report.deviations == []


def test_is_forwarding_ctor_template_unit(holder_ctors):
    assert audit._is_forwarding_ctor_template(holder_ctors[FORWARDING_LINE])
    assert audit._is_forwarding_ctor_template(holder_ctors[CONST_REF_LINE])
    assert not audit._is_forwarding_ctor_template(holder_ctors[TWO_PARAM_LINE])
    assert not audit._is_forwarding_ctor_template(holder_ctors[PLAIN_CTOR_LINE])
    for line in (CLASS_PARAM_LINE, POINTER_LINE):
        m = holder_ctors[line]
        assert audit._is_ctor(m, m.semantic_parent)  # rejected by its parameter, not as a non-ctor
        assert not audit._is_forwarding_ctor_template(m)
