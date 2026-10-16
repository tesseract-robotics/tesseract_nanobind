"""Members inherited from an unbound base (#256 A)."""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
# Its own stub root: `_typed.pyi` binds another `Shape`, which would read as bound elsewhere.
INHERIT = FIXTURES / "inherit"
SHAPES = (FIXTURE_INCLUDE / "tesseract" / "inherit" / "shapes.h").resolve()

A_GAPS = {
    ("Square.__eq__", "operator"), ("Square.__ne__", "operator"), ("Square.sides", "method"),
    ("Square.area", "field"), ("Circle.__ne__", "operator"), ("Circle.sides", "method"),
    ("Circle.area", "field"),
}  # fmt: skip


@pytest.fixture(scope="module")
def inherit_report():
    return audit.audit_tu(
        "inherit",
        INHERIT / "inherit_bindings.cpp",
        INHERIT / "_inherit.pyi",
        INHERIT / "inherit_init.py",
        ("tesseract/inherit/",),
        (FIXTURE_INCLUDE,),
        stub_roots=(INHERIT,),
    )


@pytest.fixture(scope="module")
def shapes_cpp():
    tu = audit.parse_tu(INHERIT / "inherit_bindings.cpp", (FIXTURE_INCLUDE,))
    return audit.cpp_api(tu, frozenset({SHAPES}))


def test_inherited_symbol_records_its_base_and_the_base_keeps_its_own(shapes_cpp):
    assert shapes_cpp["Square.__eq__"].inherited_from == "Shape"
    assert shapes_cpp["Shape.__eq__"].inherited_from is None  # USR trap: base keeps its member
    assert len(shapes_cpp["Shape.__eq__"].overloads) == 1
    assert shapes_cpp["Circle.__eq__"].inherited_from is None  # own declaration hides the base's


def test_private_base_and_protected_members_are_not_inherited(shapes_cpp):
    assert not {n for n in shapes_cpp if n.startswith("Hidden.") and n != "Hidden.__init__"}
    assert not {n for n in shapes_cpp if n.endswith(".equals")}


def test_inherited_members_of_an_unbound_base_are_gaps(inherit_report):
    rows = {(g.symbol, g.kind.value) for g in inherit_report.gaps}
    assert A_GAPS <= rows
    assert {s for s, _ in rows} - {s for s, _ in A_GAPS} == {"Shape", "Middle"}


def test_bound_hidden_and_bound_base_members_are_no_rows(inherit_report):
    gaps = {g.symbol for g in inherit_report.gaps}
    assert "Circle.__eq__" not in gaps  # A2: declared and bound
    assert not {g for g in gaps if g.startswith("Hexagon.")}  # A3: bound on the derived class
    assert "SlowRunner.laps" not in gaps  # A6: base Runner is bound, it audits laps
    assert "DeepRunner.depth" not in gaps  # A7: inherited from unbound Middle, bound here


def test_python_only_overload_of_an_inherited_bound_member_is_reported_once(inherit_report):
    """A8 (RESULTS.md Finding 1): the deviation belongs to the bound base, not to each subclass
    whose stub re-declares the member."""
    laps = {d.name for d in inherit_report.deviations if d.name.endswith(".laps")}
    assert laps == {"Runner.laps"}


def test_own_header_lines_ignores_inherited_symbols():
    here, base = "tesseract/inherit/leaf.h", "tesseract/inherit/shapes.h"
    cpp = {
        "Leaf": audit.CppSymbol("Leaf", audit.Kind.CLASS, f"{here}:8"),
        "Leaf.sides": audit.CppSymbol(
            "Leaf.sides", audit.Kind.METHOD, f"{base}:3", inherited_from="Shape"
        ),
        "Shape": audit.CppSymbol("Shape", audit.Kind.CLASS, f"{base}:5"),
    }
    assert audit.own_header_lines(cpp) == {here: [8], base: [5]}
