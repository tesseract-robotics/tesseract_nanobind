"""Members inherited from an unbound base (#256 A)."""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
# Its own stub root: `_typed.pyi` binds another `Shape`, which would read as bound elsewhere.
INHERIT = FIXTURES / "inherit"
SHAPES = (FIXTURE_INCLUDE / "tesseract" / "inherit" / "shapes.h").resolve()
SHAPES_REL = "tesseract/inherit/shapes.h"

A_GAPS = {
    ("Tile.__eq__", "operator"), ("Tile.__ne__", "operator"), ("Tile.sides", "method"),
    ("Tile.area", "field"), ("Disc.__ne__", "operator"), ("Disc.sides", "method"),
    ("Disc.area", "field"), ("Exposer.area", "field"),
}  # fmt: skip
CLASS_GAPS = {("Shape", "class"), ("Middle", "class")}
# Shape's members, where every inherited row points (A1).
TILE_LINES = {"Tile.__eq__": 9, "Tile.__ne__": 10, "Tile.sides": 11, "Tile.area": 12}


def _rows(report):
    """Gaps other than T5's base rows, which `test_audit_bases.py` audits."""
    return {(g.symbol, g.kind.value) for g in report.gaps if g.kind.value != "base"}


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
    assert shapes_cpp["Tile.__eq__"].inherited_from == "Shape"
    assert len(shapes_cpp["Tile.__eq__"].overloads) == 1
    assert shapes_cpp["Shape.__eq__"].inherited_from is None  # USR trap: base keeps its member
    assert len(shapes_cpp["Shape.__eq__"].overloads) == 1
    assert shapes_cpp["Disc.__eq__"].inherited_from is None  # own declaration hides the base's


def test_inherited_through_an_unbound_intermediate_records_the_declaring_base(shapes_cpp):
    assert shapes_cpp["DeepSprinter.laps"].inherited_from == "Sprinter"
    assert shapes_cpp["DeepSprinter.depth"].inherited_from == "Middle"
    assert shapes_cpp["DeepSprinter.go"].inherited_from is None  # own override hides Sprinter::go


def test_private_base_and_protected_members_are_not_inherited(shapes_cpp):
    assert not {n for n in shapes_cpp if n.startswith("Concealed.") and n != "Concealed.__init__"}
    assert not {n for n in shapes_cpp if n.endswith(".equals")}


def test_filtered_base_members_are_not_inherited(shapes_cpp):
    """A9: copy assignment, deleted, deprecated and cereal-hook members of the base."""
    tile = {n for n in shapes_cpp if n.startswith("Tile.")}
    assert not {n for n in tile if n.endswith(("operator=", ".reset", ".legacy", ".serialize"))}
    assert tile == {"Tile.__init__", *TILE_LINES}


def test_inherited_members_of_an_unbound_base_are_gaps(inherit_report):
    assert _rows(inherit_report) == A_GAPS | CLASS_GAPS


def test_inherited_gap_is_located_at_the_base_member(inherit_report):
    where = {g.symbol: g.location for g in inherit_report.gaps if g.symbol in TILE_LINES}
    assert where.keys() == TILE_LINES.keys()
    for name, line in TILE_LINES.items():
        assert where[name].endswith(f"{SHAPES_REL}:{line}"), where[name]


def test_bound_concealed_and_bound_base_members_are_no_rows(inherit_report):
    gaps = {g.symbol for g in inherit_report.gaps}
    assert "Disc.__eq__" not in gaps  # A2: declared and bound
    assert not {g for g in gaps if g.startswith("Hexagon.")}  # A3: bound on the derived class
    assert "SlowSprinter.laps" not in gaps  # A6: base Sprinter is bound, it audits laps
    assert "DeepSprinter.depth" not in gaps  # A7: inherited from unbound Middle, bound here
    assert not {"Remote", "Courier.ping"} & gaps  # A11: base bound in another extension's stub


def test_python_only_overload_of_an_inherited_bound_member_is_reported_once(inherit_report):
    """A8 (RESULTS.md Finding 1): the deviation belongs to the bound base, not to each subclass
    whose stub re-declares the member."""
    rows = {(d.name, d.kind.value) for d in inherit_report.deviations if d.kind.value != "base"}
    assert rows == {("Sprinter.laps", "overload")}


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
