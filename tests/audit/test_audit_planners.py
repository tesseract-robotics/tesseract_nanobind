"""Contract tests of the audit's motion-planner mechanisms (#256 group 3).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
PLANNER_PREFIX = ("tesseract/planner/",)


def _planner_report():
    return audit.audit_tu(
        "planner",
        FIXTURES / "planner_bindings.cpp",
        FIXTURES / "_planner.pyi",
        FIXTURES / "planner_init.py",
        PLANNER_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(FIXTURES,),
    )


@pytest.fixture(scope="module")
def planner_report():
    return _planner_report()


@pytest.fixture(scope="module")
def planner_cpp():
    tu = audit.parse_tu(FIXTURES / "planner_bindings.cpp", (FIXTURE_INCLUDE,))
    headers = audit.audited_headers(
        tu, "planner", {"planner": PLANNER_PREFIX}, (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS)
    )
    return audit.cpp_api(tu, headers)


# T1: the `yaml-node` premise.


def test_yaml_position_is_the_python_index_of_the_yaml_node(planner_cpp):
    positions = {
        str(ov.arity): ov.yaml_position for ov in planner_cpp["Profile.__init__"].overloads
    }
    assert positions == {"0": None, "2": 0}


def test_yaml_ctor_with_a_bound_text_sibling_is_accepted(planner_report):
    assert ("Loader.__init__", "yaml-node") in {(a.symbol, a.rule) for a in planner_report.accepted}
    assert "Loader.__init__" not in {g.symbol for g in planner_report.gaps}


def test_yaml_ctor_without_a_sibling_is_a_gap(planner_report):
    assert ("Profile.__init__", "2") in {(g.symbol, g.arity) for g in planner_report.gaps}
    assert "Profile.__init__" not in {a.symbol for a in planner_report.accepted}


def test_yaml_ctor_with_an_unbound_sibling_is_a_gap(planner_report):
    gaps = {(g.symbol, g.arity) for g in planner_report.gaps}
    assert ("Orphan.__init__", "1") in gaps  # both 1-arity overloads: one row per C++ overload
    assert "Orphan.__init__" not in {a.symbol for a in planner_report.accepted}


def test_failed_yaml_premise_falls_through_to_the_overload_table(monkeypatch):
    """No sibling, but an `ACCEPTED_CPP_OVERLOADS` entry: the entry's rule accepts the overload."""
    monkeypatch.setitem(audit.ACCEPTED, "fixture-plugin-config-ctor", "fixture")
    monkeypatch.setitem(
        audit.ACCEPTED_CPP_OVERLOADS,
        ("planner", "Profile.__init__", "2"),
        "fixture-plugin-config-ctor",
    )
    report = _planner_report()
    assert ("Profile.__init__", "fixture-plugin-config-ctor") in {
        (a.symbol, a.rule) for a in report.accepted
    }
    assert ("Profile.__init__", "2") not in {(g.symbol, g.arity) for g in report.gaps}


@pytest.mark.parametrize(
    ("module", "symbol"),
    [
        ("tesseract_collision", "ContactManagersPluginFactory.__init__"),
        ("tesseract_kinematics", "KinematicsPluginFactory.__init__"),
    ],
)
def test_yaml_premise_keeps_core_rows(module, symbol):
    """Both core plugin factories bind a `(str, locator)` and `(os.PathLike, locator)` sibling."""
    report = audit.audit_module(module)
    assert (symbol, "yaml-node") in {(a.symbol, a.rule) for a in report.accepted}


# T13: a class template's constructor template.


def test_class_template_ctor_template_is_a_constructor(planner_cpp):
    assert not [s for s in planner_cpp if s.startswith("Box.Box")]
    assert {str(ov.arity) for ov in planner_cpp["Box.__init__"].overloads} == {"0", "1"}


# T12: never audit `operator=`.


def test_no_operator_assign_symbol(planner_cpp):
    """T12: Python has no assignment operator, so no `operator=` is ever a row."""
    assert not [s for s in planner_cpp if s.endswith(".operator=")]


def test_other_operators_keep_their_rows(planner_report):
    gaps = {g.symbol for g in planner_report.gaps}
    assert "Holder.__eq__" in gaps
    # whichever spelling OPERATOR_DUNDERS gives it
    assert {"Holder.operator+", "Holder.__add__"} & gaps
    assert not [s for s in gaps if s.endswith(".operator=")]
