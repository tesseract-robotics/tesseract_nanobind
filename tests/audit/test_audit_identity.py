"""Class identity by qualified C++ name (#256 T3, TF-mp-plan-2).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
RENAMED = FIXTURES / "renamed"
BOUND_ELSEWHERE = FIXTURES / "bound_elsewhere"
HERE = "tests/audit/fixtures/renamed/widget.h"


def test_qualified_path_names_namespaces():
    tu = audit.parse_tu(audit.binding_source("trajopt_ifopt"))
    var = next(
        c for c in tu.cursor.walk_preorder()
        if c.kind == audit.ci.CursorKind.CLASS_DECL and c.spelling == "Var" and c.is_definition()
    )  # fmt: skip
    assert audit.qualified_path(var) == "trajopt_ifopt::Var"


def test_bound_classes_reads_each_top_level_construction():
    """Renames, template instances, enums and listed bases; the nested `Part` is skipped."""
    tu = audit.parse_tu(FIXTURES / "identity_bindings.cpp", (FIXTURE_INCLUDE,), bodies=True)
    assert audit.bound_classes(tu) == {
        "Gizmo": "tesseract::identity::LongWidgetName",
        "GadgetD": "tesseract::identity::Gadget",
        "Color": "tesseract::identity::Color",
        "Derived": "tesseract::identity::Derived",
    }


def test_bound_classes_rejects_a_tu_without_function_bodies():
    """The default parse skips bodies, NB_MODULE's included: no construction would be seen."""
    tu = audit.parse_tu(FIXTURES / "identity_bindings.cpp", (FIXTURE_INCLUDE,))
    with pytest.raises(audit.SkippedBodiesError, match="identity_bindings.cpp"):
        audit.bound_classes(tu)


def test_bound_classes_reads_each_nb_class():
    tu = audit.parse_tu(audit.binding_source("tesseract_time_parameterization"), bodies=True)
    bound = audit.bound_classes(tu)
    assert bound["TOTGCompositeProfile"] == (
        "tesseract::time_parameterization::TimeOptimalTrajectoryGenerationCompositeProfile"
    )
    assert bound["TimeOptimalTrajectoryGeneration"] == (
        "tesseract::time_parameterization::TimeOptimalTrajectoryGeneration"
    )


def test_bound_classes_resolves_namespace_aliases():
    tu = audit.parse_tu(audit.binding_source("trajopt_ifopt"), bodies=True)
    assert audit.bound_classes(tu)["Var"] == "trajopt_ifopt::Var"  # bound as nb::class_<ti::Var>


def test_bound_index_knows_binding_modules_only():
    assert audit.bound_index("trajopt_ifopt")["Var"] == "trajopt_ifopt::Var"
    assert audit.bound_index("snapshot_owner") == {}  # a fixture stub, no binding source


def test_cpp_api_sets_qualified_on_classes_and_enums():
    tu = audit.parse_tu(FIXTURES / "identity_bindings.cpp", (FIXTURE_INCLUDE,))
    header = (FIXTURE_INCLUDE / "tesseract" / "identity" / "widgets.h").resolve()
    cpp = audit.cpp_api(tu, frozenset({header}))
    assert cpp["LongWidgetName"].qualified == "tesseract::identity::LongWidgetName"
    assert cpp["LongWidgetName.Part"].qualified == "tesseract::identity::LongWidgetName.Part"
    assert cpp["Gadget"].qualified == "tesseract::identity::Gadget"
    assert cpp["Color"].qualified == "tesseract::identity::Color"
    assert cpp["LongWidgetName.size"].qualified == ""


def test_cpp_api_is_the_same_on_a_tu_with_function_bodies():
    """One TU may feed both `cpp_api` and `bound_classes`: bodies add no API symbol."""
    cpp = audit.binding_source("tesseract_time_parameterization")
    plain, bodies = audit.parse_tu(cpp), audit.parse_tu(cpp, bodies=True)

    def owned(tu):
        return frozenset(
            Path(i.include.name).resolve()
            for i in tu.get_includes()
            if "tesseract/time_parameterization/" in i.include.name
        )

    assert owned(plain) and owned(plain) == owned(bodies)
    assert audit.cpp_api(plain, owned(plain)) == audit.cpp_api(bodies, owned(bodies))
    fixture = FIXTURES / "identity_bindings.cpp"
    header = frozenset({(FIXTURE_INCLUDE / "tesseract" / "identity" / "widgets.h").resolve()})
    assert audit.cpp_api(audit.parse_tu(fixture, (FIXTURE_INCLUDE,)), header) == audit.cpp_api(
        audit.parse_tu(fixture, (FIXTURE_INCLUDE,), bodies=True), header
    )


def _widget_cpp():
    method = [audit.CppOverload(audit.Arity(0, 0), (), f"{HERE}:3")]
    return {
        "LongWidgetName": audit.CppSymbol(
            "LongWidgetName", audit.Kind.CLASS, f"{HERE}:1", qualified="tesseract::w::LongWidgetName"
        ),
        "LongWidgetName.size": audit.CppSymbol("LongWidgetName.size", audit.Kind.METHOD, "", method),
        "LongWidgetName.color": audit.CppSymbol("LongWidgetName.color", audit.Kind.METHOD, "", method),
    }  # fmt: skip


def _renamed_report(**kw):
    stub = RENAMED / "_renamed.pyi"
    py = audit.py_api(audit.load_stub(stub), audit.rel(stub))
    bound = {"renamed": {"Gizmo": "tesseract::w::LongWidgetName"}}
    return audit.match(
        "renamed", _widget_cpp(), frozenset({"size", "color"}), py, audit.rel(stub),
        stub_roots=(RENAMED,), bound=lambda m: bound.get(m, {}), **kw,
    )  # fmt: skip


def test_renamed_class_members_are_compared_against_the_python_class():
    report = _renamed_report()
    assert {g.symbol for g in report.gaps} == {"LongWidgetName.color"}
    assert {d.name for d in report.deviations} == {"Gizmo"}  # the rename itself stays visible
    assert report.covered == 2  # LongWidgetName as Gizmo, LongWidgetName.size as Gizmo.size


def test_renamed_class_accepted_by_rule(monkeypatch):
    monkeypatch.setitem(audit.ACCEPTED_SYMBOLS, ("renamed", "Gizmo"), "template-instance-name")
    report = _renamed_report()
    assert not report.deviations
    assert ("Gizmo", "template-instance-name") in {(a.symbol, a.rule) for a in report.accepted}


def test_renamed_class_python_only_overload_is_a_deviation():
    """The deviation loop maps `Gizmo.size` back to `LongWidgetName.size` to compare overloads."""
    cpp = _widget_cpp()
    unary = [audit.CppOverload(audit.Arity(1, 1), (), f"{HERE}:3")]
    cpp["LongWidgetName.size"] = audit.CppSymbol(
        "LongWidgetName.size", audit.Kind.METHOD, "", unary
    )
    stub = RENAMED / "_renamed.pyi"
    py = audit.py_api(audit.load_stub(stub), audit.rel(stub))
    bound = {"renamed": {"Gizmo": "tesseract::w::LongWidgetName"}}
    report = audit.match(
        "renamed", cpp, frozenset({"size", "color"}), py, audit.rel(stub),
        stub_roots=(RENAMED,), bound=lambda m: bound.get(m, {}),
    )  # fmt: skip
    assert ("Gizmo.size", audit.Kind.OVERLOAD) in {(d.name, d.kind) for d in report.deviations}


def _snapshot(qualified, bound):
    here = "tests/audit/fixtures/bound_elsewhere/snapshot.h"
    cpp = {
        "Snapshot": audit.CppSymbol("Snapshot", audit.Kind.CLASS, f"{here}:1", qualified=qualified)
    }
    stub = BOUND_ELSEWHERE / "_snapshot_header.pyi"
    py = audit.py_api(audit.load_stub(stub), audit.rel(stub))
    return audit.match(
        "snapshot_header", cpp, frozenset(), py, audit.rel(stub),
        stub_roots=(BOUND_ELSEWHERE,), bound=lambda m: bound.get(m, {}),
    )  # fmt: skip


def test_same_leaf_different_class_is_not_bound_elsewhere():
    """TF-mp-plan-2: sco::Var is not trajopt_ifopt::Var."""
    report = _snapshot("a::Snapshot", {"snapshot_owner": {"Snapshot": "b::Snapshot"}})
    assert ("Snapshot", audit.Kind.CLASS) in {(g.symbol, g.kind) for g in report.gaps}


def test_own_rename_wins_over_a_leaf_hit_in_another_stub():
    """T3 rule 2: `_renamed_elsewhere.pyi` declares `class LongWidgetName`, but this module binds
    `tesseract::w::LongWidgetName` as `Gizmo`: its members are compared against `Gizmo`."""
    assert (RENAMED / "_renamed_elsewhere.pyi").is_file()
    report = _renamed_report()
    accepted = {(a.symbol, a.rule) for a in report.accepted}
    assert ("LongWidgetName", "bound-in-other-module") not in accepted
    assert "LongWidgetName.color" in {g.symbol for g in report.gaps}


@pytest.mark.parametrize("bound", [{"snapshot_owner": {"Snapshot": "a::Snapshot"}}, {}])
def test_same_class_or_no_evidence_stays_bound_elsewhere(bound):
    report = _snapshot("a::Snapshot", bound)
    assert ("Snapshot", "bound-in-other-module") in {(a.symbol, a.rule) for a in report.accepted}
