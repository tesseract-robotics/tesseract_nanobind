"""Contract tests of scripts/audit_bindings.py.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
The default env ignores this directory via `addopts`.
"""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"

_spec = importlib.util.spec_from_file_location(
    "audit_bindings", REPO_ROOT / "scripts" / "audit_bindings.py"
)
audit = importlib.util.module_from_spec(_spec)
sys.modules["audit_bindings"] = audit  # dataclasses resolve annotations via sys.modules
_spec.loader.exec_module(audit)

FIRST_PASS = ("tesseract_collision", "tesseract_common", "tesseract_environment")


@pytest.mark.parametrize("module", FIRST_PASS)
def test_real_binding_tu_parses_clean(module):
    """No error diagnostics: a partial AST would silently under-report."""
    tu = audit.parse_tu(REPO_ROOT / "src" / module / f"{module}_bindings.cpp")
    assert tu.cursor is not None


def test_syntax_error_raises_header_parse_error():
    with pytest.raises(audit.HeaderParseError, match="broken_bindings.cpp"):
        audit.parse_tu(FIXTURES / "broken_bindings.cpp")


def test_binding_modules_are_the_23_extension_modules():
    modules = audit.binding_modules()
    assert len(modules) == 23
    assert {"tesseract_collision", "ompl_base", "trajopt_sqp"} <= set(modules)


def test_unknown_module_raises():
    with pytest.raises(audit.UnknownModuleError, match="tesseract_nope"):
        audit.resolve_module("tesseract_nope")


def test_unmapped_module_raises():
    """A2: a binding module without an AUDITED_HEADER_PREFIX entry is not auditable yet."""
    with pytest.raises(audit.UnknownModuleError, match="AUDITED_HEADER_PREFIX"):
        audit.resolve_module("tesseract_geometry")


@pytest.mark.parametrize("module", FIRST_PASS)
def test_first_pass_modules_resolve(module):
    assert audit.resolve_module(module) == module


def test_missing_stub_raises(tmp_path):
    with pytest.raises(audit.StubMissingError, match="missing.pyi"):
        audit.load_stub(tmp_path / "missing.pyi")


FIXTURE_PREFIX = "tesseract/fixture/"


@pytest.fixture(scope="module")
def fixture_tu():
    return audit.parse_tu(FIXTURES / "fixture_bindings.cpp", (FIXTURE_INCLUDE,))


@pytest.fixture(scope="module")
def fixture_cpp(fixture_tu):
    headers = audit.audited_headers(
        fixture_tu, FIXTURE_PREFIX, (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS)
    )
    return audit.cpp_api(fixture_tu, headers)


def test_audited_headers_are_direct_includes_under_prefix(fixture_tu):
    headers = audit.audited_headers(
        fixture_tu, FIXTURE_PREFIX, (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS)
    )
    # gadget.h is first included by widget.h, then by the TU itself (C1);
    # detail.h is only included transitively.
    assert {h.name for h in headers} == {"widget.h", "gadget.h"}


def test_cpp_symbols_exact_set(fixture_cpp):
    assert set(fixture_cpp) == {
        "Base",
        "Base.run",  # abstract: no __init__ (I3)
        "Gadget",
        "Gadget.__init__",  # directly included after a transitive include (C1)
        "Runner",
        "Runner.go",
        "Runner.__call__",  # abstract (I3); operator() -> __call__ (I5)
        "FastRunner",
        "FastRunner.__init__",
        "FastRunner.go",
        "FastRunner.__call__",
        "Owner",
        "Owner.__init__",
        "Plain",
        "Plain.__init__",
        "Plain.x",
        "Widget",
        "Widget.__init__",
        "Widget.size",
        "Widget.resize",
        "Widget.__eq__",
        "Widget.operator+",
        "Widget.__bool__",
        "Widget.owner",
        "Widget.count",
        "Color",
        "Color.RED",
        "Color.GREEN",
        "scale",
        "area",
        "collect",
        "describe",
        "flatten",
    }  # fmt: skip  (no std::hash (I1), no free serialize (I2), no Detail (C1))


def test_constructor_overloads_exclude_copy(fixture_cpp):
    arities = sorted(str(o.arity) for o in fixture_cpp["Widget.__init__"].overloads)
    assert arities == ["0", "1"]


def test_implicit_default_constructor(fixture_cpp):
    [ov] = fixture_cpp["Plain.__init__"].overloads
    assert str(ov.arity) == "0"


def test_defaulted_argument_gives_arity_range_and_redeclaration_dedups(fixture_cpp):
    [ov] = fixture_cpp["scale"].overloads
    assert str(ov.arity) == "1-2"


def test_out_param_excludes_abstract_reference(fixture_cpp):
    [ov] = fixture_cpp["collect"].overloads
    assert (str(ov.arity), len(ov.out_params)) == ("3", 1)
    assert str(ov.arity.reduced(len(ov.out_params))) == "2"


def test_stringstream_out_param_recorded(fixture_cpp):
    [ov] = fixture_cpp["describe"].overloads
    assert ov.out_params == (audit.STRINGSTREAM,)


def test_location_is_header_line(fixture_cpp):
    path, line = fixture_cpp["Widget.resize"].location.rsplit(":", 1)
    assert path == "tests/audit/fixtures/include/tesseract/fixture/widget.h"
    source = (REPO_ROOT / path).read_text(encoding="utf-8").splitlines()
    assert "void resize(int n);" in source[int(line) - 1]


FIXTURE_STUB = FIXTURES / "_fixture.pyi"


@pytest.fixture(scope="module")
def fixture_py():
    return audit.py_api(audit.load_stub(FIXTURE_STUB), audit.rel(FIXTURE_STUB))


def test_py_symbols_and_kinds(fixture_py):
    kinds = {n: s.kind for n, s in fixture_py.symbols.items()}
    assert kinds["Widget"] is audit.Kind.CLASS
    assert kinds["Color"] is audit.Kind.ENUM
    assert kinds["Color.RED"] is audit.Kind.ENUMERATOR
    assert kinds["Widget.count"] is audit.Kind.FIELD  # property; setter folded in
    assert kinds["Widget.__init__"] is audit.Kind.CONSTRUCTOR
    assert kinds["Widget.__eq__"] is audit.Kind.OPERATOR
    assert kinds["Widget.__repr__"] is audit.Kind.PROTOCOL
    assert kinds["Color_RED"] is audit.Kind.CONSTANT
    assert kinds["scale"] is audit.Kind.FUNCTION


def test_py_arity_excludes_self_and_counts_defaults(fixture_py):
    assert [str(o.arity) for o in fixture_py.symbols["Widget.__init__"].overloads] == ["0", "1"]
    assert [str(o.arity) for o in fixture_py.symbols["scale"].overloads] == ["1-2"]
    assert [str(o.arity) for o in fixture_py.symbols["area"].overloads] == ["1", "2"]


def test_py_return_annotation_kept(fixture_py):
    [ov] = fixture_py.symbols["collect"].overloads
    assert ov.returns == "tuple[bool, list[int]]"


def test_quoted_type_found_by_ast(fixture_py):
    assert [(q.name, q.annotation) for q in fixture_py.quoted] == [
        ("Widget.owner", "tesseract::fixture::Owner")
    ]


def test_docstring_with_quoted_cpp_name_is_not_a_quoted_type(fixture_py):
    assert all(q.name != "Widget.size" for q in fixture_py.quoted)


def test_numpy_order_literal_is_not_a_quoted_type(fixture_py):
    assert all(q.name != "Widget.widget_samples" for q in fixture_py.quoted)


def test_init_findings():
    rows = {(d.name, d.kind) for d in audit.init_findings(FIXTURES / "package_init.py")}
    assert rows == {
        ("FilesystemPath", audit.Kind.CLASS),
        ("try: import … except ImportError", audit.Kind.FAIL_LOUD),
    }


@pytest.fixture(scope="module")
def fixture_report():
    return audit.audit_tu(
        "fixture",
        FIXTURES / "fixture_bindings.cpp",
        FIXTURE_STUB,
        FIXTURES / "package_init.py",
        FIXTURE_PREFIX,
        (FIXTURE_INCLUDE,),
        other_bindings=(FIXTURES / "other_bindings.cpp",),
    )


ORPHAN_HEADER = "tesseract/fixture/orphan.h"


def test_fixture_gaps_exact(fixture_report):
    assert {(g.symbol, g.kind, g.arity) for g in fixture_report.gaps} == {
        ("Base", audit.Kind.CLASS, "—"),  # whole class: one row, members not listed
        ("Detail", audit.Kind.CLASS, "—"),  # transitive include under the prefix (E0)
        ("Owner", audit.Kind.CLASS, "—"),
        ("Widget.resize", audit.Kind.METHOD, "—"),
        ("Widget.operator+", audit.Kind.OPERATOR, "—"),
        (ORPHAN_HEADER, audit.Kind.HEADER, "—"),  # never included: one row (E0)
    }


def test_unincluded_header_row_locates_its_first_declaration(fixture_report):
    """E0: the row points at the header's first auditable declaration, not line 1."""
    [row] = [g for g in fixture_report.gaps if g.kind is audit.Kind.HEADER]
    path, line = row.location.rsplit(":", 1)
    assert path == f"tests/audit/fixtures/include/{ORPHAN_HEADER}"
    source = (REPO_ROOT / path).read_text(encoding="utf-8").splitlines()
    assert "struct Orphan" in source[int(line) - 1]


def test_unincluded_header_members_are_not_listed(fixture_report):
    """E0: like a missing class, the missing header is the one row."""
    assert not {g.symbol for g in fixture_report.gaps} & {"Orphan", "orphanHelper"}


def test_forward_declaration_header_has_no_row(fixture_report):
    """E0: fwd.h declares nothing auditable."""
    assert not [g for g in fixture_report.gaps if g.symbol.endswith("fwd.h")]


def test_header_owned_by_another_binding_has_no_row(fixture_report):
    """E0: shared.h is #included directly by other_bindings.cpp, so it is audited there."""
    assert not [g for g in fixture_report.gaps if "Shared" in g.symbol or "shared.h" in g.symbol]


def test_unaudited_header_pattern_is_never_parsed(fixture_report):
    """E0: test_suite/broken_unit.hpp has a syntax error; matching UNAUDITED_HEADERS skips it."""
    assert not [g for g in fixture_report.gaps if "broken_unit" in g.symbol]


def test_unaudited_header_patterns_rendered_with_reasons(fixture_report):
    text = audit.render_markdown([fixture_report], PROV)
    for pattern, reason in audit.UNAUDITED_HEADERS.items():
        assert f"`{pattern}`: {reason}" in text


def test_fixture_deviations_exact(fixture_report):
    assert {(d.name, d.kind, d.arity) for d in fixture_report.deviations} == {
        ("Widget.widget_grow", audit.Kind.METHOD, "—"),
        ("Widget.widget_samples", audit.Kind.METHOD, "—"),
        ("Widget.__repr__", audit.Kind.PROTOCOL, "—"),
        ("Color_RED", audit.Kind.CONSTANT, "—"),
        ("scale_twice", audit.Kind.FUNCTION, "—"),
        ("area", audit.Kind.OVERLOAD, "2"),
        ("FilesystemPath", audit.Kind.CLASS, "—"),
        (audit.TRY_IMPORT, audit.Kind.FAIL_LOUD, "—"),
    }


def test_fixture_accepted_exact(fixture_report):
    assert {(a.symbol, a.rule) for a in fixture_report.accepted} == {
        ("collect", "out-param"),
        ("describe", "stringstream"),
        ("flatten", "out-param"),  # void + one out-param returned directly (I6)
    }


def test_fixture_quoted(fixture_report):
    assert [q.name for q in fixture_report.quoted] == ["Widget.owner"]


def test_every_row_has_a_location(fixture_report):
    rows = [*fixture_report.gaps, *fixture_report.deviations, *fixture_report.accepted]
    assert all(re.search(r"\.(h|pyi|py):\d+$", r.location) for r in rows)


@pytest.fixture(scope="module")
def real_reports():
    return {m: audit.audit_module(m) for m in FIRST_PASS}


def test_contact_trajectory_results_has_zero_gaps(real_reports):
    gaps = [g for g in real_reports["tesseract_collision"].gaps
            if g.symbol.split(".")[0] == "ContactTrajectoryResults"]  # fmt: skip
    assert gaps == []


def test_contact_result_cc_fields_covered(real_reports):
    names = {g.symbol for g in real_reports["tesseract_collision"].gaps}
    assert not {"ContactResult.cc_time", "ContactResult.cc_type"} & names


def test_check_trajectory_covered_via_out_param(real_reports):
    report = real_reports["tesseract_environment"]
    assert [g for g in report.gaps if g.symbol == "checkTrajectory"] == []
    assert [d for d in report.deviations if d.name == "checkTrajectory"] == []
    rules = [a.rule for a in report.accepted if a.symbol == "checkTrajectory"]
    assert rules == ["out-param"] * 4


def test_satisfies_limits_all_overloads_covered(real_reports):
    report = real_reports["tesseract_common"]
    assert [g for g in report.gaps if g.symbol == "satisfiesLimits"] == []
    assert [d for d in report.deviations if d.name == "satisfiesLimits"] == []


def test_environment_init_try_import_reported(real_reports):
    kinds = {d.kind for d in real_reports["tesseract_environment"].deviations}
    assert audit.Kind.FAIL_LOUD in kinds


def test_quaternion_scalar_last_accepted(real_reports):
    accepted = {(a.symbol, a.rule) for a in real_reports["tesseract_common"].accepted}
    assert ("Quaterniond.from_xyzw", "scalar-last-quaternion") in accepted


PROV = {"tesseract-robotics": "==0.35.0", "libclang": "clang version 23", "stubs": "abc1234"}


def test_render_is_deterministic_and_sorted(fixture_report):
    a = audit.render_markdown([fixture_report], PROV)
    b = audit.render_markdown([fixture_report], PROV)
    assert a == b
    assert "abc1234" in a and "==0.35.0" in a
    gaps = a[a.index("### Gaps") : a.index("### Deviations")]
    gap_rows = [line for line in gaps.splitlines() if line.startswith("| `")]
    assert len(gap_rows) == 6
    assert gap_rows == sorted(gap_rows)


def test_render_conclusion_precedes_tables(fixture_report):
    text = audit.render_markdown([fixture_report], PROV)
    assert text.index("| fixture |") < text.index("### Gaps")


def test_json_roundtrip(fixture_report):
    data = json.loads(audit.to_json([fixture_report], PROV))
    assert data["provenance"] == PROV
    module = data["modules"]["fixture"]
    assert {g["symbol"] for g in module["gaps"]} == {
        "Base",
        "Detail",
        "Owner",
        "Widget.resize",
        "Widget.operator+",
        ORPHAN_HEADER,
    }
    assert set(module) == {"covered", "gaps", "deviations", "accepted", "quoted"}


def test_cli_rejects_unknown_module():
    with pytest.raises(audit.UnknownModuleError):
        audit.main(["tesseract_nope"])


def test_inherited_bound_members_are_not_gaps(fixture_report):
    """I4: FastRunner.go/__call__ are covered by the stub's base class Runner."""
    assert not {g.symbol for g in fixture_report.gaps} & {"FastRunner.go", "FastRunner.__call__"}


def test_call_operator_is_neither_gap_nor_deviation(fixture_report):
    """I5: operator() is bound as __call__."""
    assert "Runner.__call__" not in {g.symbol for g in fixture_report.gaps}
    assert "Runner.__call__" not in {d.name for d in fixture_report.deviations}


def test_common_audits_directly_included_acm():
    """C1: allowed_collision_matrix.h reaches the TU via utils.h before its own #include."""
    tu = audit.parse_tu(audit.binding_source("tesseract_common"))
    headers = audit.audited_headers(tu, "tesseract/common/", audit.INCLUDE_DIRS)
    assert "allowed_collision_matrix.h" in {h.name for h in headers}
