"""Contract tests of scripts/audit_bindings.py.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
The default env ignores this directory via `addopts`.
"""

import ast
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


# Phase C: the tesseract core modules and their header prefixes.
CORE_PREFIX = {
    "tesseract_geometry": "tesseract/geometry/",
    "tesseract_scene_graph": "tesseract/scene_graph/",
    "tesseract_state_solver": "tesseract/state_solver/",
    "tesseract_srdf": "tesseract/srdf/",
    "tesseract_urdf": "tesseract/urdf/",
    "tesseract_kinematics": "tesseract/kinematics/",
}


@pytest.mark.parametrize("module", sorted(CORE_PREFIX))
def test_core_binding_tu_parses_clean(module):
    """No error diagnostics in the binding TU itself."""
    tu = audit.parse_tu(REPO_ROOT / "src" / module / f"{module}_bindings.cpp")
    assert tu.cursor is not None


@pytest.mark.parametrize("module", sorted(CORE_PREFIX))
def test_core_unincluded_headers_parse_clean(module):
    """E0's synthetic TU (every unincluded, non-unaudited header) parses: one error aborts the module."""
    prefix = CORE_PREFIX[module]
    cpp = REPO_ROOT / "src" / module / f"{module}_bindings.cpp"
    tu = audit.parse_tu(cpp)
    included = {Path(i.include.name).resolve() for i in tu.get_includes()}
    headers = {
        h: s
        for h, s in audit.prefix_headers(prefix, audit.INCLUDE_DIRS).items()
        if h not in included and not audit.is_unaudited(s, prefix)
    }
    audit.unincluded_header_gaps(cpp, headers)  # raises HeaderParseError on any error


@pytest.mark.parametrize("module", sorted(CORE_PREFIX))
def test_core_modules_resolve(module):
    assert audit.resolve_module(module) == module
    assert audit.AUDITED_HEADER_PREFIX[module] == CORE_PREFIX[module]


@pytest.mark.parametrize("module", sorted(CORE_PREFIX))
def test_core_module_audits(module):
    """Every mapped core module produces a report (no parse error, stub found)."""
    assert audit.audit_module(module).covered > 0


def test_serialization_audits_the_serialization_headers_it_includes():
    """tesseract_serialization has no header directory of its own: it audits the foreign
    headers it is first to #include, and tesseract_common lists serialization.h as audited there."""
    common = audit.audit_module("tesseract_common")
    assert common.delegated["tesseract/common/serialization.h"] == "tesseract_serialization"
    report = audit.audit_module("tesseract_serialization")
    assert ("Serialization", "tesseract/common/serialization.h") in {
        (g.symbol, g.location.rsplit(":", 1)[0]) for g in report.gaps
    }


@pytest.fixture(scope="module")
def core_reports():
    return {m: audit.audit_module(m) for m in [*CORE_PREFIX, "tesseract_serialization"]}


@pytest.mark.parametrize(
    ("module", "symbol", "rule"),
    [
        # G2: Mesh/SDFMesh befriend cereal through their base PolygonMesh.
        ("tesseract_geometry", "Mesh.__init__", "serialization-default-ctor"),
        ("tesseract_geometry", "SDFMesh.__init__", "serialization-default-ctor"),
        # G8 + Z1: one Python function per instance of a C++ template.
        ("tesseract_geometry", "createConvexMeshFromPath", "template-instance-name"),
        ("tesseract_geometry", "createSDFMeshFromResource", "template-instance-name"),
        ("tesseract_serialization", "environment_to_xml", "template-instance-name"),
        ("tesseract_serialization", "scene_state_from_binary", "template-instance-name"),
        # S4: the vector fields convert to a fresh list on each access.
        ("tesseract_scene_graph", "Link.addVisual", "copied-vector-field-mutator"),
        ("tesseract_scene_graph", "Link.clearCollision", "copied-vector-field-mutator"),
        # S6: boost graph property tags.
        ("tesseract_scene_graph", "vertex_link_t", "boost-graph-property-tag"),
        ("tesseract_scene_graph", "property_kind", "boost-graph-property-tag"),
        # S7: SceneState is bound in tesseract_state_solver.
        ("tesseract_scene_graph", "SceneState", "bound-in-other-module"),
    ],
)
def test_core_rows_accepted_by_phase_c_rules(core_reports, module, symbol, rule):
    report = core_reports[module]
    assert (symbol, rule) in {(a.symbol, a.rule) for a in report.accepted}
    if rule == "serialization-default-ctor":
        # Only the arity-0 overload is accepted; the full constructor stays a gap (G2).
        assert (symbol, "0") not in {(g.symbol, g.arity) for g in report.gaps}
        return
    assert symbol not in {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


@pytest.mark.parametrize(
    ("module", "header"),
    [
        ("tesseract_kinematics", "tesseract/kinematics/ikfast/ikfast_inv_kin.h"),
        ("tesseract_kinematics", "tesseract/kinematics/kdl/kdl_fwd_kin_chain.h"),
        ("tesseract_kinematics", "tesseract/kinematics/opw/opw_inv_kin.h"),
        ("tesseract_kinematics", "tesseract/kinematics/ur/ur_inv_kin.h"),
        ("tesseract_kinematics", "tesseract/kinematics/rep_inv_kin.h"),
        ("tesseract_kinematics", "tesseract/kinematics/rop_factory.h"),
        ("tesseract_state_solver", "tesseract/state_solver/ofkt/ofkt_nodes.h"),
    ],
)
def test_core_plugin_and_internal_headers_unaudited(core_reports, module, header):
    """K7, SS4: solver plugin implementations and OFKT tree internals are not Python API."""
    assert header not in {g.symbol for g in core_reports[module].gaps}


def test_ofkt_node_class_unaudited(core_reports):
    assert "OFKTNode" not in {g.symbol for g in core_reports["tesseract_state_solver"].gaps}


def test_state_solver_kdl_still_audited(core_reports):
    """K7's `kdl/*` pattern must not hide `KDLStateSolver`, which the binding includes directly."""
    names = {g.symbol for g in core_reports["tesseract_state_solver"].gaps}
    assert any(n.startswith("KDLStateSolver.") for n in names)


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
        audit.resolve_module("ompl_base")


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
        "RemoteRunner",
        "RemoteRunner.__init__",
        "RemoteRunner.go",
        "RemoteRunner.__call__",
        "LostRunner",
        "LostRunner.__init__",
        "LostRunner.go",
        "LostRunner.__call__",
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
        "Level",
        "Level.LEVEL_LOW",
        "Level.LEVEL_HIGH",
        "scale",
        "area",
        "collect",
        "describe",
        "flatten",
        "fill",
        "shift",
        "norm",
        "trace",
        "Bag",
        "Bag.__init__",
        "Bag.size",
        "Bag.__getitem__",
        "Bag.begin",
        "Bag.end",
        "Bag.__mul__",  # templated operator* (M13)
        "Bag.__str__",  # free operator<<(std::ostream&, const Bag&) (M5)
        "Record",
        "Record.__init__",
        "Buffer",
        "Buffer.__init__",  # incl. the constructor template, not a "Buffer.Buffer" method
    }  # fmt: skip  (no std::hash (I1), no free serialize (I2), no Detail (C1))


def test_constructor_template_is_a_constructor(fixture_cpp):
    arities = sorted(str(o.arity) for o in fixture_cpp["Buffer.__init__"].overloads)
    assert arities == ["2", "2-3", "3-4"]


def test_raw_buffer_overload_recorded(fixture_cpp):
    raw = [o for o in fixture_cpp["Buffer.__init__"].overloads if o.raw_buffer]
    assert [str(o.arity) for o in raw] == ["3-4"]


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


@pytest.mark.parametrize(
    ("symbol", "n_out"), [("fill", 1), ("shift", 1), ("norm", 0), ("trace", 0)]
)
def test_eigen_ref_of_non_const_is_an_out_param(fixture_cpp, symbol, n_out):
    """gh-213: a by-value `Eigen::Ref<T>` with non-const T is writable, like a non-const `T&`."""
    [ov] = fixture_cpp[symbol].overloads
    assert len(ov.out_params) == n_out


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


def test_init_module_getattr_is_accepted():
    """A PEP 562 module `__getattr__` (a lazy re-export, gh-218) is a lookup hook, not API."""
    rows = {(a.symbol, a.rule) for a in audit.init_accepted(FIXTURES / "package_init.py")}
    assert rows == {("__getattr__", "module-getattr")}
    assert audit.ACCEPTED["module-getattr"]


def test_fixture_report_accepts_module_getattr(fixture_report):
    assert ("__getattr__", "module-getattr") in {
        (a.symbol, a.rule) for a in fixture_report.accepted
    }
    assert "__getattr__" not in {d.name for d in fixture_report.deviations}


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
        stub_roots=(FIXTURES,),
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
        # raw-buffer ctor: the bound arity-2-3 ctor overlaps it but cannot cover it (#166)
        ("Buffer.__init__", audit.Kind.CONSTRUCTOR, "3-4"),
        # stub base in a module without a stub: inherited members cannot be checked (I7)
        ("LostRunner.go", audit.Kind.METHOD, "—"),
        ("LostRunner.__call__", audit.Kind.OPERATOR, "—"),
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


def test_header_owned_by_another_binding_is_listed_as_delegated(fixture_report):
    """Phase C: the prefix owner names the module that audits shared.h, so no header is silent."""
    assert fixture_report.delegated == {"tesseract/fixture/shared.h": "other"}


def test_including_binding_audits_a_foreign_header_its_owner_does_not_include():
    """Phase C: other_bindings.cpp #includes shared.h, which fixture_bindings.cpp (its prefix
    owner) does not, so the `other` module audits it: the unbound `Shared` is a gap there."""
    report = audit.audit_tu(
        "other",
        FIXTURES / "other_bindings.cpp",
        FIXTURES / "_other.pyi",
        FIXTURES / "package_init.py",
        "tesseract/other/",
        (FIXTURE_INCLUDE,),
        other_bindings=(FIXTURES / "fixture_bindings.cpp",),
        stub_roots=(FIXTURES,),
        prefixes={"fixture": FIXTURE_PREFIX, "other": "tesseract/other/"},
    )
    assert ("Shared", audit.Kind.CLASS) in {(g.symbol, g.kind) for g in report.gaps}


def test_foreign_header_auditor_is_the_first_including_module():
    """Several bindings include the same foreign header: exactly one audits it."""
    assert audit.header_auditor(["tesseract_task_composer", "tesseract_command_language"]) == (
        "tesseract_command_language"
    )


def test_unaudited_header_pattern_is_never_parsed(fixture_report):
    """E0: test_suite/broken_unit.hpp has a syntax error; matching UNAUDITED_HEADERS skips it."""
    assert not [g for g in fixture_report.gaps if "broken_unit" in g.symbol]


def test_yaml_only_header_has_no_row(fixture_report):
    """PE-C4: `YAML::convert` specialisations are yaml-cpp's namespace, not module API."""
    assert not [g for g in fixture_report.gaps if "yaml_convert" in g.symbol]


def test_third_party_header_never_audited_even_when_included_directly(fixture_report):
    """#179: the binding #includes vhacd/VHACD.h directly to switch its implementation off;
    the vendored library is still not module API."""
    assert not [g for g in fixture_report.gaps if "VHACD" in g.symbol]


def test_third_party_header_patterns_rendered_with_reasons(fixture_report):
    text = audit.render_markdown([fixture_report], PROV)
    assert audit.THIRD_PARTY_HEADERS
    for pattern, reason in audit.THIRD_PARTY_HEADERS.items():
        assert f"`{pattern}`: {reason}" in text


def test_hacd_header_has_its_own_reason():
    """Q1: declared but not built in 0.35.0, stated apart from the Bullet internals."""
    assert "not built" in audit.UNAUDITED_HEADERS.get("bullet/convex_decomposition_hacd.h", "")


def test_unaudited_header_patterns_rendered_with_reasons(fixture_report):
    text = audit.render_markdown([fixture_report], PROV)
    for pattern, reason in audit.UNAUDITED_HEADERS.items():
        assert f"`{pattern}`: {reason}" in text


def test_limitations_say_overloads_match_by_arity_only(fixture_report):
    """Note 2: an overload with a bound overload of the same arity is never reported."""
    text = audit.render_markdown([fixture_report], PROV)
    assert "Overloads are matched by arity only" in text


def test_fixture_deviations_exact(fixture_report):
    assert {(d.name, d.kind, d.arity) for d in fixture_report.deviations} == {
        ("Widget.widget_grow", audit.Kind.METHOD, "—"),
        ("Widget.widget_samples", audit.Kind.METHOD, "—"),
        ("Gadget.__len__", audit.Kind.PROTOCOL, "—"),  # C++ Gadget has no size()
        ("Gadget.__hash__", audit.Kind.FIELD, "—"),  # unhashable without a bound __eq__
        ("Color_RED", audit.Kind.CONSTANT, "—"),
        ("LEVEL_LOW", audit.Kind.CONSTANT, "—"),  # aliases an unscoped enum value (Note 1)
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
        ("fill", "out-param"),  # Eigen::Ref<MatrixXd> out-param returned (gh-213)
        ("Bag.__len__", "container-protocol"),  # C14
        ("Bag.__setitem__", "container-protocol"),  # M18
        ("Bag.__iter__", "iterator-pair"),  # C6
        ("Bag.__str__", "stream-insertion"),  # M5
        ("Widget.__repr__", "presentation-dunder"),  # M12
        ("Record.__init__", "serialization-default-ctor"),  # E2
        ("Widget.__hash__", "value-equality-unhashable"),
        ("__getattr__", "module-getattr"),  # package_init.py, gh-218
    }


@pytest.mark.parametrize(
    ("hash_line", "accepted"),
    [("__hash__: None = None", True), ("__hash__: int", False)],
)
def test_only_a_none_hash_next_to_eq_is_value_equality_unhashable(hash_line, accepted):
    stub = f"class V:\n    def __eq__(self, other: V) -> bool: ...\n    {hash_line}\n"
    py = audit.py_api(ast.parse(stub), "v.pyi")
    report = audit.match("v", {}, frozenset(), py, "v.pyi")
    rows = {(a.symbol, a.rule) for a in report.accepted}
    assert (("V.__hash__", "value-equality-unhashable") in rows) is accepted
    assert ("V.__hash__" in {d.name for d in report.deviations}) is not accepted


def test_container_members_covered_by_protocol_dunders(fixture_report):
    """C14/C6: size and begin/end are covered by __len__/__iter__, neither gap nor row."""
    assert not {g.symbol for g in fixture_report.gaps} & {"Bag.size", "Bag.begin", "Bag.end"}


def test_stream_insertion_operator_keyed_on_its_class(fixture_cpp):
    """M5: `operator<<(std::ostream&, const Bag&)` is Bag.__str__, arity 0."""
    assert "operator<<" not in fixture_cpp
    assert [
        str(ov.arity)
        for ov in fixture_cpp.get(
            "Bag.__str__", audit.CppSymbol("", audit.Kind.OPERATOR, "")
        ).overloads
    ] == ["0"]


def test_new_accepted_rules_rendered_with_reasons(fixture_report):
    text = audit.render_markdown([fixture_report], PROV)
    for rule in (
        "container-protocol",
        "iterator-pair",
        "stream-insertion",
        "presentation-dunder",
        "serialization-default-ctor",
        "value-equality-unhashable",
    ):
        assert audit.ACCEPTED.get(rule), rule  # a non-empty reason
        assert f"`{rule}`: {audit.ACCEPTED[rule]}" in text


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


def test_eigen_ref_out_params_covered_in_kinematics():
    """gh-213: numericalJacobian's three overloads and ForwardKinematics.calcJacobian :104 return
    their `Eigen::Ref<Eigen::MatrixXd>` out-param, so none is a gap or a deviation."""
    report = audit.audit_module("tesseract_kinematics")
    names = {"numericalJacobian", "ForwardKinematics.calcJacobian"}
    assert [g for g in report.gaps if g.symbol in names] == []
    assert [d for d in report.deviations if d.name in names] == []
    rules = sorted((a.symbol, a.rule) for a in report.accepted if a.symbol in names)
    assert (
        rules
        == [("ForwardKinematics.calcJacobian", "out-param")]
        + [("numericalJacobian", "out-param")] * 3
    )


def test_satisfies_limits_all_overloads_covered(real_reports):
    report = real_reports["tesseract_common"]
    assert [g for g in report.gaps if g.symbol == "satisfiesLimits"] == []
    assert [d for d in report.deviations if d.name == "satisfiesLimits"] == []


def test_quaternion_scalar_last_accepted(real_reports):
    accepted = {(a.symbol, a.rule) for a in real_reports["tesseract_common"].accepted}
    assert ("Quaterniond.from_xyzw", "scalar-last-quaternion") in accepted


@pytest.mark.parametrize(
    ("module", "symbol", "rule"),
    [
        ("tesseract_collision", "ContactResultMap.__len__", "container-protocol"),
        ("tesseract_collision", "ContactResultVector.__len__", "container-protocol"),
        ("tesseract_common", "VectorVector3d.__setitem__", "container-protocol"),
        ("tesseract_common", "Isometry3d.__repr__", "presentation-dunder"),
        ("tesseract_common", "SimpleLocatedResource.__init__", "serialization-default-ctor"),
        ("tesseract_environment", "AddLinkCommand.__init__", "serialization-default-ctor"),
        ("tesseract_common", "ContactManagersPluginInfo.__hash__", "value-equality-unhashable"),
        ("tesseract_common", "TaskComposerPluginInfo.__hash__", "value-equality-unhashable"),
        ("tesseract_common", "ProfilesPluginInfo.__hash__", "value-equality-unhashable"),
        (
            "tesseract_environment",
            "AddContactManagersPluginInfoCommand.__hash__",
            "value-equality-unhashable",
        ),
        ("tesseract_common", "Hyperplane3d", "eigen-template-instance"),
        ("tesseract_common", "ParametrizedLine3d", "eigen-template-instance"),
        ("tesseract_common", "Quaterniond.from_rpy", "quaternion-rpy"),
        ("tesseract_common", "Quaterniond.to_rpy", "quaternion-rpy"),
        ("tesseract_common", "EIGEN_DEFAULT_PREC", "eigen-default-precision"),
    ],
)
def test_real_rows_accepted_by_new_rules(real_reports, module, symbol, rule):
    report = real_reports[module]
    assert (symbol, rule) in {(a.symbol, a.rule) for a in report.accepted}
    assert symbol not in {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


@pytest.mark.parametrize(
    "symbol", ["Isometry3d.__mul__", "Quaterniond.__mul__", "Translation3d.__mul__"]
)
def test_eigen_templated_mul_is_not_a_deviation(real_reports, symbol):
    """M13: Eigen declares `operator*` as a member template; `__mul__` is its binding."""
    assert symbol not in {d.name for d in real_reports["tesseract_common"].deviations}


@pytest.mark.parametrize(
    ("module", "header"),
    [
        ("tesseract_collision", "tesseract/collision/yaml_extensions.h"),  # PE-C4
        ("tesseract_common", "tesseract/common/yaml_extensions.h"),  # PE-M9
        ("tesseract_common", "tesseract/common/cereal_make_array.h"),  # PE-M8
        ("tesseract_common", "tesseract/common/serialization_extensions.h"),
        ("tesseract_common", "tesseract/common/sfinae_utils.h"),
        ("tesseract_common", "tesseract/common/unit_test_utils.h"),
        ("tesseract_environment", "tesseract/environment/environment_cache.h"),  # PE-E4
        ("tesseract_environment", "tesseract/environment/environment_monitor.h"),  # PE-E5
        ("tesseract_environment", "tesseract/environment/environment_monitor_interface.h"),
    ],
)
def test_post_e0_wont_fix_headers_have_no_row(real_reports, module, header):
    assert header not in {g.symbol for g in real_reports[module].gaps}


def test_raw_pointer_bytes_resource_ctor_stays_a_gap(real_reports):
    """E2 covers only the arity-0 ctor; BytesResource(url, ptr, len, parent) is M3 won't fix."""
    accepted = {(a.symbol, a.rule) for a in real_reports["tesseract_common"].accepted}
    assert ("BytesResource.__init__", "serialization-default-ctor") in accepted
    rows = [
        g for g in real_reports["tesseract_common"].gaps if g.symbol == "BytesResource.__init__"
    ]
    assert [g.arity for g in rows] == ["3-4"]


PROV = {"tesseract-robotics": "==0.35.0", "libclang": "clang version 23", "stubs": "abc1234"}


def test_render_lists_delegated_headers(fixture_report):
    """Phase C: a header audited with another module is named in its owner's section, with
    whether that module is audited yet, so no header is silently dropped."""
    text = audit.render_markdown([fixture_report], PROV)
    assert "| `tesseract/fixture/shared.h` | other | not yet: module unmapped |" in text
    data = json.loads(audit.to_json([fixture_report], PROV))
    assert data["modules"]["fixture"]["delegated"] == {"tesseract/fixture/shared.h": "other"}


def test_render_is_deterministic_and_sorted(fixture_report):
    a = audit.render_markdown([fixture_report], PROV)
    b = audit.render_markdown([fixture_report], PROV)
    assert a == b
    assert "abc1234" in a and "==0.35.0" in a
    gaps = a[a.index("### Gaps") : a.index("### Deviations")]
    gap_rows = [line for line in gaps.splitlines() if line.startswith("| `")]
    assert len(gap_rows) == len(fixture_report.gaps)
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
        "Buffer.__init__",
        "LostRunner.go",
        "LostRunner.__call__",
    }
    assert set(module) == {"covered", "gaps", "deviations", "accepted", "quoted", "delegated"}


def test_cli_rejects_unknown_module():
    with pytest.raises(audit.UnknownModuleError):
        audit.main(["tesseract_nope"])


def test_inherited_bound_members_are_not_gaps(fixture_report):
    """I4: FastRunner.go/__call__ are covered by the stub's base class Runner."""
    assert not {g.symbol for g in fixture_report.gaps} & {"FastRunner.go", "FastRunner.__call__"}


def test_members_inherited_from_another_module_stub_are_not_gaps(fixture_report):
    """I7: RemoteRunner's stub base is `fixture_remote._fixture_remote.Runner`, another
    module's stub; nanobind subclasses inherit across modules, so go/__call__ are covered."""
    gaps = {g.symbol for g in fixture_report.gaps}
    assert not gaps & {"RemoteRunner.go", "RemoteRunner.__call__"}
    assert {"LostRunner.go", "LostRunner.__call__"} <= gaps


def test_call_operator_is_neither_gap_nor_deviation(fixture_report):
    """I5: operator() is bound as __call__."""
    assert "Runner.__call__" not in {g.symbol for g in fixture_report.gaps}
    assert "Runner.__call__" not in {d.name for d in fixture_report.deviations}


def test_common_audits_directly_included_acm():
    """C1: allowed_collision_matrix.h reaches the TU via utils.h before its own #include."""
    tu = audit.parse_tu(audit.binding_source("tesseract_common"))
    headers = audit.audited_headers(tu, "tesseract/common/", audit.INCLUDE_DIRS)
    assert "allowed_collision_matrix.h" in {h.name for h in headers}
