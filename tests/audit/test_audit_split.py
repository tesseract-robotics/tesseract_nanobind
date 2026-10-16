"""Contract tests of the audit's module split (PR 0b, #256)."""

import ast
import sys
from pathlib import Path

import pytest

from tests.audit._load import SCRIPTS, audit  # noqa: F401  (loads every script)

SCRIPTS_DIR = Path(__file__).resolve().parent.parent.parent / "scripts"


def defined_names(module: str) -> set[str]:
    """Top-level names a script defines itself (defs, classes, assignments), not imports."""
    tree = ast.parse((SCRIPTS_DIR / f"{module}.py").read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            names |= {t.id for t in node.targets if isinstance(t, ast.Name)}
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
    return names


# Grows with each extraction (Tasks 2-5) to the final import order:
# audit_types, audit_ownership, audit_parse, audit_stub, audit_rules, audit_match, audit_report,
# audit_bindings.
EXPECTED_SCRIPTS = (
    "audit_types",
    "audit_ownership",
    "audit_parse",
    "audit_stub",
    "audit_rules",
    "audit_match",
    "audit_report",
    "audit_bindings",
)


def test_every_script_is_loaded_once():
    assert SCRIPTS == EXPECTED_SCRIPTS
    assert all(name in sys.modules for name in SCRIPTS)


def test_combined_namespace_rejects_two_definitions_of_one_name():
    from tests.audit._load import DuplicateNameError, combine

    a, b = type(sys)("a"), type(sys)("b")
    a.x, b.x = object(), object()
    with pytest.raises(DuplicateNameError, match="x"):
        combine([a, b])


PARSE = {
    "CONDA_PREFIX", "CONDA_INCLUDE", "LIBCLANG_MAJOR", "CLANG_RESOURCE_DIR", "CXX_STD",
    "IncludeDirError", "versioned_include", "INCLUDE_DIRS", "PARSE_ARGS", "PARSE_OPTIONS",
    "MAX_REPORTED_DIAGNOSTICS", "HeaderParseError", "parse_tu", "STRINGSTREAM", "CEREAL_HOOKS",
    "ALLOCATION_OPERATORS", "OPERATOR_DUNDERS", "OSTREAM_DECL", "FOREIGN_NAMESPACES",
    "FUNCTION_KINDS", "METHOD_KINDS", "FIELD_KINDS", "RAW_BUFFER_POINTEES",
    "SIZE_SPELLINGS", "YAML_NODE", "BYTES_KEYS", "Kind", "Arity", "CppOverload", "CppSymbol",
    "header_rel", "location", "audited_headers", "_owns", "prefix_headers",
    "direct_include_spellings", "binding_module", "foreign_headers",
    "_is_eigen_ref_of_non_const", "_is_out_param", "_out_param_type", "_is_raw_buffer",
    "_is_byte_pointer", "_byte_buffer_pairs", "_has_default", "_is_yaml_node",
    "_same_python_form", "_stream_insertion_owner", "_befriends_cereal", "cpp_api", "decl_names",
    # 0b Task 6 (class identity)
    "BODY_PARSE_OPTIONS", "NB_CLASS_TEMPLATES", "NB_CLASS_CTORS", "NB_MODULE_SCOPE",
    "NB_MODULE_INIT_PREFIX", "SkippedBodiesError", "_in_main_file", "_scope_type", "bound_classes",
    # 0b Task 7 (inherited members)
    "OWNED_ROOTS", "_owned_headers", "_is_ctor", "_member", "_declared_leaves",
    "_base_definition", "_overload", "own_header_lines",
    "TYPE_INDEX", "_is_reference_wrapper_vector",  # #256 group 1
    # G2 (R13)
    "UNBOUND_DEPENDENCY_ROOTS", "UNBOUND_DEPENDENCY", "_names_unbound_dependency",
    # G4 (forwarding ctor templates)
    "REFERENCE_TYPE_KINDS", "_is_forwarding_ctor_template",
}  # fmt: skip


def test_parse_defines_exactly():
    assert defined_names("audit_parse") == PARSE
    assert not defined_names("audit_match") & PARSE


STUB = {
    "StubMissingError", "load_stub", "ENUM_BASES", "DUNDER_OPERATORS", "NOT_AN_OVERLOAD",
    "TRY_IMPORT", "PyOverload", "PySymbol", "QuotedType", "Deviation", "Accepted", "PyApi",
    "rel", "_decorators", "_is_setter", "_py_arity", "_py_param_nodes", "_function_kind",
    "_annotations", "_quoted_names", "_enum_member", "py_api", "_catches_import_error",
    "INIT_HOOKS", "init_findings", "init_accepted",
}  # fmt: skip


def test_stub_defines_exactly():
    assert defined_names("audit_stub") == STUB
    assert not defined_names("audit_match") & STUB


def test_cli_defines_only_main():
    # `resolve_module` stays in audit_match: `audit_module` calls it, and audit_match must not
    # import the CLI (layer order).
    assert defined_names("audit_bindings") == {"main"}


# `_gap_arity` joins this set when #251 lands (0b plan Task 0 table).
REPORT = {
    "REPORT_PATH", "LIMITATION", "_git", "libclang_version", "provenance", "_table",
    "render_markdown", "to_json", "_unaudited_patterns",
}  # fmt: skip


def test_report_defines_exactly():
    assert defined_names("audit_report") == REPORT
    assert not defined_names("audit_match") & REPORT


# `ACCEPTED_DEFAULTS` joins this set when #251 lands.
RULES = {
    "PROTOCOL_MEMBERS", "PROTOCOL_RULES", "ACCEPTED", "ACCEPTED_SYMBOLS",
    # #256 group 1
    "TEMPLATE_INSTANCES", "TEMPLATE_INSTANCE_RULE", "ACCEPTED_CPP_OVERLOADS",
    "ACCEPTED_PY_OVERLOADS", "POLY_ALTERNATIVES", "WHOLE_SYMBOL_ABSENT_OK",
    "VECTOR_FORWARDING_MEMBERS", "VECTOR_FORWARDER_MEMBERS", "UnknownAcceptedRuleError",
    "_check_rule_keys",
    # G2 (T10, R15)
    "K_ENUMERATOR", "CAMEL_BOUNDARY", "k_enumerator_python_name", "_k_alias",
}  # fmt: skip


def test_rules_defines_exactly():
    assert defined_names("audit_rules") == RULES
    assert not defined_names("audit_match") & RULES


MATCH = {
    "PrefixMismatchError", "resolve_module", "Gap", "ModuleReport", "Cover", "_fits", "_upcasts_to",
    "_derived_element_sibling", "_covered", "_cover", "_protocol_rule", "_stub_py_api",
    "_base_stub", "_py_member", "_binding_stub", "_stubs_under", "_ancestor_missing", "match",
    "unincluded_header_gaps", "_header_gaps", "audit_tu", "audit_module",
    # 0b Task 6 (class identity)
    "BoundLookup", "NO_ALIAS", "no_bound", "_bound_classes_of", "bound_index", "_same_class",
    "_renamed",
    # 0b Task 8 (stub bases)
    "_stub_base", "_base_rows",
    # 0b Task 7 (inherited members)
    "_inherited_from_bound",
    # #256 group 1
    "poly_ancestors", "_absent_ok_accepts", "_whole_symbol_rule", "_bound_as_method",
    "_unbound_rule", "_uncovered_rule",
    # G3 T1: the yaml-node premise.
    "YAML_TEXT_KEYS", "_yaml_sibling",
}  # fmt: skip


def test_match_defines_exactly():
    assert defined_names("audit_match") == MATCH


@pytest.mark.parametrize("module", SCRIPTS)
def test_each_script_stays_below_1k_lines(module):
    assert len((SCRIPTS_DIR / f"{module}.py").read_text().splitlines()) < 1000
