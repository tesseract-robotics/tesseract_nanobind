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
# audit_types, audit_ownership, audit_parse, audit_stub, audit_match, audit_report, audit_bindings.
EXPECTED_SCRIPTS = ("audit_types", "audit_ownership", "audit_match")


def test_every_script_is_loaded_once():
    assert SCRIPTS == EXPECTED_SCRIPTS
    assert all(name in sys.modules for name in SCRIPTS)


def test_combined_namespace_rejects_two_definitions_of_one_name():
    from tests.audit._load import DuplicateNameError, combine

    a, b = type(sys)("a"), type(sys)("b")
    a.x, b.x = object(), object()
    with pytest.raises(DuplicateNameError, match="x"):
        combine([a, b])
