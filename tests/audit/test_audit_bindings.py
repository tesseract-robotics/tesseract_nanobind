"""Contract tests of scripts/audit_bindings.py.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
The default env ignores this directory via `addopts`.
"""

import importlib.util
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
