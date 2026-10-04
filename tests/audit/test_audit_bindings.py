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
    assert {h.name for h in headers} == {"widget.h"}


def test_cpp_symbols_exact_set(fixture_cpp):
    assert set(fixture_cpp) == {
        "Base", "Base.run", "Base.__init__",
        "Owner", "Owner.__init__",
        "Plain", "Plain.__init__", "Plain.x",
        "Widget", "Widget.__init__", "Widget.size", "Widget.resize", "Widget.__eq__",
        "Widget.operator+", "Widget.__bool__", "Widget.owner", "Widget.count",
        "Color", "Color.RED", "Color.GREEN",
        "scale", "area", "collect", "describe",
    }  # fmt: skip


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
