"""Contract tests on the committed .pyi stubs.

Stubs are generated, never hand-edited. gh-157: the committed
ContinuousContactManager stub lacked two methods the binding registers, because
nothing compared the committed stubs against the build (gh-159).

That comparison needs nanobind's stubgen, which the wheel test venvs don't have,
so it runs as `pixi run stubs-check` in the CI build jobs. The checks here read
only the committed files and run everywhere.
"""

import ast
import importlib.util
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_SCRIPT = REPO_ROOT / "scripts" / "generate_stubs.py"

_spec = importlib.util.spec_from_file_location("generate_stubs", _SCRIPT)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

MODULES = _mod.discover_modules()

# A C++ type nanobind could not map to Python is emitted as its quoted C++ name.
# Standard-library spellings differ per platform (libc++ `std::__1::`, libstdc++
# `std::__cxx11::`), so any such leak makes the stubs platform-dependent. Eigen
# template names too: MSVC drops the spaces (`Transform<double,3,1,0>`).
PLATFORM_TYPE_LEAK = re.compile(r'"[^"\n]*\b(?:std|Eigen)::[^"\n]*"')

# A tesseract type quoted by its C++ name: the module defining it was not imported
# before stub generation (gh-168).
QUOTED_TESSERACT_TYPE = re.compile(r'"[^"\n]*\btesseract::[^"\n]*"')


def test_modules_discovered():
    assert "tesseract_robotics.tesseract_collision._tesseract_collision" in MODULES


@pytest.mark.parametrize("module", MODULES)
def test_committed_stub_exists(module):
    assert _mod.stub_path(module).is_file(), f"no committed stub for {module}; run `pixi run stubs`"


@pytest.mark.parametrize("module", MODULES)
def test_stub_is_valid_python(module):
    """A binding arg named after a keyword (`"from"_a`) yields a stub that does not parse."""
    path = _mod.stub_path(module)
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


@pytest.mark.parametrize("module", MODULES)
def test_stub_has_no_std_type_leak(module):
    leaks = PLATFORM_TYPE_LEAK.findall(_mod.stub_path(module).read_text(encoding="utf-8"))
    assert not leaks, f"unconverted std::/Eigen:: types in {module}: {leaks}"


def test_continuous_manager_geometry_getters_declared():
    """gh-157."""
    text = _mod.stub_path("tesseract_robotics.tesseract_collision._tesseract_collision").read_text(
        encoding="utf-8"
    )
    body = text.split("class ContinuousContactManager:", 1)[1].split("\nclass ", 1)[0]
    assert "def getCollisionObjectGeometries(" in body
    assert "def getCollisionObjectGeometriesTransforms(" in body


def test_collision_stub_has_no_quoted_tesseract_types():
    """gh-168. Scoped to tesseract_collision: other modules still quote `tesseract::`
    names, which belong to other findings."""
    text = _mod.stub_path("tesseract_robotics.tesseract_collision._tesseract_collision").read_text(
        encoding="utf-8"
    )
    leaks = QUOTED_TESSERACT_TYPE.findall(text)
    assert not leaks, f"quoted tesseract:: types in tesseract_collision: {leaks}"


def test_executor_thread_default_is_machine_independent():
    """`hardware_concurrency()` is evaluated at import; its value must not reach the stub."""
    text = _mod.stub_path(
        "tesseract_robotics.tesseract_task_composer._tesseract_task_composer"
    ).read_text(encoding="utf-8")
    assert "name: str = 'TaskflowExecutor', num_threads: int = ...)" in text


def test_check_reports_unified_diff():
    """A failing drift gate must show what differs, not only which file (CI logs are the only view)."""
    report = _mod.drift_report(Path("pkg/_m.pyi"), "a\nkeep\n", "b\nkeep\n")
    assert "--- committed/pkg/_m.pyi" in report
    assert "+++ rendered/pkg/_m.pyi" in report
    assert "-a\n" in report and "+b\n" in report


def test_binding_sources_match_extension_modules():
    """The audit (scripts/audit_bindings.py, BINDING_GLOB) takes its module list from
    src/*/*_bindings.cpp because it must not import the package; this pins that set
    to the modules the build actually produces."""
    sources = {p.parent.name for p in (REPO_ROOT / "src").glob("*/*_bindings.cpp")}
    built = {m.rsplit(".", 1)[-1].removeprefix("_") for m in MODULES}
    assert sources == built
