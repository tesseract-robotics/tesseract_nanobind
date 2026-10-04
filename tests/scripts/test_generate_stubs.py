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
# `std::__cxx11::`), so any such leak makes the stubs platform-dependent.
STD_TYPE_LEAK = re.compile(r'"[^"\n]*\bstd::[^"\n]*"')


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
    leaks = STD_TYPE_LEAK.findall(_mod.stub_path(module).read_text(encoding="utf-8"))
    assert not leaks, f"unconverted std:: types in {module}: {leaks}"


def test_continuous_manager_geometry_getters_declared():
    """gh-157."""
    text = _mod.stub_path("tesseract_robotics.tesseract_collision._tesseract_collision").read_text(
        encoding="utf-8"
    )
    body = text.split("class ContinuousContactManager:", 1)[1].split("\nclass ", 1)[0]
    assert "def getCollisionObjectGeometries(" in body
    assert "def getCollisionObjectGeometriesTransforms(" in body
