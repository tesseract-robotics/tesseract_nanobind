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
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def _load(name: str):
    """`scripts/<name>.py` as module `name`; registered so a sibling script can import it."""
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_widen = _load("widen_implicit_id_stubs")
_mod = _load("generate_stubs")
_own = _load("audit_ownership")

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
    assert "tesseract_robotics.collision" in MODULES


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


@pytest.mark.parametrize("module", MODULES)
def test_stub_id_params_are_widened(module):
    """generate_stubs.py widens `LinkId` / `JointId` / `LinkIdPair` parameters to the
    `str` / `tuple` nanobind converts implicitly; a committed stub is its fixed point."""
    text = _mod.stub_path(module).read_text(encoding="utf-8")
    assert _widen.widen(text) == text


def test_continuous_manager_geometry_getters_declared():
    """gh-157."""
    text = _mod.stub_path("tesseract_robotics.collision").read_text(encoding="utf-8")
    body = text.split("class ContinuousContactManager:", 1)[1].split("\nclass ", 1)[0]
    assert "def getCollisionObjectGeometries(" in body
    assert "def getCollisionObjectGeometriesTransforms(" in body


# Modules whose stubs must name every tesseract type: gh-168 (collision), gh-218 (the core
# modules that use types another extension defines). srdf's last quoted type was
# the unbound CalibrationInfo (gh-217).
QUOTE_FREE_MODULES = [
    "collision",
    "kinematics",
    "serialization",
    "srdf",
    "state_solver",
    "urdf",  # writeMeshToFile takes a PolygonMesh (gh-220)
]

# gh-218: the defining modules each extension imports itself, so that its stub, rendered in
# a fresh interpreter, names their types (SceneGraph, Joint, Link: scene_graph; SceneState:
# state_solver; Environment: environment; CompositeInstruction: command_language).
DEFINING_MODULE_IMPORTS = {
    "kinematics": ["scene_graph", "state_solver"],
    "serialization": [
        "command_language",
        "environment",
        "state_solver",
    ],
    "srdf": ["scene_graph"],
    "state_solver": ["scene_graph"],
    "urdf": ["common", "geometry", "scene_graph"],
}


# Packages that wrap their extension `_<name>`; every other module is itself the extension.
WRAPPED_PACKAGES = frozenset(
    {
        "common",
        "scene_graph",
        "motion_planners",
        "motion_planners_simple",
        "motion_planners_descartes",
        "motion_planners_trajopt_ifopt",
        "trajopt_ifopt",
    }
)


def _extension(package: str) -> str:
    """Dotted name of the compiled module of `package`."""
    if package in WRAPPED_PACKAGES:
        return f"tesseract_robotics.{package}._{package}"
    return f"tesseract_robotics.{package}"


@pytest.mark.parametrize("package", QUOTE_FREE_MODULES)
def test_stub_has_no_quoted_tesseract_types(package):
    """gh-168, gh-218."""
    text = _mod.stub_path(_extension(package)).read_text(encoding="utf-8")
    leaks = QUOTED_TESSERACT_TYPE.findall(text)
    assert not leaks, f"quoted tesseract:: types in {package}: {leaks}"


@pytest.mark.parametrize("package", sorted(DEFINING_MODULE_IMPORTS))
def test_extension_imports_its_defining_modules(package):
    """gh-218. A fresh interpreter, as the stub generator uses: importing the extension alone
    must load every module whose types it uses."""
    expected = [_extension(dep) for dep in DEFINING_MODULE_IMPORTS[package]]
    code = (
        "import sys\n"
        f"import {_extension(package)}\n"
        f"missing = [m for m in {expected!r} if m not in sys.modules]\n"
        "print(missing)\n"
        "sys.exit(1 if missing else 0)\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, f"{package} misses imports: {result.stdout}{result.stderr}"


def test_executor_thread_default_is_machine_independent():
    """`hardware_concurrency()` is evaluated at import; its value must not reach the stub."""
    text = _mod.stub_path("tesseract_robotics.task_composer").read_text(encoding="utf-8")
    assert "name: str = 'TaskflowExecutor', num_threads: int = ...)" in text


def test_check_reports_unified_diff():
    """A failing drift gate must show what differs, not only which file (CI logs are the only view)."""
    report = _mod.drift_report(Path("pkg/_m.pyi"), "a\nkeep\n", "b\nkeep\n")
    assert "--- committed/pkg/_m.pyi" in report
    assert "+++ rendered/pkg/_m.pyi" in report
    assert "-a\n" in report and "+b\n" in report


def test_binding_sources_match_extension_modules():
    """The audit (scripts/audit_ownership.py, binding_modules) takes its module list from
    src/bindings/*.cpp because it must not import the package: each module's primary, minus the
    parts linked into a module (MODULE_PARTS) and the helpers (NON_MODULE_SOURCES). This pins
    that set to the modules the build actually produces."""
    parts = {s for stems in _own.MODULE_PARTS.values() for s in stems}
    sources = {p.stem for p in (REPO_ROOT / "src" / "bindings").glob("*.cpp")}
    built = {m.rsplit(".", 1)[-1].removeprefix("_") for m in MODULES}
    assert sources - parts - _own.NON_MODULE_SOURCES == built
    assert set(_own.binding_modules()) == built


_PRIVATE_EXTENSION_REF = re.compile(r"\btesseract_robotics\.(\w+)\._\1\b")


def test_stubs_name_public_paths_not_private_extensions():
    """Cross references read `tesseract_robotics.common.Isometry3d`, never `common._common`.

    A wrapped package star-imports its `_<pkg>` extension, so the public path names the same
    object; users and type checkers should never see the private module.
    """
    offenders = []
    for stub in sorted((REPO_ROOT / "src" / "tesseract_robotics").rglob("*.pyi")):
        for lineno, line in enumerate(stub.read_text(encoding="utf-8").splitlines(), 1):
            if _PRIVATE_EXTENSION_REF.search(line):
                offenders.append(f"{stub.relative_to(REPO_ROOT)}:{lineno}: {line.strip()}")
    assert not offenders, "\n".join(offenders[:20])


def test_publish_paths_rewrites_only_wrapped_extension_references():
    publish_paths = _load("generate_stubs").publish_paths
    assert (
        publish_paths("x: tesseract_robotics.common._common.Isometry3d")
        == "x: tesseract_robotics.common.Isometry3d"
    )
    assert publish_paths("import tesseract_robotics.common._common") == (
        "import tesseract_robotics.common"
    )
    unchanged = "tesseract_robotics.collision.ContactResult; tesseract_robotics.common._x.Y"
    assert publish_paths(unchanged) == unchanged
