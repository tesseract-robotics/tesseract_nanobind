"""Contract test: every third-party module the package imports is a declared dependency.

An import test caught `tesseract_robotics.planning` importing `yaml`
without `pyyaml` in `[project] dependencies`: a plain `pip install` of the wheel
failed on `import tesseract_robotics.planning`. An import counts as declared when the
dependencies `scripts/support_metadata.py` generates (the `requires` list in
`[tool.scikit-build.metadata.dependencies]` plus the support wheel pin) name its distribution.
"""

import ast
import sys
import tomllib
from importlib.metadata import packages_distributions
from pathlib import Path

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
PACKAGE = REPO_ROOT / "src" / "tesseract_robotics"
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from support_metadata import dynamic_metadata  # noqa: E402

# This repository's own distributions, by import name: the dev env need not have them
# installed (it reads the support data from the conda env's share/).
OWN_DISTRIBUTIONS = {"tesseract_robotics_support": "tesseract-robotics-nanobind-support"}

# Imports a module may make without a declared dependency, with the reason.
# selftest.py runs the bundled test suite through pytest; it is not part of the runtime API.
UNDECLARED_ALLOWED = {"src/tesseract_robotics/selftest.py": {"pytest"}}


def _top_level_imports(path: Path) -> set[str]:
    """Absolute top-level module names imported anywhere in `path`."""
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(), filename=str(path))):
        if isinstance(node, ast.Import):
            names |= {alias.name.partition(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names.add(node.module.partition(".")[0])
    return names


SUPPORT_DISTRIBUTION = "tesseract-robotics-nanobind-support"
PROVIDER_KEYS = {"provider", "provider-path"}


def _requirements(version: str) -> list[Requirement]:
    """The wheel's Requires-Dist, as scikit-build-core builds it for `version`."""
    pyproject = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    config = pyproject["tool"]["scikit-build"]["metadata"]["dependencies"]
    settings = {k: v for k, v in config.items() if k not in PROVIDER_KEYS}
    return [
        Requirement(s) for s in dynamic_metadata("dependencies", settings, {"version": version})
    ]


def test_support_wheel_is_a_hard_dependency() -> None:
    """`pip install tesseract-robotics-nanobind` installs the support wheel of the same version."""
    pyproject = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    assert "optional-dependencies" not in pyproject["project"].get("dynamic", [])
    assert "optional-dependencies" not in pyproject["project"]
    pins = [
        r for r in _requirements("1.2.3.4") if canonicalize_name(r.name) == SUPPORT_DISTRIBUTION
    ]
    assert len(pins) == 1, pins
    assert str(pins[0].specifier) == "==1.2.3.4"
    assert pins[0].marker is None and not pins[0].extras


def test_third_party_imports_are_declared() -> None:
    declared = {canonicalize_name(r.name) for r in _requirements("0")}
    module_to_dists = packages_distributions()

    missing: dict[str, list[str]] = {}
    for path in sorted(PACKAGE.rglob("*.py")):
        relative = path.relative_to(REPO_ROOT).as_posix()
        allowed = UNDECLARED_ALLOWED.get(relative, set())
        third_party = _top_level_imports(path) - set(sys.stdlib_module_names)
        for module in third_party - {"tesseract_robotics"} - allowed:
            dists = {canonicalize_name(d) for d in module_to_dists.get(module, [])}
            if module in OWN_DISTRIBUTIONS:
                dists.add(canonicalize_name(OWN_DISTRIBUTIONS[module]))
            if not dists & declared:
                missing.setdefault(module, []).append(relative)
    assert missing == {}, f"imported but not a declared dependency: {missing}"
