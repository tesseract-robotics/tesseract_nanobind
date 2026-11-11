"""Contract test: every third-party module the package imports is a declared dependency.

An import test caught `tesseract_robotics.planning` importing `yaml`
without `pyyaml` in `[project] dependencies`: a plain `pip install` of the wheel
failed on `import tesseract_robotics.planning`. An import counts as declared when a
`[project] dependencies` entry or an extra (`support`, from `scripts/support_metadata.py`)
names its distribution.
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


def test_third_party_imports_are_declared() -> None:
    pyproject = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    extras = dynamic_metadata("optional-dependencies", {}, {"version": "0"})
    specs = [*pyproject["project"]["dependencies"], *(s for reqs in extras.values() for s in reqs)]
    declared = {canonicalize_name(Requirement(spec).name) for spec in specs}
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
    assert missing == {}, f"imported but not in [project] dependencies or extras: {missing}"
