"""Every example's `main()`: the viewer half the `run()` tests in tests/examples never reach.

Lives here for the Python >= 3.11 gate in conftest.py. Slow: most mains plan.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import numpy as np
import pytest

import tesseract_robotics.examples
from tesseract_robotics.viewer import TesseractViewer

EXAMPLES_DIR = Path(tesseract_robotics.examples.__file__).parent

# Needs ws/src/tesseract_ros_workcell (`git submodule update --init`), absent from CI's
# wheel tests; hand-checked instead. Not a pytest skip: awaiting a go for one.
NEEDS_SUBMODULE = {"twc_workcell_positioner_viewer"}

# Trajectory positions the fake input() shows, as fractions of the duration.
SWEEP_FRACTIONS = np.linspace(0.0, 1.0, 5)


def _module_name(path: Path) -> str:
    return ".".join(path.relative_to(EXAMPLES_DIR).with_suffix("").parts)


def _main_def(path: Path) -> ast.FunctionDef | None:
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.FunctionDef) and node.name == "main":
            return node
    return None


EXAMPLE_FILES = sorted(p for p in EXAMPLES_DIR.rglob("*.py") if p.name != "__init__.py")
MAIN_FILES = [p for p in EXAMPLE_FILES if _main_def(p) is not None]
VIEWER_MAIN_FILES = [p for p in MAIN_FILES if "TesseractViewer" in p.read_text(encoding="utf-8")]


@pytest.mark.parametrize("path", VIEWER_MAIN_FILES, ids=_module_name)
def test_viewer_main_returns_an_exit_code(path):
    """`sys.exit(True)` is exit 1: a console script's `main` must return an int."""
    main = _main_def(path)
    assert main is not None and main.returns is not None and ast.unparse(main.returns) == "int"
    returns = [n for n in ast.walk(main) if isinstance(n, ast.Return)]
    assert returns, "falls off the end: exit 0 even when planning failed"
    assert "sys.exit(main())" in path.read_text(encoding="utf-8")


@pytest.mark.viewer
@pytest.mark.parametrize(
    "module_name",
    [_module_name(p) for p in MAIN_FILES if _module_name(p) not in NEEDS_SUBMODULE],
)
def test_main_runs_and_exits_zero(module_name, monkeypatch, tmp_path):
    viewers: list[TesseractViewer] = []

    class AnyPortViewer(TesseractViewer):
        # Port 8000 may be taken; 0 lets the OS pick.
        def __init__(self):
            super().__init__(port=0)
            viewers.append(self)

    def fake_input(_prompt=""):
        # Reached with the scene up: walk the trajectory, as the player thread would.
        viewer = viewers[-1]
        assert viewer._env is not None
        if viewer._trajectory is not None:
            for fraction in SWEEP_FRACTIONS:
                viewer.show_time(fraction * viewer._trajectory.duration_s)
        return ""

    monkeypatch.setattr("tesseract_robotics.viewer.TesseractViewer", AnyPortViewer)
    monkeypatch.setattr("builtins.input", fake_input)
    monkeypatch.setattr(sys, "argv", [module_name])
    monkeypatch.chdir(tmp_path)

    module = __import__(f"tesseract_robotics.examples.{module_name}", fromlist=["main"])
    exit_code = module.main()

    assert exit_code is None or (type(exit_code) is int and exit_code == 0)
