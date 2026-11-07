"""The low-level examples use the bindings only, never `tesseract_robotics.planning` (#42 item 6).

They are the 1:1 counterparts of the C++ tesseract_examples; the high-level API has its own
examples one directory up.
"""

import ast
from pathlib import Path

import pytest

LOWLEVEL = (
    Path(__file__).resolve().parents[2] / "src" / "tesseract_robotics" / "examples" / "lowlevel"
)
FORBIDDEN = "tesseract_robotics.planning"


def _planning_imports(path: Path) -> list[str]:
    found = []
    for node in ast.walk(ast.parse(path.read_text(), filename=str(path))):
        if isinstance(node, ast.ImportFrom) and node.module:
            names = [node.module]
            if node.module == "tesseract_robotics":
                names += [f"tesseract_robotics.{a.name}" for a in node.names]
        elif isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        else:
            continue
        found += [
            f"line {node.lineno}: {n}"
            for n in names
            if n == FORBIDDEN or n.startswith(FORBIDDEN + ".")
        ]
    return found


@pytest.mark.parametrize("path", sorted(LOWLEVEL.glob("*.py")), ids=lambda p: p.name)
def test_lowlevel_examples_use_bindings_only(path: Path) -> None:
    assert _planning_imports(path) == []
