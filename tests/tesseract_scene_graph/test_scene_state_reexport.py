"""`tesseract_scene_graph.SceneState` re-exports the class bound in `tesseract_state_solver`,
whichever package a fresh interpreter imports first (gh-218: the state_solver extension
imports the scene_graph extension, so an eager re-export was an import cycle).

The re-export is lazy (module `__getattr__`), so it is reached by name, not by a star import:
the package defines no `__all__`, and a star import only copies names already bound."""

import subprocess
import sys

import pytest

FIRST_IMPORTS = [
    "tesseract_robotics.scene_graph",
    "tesseract_robotics.state_solver",
]


@pytest.mark.parametrize("first", FIRST_IMPORTS)
def test_scene_state_reexport_in_fresh_interpreter(first):
    code = (
        f"import {first}\n"
        "from tesseract_robotics.scene_graph import SceneState\n"
        "from tesseract_robotics.state_solver import SceneState as Bound\n"
        "assert SceneState is Bound\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_scene_graph_unknown_attribute_raises():
    import tesseract_robotics.scene_graph as sg

    with pytest.raises(AttributeError, match="no_such_name"):
        sg.no_such_name
