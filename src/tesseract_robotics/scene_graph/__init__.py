"""tesseract_scene_graph Python bindings (nanobind)"""

# Import tesseract_geometry first to ensure cross-module type resolution works
import tesseract_robotics.geometry  # noqa: F401
from tesseract_robotics.scene_graph._scene_graph import *


def __getattr__(name: str):
    """Re-export SceneState (bound in tesseract_state_solver) on first access.

    Lazy because the state_solver extension imports this package's extension for its own
    signatures (gh-218): an eager import here re-enters a half-initialised state_solver.
    """
    if name == "SceneState":
        from tesseract_robotics.state_solver import SceneState

        return SceneState
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
