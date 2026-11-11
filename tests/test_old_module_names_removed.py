"""0.36 is a breaking release: the 0.35 module names (`tesseract_robotics.tesseract_x`) are gone.

Each of the 20 renamed modules imports only under its 0.36 name (`tesseract_robotics.x`); the
old spelling raises `ModuleNotFoundError`, with no alias and no deprecation shim.
"""

from __future__ import annotations

import importlib

import pytest

RENAMED = (
    "collision",
    "command_language",
    "common",
    "environment",
    "geometry",
    "kinematics",
    "motion_planners",
    "motion_planners_descartes",
    "motion_planners_ompl",
    "motion_planners_simple",
    "motion_planners_trajopt",
    "motion_planners_trajopt_ifopt",
    "scene_graph",
    "serialization",
    "srdf",
    "state_solver",
    "task_composer",
    "task_composer_planning",
    "time_parameterization",
    "urdf",
)


@pytest.mark.parametrize("name", RENAMED)
def test_new_name_imports(name: str) -> None:
    importlib.import_module(f"tesseract_robotics.{name}")


@pytest.mark.parametrize("name", RENAMED)
def test_old_name_raises(name: str) -> None:
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(f"tesseract_robotics.tesseract_{name}")
