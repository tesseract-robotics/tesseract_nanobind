"""#258: `tesseract_robotics.ompl_base` is gone; OMPL stays a dependency of the OMPL planner."""

import importlib.util
import tomllib
from pathlib import Path

REPO = Path(__file__).parents[1]


def test_ompl_base_is_not_a_module():
    assert importlib.util.find_spec("tesseract_robotics.ompl_base") is None


def test_reeds_shepp_example_has_no_entry_point():
    pyproject = tomllib.loads((REPO / "pyproject.toml").read_text())
    assert "tesseract_reeds_shepp_example" not in pyproject["project"]["scripts"]


def test_the_ompl_planner_still_imports():
    assert importlib.util.find_spec("tesseract_robotics.motion_planners_ompl") is not None
