"""Every viewer example's `main()`: the half the `run()` tests in tests/examples never reach.

Planning is tests/examples' job: mains that plan get a canned result here.
"""

from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import tesseract_robotics.examples
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import TesseractViewer

EXAMPLES_DIR = Path(tesseract_robotics.examples.__file__).parent
# Its main takes a cell from argv and needs ws/src/tesseract_ros_workcell, so it stays out of
# this canned run. tests/examples/test_twc_rail.py loads both of its cells for real.
NEEDS_SUBMODULE = {"twc_workcell_positioner_viewer"}

# Mains that keep their real run(): it plans nothing through a planner, and the main reads
# the example's own scene (online_planning_sqp: its gantry and estop zones).
REAL_RUN = {
    "geometry_showcase_example",
    "lowlevel.scene_graph_c_api_example",
    "online_planning_sqp_example",
    "scene_graph_example",
    "shapes_viewer",
    "tesseract_material_mesh_viewer",
    "sdf_collision_example",
}

JOINT_NAMES = [f"joint_{i + 1}" for i in range(6)]
# Two IRB2400 configurations for the canned plan [rad].
CANNED_Q = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.3, -0.2, 0.1, 0.4, -0.5, 0.6]])

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


def _canned_results() -> dict[str, object]:
    # Every key a planning main reads from run(), holding real viewer inputs.
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)

    composite = CompositeInstruction()
    for q in CANNED_Q:
        waypoint = StateWaypointPoly(StateWaypoint(JOINT_NAMES, q))
        move = MoveInstruction(waypoint, MoveInstructionType.FREESPACE)
        composite.push_back(MoveInstructionPoly(move))

    planned = SimpleNamespace(successful=True, raw_results=composite)
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    robot = SimpleNamespace(env=env, get_manipulator_info=lambda *_args, **_kwargs: manip_info)
    return {
        "env": env,
        "robot": robot,
        "success": True,
        "trajectory": composite,
        "manip_info": manip_info,
        "result": planned,
        "colliding_result": planned,
        "pick_result": planned,
        "place_result": planned,
        "trajectories": [planned],
        # freespace_clearance_objective_example: case 2 and one cube's closest approach.
        "threshold": {"robot": robot, "result": planned, "success": True},
        "clearances": [
            SimpleNamespace(
                cube="cube_0",
                link="link_6",
                distance_m=0.05,
                robot_point_m=np.array([0.9, 0.0, 1.2]),
                cube_point_m=np.array([0.9, 0.0, 1.15]),
            )
        ],
    }


@pytest.mark.parametrize(
    "module_name",
    [
        "lowlevel.puzzle_piece_auxillary_axes_c_api_example",
        "lowlevel.puzzle_piece_c_api_example",
        "lowlevel.tesseract_planning_composer_c_api_example",
    ],
)
def test_main_exits_one_when_run_reports_failure(module_name, monkeypatch):
    """These run()s return False when planning fails; main must not index it."""
    module = importlib.import_module(f"tesseract_robotics.examples.{module_name}")
    monkeypatch.setattr(module, "run", lambda: False)
    assert module.main() == 1


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
    [_module_name(p) for p in VIEWER_MAIN_FILES if _module_name(p) not in NEEDS_SUBMODULE],
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

    # import_module: the package re-exports each example's run() under the module's name.
    module = importlib.import_module(f"tesseract_robotics.examples.{module_name}")
    if module_name not in REAL_RUN:
        results = _canned_results()
        monkeypatch.setattr(module, "run", lambda *_args, **_kwargs: results)

    assert module.main() == 0
    assert viewers, "main returned without opening the viewer"
