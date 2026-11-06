"""What each gallery entry shows: the phases to play and the paths to draw."""

from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass, field
from types import ModuleType

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import RecordingPhase


@dataclass(frozen=True)
class Scene:
    """A recording's content. `env` is what the viewer's scene is built from."""

    env: Environment
    phases: list[RecordingPhase]
    # Programs whose commanded TCP path gets drawn.
    plots: list[CompositeInstruction] = field(default_factory=list)


def _example(module: str) -> ModuleType:
    # Through importlib: the package attribute of this name is the example's main function.
    return importlib.import_module(f"tesseract_robotics.examples.{module}")


def _pick_then_place(module: str, composer: TaskComposer) -> Scene:
    """The whole story: the part rests during the pick, and the tool carries it after."""
    results = _example(module).run(composer=composer)
    pick, place = results["pick_result"].raw_results, results["place_result"].raw_results
    env = results["robot"].env
    return Scene(
        env,
        [RecordingPhase(results["pick_env"], pick), RecordingPhase(env, place)],
        [pick, place],
    )


def _planned(module: str, composer: TaskComposer, *, plot: bool = True) -> Scene:
    results = _example(module).run(composer=composer)
    program = results["result"].raw_results
    env = results["robot"].env
    return Scene(env, [RecordingPhase(env, program)], [program] if plot else [])


def _static(module: str, _composer: TaskComposer) -> Scene:
    return Scene(_example(module).run()["env"], [])


SCENES: dict[str, Callable[[TaskComposer], Scene]] = {
    "car_seat_example": lambda c: _pick_then_place("car_seat_example", c),
    "pick_and_place_example": lambda c: _pick_then_place("pick_and_place_example", c),
    "raster_example": lambda c: _planned("raster_example", c),
    # No plot: its MoveL working frame moves with the group (MovingWorkingFrameError).
    "puzzle_piece_auxillary_axes_example": lambda c: _planned(
        "puzzle_piece_auxillary_axes_example", c, plot=False
    ),
    "shapes_viewer": lambda c: _static("shapes_viewer", c),
    "tesseract_material_mesh_viewer": lambda c: _static("tesseract_material_mesh_viewer", c),
}
