"""What each demo shows: the phases to play and the paths to draw."""

from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass, field
from types import ModuleType

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import RecordingPhase, Trajectory

# The TWC cell the demo shows.
TWC_CELL = "positioner"


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


def _result_scene(results: dict, *, plot: bool = True) -> Scene:
    program = results["result"].raw_results
    env = results["robot"].env
    return Scene(env, [RecordingPhase(env, program)], [program] if plot else [])


def _planned(module: str, composer: TaskComposer, *, plot: bool = True) -> Scene:
    return _result_scene(_example(module).run(composer=composer), plot=plot)


def _static(module: str, _composer: TaskComposer) -> Scene:
    return Scene(_example(module).run()["env"], [])


def _static_robot(module: str, _composer: TaskComposer) -> Scene:
    return Scene(_example(module).run()["robot"].env, [])


def _abb(_composer: TaskComposer) -> Scene:
    # Plans through the low-level planners, without a composer.
    results = _example("abb_irb2400_viewer").run()
    env, program = results["env"], results["trajectory"]
    return Scene(env, [RecordingPhase(env, program)], [program])


def _online(composer: TaskComposer) -> Scene:
    # The last replan, as the example's main shows it.
    results = _example("online_planning_example").run(composer=composer)
    program = results["trajectories"][-1].raw_results
    env = results["robot"].env
    return Scene(env, [RecordingPhase(env, program)], [program])


def _online_sqp(_composer: TaskComposer) -> Scene:
    example = _example("online_planning_sqp_example")
    results = example.run()
    env = results["robot"].env
    example.style_for_viewer(env)
    trajectory = Trajectory.from_rows(results["joint_names"], example.final_rows(results))
    return Scene(env, [RecordingPhase(env, trajectory)])


def _pointcloud(_composer: TaskComposer) -> Scene:
    # The collision-aware plan. The example's run builds its own composer.
    return _result_scene(_example("pointcloud_octree_collision_example").run())


def _sdf(_composer: TaskComposer) -> Scene:
    example = _example("sdf_collision_example")
    env = example.run()["robot"].env
    trajectory = Trajectory.from_rows(["probe_x"], example.sweep_trajectory())
    return Scene(env, [RecordingPhase(env, trajectory)])


def _twc(_composer: TaskComposer) -> Scene:
    example = _example("twc_workcell_positioner_viewer")
    env = example.run(TWC_CELL)["env"]
    return Scene(
        env, [RecordingPhase(env, Trajectory.from_rows(*example.animation_rows(TWC_CELL)))]
    )


SCENES: dict[str, Callable[[TaskComposer], Scene]] = {
    "car_seat_example": lambda c: _pick_then_place("car_seat_example", c),
    "pick_and_place_example": lambda c: _pick_then_place("pick_and_place_example", c),
    "raster_example": lambda c: _planned("raster_example", c),
    "basic_cartesian_example": lambda c: _planned("basic_cartesian_example", c),
    "glass_upright_example": lambda c: _planned("glass_upright_example", c),
    "freespace_ompl_example": lambda c: _planned("freespace_ompl_example", c),
    "freespace_hybrid_example": lambda c: _planned("freespace_hybrid_example", c),
    "chain_example": lambda c: _planned("chain_example", c),
    # No plot: its MoveL working frame moves with the group (MovingWorkingFrameError).
    "puzzle_piece_auxillary_axes_example": lambda c: _planned(
        "puzzle_piece_auxillary_axes_example", c, plot=False
    ),
    "abb_irb2400_viewer": _abb,
    "online_planning_example": _online,
    "online_planning_sqp_example": _online_sqp,
    "pointcloud_octree_collision_example": _pointcloud,
    "sdf_collision_example": _sdf,
    "twc_workcell_positioner_viewer": _twc,
    "geometry_showcase_example": lambda c: _static_robot("geometry_showcase_example", c),
    "scene_graph_example": lambda c: _static_robot("scene_graph_example", c),
    "shapes_viewer": lambda c: _static("shapes_viewer", c),
    "tesseract_material_mesh_viewer": lambda c: _static("tesseract_material_mesh_viewer", c),
}
