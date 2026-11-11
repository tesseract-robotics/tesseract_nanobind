"""The demo scenes: every manifest entry has one, and those without a planner build here."""

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import manifest  # noqa: E402
from scenes import SCENES  # noqa: E402

from tesseract_robotics.planning import Pose, TaskComposer  # noqa: E402
from tesseract_robotics.viewer import Trajectory  # noqa: E402

STATIC = [
    "geometry_showcase_example",
    "scene_graph_example",
    "shapes_viewer",
    "tesseract_material_mesh_viewer",
]
# Animated without planning: a hand-built sweep and a sinusoid.
needs_twc = pytest.mark.usefixtures("twc_support")
SCRIPTED = ["sdf_collision_example", "twc_workcell_positioner_viewer"]


def test_every_manifest_entry_has_a_scene():
    assert {entry.module for entry in manifest.load_manifest()} == set(SCENES)


@pytest.mark.parametrize("module", STATIC)
def test_a_static_scene_has_an_environment_and_no_phases(module):
    scene = SCENES[module](None)
    assert scene.env.getLinkIds()
    assert scene.phases == []


@pytest.mark.parametrize("module", SCRIPTED)
def test_a_scripted_scene_plays_one_trajectory_in_its_environment(module, request):
    if module == "twc_workcell_positioner_viewer":
        request.getfixturevalue("twc_support")
    scene = SCENES[module](None)
    assert [phase.env for phase in scene.phases] == [scene.env]
    assert isinstance(scene.phases[0].trajectory, Trajectory)


@needs_twc
def test_the_twc_scene_is_the_rail_cell():
    scene = SCENES["twc_workcell_positioner_viewer"](None)
    assert "rail" in scene.env.getName()


def test_the_python_constraints_scene_plays_one_phase_per_act_in_one_environment():
    example = importlib.import_module("tesseract_robotics.examples.python_constraints_example")

    scene = SCENES["python_constraints_example"](None)

    assert [phase.env for phase in scene.phases] == [scene.env] * len(example.ACTS)
    for phase, act in zip(scene.phases, example.ACTS):
        assert isinstance(phase.trajectory, Trajectory)
        assert phase.trajectory.duration_s == pytest.approx(act.seconds - 1.0 / example.FRAME_HZ)
    # The captions, gauges and glyphs are drawn by one decoration, onto links of that scene.
    assert len(scene.decorations) == 1
    link_names = {link_id.name() for link_id in scene.env.getLinkIds()}
    assert {f"caption_{index}" for index in range(len(example.ACTS))} <= link_names


@pytest.fixture(scope="module")
def composer():
    """One composer for the planned scenes: each from_config() reloads the plugins."""
    return TaskComposer.from_config()


def _collision_boxes(env, link: str):
    """(edges, world pose) of each collision box of `link`."""
    pose = env.getState().link_transforms[link]
    shapes = env.getSceneGraph().getLink(link).collision
    return [
        (
            np.array([c.geometry.getX(), c.geometry.getY(), c.geometry.getZ()]),
            np.asarray((pose * c.origin).matrix),
        )
        for c in shapes
    ]


@pytest.mark.planning
def test_the_clearance_recording_holds_the_examples_cubes(composer):
    """The recorder builds no scene of its own: the recorded obstacles are the example's."""
    example = importlib.import_module(
        "tesseract_robotics.examples.freespace_clearance_objective_example"
    )

    scene = SCENES["freespace_clearance_objective_example"](composer)

    for index, (edge_m, centre_m, rpy_rad) in enumerate(example.CUBES):
        ((edges, pose),) = _collision_boxes(scene.env, f"cube_{index}")
        assert edges == pytest.approx([edge_m] * 3)
        assert pose == pytest.approx(np.asarray(Pose.from_xyz_rpy(centre_m, rpy_rad).matrix))


@pytest.mark.planning
def test_the_ompl_recording_holds_the_cpp_examples_sphere(composer):
    """freespace_ompl_example.cpp's obstacle: a 0.15 m sphere at (0.5, 0, 0.55) m."""
    scene = SCENES["freespace_ompl_example"](composer)

    (shape,) = scene.env.getSceneGraph().getLink("sphere_attached").collision
    assert shape.geometry.getRadius() == pytest.approx(0.15)
    centre = scene.env.getState().link_transforms["sphere_attached"] * shape.origin
    assert np.asarray(centre.translation) == pytest.approx([0.5, 0.0, 0.55])
