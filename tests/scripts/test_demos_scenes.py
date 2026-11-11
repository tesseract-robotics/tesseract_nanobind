"""The demo scenes: every manifest entry has one, and those without a planner build here."""

import importlib
import sys
import types
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


def _clearance(cube: str, robot_point, cube_point):
    example = importlib.import_module(
        "tesseract_robotics.examples.freespace_clearance_objective_example"
    )
    return example.Clearance(
        cube=cube,
        link="link_7",
        distance_m=float(np.linalg.norm(np.subtract(cube_point, robot_point))),
        robot_point_m=np.asarray(robot_point, dtype=float),
        cube_point_m=np.asarray(cube_point, dtype=float),
    )


CLEARANCE_MODULE = "tesseract_robotics.examples.freespace_clearance_objective_example"


def _no_two_overlap(view, anchors) -> bool:
    return not any(view.overlap(a, b) for i, a in enumerate(anchors) for b in anchors[i + 1 :])


def test_labels_of_cubes_behind_one_another_are_stacked_apart():
    """Two nearest points on one line of sight, far apart in space: their labels still meet."""
    example = importlib.import_module(CLEARANCE_MODULE)
    view = example.DEFAULT_VIEW
    eye = np.asarray(view.position_m)
    toward = np.asarray(view.look_at_m) - eye
    near, far = eye + 0.6 * toward, eye + 1.2 * toward
    clearances = [
        _clearance("cube_0", near - (0.0, 0.0, 0.05), near),
        _clearance("cube_1", far - (0.0, 0.0, 0.05), far),
        _clearance("cube_2", (0.0, 1.0, 0.4), (0.0, 1.0, 0.45)),
    ]
    assert view.overlap(near, far)

    anchors = example.label_anchors(clearances)

    assert _no_two_overlap(view, anchors)
    # The first label, and one with room, stay at their cube's nearest point.
    assert anchors[0] == pytest.approx(clearances[0].cube_point_m)
    assert anchors[2] == pytest.approx(clearances[2].cube_point_m)
    # Labels only rise, so each stays above the end of its own line.
    for anchor, clearance in zip(anchors, clearances, strict=True):
        assert anchor[:2] == pytest.approx(clearance.cube_point_m[:2])
        assert anchor[2] >= clearance.cube_point_m[2]


def test_the_labels_are_laid_out_for_the_recordings_camera():
    example = importlib.import_module(CLEARANCE_MODULE)
    (entry,) = [
        e for e in manifest.load_manifest() if e.module == "freespace_clearance_objective_example"
    ]
    assert tuple(entry.camera.position_m) == pytest.approx(example.VIEW_POSITION_M)
    assert tuple(entry.camera.look_at_m) == pytest.approx(example.VIEW_LOOK_AT_M)
    assert entry.camera.fov_rad == pytest.approx(example.VIEW_FOV_RAD)


@pytest.mark.planning
def test_no_two_recorded_clearance_labels_overlap(composer):
    example = importlib.import_module(CLEARANCE_MODULE)
    labels = {}

    class Recorder:
        """Stands in for the viewer's scene: keeps each label's anchor."""

        def add_line_segments(self, *_args, **_kwargs):
            pass

        def add_label(self, name, _text, *, position):
            labels[name] = np.asarray(position, dtype=float)

    viewer = types.SimpleNamespace(server=types.SimpleNamespace(scene=Recorder()))
    scene = SCENES["freespace_clearance_objective_example"](composer)
    for decorate in scene.decorations:
        decorate(viewer)

    assert len(labels) == len(example.CUBES)
    assert _no_two_overlap(example.DEFAULT_VIEW, list(labels.values()))
