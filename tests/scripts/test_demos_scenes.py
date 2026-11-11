"""The demo scenes: every manifest entry has one, and those without a planner build here."""

import importlib
import os
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import manifest  # noqa: E402
from scenes import SCENES  # noqa: E402

from tesseract_robotics.viewer import Trajectory  # noqa: E402

STATIC = [
    "geometry_showcase_example",
    "scene_graph_example",
    "shapes_viewer",
    "tesseract_material_mesh_viewer",
]
# Animated without planning: a hand-built sweep and a sinusoid.
TWC_SUPPORT = REPO / "ws/src/tesseract_ros_workcell/twc_support"
# CI sets the variable, so a broken path there fails rather than skips.
needs_twc = pytest.mark.skipif(
    "TWC_SUPPORT_PATH" not in os.environ and not TWC_SUPPORT.exists(),
    reason="needs the TWC workcell: git submodule update --init ws/src/tesseract_ros_workcell",
)
SCRIPTED = [
    "sdf_collision_example",
    pytest.param("twc_workcell_positioner_viewer", marks=needs_twc),
]


def test_every_manifest_entry_has_a_scene():
    assert {entry.module for entry in manifest.load_manifest()} == set(SCENES)


@pytest.mark.parametrize("module", STATIC)
def test_a_static_scene_has_an_environment_and_no_phases(module):
    scene = SCENES[module](None)
    assert scene.env.getLinkIds()
    assert scene.phases == []


@pytest.mark.parametrize("module", SCRIPTED)
def test_a_scripted_scene_plays_one_trajectory_in_its_environment(module):
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


def test_the_sqp_rows_are_the_final_solve_with_a_timestamp_last():
    example = importlib.import_module("tesseract_robotics.examples.online_planning_sqp_example")
    joints = ["a", "b"]
    steps = 3
    first = np.zeros(steps * len(joints))
    final = np.arange(steps * len(joints), dtype=float)

    rows = example.final_rows({"joint_names": joints, "trajectories": [first, final]}, steps=steps)

    assert rows == [
        [0.0, 1.0, 0.0 * example.VIEWER_STEP_S],
        [2.0, 3.0, 1.0 * example.VIEWER_STEP_S],
        [4.0, 5.0, 2.0 * example.VIEWER_STEP_S],
    ]
