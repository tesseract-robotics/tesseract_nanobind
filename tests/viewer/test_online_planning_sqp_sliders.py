"""online_planning_sqp_example --sliders: the robot replans live as a slider moves the person."""

from __future__ import annotations

import importlib
import threading

import numpy as np
import pytest

from tesseract_robotics.viewer import TesseractViewer

example = importlib.import_module("tesseract_robotics.examples.online_planning_sqp_example")
scene = example.scene

# Ticks the driver replans with the slider at each end. At walking speed 10 ticks cover 1.3 m,
# past the walk's 0.8 m, so the person reaches the near end.
DRIVE_TICKS = 10
# Longest wait for those ticks [s]; generous for loaded CI.
WAIT_TIMEOUT_S = 30.0
# Float slack on a step's length and on where the walk ends [m].
STEP_SLACK_M = 1e-9

# [s] Per-test bound: never under 60 s, as tests/viewer/test_python_constraints_sliders.py sets
# it. The bound includes fixture setup, and a cold CI worker spends seconds loading plugins. A
# deadlock or hang never ends; this fails it instead of waiting for the session-wide timeout.
TEST_TIMEOUT_S = 60
pytestmark = pytest.mark.timeout(TEST_TIMEOUT_S)


@pytest.mark.viewer
def test_the_slider_walks_the_person_to_the_arm_while_the_robot_replans(monkeypatch):
    viewers: list[TesseractViewer] = []
    sliders: list = []
    failures: list[threading.ExceptHookArgs] = []
    places: list[np.ndarray] = []

    class AnyPortViewer(TesseractViewer):
        def __init__(self):
            super().__init__(port=0)
            viewers.append(self)
            gui = self.server.gui
            add_slider = gui.add_slider

            def recording_add_slider(*args, **kwargs):
                handle = add_slider(*args, **kwargs)
                sliders.append(handle)
                return handle

            gui.add_slider = recording_add_slider

    ticks = [0]
    ticked = threading.Condition()
    replan = example.replan
    place_person = scene.place_person

    def counting_replan(*args, **kwargs):
        problem_data = replan(*args, **kwargs)
        with ticked:
            ticks[0] += 1
            ticked.notify_all()
        return problem_data

    def recording_place_person(env, position_m):
        places.append(np.array(position_m, dtype=np.float64))
        place_person(env, position_m)

    def wait_ticks(count: int) -> None:
        with ticked:
            assert ticked.wait_for(lambda: ticks[0] >= count, WAIT_TIMEOUT_S), (
                f"the driver replanned {ticks[0]} of {count} ticks"
            )

    def fake_input(_prompt=""):
        wait_ticks(DRIVE_TICKS)
        (person,) = sliders
        person.value = person.max
        wait_ticks(ticks[0] + DRIVE_TICKS)
        return ""

    monkeypatch.setattr(example, "replan", counting_replan)
    monkeypatch.setattr(scene, "place_person", recording_place_person)
    monkeypatch.setattr("tesseract_robotics.viewer.TesseractViewer", AnyPortViewer)
    monkeypatch.setattr("builtins.input", fake_input)
    monkeypatch.setattr(threading, "excepthook", failures.append)

    assert example.main(["--sliders"]) == 0

    assert not failures, failures[0].exc_value
    assert viewers, "main returned without opening the viewer"
    walk = np.array(places)
    # On the walk's line, from its far end, where the slider's zero puts it, to its near end.
    assert walk[:, 0] == pytest.approx(scene.WALK_X_M)
    assert walk[0, 1] == pytest.approx(scene.WALK_FAR_Y_M)
    assert walk[-1, 1] == pytest.approx(scene.WALK_NEAR_Y_M)
    assert walk[:, 1].min() >= scene.WALK_NEAR_Y_M - STEP_SLACK_M
    # At walking speed: one tick's step at most.
    step_m = scene.WALKING_SPEED_M_S * scene.TICK_S
    assert np.abs(np.diff(walk[:, 1])).max() <= step_m + STEP_SLACK_M
