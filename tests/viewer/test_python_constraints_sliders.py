"""python_constraints_example --sliders: the loop solves from the viewer's control panel."""

from __future__ import annotations

import importlib
import threading

import numpy as np
import pytest

from tesseract_robotics.viewer import TesseractViewer

example = importlib.import_module("tesseract_robotics.examples.python_constraints_example")

# Frames the driver solves before the fake Enter.
DRIVE_FRAMES = 10
# Longest wait for those frames, or for a slider callback [s]; generous for loaded CI.
WAIT_TIMEOUT_S = 30.0


# [s] Per-test bound: ten times the slowest test measured in this file (1.09 s, test_sliders_mode_drives_the_cell_until_enter),
# rounded up to a multiple of 5 s, and never under 60 s: the bound includes fixture setup, and a
# cold CI worker (Windows above all) spends seconds loading plugins before the first test runs.
# A deadlock or hang never ends; this fails it instead of
# waiting for the session-wide timeout.
TEST_TIMEOUT_S = 60
pytestmark = pytest.mark.timeout(TEST_TIMEOUT_S)


@pytest.mark.viewer
def test_sliders_mode_drives_the_cell_until_enter(monkeypatch):
    viewers: list[TesseractViewer] = []
    failures: list[threading.ExceptHookArgs] = []

    class AnyPortViewer(TesseractViewer):
        def __init__(self):
            super().__init__(port=0)
            viewers.append(self)

    solved = threading.Event()
    frames = [0]
    step = example.LiveLoop.step

    def counting_step(self, inputs):
        frame = step(self, inputs)
        frames[0] += 1
        if frames[0] >= DRIVE_FRAMES:
            solved.set()
        return frame

    def fake_input(_prompt=""):
        assert solved.wait(WAIT_TIMEOUT_S), f"the driver solved {frames[0]} frames"
        return ""

    monkeypatch.setattr(example.LiveLoop, "step", counting_step)
    monkeypatch.setattr("tesseract_robotics.viewer.TesseractViewer", AnyPortViewer)
    monkeypatch.setattr("builtins.input", fake_input)
    monkeypatch.setattr(threading, "excepthook", failures.append)

    assert example.main(["--sliders"]) == 0

    assert not failures, failures[0].exc_value
    assert viewers, "main returned without opening the viewer"


@pytest.mark.viewer
def test_the_opacity_slider_sets_every_glyph():
    loop = example.LiveLoop()
    with TesseractViewer(port=0) as viewer:
        viewer.update_environment(loop.robot.env)
        glyphs = example.decorate(viewer, loop.target)
        assert glyphs
        assert all(glyph.opacity == example.GLYPH_OPACITY for glyph in glyphs)

        slider = example.add_opacity_slider(viewer, glyphs)
        # viser runs a server-side value change's callbacks in registration order (on a
        # coroutine callback, on its event loop): this one fires after the example's.
        applied = threading.Event()
        slider.on_update(lambda _event: applied.set())
        slider.value = 0.6
        assert applied.wait(WAIT_TIMEOUT_S)

        assert all(np.isclose(glyph.opacity, 0.6) for glyph in glyphs)
