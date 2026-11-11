"""python_constraints_example --sliders: the loop solves from the viewer's control panel."""

from __future__ import annotations

import importlib
import threading
import time

import numpy as np
import pytest

from tesseract_robotics.viewer import TesseractViewer

example = importlib.import_module("tesseract_robotics.examples.python_constraints_example")

# How long the driver gets to solve some frames before the fake Enter [s].
DRIVE_S = 1.5


@pytest.mark.viewer
def test_sliders_mode_drives_the_cell_until_enter(monkeypatch):
    viewers: list[TesseractViewer] = []
    failures: list[threading.ExceptHookArgs] = []

    class AnyPortViewer(TesseractViewer):
        def __init__(self):
            super().__init__(port=0)
            viewers.append(self)

    def fake_input(_prompt=""):
        time.sleep(DRIVE_S)
        return ""

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
        slider.value = 0.6
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not np.isclose(glyphs[0].opacity, 0.6):
            time.sleep(0.05)

        assert all(np.isclose(glyph.opacity, 0.6) for glyph in glyphs)
