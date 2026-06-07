"""JBI target formatters: pure joints → PULSE position-tuple strings."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.jbi.targets import pulse_pos


def test_pulse_pos_degrees_three_decimals() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert pulse_pos(joints) == "0.000,-20.000,30.000,0.000,50.000,0.000"


def test_pulse_pos_negative_values() -> None:
    joints = tuple(np.deg2rad([30.0, 10.0, 15.0, 20.0, 40.0, -10.0]))
    assert pulse_pos(joints) == "30.000,10.000,15.000,20.000,40.000,-10.000"


def test_pulse_pos_normalizes_negative_zero() -> None:
    # A tiny negative angle must round to a clean 0.000, never -0.000.
    joints = (-1e-12, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert pulse_pos(joints).split(",")[0] == "0.000"


def test_pulse_pos_requires_six_joints() -> None:
    with pytest.raises(ValueError):
        pulse_pos((0.0, 0.0, 0.0))
