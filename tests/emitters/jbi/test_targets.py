"""JBI pulse rows: exact integer pulses from the calibration."""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import ExternalAxisError, InvalidCalibrationError
from tesseract_robotics.emitters.jbi import JbiPulseCalibration
from tesseract_robotics.emitters.jbi.targets import pulse_pos

_CAL = JbiPulseCalibration.build(
    pulses_per_rad=[1000.0 * 180.0 / math.pi] * 6, zero_pulses=[0, 0, 0, 0, 0, 0]
)


def test_pulse_pos_integer_pulses() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert pulse_pos(joints, _CAL) == "0,-20000,30000,0,50000,0"


def test_pulse_pos_zero_offset_and_sign() -> None:
    cal = JbiPulseCalibration.build(
        pulses_per_rad=[-1000.0 * 180.0 / math.pi] + [1.0] * 5, zero_pulses=[7, 0, 0, 0, 0, 0]
    )
    assert pulse_pos(tuple(np.deg2rad([1.0, 0, 0, 0, 0, 0])), cal).split(",")[0] == "-993"


def test_pulse_pos_requires_six_joints() -> None:
    with pytest.raises(ExternalAxisError):
        pulse_pos((0.0, 0.0, 0.0), _CAL)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"pulses_per_rad": [1.0] * 5, "zero_pulses": [0] * 6},
        {"pulses_per_rad": [0.0] + [1.0] * 5, "zero_pulses": [0] * 6},
        {"pulses_per_rad": [math.nan] + [1.0] * 5, "zero_pulses": [0] * 6},
        {"pulses_per_rad": [1.0] * 6, "zero_pulses": [0.5] + [0] * 5},
    ],
)
def test_calibration_validated(kwargs: dict[str, list[float]]) -> None:
    with pytest.raises(InvalidCalibrationError):
        JbiPulseCalibration.build(**kwargs)  # type: ignore[arg-type]
