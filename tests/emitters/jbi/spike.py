"""The JBI spike program and test calibration shared by the JBI tests."""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.command_language import CompositeInstruction
from tesseract_robotics.emitters.jbi import JbiProfile, JbiPulseCalibration, jbi_frames
from tests.emitters.programs import composite, joint_instruction

ROWS_DEG = [
    [0.0, -20.0, 30.0, 0.0, 50.0, 0.0],
    [30.0, 10.0, 15.0, 20.0, 40.0, -10.0],
    [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]
#: Test calibration: pulses per degree per axis (axis 3 counts backwards) and zero offsets.
PULSES_PER_DEG = (1341.0, 1341.0, -1341.0, 1120.5, 1120.5, 655.2)
ZERO_PULSES = (0, 0, 0, 0, 0, 100)
CALIBRATION = JbiPulseCalibration.build(
    pulses_per_rad=[p * 180.0 / math.pi for p in PULSES_PER_DEG], zero_pulses=ZERO_PULSES
)
FRAMES = jbi_frames(tools={"tool0": 0})


def spike_composite() -> CompositeInstruction:
    return composite(
        *(joint_instruction(np.deg2rad(row), profile="JMOVE") for row in ROWS_DEG),
        description="SPIKE",
    )


def spike_profiles() -> dict[str, JbiProfile]:
    return {"JMOVE": JbiProfile.build(joint_speed_percent=50.0)}
