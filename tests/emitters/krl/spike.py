"""The KRL spike program shared by the golden and parse-back tests."""

from __future__ import annotations

import numpy as np

from tesseract_robotics.command_language import CompositeInstruction
from tesseract_robotics.emitters.krl import KrlNullFrame, KrlProfile, krl_frames
from tesseract_robotics.planning import Pose
from tests.emitters.programs import cart_instruction, composite, joint_instruction

J1_DEG = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
J2_DEG = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
P1 = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0])
P2 = Pose.from_xyz_rpy([0.6, -0.02, 0.8], [np.pi, 0, 0])
FRAMES = krl_frames(bases={"base_link": KrlNullFrame.NULLFRAME}, tools={"tool0": 1})


def spike_composite() -> CompositeInstruction:
    return composite(
        joint_instruction(np.deg2rad(J1_DEG), profile="JMOVE"),
        joint_instruction(np.deg2rad(J2_DEG), profile="JMOVE"),
        cart_instruction(P1, profile="LMOVE"),
        cart_instruction(P2, profile="LBLEND"),
        description="SPIKE",
    )


def spike_profiles() -> dict[str, KrlProfile]:
    return {
        "JMOVE": KrlProfile.build(ptp_speed_percent=100.0),
        "LMOVE": KrlProfile.build(cp_speed_mms=100.0),
        "LBLEND": KrlProfile.build(cp_speed_mms=100.0, approximation_mm=5.0),
    }
