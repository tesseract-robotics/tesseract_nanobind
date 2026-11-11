"""The URScript spike program shared by the golden and parse-back tests."""

from __future__ import annotations

import numpy as np

from tesseract_robotics.command_language import CompositeInstruction, MoveInstructionType
from tesseract_robotics.emitters.urscript import UrScriptProfile, ur_frames
from tesseract_robotics.planning import Pose
from tests.emitters.programs import cart_instruction, composite, joint_instruction

J1_DEG = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
J2_DEG = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
P1 = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0])
P2 = Pose.from_xyz_rpy([0.6, -0.02, 0.8], [np.pi, 0, 0])
SEED2_DEG = [-2.0, -60.0, 100.0, -130.0, -90.0, 0.0]
#: A 0.15 m tool along the flange z axis.
TCP = Pose.from_xyz(0.0, 0.0, 0.15)
FRAMES = ur_frames(features={"base_link": Pose()}, tcps={"tool0": TCP})


def spike_composite() -> CompositeInstruction:
    return composite(
        joint_instruction(np.deg2rad(J1_DEG), profile="JMOVE"),
        joint_instruction(np.deg2rad(J2_DEG), profile="JMOVE"),
        cart_instruction(P1, profile="LMOVE"),
        cart_instruction(
            P2, MoveInstructionType.FREESPACE, profile="JMOVE", seed_rad=np.deg2rad(SEED2_DEG)
        ),
        description="SPIKE",
    )


def spike_profiles() -> dict[str, UrScriptProfile]:
    return {"JMOVE": UrScriptProfile.build(), "LMOVE": UrScriptProfile.build(linear_speed_ms=0.1)}
