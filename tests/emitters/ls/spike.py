"""The LS spike program shared by the golden and parse-back tests."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from tesseract_robotics.command_language import CompositeInstruction, MoveInstructionType
from tesseract_robotics.emitters.ls import FanucArmConfig, LsProfile, ls_frames
from tesseract_robotics.planning import Pose
from tests.emitters.programs import IRB2400_POSTURE, cart_instruction, composite, joint_instruction

J1_DEG = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
J2_DEG = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
P1 = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0])
P2 = Pose.from_xyz_rpy([0.6, -0.02, 0.8], [np.pi, 0, 0])
#: Seeds of P1/P2 (radians); J6 at 200 deg exercises a nonzero turn number.
SEED1_DEG = [-9.5, 17.0, 23.0, 0.0, 50.0, 200.0]
SEED2_DEG = [-1.9, 16.0, 24.0, 0.0, 50.0, 190.0]
FRAMES = ls_frames(user_frames={"base_link": 0}, tools={"tool0": 1})


def posture_config(joints: Sequence[float]) -> FanucArmConfig:
    """Test resolver: placement letters from the OPW posture of the IRB 2400 geometry."""
    p = IRB2400_POSTURE.posture(joints)
    return FanucArmConfig(
        flip=not p.axis5_positive, up=p.wrist_in_front_of_lower_arm, front=p.wrist_in_front_of_axis1
    )


def spike_composite() -> CompositeInstruction:
    return composite(
        joint_instruction(np.deg2rad(J1_DEG), profile="JMOVE"),
        joint_instruction(np.deg2rad(J2_DEG), profile="JMOVE"),
        cart_instruction(P1, profile="LMOVE", seed_rad=np.deg2rad(SEED1_DEG)),
        cart_instruction(
            P2, MoveInstructionType.FREESPACE, profile="JMOVE", seed_rad=np.deg2rad(SEED2_DEG)
        ),
        description="SPIKE",
    )


def spike_profiles() -> dict[str, LsProfile]:
    return {
        "JMOVE": LsProfile.build(joint_speed_percent=100.0),
        "LMOVE": LsProfile.build(linear_speed_mms=100.0, termination="CNT50"),
    }
