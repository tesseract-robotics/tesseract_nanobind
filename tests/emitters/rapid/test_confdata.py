"""ABB confdata: quadrant numbering and the cfx table (3HAC050917, confdata)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.core.posture import ArmPosture
from tesseract_robotics.emitters.core.units import Radians
from tesseract_robotics.emitters.rapid.targets import Config, cfx, quadrant
from tests.emitters.programs import IRB2400_POSTURE


@pytest.mark.parametrize(
    ("deg", "expected"),
    [(0, 0), (45, 0), (90, 1), (179.9, 1), (180, 2), (-0.1, -1), (-45, -1), (-90, -1),
     (-90.1, -2), (-180, -2), (270, 3)],
)  # fmt: skip
def test_quadrant_boundaries(deg: float, expected: int) -> None:
    assert quadrant(Radians(math.radians(deg))) == expected


def test_quadrant_exact_quarter_turns() -> None:
    assert quadrant(Radians(-math.pi / 2)) == -1
    assert quadrant(Radians(math.pi / 2)) == 1


@pytest.mark.parametrize(
    ("front_axis1", "front_lower_arm", "a5_positive", "expected"),
    [
        (True, True, True, 0),
        (True, True, False, 1),
        (True, False, True, 2),
        (True, False, False, 3),
        (False, True, True, 4),
        (False, True, False, 5),
        (False, False, True, 6),
        (False, False, False, 7),
    ],
)
def test_cfx_table(
    front_axis1: bool, front_lower_arm: bool, a5_positive: bool, expected: int
) -> None:
    assert cfx(ArmPosture(front_axis1, front_lower_arm, a5_positive)) == expected


def test_config_from_irb2400_joints() -> None:
    q = tuple(np.deg2rad([-100, 30, 10, 200, -40, -95]))
    config = Config.from_joints(q, IRB2400_POSTURE.posture(q))
    assert config == Config(cf1=-2, cf4=2, cf6=-2, cfx=1)
    assert config.literal() == "[-2, 2, -2, 1]"


def test_config_behind_axis1() -> None:
    # Arm leaning far back over the base puts the wrist centre behind axis 1.
    q = tuple(np.deg2rad([0, -100, 60, 0, 30, 0]))
    assert Config.from_joints(q, IRB2400_POSTURE.posture(q)).cfx == 4
