"""Arm posture: OPW closed form against tesseract forward kinematics (IRB 2400)."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import InvalidProfileError, SingularPostureError
from tesseract_robotics.emitters.core.posture import OpwParameters
from tests.emitters.programs import IRB2400_POSTURE, irb2400_fk

#: IRB 2400 joint limits (deg) the samples stay inside, from the URDF.
_LIMITS_DEG = np.array([[-180, 180], [-100, 110], [-60, 65], [-200, 200], [-120, 120], [-400, 400]])


def _geometric_posture(q: np.ndarray) -> tuple[bool, bool, bool]:
    """Posture bits from FK link origins, independent of the OPW closed form.

    Wrist centre = link_5 origin (axes 4, 5, 6 intersect there). Expressed in the
    link_1 frame (which rotates with axis 1): in front of axis 1 iff x > 0; in
    front of the lower arm iff (lower arm x forearm) has a positive y component.
    """
    t1 = irb2400_fk(q, "link_1")
    inv = np.linalg.inv(t1)
    p2, p3, wc = ((inv @ irb2400_fk(q, link))[:3, 3] for link in ("link_2", "link_3", "link_5"))
    lower, fore = p3 - p2, wc - p3
    return bool(wc[0] > 0.0), bool(np.cross(lower, fore)[1] > 0.0), bool(q[4] > 0.0)


def test_opw_posture_matches_forward_kinematics() -> None:
    rng = np.random.default_rng(7)
    for _ in range(200):
        q = np.deg2rad(rng.uniform(_LIMITS_DEG[:, 0], _LIMITS_DEG[:, 1]))
        p = IRB2400_POSTURE.posture(tuple(q))
        expected = _geometric_posture(q)
        got = (p.wrist_in_front_of_axis1, p.wrist_in_front_of_lower_arm, p.axis5_positive)
        assert got == expected, np.rad2deg(q)


def test_home_posture_is_front_front_positive() -> None:
    p = IRB2400_POSTURE.posture(tuple(np.deg2rad([0, 0, 0, 0, 30, 0])))
    assert p.wrist_in_front_of_axis1 and p.wrist_in_front_of_lower_arm and p.axis5_positive


def test_axis5_exactly_zero_is_singular() -> None:
    with pytest.raises(SingularPostureError, match="axis 5"):
        IRB2400_POSTURE.posture((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def test_opw_parameters_reject_bad_input() -> None:
    with pytest.raises(InvalidProfileError):
        OpwParameters.build(
            a1=0.1, a2=0.0, b=0.0, c1=0.6, c2=0.0, c3=0.7, c4=0.1,
            offsets=[0] * 6, sign_corrections=[1] * 6,
        )  # fmt: skip
    with pytest.raises(InvalidProfileError):
        OpwParameters.build(
            a1=0.1, a2=0.0, b=0.0, c1=0.6, c2=0.7, c3=0.7, c4=0.1,
            offsets=[0] * 6, sign_corrections=[1, 1, 2, 1, 1, 1],
        )  # fmt: skip
