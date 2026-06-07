"""IR event dataclasses: frozen, typed, canonically serializable."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    Note,
    ProgramIR,
    SetDigital,
    WaitDigital,
)
from tesseract_robotics.planning import Pose


def test_joint_move_is_frozen() -> None:
    m = JointMove(joints=(0.0, 0.1, 0.2, 0.3, 0.4, 0.5), profile="P")
    with pytest.raises(Exception):
        m.profile = "Q"  # type: ignore[misc]


def test_set_digital_key_optional() -> None:
    unnamed = SetDigital(key=None, index=7, value=True)
    named = SetDigital(key="do_array", index=3, value=False)
    assert unnamed.key is None and unnamed.index == 7
    assert named.key == "do_array"


def test_wait_digital_carries_is_input() -> None:
    w = WaitDigital(is_input=True, index=2, value=True, timeout=None)
    assert w.is_input and w.index == 2 and w.timeout is None


def test_cartesian_move_holds_pose_and_kind() -> None:
    m = CartesianMove(
        pose=Pose.from_xyz(0.6, -0.1, 0.8), kind=MoveKind.LINEAR, profile="P", seed_joints=None
    )
    assert m.kind is MoveKind.LINEAR
    assert np.allclose(m.pose.translation, [0.6, -0.1, 0.8])


def test_program_ir_canonical_is_stable_and_value_sensitive() -> None:
    events = (Note("hi"), Dwell(seconds=1.5), SetDigital(key="do", index=0, value=True))
    a = ProgramIR(name="p", events=events).canonical()
    b = ProgramIR(name="p", events=events).canonical()
    assert a == b and "Dwell" in a
    c = ProgramIR(name="p", events=(Dwell(seconds=2.0),)).canonical()
    assert c != a
