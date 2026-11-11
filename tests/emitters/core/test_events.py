"""IR event dataclasses: frozen, typed, canonically serializable, validated."""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import InvalidEventError
from tesseract_robotics.emitters.core.events import (
    Dwell,
    MoveKind,
    Note,
    ProgramIR,
    SetDigital,
    WaitDigital,
)
from tesseract_robotics.planning import Pose
from tests.emitters.programs import FRAMES, cart_move, joint_move


def test_joint_move_is_frozen() -> None:
    m = joint_move([0, 10, 20, 30, 40, 50])
    with pytest.raises(AttributeError):
        m.profile = "Q"  # type: ignore[misc]


def test_set_digital_key_optional() -> None:
    unnamed = SetDigital(key=None, index=7, value=True)
    named = SetDigital(key="do_array", index=3, value=False)
    assert unnamed.key is None and unnamed.index == 7
    assert named.key == "do_array"


def test_wait_digital_carries_is_input() -> None:
    w = WaitDigital(is_input=True, index=2, value=True, timeout=None)
    assert w.is_input and w.index == 2 and w.timeout is None


def test_cartesian_move_carries_framed_pose() -> None:
    m = cart_move(Pose.from_xyz(0.6, -0.1, 0.8))
    assert m.kind is MoveKind.LINEAR
    assert m.target.frames == FRAMES
    assert np.allclose(m.target.pose.translation, [0.6, -0.1, 0.8])


@pytest.mark.parametrize("seconds", [-0.1, math.nan, math.inf])
def test_dwell_rejects_out_of_domain(seconds: float) -> None:
    with pytest.raises(InvalidEventError):
        Dwell.build(seconds)


def test_dwell_accepts_zero() -> None:
    assert Dwell.build(0.0).seconds == 0.0


def test_program_ir_canonical_is_stable_and_value_sensitive() -> None:
    events = (Note("hi"), Dwell(seconds=1.5), SetDigital(key="do", index=0, value=True))
    a = ProgramIR(name="p", events=events).canonical()
    assert a == ProgramIR(name="p", events=events).canonical() and "Dwell" in a
    assert ProgramIR(name="p", events=(Dwell(seconds=2.0),)).canonical() != a


def test_canonical_includes_frames() -> None:
    a = ProgramIR(name="p", events=(cart_move(Pose.from_xyz(0.5, 0, 0.5)),)).canonical()
    assert "base_link" in a and "tool0" in a
