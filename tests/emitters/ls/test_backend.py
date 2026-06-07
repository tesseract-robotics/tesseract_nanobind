"""LsBackend: consume IR events, accumulate /MN logic + /POS position table."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.core.backend import _dispatch
from tesseract_robotics.emitters.core.errors import MissingProfileError
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    Note,
    SetDigital,
)
from tesseract_robotics.emitters.ls.backend import LsBackend
from tesseract_robotics.emitters.ls.profile import LsProfile
from tesseract_robotics.planning import Pose


def _run(events, profiles) -> str:
    backend = LsBackend(profiles=profiles, program_name="SPIKE")
    backend.prog_start()
    for e in events:
        _dispatch(e, backend)
    return backend.prog_finish()["SPIKE.ls"]


def test_joint_move_emits_j_and_joint_pos() -> None:
    ls = _run([JointMove(joints=tuple(np.zeros(6)), profile="J")], {"J": LsProfile.build()})
    assert "   1:J P[1] 100% FINE ;" in ls
    assert ls.startswith("/PROG  SPIKE") and ls.rstrip().endswith("/END")
    assert "/MN" in ls and "/POS" in ls
    assert "P[1]{" in ls and "J1 = 0.000 deg" in ls


def test_linear_move_emits_l_and_cartesian_pos() -> None:
    ls = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.LINEAR,
                profile="L",
                seed_joints=None,
            )
        ],
        {"L": LsProfile.build(linear_speed_mms=100.0)},
    )
    assert "   1:L P[1] 100mm/sec FINE ;" in ls
    assert "P[1]{" in ls and "CONFIG : 'N U T, 0, 0, 0'" in ls
    assert "X = 600.000 mm, Y = -100.000 mm, Z = 800.000 mm," in ls


def test_freespace_cartesian_emits_j() -> None:
    ls = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.FREESPACE,
                profile="F",
                seed_joints=None,
            )
        ],
        {"F": LsProfile.build()},
    )
    assert "   1:J P[1] 100% FINE ;" in ls


def test_dwell_setdigital_note() -> None:
    ls = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)],
        {},
    )
    assert "! hello ;" in ls and "WAIT 2.5(sec) ;" in ls and "DO[3]=ON ;" in ls


def test_positions_indexed_in_emit_order() -> None:
    ls = _run(
        [
            JointMove(joints=tuple(np.zeros(6)), profile="J"),
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.LINEAR,
                profile="J",
                seed_joints=None,
            ),
        ],
        {"J": LsProfile.build()},
    )
    assert "   1:J P[1] 100% FINE ;" in ls
    assert "   2:L P[2] 100mm/sec FINE ;" in ls
    assert "P[1]{" in ls and "P[2]{" in ls


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([JointMove(joints=tuple(np.zeros(6)), profile="NOPE")], {"J": LsProfile.build()})
