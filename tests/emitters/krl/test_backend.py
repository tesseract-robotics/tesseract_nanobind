"""KrlBackend: consume IR events, accumulate a KRL .src body."""

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
from tesseract_robotics.emitters.krl.backend import KrlBackend
from tesseract_robotics.emitters.krl.profile import KrlProfile
from tesseract_robotics.planning import Pose


def _run(events, profiles) -> str:
    backend = KrlBackend(profiles=profiles, program_name="SPIKE")
    backend.prog_start("SPIKE")
    for e in events:
        _dispatch(e, backend)
    return backend.prog_finish()["SPIKE.src"]


def test_joint_move_emits_ptp() -> None:
    src = _run([JointMove(joints=tuple(np.zeros(6)), profile="J")], {"J": KrlProfile.build()})
    assert "PTP {A1 0.00000,A2 0.00000,A3 0.00000,A4 0.00000,A5 0.00000,A6 0.00000}" in src
    assert src.startswith("DEF SPIKE()") and src.rstrip().endswith("END")


def test_linear_move_emits_lin_and_vel_cp() -> None:
    src = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.LINEAR,
                profile="L",
                seed_joints=None,
            )
        ],
        {"L": KrlProfile.build(cp_speed_mms=100.0)},
    )
    assert "$VEL.CP = 0.10000" in src
    assert "LIN {X 600.000,Y -100.000,Z 800.000" in src


def test_dwell_setdigital_note() -> None:
    src = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)],
        {},
    )
    assert "; hello" in src and "WAIT SEC 2.5" in src and "$OUT[3] = TRUE" in src


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([JointMove(joints=tuple(np.zeros(6)), profile="NOPE")], {"J": KrlProfile.build()})
