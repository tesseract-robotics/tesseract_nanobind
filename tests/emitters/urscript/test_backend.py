"""UrScriptBackend: consume IR events, accumulate a URScript .script body."""

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
from tesseract_robotics.emitters.urscript.backend import UrScriptBackend
from tesseract_robotics.emitters.urscript.profile import UrScriptProfile
from tesseract_robotics.planning import Pose


def _run(events, profiles) -> str:
    backend = UrScriptBackend(profiles=profiles, program_name="SPIKE")
    backend.prog_start()
    for e in events:
        _dispatch(e, backend)
    return backend.prog_finish()["SPIKE.script"]


def test_joint_move_emits_movej() -> None:
    src = _run(
        [JointMove(joints=tuple(np.zeros(6)), profile="J")],
        {"J": UrScriptProfile.build()},
    )
    assert "movej([0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]" in src
    assert "a=1.400000" in src and "v=1.050000" in src
    assert src.startswith("def SPIKE():") and src.rstrip().endswith("end")


def test_linear_move_emits_movel() -> None:
    src = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0.0, 0.0]),
                kind=MoveKind.LINEAR,
                profile="L",
                seed_joints=None,
            )
        ],
        {"L": UrScriptProfile.build(linear_speed_ms=0.1, linear_accel_mss=0.5)},
    )
    assert "movel(p[0.600000, -0.100000, 0.800000, 3.141593, 0.000000, 0.000000]" in src
    assert "a=0.500000" in src and "v=0.100000" in src


def test_freespace_cartesian_emits_movej_inverse_kin() -> None:
    src = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0.0, 0.0]),
                kind=MoveKind.FREESPACE,
                profile="F",
                seed_joints=None,
            )
        ],
        {"F": UrScriptProfile.build()},
    )
    assert "movej(get_inverse_kin(p[0.600000, -0.100000, 0.800000" in src


def test_dwell_setdigital_note() -> None:
    src = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)],
        {},
    )
    assert "# hello" in src
    assert "sleep(2.500)" in src
    assert "set_standard_digital_out(3, True)" in src


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run(
            [JointMove(joints=tuple(np.zeros(6)), profile="NOPE")],
            {"J": UrScriptProfile.build()},
        )
