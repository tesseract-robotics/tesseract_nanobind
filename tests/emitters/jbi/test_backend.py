"""JbiBackend: consume IR events, accumulate a two-section .jbi body."""

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
from tesseract_robotics.emitters.jbi.backend import JbiBackend, MissingSeedError
from tesseract_robotics.emitters.jbi.profile import JbiProfile
from tesseract_robotics.planning import Pose


def _run(events, profiles) -> str:
    backend = JbiBackend(profiles=profiles, program_name="SPIKE")
    backend.prog_start()
    for e in events:
        _dispatch(e, backend)
    return backend.prog_finish()["SPIKE.jbi"]


def test_joint_move_emits_movj_and_position() -> None:
    jbi = _run(
        [JointMove(joints=tuple(np.zeros(6)), profile="J")],
        {"J": JbiProfile.build(joint_speed_percent=50.0)},
    )
    # File scaffolding.
    assert jbi.startswith("/JOB")
    assert "//NAME SPIKE" in jbi
    assert "//POS" in jbi and "//INST" in jbi
    assert "///POSTYPE PULSE" in jbi and "///PULSE" in jbi
    assert jbi.rstrip().endswith("END")
    # The move references position C00000, and the table holds that row.
    assert "MOVJ C00000 VJ=50.00" in jbi
    assert "C00000=0.000,0.000,0.000,0.000,0.000,0.000" in jbi
    assert "///NPOS 1,0,0,0,0,0" in jbi


def test_linear_move_with_seed_emits_movl() -> None:
    seed = tuple(np.deg2rad([10, 20, 30, 40, 50, 60]))
    jbi = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.LINEAR,
                profile="L",
                seed_joints=seed,
            )
        ],
        {"L": JbiProfile.build(linear_speed_mms=123.0)},
    )
    assert "MOVL C00000 V=123.0" in jbi
    assert "C00000=10.000,20.000,30.000,40.000,50.000,60.000" in jbi


def test_freespace_cartesian_with_seed_emits_movj() -> None:
    seed = tuple(np.zeros(6))
    jbi = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz(0.6, -0.1, 0.8),
                kind=MoveKind.FREESPACE,
                profile="F",
                seed_joints=seed,
            )
        ],
        {"F": JbiProfile.build(joint_speed_percent=25.0)},
    )
    assert "MOVJ C00000 VJ=25.00" in jbi


def test_dwell_setdigital_note() -> None:
    jbi = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)],
        {},
    )
    assert "'hello" in jbi and "TIMER T=2.5" in jbi and "DOUT OT#(3) ON" in jbi


def test_position_indices_increment() -> None:
    jbi = _run(
        [
            JointMove(joints=tuple(np.zeros(6)), profile="J"),
            JointMove(joints=tuple(np.deg2rad([1, 2, 3, 4, 5, 6])), profile="J"),
        ],
        {"J": JbiProfile.build()},
    )
    assert "MOVJ C00000 VJ=" in jbi and "MOVJ C00001 VJ=" in jbi
    assert "C00000=" in jbi and "C00001=" in jbi
    assert "///NPOS 2,0,0,0,0,0" in jbi


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([JointMove(joints=tuple(np.zeros(6)), profile="NOPE")], {"J": JbiProfile.build()})


def test_cartesian_without_seed_raises() -> None:
    with pytest.raises(MissingSeedError):
        _run(
            [
                CartesianMove(
                    pose=Pose.from_xyz(0.6, -0.1, 0.8),
                    kind=MoveKind.LINEAR,
                    profile="L",
                    seed_joints=None,
                )
            ],
            {"L": JbiProfile.build()},
        )
