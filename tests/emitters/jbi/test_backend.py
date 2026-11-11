"""JbiBackend: IR events to a two-section .jbi body."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.backend import drive
from tesseract_robotics.emitters.core.errors import (
    MissingProfileError,
    ToolConflictError,
    UnemittableTextError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    Dwell,
    JointMove,
    Note,
    ProgramIR,
    SetAnalog,
    SetDigital,
)
from tesseract_robotics.emitters.core.frames import MoveFrames
from tesseract_robotics.emitters.jbi import JbiProfile, jbi_frames
from tesseract_robotics.emitters.jbi.backend import JbiBackend
from tesseract_robotics.planning import Pose
from tests.emitters.jbi.spike import CALIBRATION
from tests.emitters.programs import cart_move, joint_move

_FRAMES = jbi_frames(tools={"tool0": 2, "gun": 3})
_PROFILES = {"J": JbiProfile.build(joint_speed_percent=50.0)}


def _run(events) -> str:  # noqa: ANN001
    ir = ProgramIR(name="SPIKE", events=tuple(events))
    backend = JbiBackend(
        profiles=_PROFILES, frames=_FRAMES, calibration=CALIBRATION, program_name="SPIKE"
    )
    return drive(ir, backend)["SPIKE.jbi"]


def test_joint_move_emits_movj_and_position() -> None:
    jbi = _run([joint_move([0, 0, 0, 0, 0, 0])])
    assert jbi.startswith("/JOB") and "//NAME SPIKE" in jbi and jbi.rstrip().endswith("END")
    assert "///NPOS 1,0,0,0,0,0" in jbi and "///TOOL 2" in jbi and "///POSTYPE PULSE" in jbi
    assert "MOVJ C00000 VJ=50.00" in jbi
    assert "C00000=0,0,0,0,0,100" in jbi


def test_cartesian_move_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="RCONF"):
        _run([cart_move(Pose.from_xyz(0.6, -0.1, 0.8), seed_deg=[0] * 6)])


def test_tool_conflict_raises() -> None:
    gun = MoveFrames("base_link", "gun", Pose())
    with pytest.raises(ToolConflictError, match=r"\[2, 3\]"):
        _run([joint_move([0] * 6), JointMove((0.1,) * 6, "J", gun)])


def test_io_lines() -> None:
    jbi = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(None, 3, True), SetAnalog(None, 1, 12.7)]
    )
    assert "'hello" in jbi and "TIMER T=2.5" in jbi and "DOUT OT#(3) ON" in jbi
    assert "AOUT AO#(1) 12.70" in jbi
    assert "///TOOL" not in jbi


def test_analog_out_of_range_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="AOUT"):
        _run([SetAnalog(None, 0, 1.0)])
    with pytest.raises(UnsupportedInstructionError, match="AOUT"):
        _run([SetAnalog(None, 1, 15.0)])


def test_long_comment_raises() -> None:
    with pytest.raises(UnemittableTextError, match="maximum is 32"):
        _run([Note("x" * 33)])


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([joint_move([0] * 6, profile="NOPE")])
