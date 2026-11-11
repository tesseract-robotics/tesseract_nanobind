"""LsBackend: IR events to /MN logic plus the /POS position table."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.backend import drive
from tesseract_robotics.emitters.core.errors import (
    MissingConfigurationError,
    MissingProfileError,
    UnemittableTextError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    MoveKind,
    Note,
    ProgramIR,
    SetDigital,
    ToolChange,
)
from tesseract_robotics.emitters.core.frames import FramedPose, MoveFrames
from tesseract_robotics.emitters.core.identity import EmitIdentity
from tesseract_robotics.emitters.ls import LsProfile, ls_frames
from tesseract_robotics.emitters.ls.backend import LsBackend
from tesseract_robotics.planning import Pose
from tests.emitters.ls.spike import posture_config
from tests.emitters.programs import cart_move, joint_move

_FRAMES = ls_frames(user_frames={"base_link": 0, "fixture": 2}, tools={"tool0": 1})
_PROFILES = {"J": LsProfile.build(), "L": LsProfile.build(linear_speed_mms=100.0)}
_SEED = [0, 20, 10, 0, 40, 0]


def _run(events, config=posture_config) -> str:  # noqa: ANN001
    ir = ProgramIR(name="SPIKE", events=tuple(events))
    backend = LsBackend(
        profiles=_PROFILES,
        frames=_FRAMES,
        config=config,
        program_name="spike",
        identity=EmitIdentity.build(ir, profiles_repr="x"),
    )
    return drive(ir, backend)["SPIKE.ls"]


def test_joint_move_emits_frames_j_and_joint_pos() -> None:
    ls = _run([joint_move([0, 0, 0, 0, 0, 0])])
    assert "   5:UFRAME_NUM=0 ;" in ls and "   6:UTOOL_NUM=1 ;" in ls
    assert "   7:J P[1] 100% FINE ;" in ls
    assert ls.startswith("/PROG  SPIKE") and ls.rstrip().endswith("/END")
    assert "P[1]{" in ls and "J1 = 0.0000 deg" in ls


def test_linear_move_emits_l_and_cartesian_pos() -> None:
    ls = _run([cart_move(Pose.from_xyz_rpy([0.6, -0.1, 0.8], [3.14159, 0, 0]), seed_deg=_SEED)])
    assert ":L P[1] 100mm/sec FINE ;" in ls
    assert "CONFIG : 'N U T, 0, 0, 0'" in ls
    assert "X = 600.000 mm, Y = -100.000 mm, Z = 800.000 mm," in ls


def test_user_frame_switch_is_emitted_and_matches_pos() -> None:
    frames = MoveFrames("fixture", "tool0", Pose())
    move = CartesianMove(
        FramedPose(Pose(), frames), MoveKind.LINEAR, "L", (0.1, 0.3, 0.2, 0, 0.5, 0)
    )
    ls = _run([joint_move([0] * 6), move])
    assert ":UFRAME_NUM=2 ;" in ls and "UF : 2, UT : 1," in ls


def test_cartesian_without_seed_raises() -> None:
    with pytest.raises(MissingConfigurationError, match="seed"):
        _run([cart_move(Pose.from_xyz(0.6, -0.1, 0.8))])


def test_cartesian_without_resolver_raises() -> None:
    with pytest.raises(MissingConfigurationError, match="config"):
        _run([cart_move(Pose.from_xyz(0.6, -0.1, 0.8), seed_deg=_SEED)], config=None)


def test_dwell_setdigital_note() -> None:
    ls = _run([Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)])
    assert "! hello ;" in ls and "WAIT 2.5(sec) ;" in ls and "DO[3]=ON ;" in ls


def test_tool_change_and_named_signal_raise() -> None:
    with pytest.raises(UnsupportedInstructionError):
        _run([ToolChange(3)])
    with pytest.raises(UnsupportedInstructionError):
        _run([SetDigital(key="torch", index=0, value=True)])


def test_comment_with_terminator_raises() -> None:
    with pytest.raises(UnemittableTextError):
        _run([Note("a ; J P[9]")])


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([joint_move([0] * 6, profile="NOPE")])
