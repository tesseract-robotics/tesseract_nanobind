"""KrlBackend: IR events to a KRL .src body."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.backend import drive
from tesseract_robotics.emitters.core.errors import (
    ExternalAxisError,
    MissingConfigurationError,
    MissingProfileError,
    UnboundFrameError,
    UnemittableTextError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    Dwell,
    JointMove,
    MoveKind,
    Note,
    ProgramIR,
    SetDigital,
    ToolChange,
)
from tesseract_robotics.emitters.core.identity import EmitIdentity
from tesseract_robotics.emitters.krl import KrlNullFrame, KrlProfile, krl_frames
from tesseract_robotics.emitters.krl.backend import KrlBackend
from tesseract_robotics.planning import Pose
from tests.emitters.programs import FRAMES, cart_move, joint_move

_FRAMES = krl_frames(bases={"base_link": 3}, tools={"tool0": KrlNullFrame.NULLFRAME})
_PROFILES = {"J": KrlProfile.build(), "L": KrlProfile.build(cp_speed_mms=100.0)}


def _run(events, frames=_FRAMES) -> str:  # noqa: ANN001
    ir = ProgramIR(name="SPIKE", events=tuple(events))
    backend = KrlBackend(
        profiles=_PROFILES,
        frames=frames,
        program_name="SPIKE",
        identity=EmitIdentity.build(ir, profiles_repr="x"),
    )
    return drive(ir, backend)["SPIKE.src"]


def test_joint_move_emits_tool_and_ptp() -> None:
    src = _run([joint_move([0, 0, 0, 0, 0, 0])])
    assert "$TOOL = $NULLFRAME" in src
    assert "PTP {A1 0.0000,A2 0.0000,A3 0.0000,A4 0.0000,A5 0.0000,A6 0.0000}" in src
    assert src.startswith("DEF SPIKE()") and src.rstrip().endswith("END")


def test_ptp_velocity_set_once() -> None:
    src = _run([joint_move([0, 0, 0, 0, 0, 0]), joint_move([1, 0, 0, 0, 0, 0])])
    assert src.count("BAS(#VEL_PTP, 100)") == 1


def test_linear_move_emits_base_lin_and_vel_cp() -> None:
    src = _run([cart_move(Pose.from_xyz(0.6, -0.1, 0.8))])
    assert "$BASE = BASE_DATA[3]" in src
    assert "$VEL.CP = 0.100000" in src
    assert "LIN {X 600.000,Y -100.000,Z 800.000" in src


def test_freespace_cartesian_raises() -> None:
    with pytest.raises(MissingConfigurationError, match="Status bit 2"):
        _run([cart_move(Pose.from_xyz(0.6, -0.1, 0.8), MoveKind.FREESPACE, seed_deg=[0] * 6)])


def test_dwell_setdigital_note() -> None:
    src = _run([Note("hello"), Dwell(seconds=2.5), SetDigital(key=None, index=3, value=True)])
    assert "; hello" in src and "WAIT SEC 2.5" in src and "$OUT[3] = TRUE" in src


def test_named_signal_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="named signal"):
        _run([SetDigital(key="do_torch", index=0, value=True)])


def test_tool_change_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="SetTool"):
        _run([ToolChange(2)])


def test_comment_newline_raises() -> None:
    with pytest.raises(UnemittableTextError):
        _run([Note("a\nEND")])


def test_external_axes_raise() -> None:
    with pytest.raises(ExternalAxisError):
        _run([JointMove((0.1,) * 7, "J", FRAMES)])


def test_unbound_tool_raises() -> None:
    with pytest.raises(UnboundFrameError, match="tool0"):
        _run([joint_move([0] * 6)], frames=krl_frames(bases={}, tools={}))


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([joint_move([0] * 6, profile="NOPE")])
