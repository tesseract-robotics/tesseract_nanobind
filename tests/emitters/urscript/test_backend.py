"""UrScriptBackend: IR events to a URScript .script body."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.core.backend import drive
from tesseract_robotics.emitters.core.errors import (
    MissingConfigurationError,
    MissingProfileError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    MoveKind,
    Note,
    ProgramIR,
    SetAnalog,
    SetDigital,
    ToolChange,
)
from tesseract_robotics.emitters.core.frames import FramedPose, MoveFrames
from tesseract_robotics.emitters.core.identity import EmitIdentity
from tesseract_robotics.emitters.urscript import UrScriptProfile, ur_frames
from tesseract_robotics.emitters.urscript.backend import UrScriptBackend
from tesseract_robotics.planning import Pose
from tests.emitters.programs import cart_move, joint_move

_FRAMES = ur_frames(
    features={"base_link": Pose(), "table": Pose.from_xyz(0.1, 0.2, 0.0)},
    tcps={"tool0": Pose()},
)
_PROFILES = {
    "J": UrScriptProfile.build(),
    "L": UrScriptProfile.build(linear_speed_ms=0.1, linear_accel_mss=0.5),
}
_DOWN = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0.0, 0.0])


def _run(events) -> str:  # noqa: ANN001
    ir = ProgramIR(name="SPIKE", events=tuple(events))
    backend = UrScriptBackend(
        profiles=_PROFILES,
        frames=_FRAMES,
        program_name="SPIKE",
        identity=EmitIdentity.build(ir, profiles_repr="x"),
    )
    return drive(ir, backend)["SPIKE.script"]


def test_joint_move_emits_tcp_and_movej() -> None:
    src = _run([joint_move([0] * 6)])
    assert "set_tcp(p[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000])" in src
    assert "movej([0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]" in src
    assert src.startswith("def SPIKE():") and src.rstrip().endswith("end")


def test_linear_move_emits_movel() -> None:
    src = _run([cart_move(_DOWN)])
    assert "movel(p[0.600000, -0.100000, 0.800000, 3.141593, 0.000000, 0.000000]" in src
    assert "a=0.500000" in src and "v=0.100000" in src


def test_feature_frame_is_applied_exactly() -> None:
    frames = MoveFrames("table", "tool0", Pose())
    move = CartesianMove(FramedPose(_DOWN, frames), MoveKind.LINEAR, "L", None)
    assert "movel(p[0.700000, 0.100000, 0.800000," in _run([move])


def test_tcp_offset_composes_with_bound_tcp() -> None:
    frames = MoveFrames("base_link", "tool0", Pose.from_xyz(0.0, 0.0, 0.2))
    move = CartesianMove(FramedPose(_DOWN, frames), MoveKind.LINEAR, "L", None)
    assert "set_tcp(p[0.000000, 0.000000, 0.200000," in _run([move])


def test_freespace_cartesian_passes_seed_as_qnear() -> None:
    src = _run([cart_move(_DOWN, MoveKind.FREESPACE, profile="J", seed_deg=[0, -90, 90, 0, 90, 0])])
    assert "movej(get_inverse_kin(p[0.600000, -0.100000, 0.800000" in src
    assert "qnear=[0.000000, -1.570796, 1.570796, 0.000000, 1.570796, 0.000000])" in src


def test_freespace_cartesian_without_seed_raises() -> None:
    with pytest.raises(MissingConfigurationError, match="qnear"):
        _run([cart_move(_DOWN, MoveKind.FREESPACE, profile="J")])


def test_io_and_note() -> None:
    src = _run(
        [Note("hello"), Dwell(seconds=2.5), SetDigital(None, 3, True), SetAnalog(None, 1, 0.25)]
    )
    assert "# hello" in src and "sleep(2.500000)" in src
    assert "set_standard_digital_out(3, True)" in src
    assert "set_standard_analog_out(1, 0.250000)" in src


@pytest.mark.parametrize(
    "event", [SetAnalog(None, 2, 0.5), SetAnalog(None, 0, 1.5), SetAnalog("ao", 0, 0.5)]
)
def test_analog_out_of_range_raises(event: SetAnalog) -> None:
    with pytest.raises(UnsupportedInstructionError):
        _run([event])


def test_tool_change_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="SetTool"):
        _run([ToolChange(1)])


def test_missing_profile_raises() -> None:
    with pytest.raises(MissingProfileError):
        _run([joint_move([0] * 6, profile="NOPE")])
