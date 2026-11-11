"""Full `tesseract_command_language` coverage for `emit_rapid`.

Builds synthetic `CompositeInstruction`s directly via the bindings (no planner)
and asserts the emitted RAPID covers every instruction kind, plus the error paths.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.command_language import (
    CompositeInstruction,
    MoveInstructionType,
    SetAnalogInstruction,
    SetDigitalInstruction,
    SetToolInstruction,
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
)
from tesseract_robotics.emitters.core.errors import (
    EmptyProgramError,
    IdentifierError,
    MissingProfileError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.rapid import RapidProfile, emit_rapid, rapid_frames
from tesseract_robotics.planning import Pose
from tests.emitters.programs import (
    IRB2400_POSTURE,
    cart_instruction,
    composite,
    irb2400_ik,
    joint_instruction,
)
from tests.emitters.rapid.rapid_grammar import check_module

_PROFILES = {"P": RapidProfile.build(speed="v200", zone="z10")}
_FRAMES = rapid_frames(work_objects={"base_link": "wobj0"}, tools={"tool0": "tool0"})
_TOOL_DOWN = [0.0, 1.0, 0.0, 0.0]


def _emit(c: CompositeInstruction, profiles=_PROFILES, **kw) -> str:  # noqa: ANN001, ANN003
    text = emit_rapid(c, profiles, frames=_FRAMES, posture=IRB2400_POSTURE, **kw).text
    check_module(text)
    return text


def _cart(pose: Pose, **kw) -> object:  # noqa: ANN003
    return cart_instruction(pose, profile="P", seed_rad=irb2400_ik(pose), **kw)


def _proc_lines(rapid_text: str) -> list[str]:
    """Statements inside the PROC (framing, comments stripped)."""
    return [
        line.strip()
        for line in rapid_text.splitlines()
        if line.strip()
        and not line.lstrip().startswith(("MODULE ", "PROC ", "ENDPROC", "ENDMODULE", "!"))
    ]


def test_linear_cartesian_lowers_to_movel() -> None:
    pose = Pose.from_xyz_quat([0.8, -0.2, 0.62], _TOOL_DOWN)
    [movel] = _proc_lines(_emit(composite(_cart(pose))))
    assert movel.startswith("MoveL [[800.000, -200.000, 620.000],")
    assert movel.endswith(", v200, z10, tool0 \\WObj:=wobj0;")


def test_freespace_cartesian_lowers_to_movej() -> None:
    pose = Pose.from_xyz_quat([0.8, 0.0, 0.62], _TOOL_DOWN)
    [movej] = _proc_lines(_emit(composite(_cart(pose, move_type=MoveInstructionType.FREESPACE))))
    assert movej.startswith("MoveJ [[800.000, 0.000, 620.000],")


def test_joint_waypoint_lowers_to_moveabsj() -> None:
    c = composite(joint_instruction([0, math.pi / 2, 0, 0, 0, 0], profile="P"))
    [moveabsj] = _proc_lines(_emit(c))
    assert moveabsj.startswith("MoveAbsJ [[0.0000, 90.0000, 0.0000, 0.0000, 0.0000, 0.0000],")
    assert moveabsj.endswith(", v200, z10, tool0;")


def test_wait_time_lowers_to_waittime() -> None:
    [line] = _proc_lines(_emit(composite(WaitInstruction(1.5)), {}))
    assert line == "WaitTime 1.5;"


def test_timer_lowers_to_waittime_plus_setdo() -> None:
    timer = TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 7)
    assert _proc_lines(_emit(composite(timer), {})) == ["WaitTime 2.0;", "SetDO do7, 1;"]


def test_setdigital_with_index_zero_emits_bare_key() -> None:
    [line] = _proc_lines(_emit(composite(SetDigitalInstruction("do_torch", 0, True)), {}))
    assert line == "SetDO do_torch, 1;"


def test_setdigital_with_nonzero_index_emits_indexed_signal() -> None:
    [line] = _proc_lines(_emit(composite(SetDigitalInstruction("do_array", 3, False)), {}))
    assert line == "SetDO do_array{3}, 0;"


def test_setanalog_emits_setao_with_value() -> None:
    [line] = _proc_lines(_emit(composite(SetAnalogInstruction("ao_voltage", 0, 24.5)), {}))
    assert line == "SetAO ao_voltage, 24.5;"


def test_illegal_signal_name_raises() -> None:
    with pytest.raises(IdentifierError):
        _emit(composite(SetDigitalInstruction("do torch", 0, True)), {})


def test_settool_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="SetTool"):
        _emit(composite(SetToolInstruction(7)), {})


def test_nested_composite_walks_recursively() -> None:
    pose = Pose.from_xyz_quat([0.8, 0.0, 0.5], _TOOL_DOWN)
    inner = CompositeInstruction("inner")
    inner.push_back(_cart(pose))
    out = _emit(composite(inner, _cart(pose)))
    assert len([line for line in out.splitlines() if "MoveL " in line]) == 2


def test_circular_move_raises_unsupported() -> None:
    pose = Pose.from_xyz_quat([0.8, 0.0, 0.5], _TOOL_DOWN)
    with pytest.raises(UnsupportedInstructionError, match="CIRCULAR"):
        _emit(composite(_cart(pose, move_type=MoveInstructionType.CIRCULAR)))


def test_missing_profile_raises() -> None:
    pose = Pose.from_xyz_quat([0.8, 0.0, 0.5], _TOOL_DOWN)
    c = composite(cart_instruction(pose, profile="UNKNOWN", seed_rad=np.zeros(6) + 0.1))
    with pytest.raises(MissingProfileError, match="'UNKNOWN'"):
        _emit(c)


def test_empty_composite_raises() -> None:
    with pytest.raises(EmptyProgramError, match="no leaf instructions"):
        emit_rapid(CompositeInstruction("empty"), {}, frames=_FRAMES)


def test_emitted_confdata_matches_seed() -> None:
    """Parse-back: every robtarget's confdata equals the seed's quadrants and posture."""
    pose = Pose.from_xyz_quat([0.3, -0.7, 0.9], _TOOL_DOWN)
    seed = irb2400_ik(pose, near=np.deg2rad([-60, 10, 10, 0, 60, 0]))
    out = _emit(composite(cart_instruction(pose, profile="P", seed_rad=seed)))
    [conf] = [line.split("],[")[2] for line in _proc_lines(out)]
    quadrants = [math.floor(seed[i] / (math.pi / 2)) for i in (0, 3, 5)]
    assert conf.split(", ")[:3] == [str(q) for q in quadrants]
