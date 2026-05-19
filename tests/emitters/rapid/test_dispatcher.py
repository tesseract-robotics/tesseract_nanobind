"""Full tesseract `tesseract_command_language` coverage for the `emit_rapid`
dispatcher.

Builds synthetic `CompositeInstruction`s directly via the C++ bindings (no
planner involved) and asserts the lowered RAPID covers every instruction kind
the dispatcher knows about, plus the three error paths.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.rapid import (
    EmptyProgramError,
    MissingProfileError,
    RapidProfile,
    UnsupportedInstructionError,
    emit_rapid,
)
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_CIRCULAR,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    SetAnalogInstruction,
    SetDigitalInstruction,
    SetToolInstruction,
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
)

_PROFILE = RapidProfile(speed="v200", zone="z10", tool="tool0", wobj="wobj0")
_PROFILES = {"P": _PROFILE}


def _cart_waypoint_poly(pose: Pose):
    # Pose IS-A Isometry3d (subclass) — passed directly, no wrapping needed.
    return CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose))


def _joint_waypoint_poly(joints_rad, names):
    jw = JointWaypoint()
    jw.setNames(names)
    jw.setPosition(np.asarray(joints_rad, dtype=np.float64))
    return JointWaypointPoly_wrap_JointWaypoint(jw)


def _move(waypoint_poly, *, profile: str = "P", move_type=MoveInstructionType_LINEAR):
    """Build a `MoveInstructionPoly` from a typed waypoint poly.

    Uses the 3-arg `MoveInstruction(waypoint, type, profile)` SWIG-compat
    overload, then wraps via `MoveInstructionPoly_wrap_MoveInstruction` —
    the canonical pattern from the existing `tests/tesseract_command_language`
    test suite.
    """
    mi = MoveInstruction(waypoint_poly, move_type, profile)
    return MoveInstructionPoly_wrap_MoveInstruction(mi)


def _composite_with(*instructions) -> CompositeInstruction:
    c = CompositeInstruction("test")
    for instr in instructions:
        c.push_back(instr)
    return c


def _proc_lines(rapid_text: str) -> list[str]:
    """Strip MODULE / PROC framing; return only the inner statements (whitespace-trimmed)."""
    return [
        line.strip()
        for line in rapid_text.splitlines()
        if line.strip()
        and not line.lstrip().startswith(("MODULE ", "PROC ", "ENDPROC", "ENDMODULE"))
    ]


# ---- Move instructions ----------------------------------------------------


def test_linear_cartesian_lowers_to_movel():
    pose = Pose.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 0, 1)
    composite = _composite_with(_move(_cart_waypoint_poly(pose)))
    out = emit_rapid(composite, _PROFILES)

    [movel] = _proc_lines(out)
    assert movel.startswith("MoveL [[500.0000, -200.0000, 620.0000],")
    assert movel.endswith(", v200, z10, tool0 \\Wobj:=wobj0;")


def test_freespace_cartesian_lowers_to_movej():
    pose = Pose.from_xyz_quat(0.5, 0.0, 0.62, 0, 0, 0, 1)
    composite = _composite_with(
        _move(_cart_waypoint_poly(pose), move_type=MoveInstructionType_FREESPACE)
    )
    out = emit_rapid(composite, _PROFILES)

    [movej] = _proc_lines(out)
    assert movej.startswith("MoveJ [[500.0000, 0.0000, 620.0000],")


def test_joint_waypoint_lowers_to_moveabsj():
    names = [f"j{i + 1}" for i in range(6)]
    composite = _composite_with(_move(_joint_waypoint_poly([0, math.pi / 2, 0, 0, 0, 0], names)))
    out = emit_rapid(composite, _PROFILES)

    [moveabsj] = _proc_lines(out)
    assert moveabsj.startswith("MoveAbsJ [[0.00, 90.00, 0.00, 0.00, 0.00, 0.00],")


# ---- I/O instructions -----------------------------------------------------


def test_wait_time_lowers_to_waittime():
    composite = _composite_with(WaitInstruction(1.5))
    [line] = _proc_lines(emit_rapid(composite, {}))
    assert line == "WaitTime 1.5;"


def test_timer_lowers_to_waittime_plus_setdo():
    """TimerInstruction = `wait N seconds, then drive digital IO i {high,low}`.
    RAPID is imperative; lower as two consecutive statements."""
    timer = TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 7)
    composite = _composite_with(timer)
    lines = _proc_lines(emit_rapid(composite, {}))
    assert lines[0] == "WaitTime 2.0;"
    assert lines[1] == "SetDO do7, 1;"


def test_setdigital_with_index_zero_emits_bare_key():
    composite = _composite_with(SetDigitalInstruction("do_torch", 0, True))
    [line] = _proc_lines(emit_rapid(composite, {}))
    assert line == "SetDO do_torch, 1;"


def test_setdigital_with_nonzero_index_emits_indexed_signal():
    """RAPID array-indexed signal name is `<key>{<index>}` (e.g. `do_array{3}`)."""
    composite = _composite_with(SetDigitalInstruction("do_array", 3, False))
    [line] = _proc_lines(emit_rapid(composite, {}))
    assert line == "SetDO do_array{3}, 0;"


def test_setanalog_emits_setao_with_value():
    composite = _composite_with(SetAnalogInstruction("ao_voltage", 0, 24.5))
    [line] = _proc_lines(emit_rapid(composite, {}))
    assert line == "SetAO ao_voltage, 24.5;"


def test_settool_emits_comment_with_int_id():
    """SetToolInstruction stores a tool ID (int), not a name. Comment marker
    only — caller swaps RapidProfile.tool on subsequent moves."""
    composite = _composite_with(SetToolInstruction(7))
    [line] = _proc_lines(emit_rapid(composite, {}))
    assert line == "! tool change to tool id 7"


# ---- Recursion ------------------------------------------------------------


def test_nested_composite_walks_recursively():
    pose = Pose.from_xyz_quat(0.5, 0.0, 0.5, 0, 0, 0, 1)
    inner = _composite_with(_move(_cart_waypoint_poly(pose)))
    outer = _composite_with(inner, _move(_cart_waypoint_poly(pose)))
    out = emit_rapid(outer, _PROFILES)
    movel_lines = [line for line in out.splitlines() if "MoveL " in line]
    assert len(movel_lines) == 2  # both the nested-composite move + the top-level move


# ---- Error paths ----------------------------------------------------------


def test_circular_move_raises_unsupported():
    pose = Pose.from_xyz_quat(0.5, 0.0, 0.5, 0, 0, 0, 1)
    composite = _composite_with(
        _move(_cart_waypoint_poly(pose), move_type=MoveInstructionType_CIRCULAR)
    )
    with pytest.raises(UnsupportedInstructionError, match="CIRCULAR"):
        emit_rapid(composite, _PROFILES)


def test_missing_profile_raises():
    pose = Pose.from_xyz_quat(0.5, 0.0, 0.5, 0, 0, 0, 1)
    composite = _composite_with(_move(_cart_waypoint_poly(pose), profile="UNKNOWN"))
    with pytest.raises(MissingProfileError, match="'UNKNOWN'"):
        emit_rapid(composite, _PROFILES)


def test_empty_composite_raises():
    with pytest.raises(EmptyProgramError, match="no leaf instructions"):
        emit_rapid(CompositeInstruction("empty"), {})
