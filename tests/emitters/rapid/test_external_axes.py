"""External-axis (``eax``) emission — coordinated track/positioner → RAPID.

Extends the dispatcher coverage to a 7-DOF coordinated joint waypoint on the ROP
reference-cell joint ordering (``[positioner_joint_1, joint_1..joint_6]`` — the
external axis comes **first**, as the native ``ROPInvKin`` solver reports it) and
asserts the emitted ``MoveAbsJ`` carries a real ``eax`` value in mm/deg (never the
``9E+09`` sentinel), that the values round-trip within display tolerance, that a
name mismatch fails loudly, and that an uncoupled emit keeps the all-sentinel
``eax`` (the byte-identical pre-external-axis behavior; the full goldens are
locked by ``test_zigzag_square`` / ``test_full_dsl_golden``).
"""

from __future__ import annotations

import math
import re

import numpy as np
import pytest

from tesseract_robotics.emitters.rapid import (
    ExternalAxisKind,
    ExternalAxisLayout,
    ExternalAxisSpec,
    RapidProfile,
    UncoordinatedTargetError,
    emit_rapid,
)
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
)

_PROFILE = RapidProfile(speed="v200", zone="z10", tool="tool0", wobj="wobj0")
_PROFILES = {"P": _PROFILE}

#: The six IRB2400 arm joints in RAPID robax order.
_ARM = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
#: The ROP reference cell's positioner joint — the native solver lists it FIRST.
_POSITIONER = "positioner_joint_1"
_COUPLED_NAMES = (_POSITIONER, *_ARM)

#: One ``.4f`` mm display step — bounds the linear-axis round-trip rounding error.
_MM_ROUNDTRIP_ATOL = 1e-4
#: One ``.2f`` deg display step — bounds the rotary-axis round-trip rounding error.
_DEG_ROUNDTRIP_ATOL = 1e-2


def _joint_move(names, positions_rad):
    """A single FREESPACE joint move whose waypoint carries ``names`` + positions."""
    jw = JointWaypoint()
    jw.setNames(list(names))
    jw.setPosition(np.asarray(positions_rad, dtype=np.float64))
    poly = JointWaypointPoly_wrap_JointWaypoint(jw)
    move = MoveInstruction(poly, MoveInstructionType_FREESPACE, "P")
    return MoveInstructionPoly_wrap_MoveInstruction(move)


def _composite(*instructions) -> CompositeInstruction:
    composite = CompositeInstruction("test")
    for instruction in instructions:
        composite.push_back(instruction)
    return composite


def _moveabsj_line(rapid_text: str) -> str:
    [line] = [ln.strip() for ln in rapid_text.splitlines() if ln.strip().startswith("MoveAbsJ")]
    return line


def _parse_eax(moveabsj_line: str) -> list[float | None]:
    """Parse the ``eax`` slots out of a ``MoveAbsJ [[robax], [eax]], ...`` line.

    Returns a float per slot, or ``None`` for the ``9E+09`` sentinel.
    """
    match = re.search(r"\[\[.*?\],\s*\[([^\]]*)\]\], v", moveabsj_line)
    assert match is not None, moveabsj_line
    return [None if tok.strip() == "9E+09" else float(tok) for tok in match.group(1).split(",")]


def test_track_external_axis_emits_mm_in_slot_a():
    """A prismatic track (0.5 m) → 500.0000 mm in eax slot a; arm keeps six robax."""
    layout = ExternalAxisLayout.build(
        arm_joint_names=_ARM,
        external_axes=(ExternalAxisSpec.build(_POSITIONER, ExternalAxisKind.LINEAR),),
    )
    # External axis FIRST in the vector — the native ROP solver joint order.
    positions = (0.5, 0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0)
    out = emit_rapid(
        _composite(_joint_move(_COUPLED_NAMES, positions)), _PROFILES, external_axes=layout
    )
    line = _moveabsj_line(out)
    # robax are the six ARM joints (joint_2 = 90 deg), NOT the positioner value.
    assert line.startswith("MoveAbsJ [[0.00, 90.00, 0.00, 0.00, 0.00, 0.00],")
    # eax slot a carries a real millimetre value; b..f stay sentinel.
    assert "[500.0000,9E+09,9E+09,9E+09,9E+09,9E+09]]" in line


def test_positioner_external_axis_emits_degrees_in_slot_a():
    """A revolute positioner (π/4 rad) → 45.00 deg in eax slot a."""
    layout = ExternalAxisLayout.build(
        arm_joint_names=_ARM,
        external_axes=(ExternalAxisSpec.build(_POSITIONER, ExternalAxisKind.ROTARY),),
    )
    positions = (math.pi / 4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    out = emit_rapid(
        _composite(_joint_move(_COUPLED_NAMES, positions)), _PROFILES, external_axes=layout
    )
    line = _moveabsj_line(out)
    assert "[45.00,9E+09,9E+09,9E+09,9E+09,9E+09]]" in line


def test_external_axis_values_round_trip_within_tolerance():
    """Emitted eax tokens parse back to the SI inputs (mm/deg) within display tol.

    Two external axes (a linear track + a rotary turntable) placed AFTER the arm
    here — the name-based split is order-agnostic, so slot order follows the
    layout, not the waypoint vector order.
    """
    layout = ExternalAxisLayout.build(
        arm_joint_names=_ARM,
        external_axes=(
            ExternalAxisSpec.build("track", ExternalAxisKind.LINEAR),
            ExternalAxisSpec.build("turn", ExternalAxisKind.ROTARY),
        ),
    )
    names = (*_ARM, "track", "turn")
    track_m = 1.2345
    turn_deg = 37.5
    positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, track_m, math.radians(turn_deg))
    out = emit_rapid(_composite(_joint_move(names, positions)), _PROFILES, external_axes=layout)

    eax = _parse_eax(_moveabsj_line(out))
    assert eax[0] is not None and eax[1] is not None
    assert math.isclose(eax[0], track_m * 1000.0, abs_tol=_MM_ROUNDTRIP_ATOL)
    assert math.isclose(eax[1], turn_deg, abs_tol=_DEG_ROUNDTRIP_ATOL)
    assert eax[2:] == [None, None, None, None]


def test_missing_external_joint_raises_uncoordinated():
    """A layout naming a joint the waypoint lacks must fail loudly, never guess."""
    layout = ExternalAxisLayout.build(
        arm_joint_names=_ARM,
        external_axes=(ExternalAxisSpec.build(_POSITIONER, ExternalAxisKind.LINEAR),),
    )
    # Waypoint omits the positioner joint — only the six arm joints are present.
    move = _joint_move(_ARM, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    with pytest.raises(UncoordinatedTargetError, match=_POSITIONER):
        emit_rapid(_composite(move), _PROFILES, external_axes=layout)


def test_unclassified_joint_raises_uncoordinated():
    """A waypoint joint the layout does not classify must also fail loudly."""
    layout = ExternalAxisLayout.build(
        arm_joint_names=_ARM,
        external_axes=(ExternalAxisSpec.build(_POSITIONER, ExternalAxisKind.LINEAR),),
    )
    # Extra, unclassified joint "wrist_extra" beyond arm + positioner.
    names = (_POSITIONER, *_ARM, "wrist_extra")
    move = _joint_move(names, (0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1))
    with pytest.raises(UncoordinatedTargetError, match="wrist_extra"):
        emit_rapid(_composite(move), _PROFILES, external_axes=layout)


def test_no_layout_keeps_all_sentinel_eax():
    """No layout: the 6-DOF joint vector stays whole and eax is all-sentinel —
    byte-identical to the pre-external-axis emitter (external_axes defaults None)."""
    move = _joint_move(_ARM, (0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0))
    out = emit_rapid(_composite(move), _PROFILES)
    line = _moveabsj_line(out)
    assert line == (
        "MoveAbsJ [[0.00, 90.00, 0.00, 0.00, 0.00, 0.00], "
        "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v200, z10, tool0;"
    )
    # Explicit None must equal the default (documents the byte-identical contract).
    assert emit_rapid(_composite(move), _PROFILES, external_axes=None) == out
