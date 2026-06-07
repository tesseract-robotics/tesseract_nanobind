"""Lower a tesseract ``CompositeInstruction`` to the brand-free event IR.

The single walk every backend shares — mirrors ``emitters.rapid.emit`` but
emits typed ``Event`` objects so brand formatting lives downstream. This is the
anti-corruption layer at the nanobind boundary: loosely-typed binding objects in,
strictly-typed dataclasses out. One instruction may expand to several events
(Timer → Dwell + SetDigital). Unknown instructions raise (no silent fallback).
"""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypointPoly,
    CompositeInstruction,
    InstructionPoly,
    MoveInstructionPoly,
    MoveInstructionType,
    TimerInstructionType,
    WaitInstructionType,
    WaypointPoly_as_CartesianWaypointPoly,
    WaypointPoly_as_JointWaypointPoly,
)

from .errors import EmptyProgramError, UnsupportedInstructionError
from .events import (
    CartesianMove,
    Dwell,
    Event,
    JointMove,
    MoveKind,
    ProgramIR,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)

#: ``ProgramIR`` name used when the ``CompositeInstruction`` has no description.
_DEFAULT_PROGRAM_NAME = "program"
#: Wait-instruction members that block on an *input* (vs an output) signal.
#: Compared explicitly rather than substring-sniffing ``kind.name`` — an upstream
#: enum rename would silently invert a name-sniff, but breaks this loudly.
_WAIT_INPUT_TYPES = frozenset(
    {WaitInstructionType.DIGITAL_INPUT_HIGH, WaitInstructionType.DIGITAL_INPUT_LOW}
)
#: Wait-instruction members that block until the signal is *HIGH* (vs LOW).
_WAIT_HIGH_TYPES = frozenset(
    {WaitInstructionType.DIGITAL_INPUT_HIGH, WaitInstructionType.DIGITAL_OUTPUT_HIGH}
)


def lower(composite: CompositeInstruction) -> ProgramIR:
    """Walk ``composite`` → ``ProgramIR``. Raises ``EmptyProgramError`` if no leaves."""
    leaves = list(_walk(composite))
    if not leaves:
        raise EmptyProgramError(composite.getDescription())
    events: list[Event] = []
    for instr in leaves:
        events.extend(_lower_instruction(instr))
    return ProgramIR(name=composite.getDescription() or _DEFAULT_PROGRAM_NAME, events=tuple(events))


def _walk(composite: CompositeInstruction) -> Iterator[InstructionPoly]:
    """Yield leaf instructions, recursing into nested composites."""
    for instr in composite.getInstructions():
        if instr.isCompositeInstruction():
            yield from _walk(instr.asCompositeInstruction())
        else:
            yield instr


def _lower_instruction(instr: InstructionPoly) -> list[Event]:
    if instr.isMoveInstruction():
        return [_lower_move(instr.asMoveInstruction())]

    if instr.isWaitInstruction():
        wait = instr.asWaitInstruction()
        kind = wait.getWaitType()
        if kind == WaitInstructionType.TIME:
            return [Dwell(seconds=float(wait.getWaitTime()))]
        return [
            WaitDigital(
                is_input=kind in _WAIT_INPUT_TYPES,
                index=int(wait.getWaitIO()),
                value=kind in _WAIT_HIGH_TYPES,
                timeout=None,
            )
        ]

    if instr.isTimerInstruction():
        # tesseract Timer = "after N s drive digital IO i {high,low}" → Dwell + SetDigital.
        # key=None ⇒ unnamed output #i (RAPID renders do<i>, KRL renders $OUT[i]).
        timer = instr.asTimerInstruction()
        return [
            Dwell(seconds=float(timer.getTimerTime())),
            SetDigital(
                key=None,
                index=int(timer.getTimerIO()),
                value=timer.getTimerType() == TimerInstructionType.DIGITAL_OUTPUT_HIGH,
            ),
        ]

    if instr.isSetDigitalInstruction():
        sd = instr.asSetDigitalInstruction()
        return [SetDigital(key=sd.getKey(), index=int(sd.getIndex()), value=bool(sd.getValue()))]

    if instr.isSetAnalogInstruction():
        sa = instr.asSetAnalogInstruction()
        return [SetAnalog(key=sa.getKey(), index=int(sa.getIndex()), value=float(sa.getValue()))]

    if instr.isSetToolInstruction():
        return [ToolChange(tool_id=int(instr.asSetToolInstruction().getTool()))]

    raise UnsupportedInstructionError(
        f"Unrecognized instruction polymorphism: {instr.getDescription()!r}"
    )


def _lower_move(move: MoveInstructionPoly) -> Event:
    profile = move.getProfile()
    waypoint = move.getWaypoint()
    move_type = move.getMoveType()

    if waypoint.isCartesianWaypoint():
        cart = WaypointPoly_as_CartesianWaypointPoly(waypoint)
        pose = Pose(cart.getTransform().matrix)
        if move_type == MoveInstructionType.LINEAR:
            kind = MoveKind.LINEAR
        elif move_type == MoveInstructionType.FREESPACE:
            kind = MoveKind.FREESPACE
        else:
            raise UnsupportedInstructionError(
                "CIRCULAR moves cannot be lowered from a CompositeInstruction "
                "(MoveInstructionPoly carries no via point)."
            )
        return CartesianMove(pose=pose, kind=kind, profile=profile, seed_joints=_seed_joints(cart))

    if waypoint.isJointWaypoint():
        joint = WaypointPoly_as_JointWaypointPoly(waypoint)
        return JointMove(joints=_to_tuple(joint.getPosition()), profile=profile)

    raise UnsupportedInstructionError(
        "Move waypoint type not supported: neither Cartesian nor Joint"
    )


def _seed_joints(cart: CartesianWaypointPoly) -> tuple[float, ...] | None:
    """The waypoint's IK seed as a radians tuple, or ``None`` if it carries none."""
    pos = np.asarray(cart.getSeed().position, dtype=np.float64).ravel()
    return tuple(float(v) for v in pos) if pos.size else None


def _to_tuple(values: ArrayLike) -> tuple[float, ...]:
    """Flatten an array-like of joint values to a plain float tuple (radians)."""
    return tuple(float(v) for v in np.asarray(values, dtype=np.float64).ravel())
