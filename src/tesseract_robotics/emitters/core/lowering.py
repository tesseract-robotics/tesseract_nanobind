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

from .errors import EmptyProgramError, UncoordinatedTargetError, UnsupportedInstructionError
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
from .external_axes import ExternalAxisLayout, ExternalAxisValue

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


def lower(
    composite: CompositeInstruction,
    *,
    external_axes: ExternalAxisLayout | None = None,
) -> ProgramIR:
    """Walk ``composite`` → ``ProgramIR``. Raises ``EmptyProgramError`` if no leaves.

    ``external_axes`` classifies which group joints are the six arm axes vs.
    track/positioner external axes. When supplied, every coordinated move's joint
    vector is split by joint *name* so ``jointtarget``/``robtarget`` keep their six
    robax and carry the external DOF in the ``eax`` field. ``None`` leaves joint
    vectors whole (plain, uncoupled 6-DOF).
    """
    leaves = list(_walk(composite))
    if not leaves:
        raise EmptyProgramError(composite.getDescription())
    events: list[Event] = []
    for instr in leaves:
        events.extend(_lower_instruction(instr, external_axes))
    return ProgramIR(name=composite.getDescription() or _DEFAULT_PROGRAM_NAME, events=tuple(events))


def _walk(composite: CompositeInstruction) -> Iterator[InstructionPoly]:
    """Yield leaf instructions, recursing into nested composites."""
    for instr in composite.getInstructions():
        if instr.isCompositeInstruction():
            yield from _walk(instr.asCompositeInstruction())
        else:
            yield instr


def _lower_instruction(instr: InstructionPoly, layout: ExternalAxisLayout | None) -> list[Event]:
    if instr.isMoveInstruction():
        return [_lower_move(instr.asMoveInstruction(), layout)]

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


def _lower_move(move: MoveInstructionPoly, layout: ExternalAxisLayout | None) -> Event:
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
        # Coordinated-Cartesian eax (a robtarget carrying the seed's external DOF)
        # is E2; the CartesianMove.external_axes field and the backend plumbing are
        # ready, so lowering leaves it empty here rather than split an unverified seed.
        return CartesianMove(pose=pose, kind=kind, profile=profile, seed_joints=_seed_joints(cart))

    if waypoint.isJointWaypoint():
        joint = WaypointPoly_as_JointWaypointPoly(waypoint)
        positions = _to_tuple(joint.getPosition())
        if layout is None:
            return JointMove(joints=positions, profile=profile)
        names = tuple(str(name) for name in joint.getNames())
        arm, external = _split_external(names, positions, layout)
        return JointMove(joints=arm, profile=profile, external_axes=external)

    raise UnsupportedInstructionError(
        "Move waypoint type not supported: neither Cartesian nor Joint"
    )


def _split_external(
    names: tuple[str, ...],
    positions: tuple[float, ...],
    layout: ExternalAxisLayout,
) -> tuple[tuple[float, ...], tuple[ExternalAxisValue, ...]]:
    """Split a coordinated joint vector into (six arm values, external values) by name.

    The waypoint's joint-name set must exactly equal the layout's (arm ∪ external);
    a missing external DOF, an unclassified joint, a length mismatch, or duplicate
    names all raise ``UncoordinatedTargetError`` — the by-name split never guesses a
    slot. Arm values follow ``layout.arm_joint_names`` (robax order); external values
    follow ``layout.external_axes`` (eax-slot order).
    """
    layout_names = layout.joint_names
    waypoint_set = set(names)
    layout_set = set(layout_names)
    missing = tuple(name for name in layout_names if name not in waypoint_set)
    extra = tuple(name for name in names if name not in layout_set)
    if missing or extra or len(names) != len(positions) or len(waypoint_set) != len(names):
        raise UncoordinatedTargetError(missing=missing, extra=extra, waypoint_names=names)
    by_name = dict(zip(names, positions))
    arm = tuple(by_name[name] for name in layout.arm_joint_names)
    external = tuple(
        ExternalAxisValue(by_name[spec.joint_name], spec.kind) for spec in layout.external_axes
    )
    return arm, external


def _seed_joints(cart: CartesianWaypointPoly) -> tuple[float, ...] | None:
    """The waypoint's IK seed as a radians tuple, or ``None`` if it carries none."""
    pos = np.asarray(cart.getSeed().position, dtype=np.float64).ravel()
    return tuple(float(v) for v in pos) if pos.size else None


def _to_tuple(values: ArrayLike) -> tuple[float, ...]:
    """Flatten an array-like of joint values to a plain float tuple (radians)."""
    return tuple(float(v) for v in np.asarray(values, dtype=np.float64).ravel())
