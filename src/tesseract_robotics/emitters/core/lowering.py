"""Lower a tesseract `CompositeInstruction` to the brand-free event IR.

The single walk every backend shares. This is the anti-corruption layer at the
nanobind boundary: loosely-typed binding objects in, strictly-typed dataclasses
out. One instruction may expand to several events (Timer becomes Dwell +
SetDigital). Unknown instructions raise; nothing falls back silently.

Frames: tesseract resolves a move's `ManipulatorInfo` by overlaying it on its
parent composite's (`getCombined`: non-empty fields of the child win). The walk
does the same down the composite tree, so every move event carries the frames
the planner used.
"""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.command_language import (
    CartesianWaypointPoly,
    CompositeInstruction,
    InstructionPoly,
    MoveInstructionPoly,
    MoveInstructionType,
    TimerInstructionType,
    WaitInstructionType,
)
from tesseract_robotics.common import Isometry3d, ManipulatorInfo
from tesseract_robotics.planning.transforms import Pose

from .errors import (
    EmptyProgramError,
    MissingManipulatorInfoError,
    TcpOffsetError,
    UncoordinatedTargetError,
    UnsupportedInstructionError,
)
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
from .frames import FramedPose, MoveFrames

#: `ProgramIR` name used when the `CompositeInstruction` has no description.
_DEFAULT_PROGRAM_NAME = "program"
#: Wait-instruction members that block on an input (vs an output) signal.
#: Compared explicitly rather than substring-sniffing `kind.name`: an upstream
#: enum rename would silently invert a name-sniff, but breaks this loudly.
_WAIT_INPUT_TYPES = frozenset(
    {WaitInstructionType.DIGITAL_INPUT_HIGH, WaitInstructionType.DIGITAL_INPUT_LOW}
)
#: Wait-instruction members that block until the signal is HIGH (vs LOW).
_WAIT_HIGH_TYPES = frozenset(
    {WaitInstructionType.DIGITAL_INPUT_HIGH, WaitInstructionType.DIGITAL_OUTPUT_HIGH}
)


def lower(
    composite: CompositeInstruction,
    *,
    external_axes: ExternalAxisLayout | None = None,
) -> ProgramIR:
    """Walk `composite` into a `ProgramIR`.

    Args:
        composite: Program to lower (planner results or a `MotionProgram` output).
        external_axes: Arm-vs-external joint classification. When supplied, every
            joint vector (targets and Cartesian seeds) is split by joint name so
            the arm keeps six axes and the external DOF travel separately. `None`
            leaves joint vectors whole.

    Returns:
        The lowered program.

    Raises:
        EmptyProgramError: no leaf instructions.
        UnsupportedInstructionError: CIRCULAR moves, unknown instruction or waypoint
            types, or a coordinated Cartesian target (tesseract stores no
            external-axis target values for it).
        MissingManipulatorInfoError: a move has no working or TCP frame.
        TcpOffsetError: a move's `tcp_offset` names a link instead of a pose.
        UncoordinatedTargetError: a joint vector does not match `external_axes`.
    """
    leaves = list(_walk(composite, composite.getManipulatorInfo()))
    if not leaves:
        raise EmptyProgramError(composite.getDescription())
    events: list[Event] = []
    move_index = 0
    for instr, info in leaves:
        if instr.isMoveInstruction():
            move_index += 1
        events.extend(_lower_instruction(instr, info, move_index, external_axes))
    return ProgramIR(name=composite.getDescription() or _DEFAULT_PROGRAM_NAME, events=tuple(events))


def _walk(
    composite: CompositeInstruction, info: ManipulatorInfo
) -> Iterator[tuple[InstructionPoly, ManipulatorInfo]]:
    """Yield `(leaf, combined ManipulatorInfo)`, recursing into nested composites."""
    for instr in composite.getInstructions():
        if instr.isCompositeInstruction():
            child = instr.asCompositeInstruction()
            yield from _walk(child, info.getCombined(child.getManipulatorInfo()))
        else:
            yield instr, info


def _lower_instruction(
    instr: InstructionPoly,
    info: ManipulatorInfo,
    move_index: int,
    layout: ExternalAxisLayout | None,
) -> list[Event]:
    if instr.isMoveInstruction():
        move = instr.asMoveInstruction()
        frames = _move_frames(info.getCombined(move.getManipulatorInfo()), move_index)
        return [_lower_move(move, frames, layout)]

    if instr.isWaitInstruction():
        wait = instr.asWaitInstruction()
        kind = wait.getWaitType()
        if kind == WaitInstructionType.TIME:
            return [Dwell.build(float(wait.getWaitTime()))]
        return [
            WaitDigital(
                is_input=kind in _WAIT_INPUT_TYPES,
                index=int(wait.getWaitIO()),
                value=kind in _WAIT_HIGH_TYPES,
                timeout=None,
            )
        ]

    if instr.isTimerInstruction():
        # tesseract Timer = "after N s drive digital IO i {high,low}": Dwell + SetDigital.
        # key=None means unnamed output #i (RAPID renders do<i>, KRL renders $OUT[i]).
        timer = instr.asTimerInstruction()
        return [
            Dwell.build(float(timer.getTimerTime())),
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


def _move_frames(info: ManipulatorInfo, move_index: int) -> MoveFrames:
    """The `MoveFrames` of a combined `ManipulatorInfo`; raise on missing frames."""
    working = info.working_frame.name()
    tcp = info.tcp_frame.name()
    if not working:
        raise MissingManipulatorInfoError("working_frame", move_index)
    if not tcp:
        raise MissingManipulatorInfoError("tcp_frame", move_index)
    offset = info.tcp_offset
    if not isinstance(offset, Isometry3d):
        raise TcpOffsetError(
            "IR",
            f"move #{move_index} names tcp_offset link {offset.name()!r}; the emitter has no "
            "environment to resolve a link offset, pass the offset as a pose",
        )
    return MoveFrames(working_frame=working, tcp_frame=tcp, tcp_offset=Pose(offset.matrix))


def _lower_move(
    move: MoveInstructionPoly, frames: MoveFrames, layout: ExternalAxisLayout | None
) -> Event:
    profile = move.getProfile()
    waypoint = move.getWaypoint()
    move_type = move.getMoveType()

    if waypoint.isCartesianWaypoint():
        cart = waypoint.asCartesianWaypoint()
        if move_type == MoveInstructionType.LINEAR:
            kind = MoveKind.LINEAR
        elif move_type == MoveInstructionType.FREESPACE:
            kind = MoveKind.FREESPACE
        else:
            raise UnsupportedInstructionError(
                "CIRCULAR moves cannot be lowered from a CompositeInstruction "
                "(MoveInstructionPoly carries no via point)."
            )
        if layout is not None and layout.external_axes:
            raise UnsupportedInstructionError(
                "coordinated Cartesian target: a tesseract CartesianWaypoint stores no "
                "external-axis target values (its seed is an IK hint, not a target); "
                "plan coordinated motion as joint waypoints"
            )
        target = FramedPose(pose=Pose(cart.getTransform().matrix), frames=frames)
        return CartesianMove(
            target=target, kind=kind, profile=profile, seed_joints=_seed_joints(cart, layout)
        )

    if waypoint.isJointWaypoint():
        joint = waypoint.asJointWaypoint()
        positions = _to_tuple(joint.getPosition())
        if layout is None:
            return JointMove(joints=positions, profile=profile, frames=frames)
        names = tuple(str(name) for name in joint.getJointIds())
        arm, external = _split_external(names, positions, layout)
        return JointMove(joints=arm, profile=profile, frames=frames, external_axes=external)

    raise UnsupportedInstructionError(
        "Move waypoint type not supported: neither Cartesian nor Joint"
    )


def _split_external(
    names: tuple[str, ...],
    positions: tuple[float, ...],
    layout: ExternalAxisLayout,
) -> tuple[tuple[float, ...], tuple[ExternalAxisValue, ...]]:
    """Split a coordinated joint vector into (six arm values, external values) by name.

    The waypoint's joint-name set must exactly equal the layout's (arm and
    external); a missing external DOF, an unclassified joint, a length mismatch,
    or duplicate names all raise `UncoordinatedTargetError`. Arm values follow
    `layout.arm_joint_names` (robax order); external values follow
    `layout.external_axes` (eax-slot order).
    """
    layout_names = layout.joint_names
    waypoint_set = set(names)
    layout_set = set(layout_names)
    missing = tuple(name for name in layout_names if name not in waypoint_set)
    extra = tuple(name for name in names if name not in layout_set)
    if missing or extra or len(names) != len(positions) or len(waypoint_set) != len(names):
        raise UncoordinatedTargetError(missing=missing, extra=extra, waypoint_names=names)
    by_name = dict(zip(names, positions, strict=True))
    arm = tuple(by_name[name] for name in layout.arm_joint_names)
    external = tuple(
        ExternalAxisValue(by_name[spec.joint_name], spec.kind) for spec in layout.external_axes
    )
    return arm, external


def _seed_joints(
    cart: CartesianWaypointPoly, layout: ExternalAxisLayout | None
) -> tuple[float, ...] | None:
    """The waypoint's seed as a radians tuple (arm axes when a layout is given), or `None`."""
    seed = cart.getSeed()
    pos = _to_tuple(seed.position)
    if not pos:
        return None
    if layout is None:
        return pos
    names = tuple(str(jid) for jid in seed.joint_ids)
    arm, _ = _split_external(names, pos, layout)
    return arm


def _to_tuple(values: ArrayLike) -> tuple[float, ...]:
    """Flatten an array-like of joint values to a plain float tuple (radians)."""
    return tuple(float(v) for v in np.asarray(values, dtype=np.float64).ravel())
