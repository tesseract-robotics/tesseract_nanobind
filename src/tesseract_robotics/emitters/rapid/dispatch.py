"""Tesseract `CompositeInstruction` → RAPID dispatcher.

Walks a tesseract `CompositeInstruction`, lowering each instruction kind to
the corresponding `RapidWriter` DSL call inside a `Module`/`Proc` scope.
"""

from __future__ import annotations

from collections.abc import Iterator

from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    InstructionPoly,
    MoveInstructionPoly,
    MoveInstructionType,
    TimerInstructionType,
    WaitInstructionType,
    WaypointPoly_as_CartesianWaypointPoly,
    WaypointPoly_as_JointWaypointPoly,
)

from .rapid_writer import (
    Comment,
    JointTarget,
    Module,
    MoveAbsJ,
    MoveJ,
    MoveL,
    Proc,
    RapidProfile,
    RapidTarget,
    RapidWriter,
    SetAO,
    SetDO,
    WaitDI,
    WaitDO,
    WaitTime,
)


class RapidEmitterError(Exception):
    """Base for all errors raised by the high-level `emit_rapid()`."""


class UnsupportedInstructionError(RapidEmitterError):
    """Raised on an instruction kind the emitter cannot represent (e.g. a
    CIRCULAR `MoveInstruction`, since tesseract's `MoveInstructionPoly` carries
    no via point — use `MoveC(via, end)` directly with a handcrafted via point
    for biarc workflows)."""


class MissingProfileError(RapidEmitterError):
    """Raised when a tesseract profile name on a `MoveInstruction` has no
    entry in the `profiles: dict[str, RapidProfile]` passed to
    `emit_rapid()`."""


class EmptyProgramError(RapidEmitterError):
    """Raised when the input `CompositeInstruction` has zero leaf instructions
    after recursive walk."""


def emit_rapid(
    composite: CompositeInstruction,
    profiles: dict[str, RapidProfile],
    *,
    module_name: str = "main_program",
    proc_name: str = "main",
) -> str:
    """Walk a tesseract `CompositeInstruction` and emit a RAPID `MODULE` string.

    Args:
        composite: Top-level `tesseract_command_language.CompositeInstruction`
            (typically the planner's `raw_results` or a `MotionProgram`'s
            `to_composite_instruction()` output).
        profiles: Mapping from tesseract profile name → `RapidProfile` with the
            RAPID variable names this profile should reference. Every profile
            name on a Move must have an entry, else `MissingProfileError`.
        module_name: RAPID `MODULE` name.
        proc_name: RAPID `PROC` name (single procedure for all instructions).

    Raises:
        EmptyProgramError: composite has no leaf instructions after walk.
        UnsupportedInstructionError: CIRCULAR moves, unrecognized polymorphism.
        MissingProfileError: profile name referenced but not in `profiles`.
    """
    leaves = list(_walk(composite))
    if not leaves:
        raise EmptyProgramError(
            f"CompositeInstruction {composite.getDescription()!r} has no leaf instructions"
        )

    rapid = RapidWriter()
    rapid.clear()
    with Module(module_name):
        with Proc(proc_name):
            for instr in leaves:
                _dispatch(instr, profiles)
    return rapid.getvalue()


def _walk(composite: CompositeInstruction) -> Iterator[InstructionPoly]:
    """Yield leaf `InstructionPoly` entries; recurse into nested `CompositeInstruction`s."""
    for instr in composite.getInstructions():
        if instr.isCompositeInstruction():
            yield from _walk(instr.asCompositeInstruction())
        else:
            yield instr


def _rapid_signal_name(key: str, index: int) -> str:
    """Combine a tesseract signal `(key, index)` into a RAPID signal name.

    RAPID names array-indexed signals as `<key>{<index>}` (e.g. `do_torch{3}`).
    Index 0 is the conventional unindexed default — emit the bare key.
    """
    return key if index == 0 else f"{key}{{{index}}}"


def _dispatch(instr: InstructionPoly, profiles: dict[str, RapidProfile]) -> None:
    if instr.isMoveInstruction():
        _emit_move(instr.asMoveInstruction(), profiles)
        return

    if instr.isWaitInstruction():
        wait = instr.asWaitInstruction()
        kind = wait.getWaitType()
        if kind == WaitInstructionType.TIME:
            WaitTime(wait.getWaitTime())
            return
        # Digital-IO waits: tesseract names them `DIGITAL_{INPUT,OUTPUT}_{HIGH,LOW}`.
        # RAPID signal naming convention: `di{N}` for inputs, `do{N}` for outputs
        # (overrideable in your RAPID system if signals have been renamed).
        is_input = "INPUT" in kind.name
        cls = WaitDI if is_input else WaitDO
        prefix = "di" if is_input else "do"
        cls(f"{prefix}{wait.getWaitIO()}", value="HIGH" in kind.name)
        return

    if instr.isTimerInstruction():
        # Tesseract's TimerInstruction is "after N seconds, drive digital IO i
        # {high,low}". RAPID is imperative — lower as two consecutive
        # statements: `WaitTime <seconds>;` then `SetDO do<i>, <0|1>;`.
        timer = instr.asTimerInstruction()
        WaitTime(timer.getTimerTime())
        SetDO(
            f"do{timer.getTimerIO()}",
            timer.getTimerType() == TimerInstructionType.DIGITAL_OUTPUT_HIGH,
        )
        return

    if instr.isSetDigitalInstruction():
        sd = instr.asSetDigitalInstruction()
        SetDO(_rapid_signal_name(sd.getKey(), sd.getIndex()), sd.getValue())
        return

    if instr.isSetAnalogInstruction():
        sa = instr.asSetAnalogInstruction()
        SetAO(_rapid_signal_name(sa.getKey(), sa.getIndex()), sa.getValue())
        return

    if instr.isSetToolInstruction():
        st = instr.asSetToolInstruction()
        # RAPID has no first-class tool-switch instruction; emit a marker
        # comment with the tool ID. Caller is responsible for swapping
        # `RapidProfile.tool` on subsequent moves if a real tool change is
        # required.
        Comment(f"tool change to tool id {st.getTool()}")
        return

    raise UnsupportedInstructionError(
        f"Unrecognized instruction polymorphism: {instr.getDescription()!r}"
    )


def _emit_move(move: MoveInstructionPoly, profiles: dict[str, RapidProfile]) -> None:
    profile_name = move.getProfile()
    if profile_name not in profiles:
        raise MissingProfileError(
            f"profile {profile_name!r} referenced by move but not in profiles dict; "
            f"available: {sorted(profiles.keys())}"
        )
    profile = profiles[profile_name]
    waypoint = move.getWaypoint()
    move_type = move.getMoveType()

    if waypoint.isCartesianWaypoint():
        cart = WaypointPoly_as_CartesianWaypointPoly(waypoint)
        # Isometry3d.matrix() is a method (not a property) — must call it.
        target = RapidTarget(pose=Pose(cart.getTransform().matrix()))
        if move_type == MoveInstructionType.LINEAR:
            MoveL(target, profile)
        elif move_type == MoveInstructionType.FREESPACE:
            MoveJ(target, profile)
        elif move_type == MoveInstructionType.CIRCULAR:
            raise UnsupportedInstructionError(
                "CIRCULAR moves cannot be lowered from tesseract CompositeInstruction "
                "(MoveInstructionPoly carries no via point). "
                "Use MoveC(via, end) directly for handcrafted biarc programs."
            )
        return

    if waypoint.isJointWaypoint():
        joint = WaypointPoly_as_JointWaypointPoly(waypoint)
        # Joint waypoints always emit MoveAbsJ regardless of LINEAR/FREESPACE
        # (controller can't do Cartesian interpolation when target is in joint space).
        MoveAbsJ(JointTarget(joint.getPosition()), profile)
        return

    raise UnsupportedInstructionError(
        "Move waypoint type not supported: neither Cartesian nor Joint"
    )
