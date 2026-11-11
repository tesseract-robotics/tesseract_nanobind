"""Tesseract `CompositeInstruction` to RAPID.

Thin orchestration over the brand-independent core: `lower()` produces the
typed event IR and `drive()` runs it through `RapidBackend`.
"""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.command_language import CompositeInstruction

from ..core.backend import drive
from ..core.external_axes import ExternalAxisLayout
from ..core.frames import FrameBinding
from ..core.identity import EmitIdentity, stable_repr
from ..core.lowering import lower
from ..core.posture import PostureSolver
from ..core.program import EmittedProgram
from .backend import RapidBackend
from .profile import RapidProfile, ToolName, WobjName


def emit_rapid(
    composite: CompositeInstruction,
    profiles: Mapping[str, RapidProfile],
    *,
    frames: FrameBinding[WobjName, ToolName],
    posture: PostureSolver | None = None,
    external_axes: ExternalAxisLayout | None = None,
    module_name: str = "main_program",
    proc_name: str = "main",
) -> EmittedProgram:
    """Emit a RAPID `MODULE` from a tesseract `CompositeInstruction`.

    Args:
        composite: Top-level program (planner results or a
            `MotionProgram.to_composite_instruction()` output).
        profiles: tesseract profile name to `RapidProfile`. Every move's profile
            must be present.
        frames: Binding of working frames to `wobjdata` and TCP frames to
            `tooldata` (build with `rapid_frames`).
        posture: Posture solver for the arm. Required as soon as the program has
            a Cartesian move: it yields `cfx`. Seed joints yield `cf1/cf4/cf6`.
        external_axes: Arm-vs-external joint classification for a coordinated
            (track/positioner) group. When given, each joint vector is split by
            joint name so `jointtarget` keeps six robax and carries the external
            DOF in `eax` (mm for LINEAR, deg for ROTARY). `None` emits a plain
            program (`eax` all `9E+09`).
        module_name: RAPID `MODULE` name.
        proc_name: RAPID `PROC` name.

    Returns:
        One `<module_name>.mod` file; the identity header is the module's first
        comment lines.

    Raises:
        EmptyProgramError: composite has no leaf instructions.
        UnsupportedInstructionError: CIRCULAR moves, tool changes, unrecognized
            instructions.
        MissingProfileError: profile referenced but absent from `profiles`.
        UnboundFrameError: a move's working or TCP frame is not in `frames`.
        MissingConfigurationError: a Cartesian move without seed joints or without
            a `posture` solver.
        SingularPostureError: a seed lies exactly on a posture boundary.
        UncoordinatedTargetError: a move's joints do not match `external_axes`.
        IdentifierError: a module, procedure or signal name is not legal RAPID.
    """
    ir = lower(composite, external_axes=external_axes)
    identity = EmitIdentity.build(
        ir,
        profiles_repr=repr(
            (
                sorted(profiles.items()),
                sorted(frames.work_frames.items()),
                sorted(frames.tools.items()),
                stable_repr(posture),
                external_axes,
            )
        ),  # fmt: skip
    )
    backend = RapidBackend(
        profiles=profiles,
        frames=frames,
        posture=posture,
        module_name=module_name,
        proc_name=proc_name,
        identity=identity,
    )
    return EmittedProgram(identity=identity, files=drive(ir, backend))
