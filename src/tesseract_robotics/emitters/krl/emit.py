"""Public KRL emission: `CompositeInstruction` to `EmittedProgram`."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.command_language import CompositeInstruction

from ..core.backend import drive
from ..core.frames import FrameBinding
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import KrlBackend
from .profile import KrlProfile
from .syntax import KrlFrameRef


def emit_krl(
    composite: CompositeInstruction,
    profiles: Mapping[str, KrlProfile],
    *,
    frames: FrameBinding[KrlFrameRef, KrlFrameRef],
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a KUKA KRL `.src` from a tesseract `CompositeInstruction`.

    Args:
        composite: Planned or authored program (joint moves, LINEAR Cartesian moves).
        profiles: tesseract profile name to `KrlProfile`; every move's profile
            must be present.
        frames: Binding of working frames to `BASE_DATA` and TCP frames to
            `TOOL_DATA` (build with `krl_frames`).
        program_name: KRL `DEF` name; defaults to the composite description.

    Returns:
        One `<name>.src` file; the identity header is the first comment lines of
        the `DEF` body.

    Raises:
        EmptyProgramError: no leaf instructions.
        MissingProfileError: a move's profile is absent from `profiles`.
        UnboundFrameError: a move's frame is not in `frames`.
        MissingConfigurationError: a FREESPACE Cartesian move (see `KrlBackend`).
        ExternalAxisError: a joint move carries more than six axes.
        UnsupportedInstructionError: CIRCULAR moves, tool changes, named signals.
        IdentifierError: the program name is not a legal KRL name.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(
        ir,
        profiles_repr=repr(
            (
                sorted(profiles.items()),
                sorted(frames.work_frames.items(), key=str),
                sorted(frames.tools.items(), key=str),
            )
        ),  # fmt: skip
    )
    backend = KrlBackend(profiles=profiles, frames=frames, program_name=name, identity=identity)
    return EmittedProgram(identity=identity, files=drive(ir, backend))
