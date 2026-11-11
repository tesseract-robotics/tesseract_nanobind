"""Public Fanuc LS emission: `CompositeInstruction` to `EmittedProgram`."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.command_language import CompositeInstruction

from ..core.backend import drive
from ..core.frames import FrameBinding
from ..core.identity import EmitIdentity, stable_repr
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import LsBackend
from .profile import LsProfile
from .syntax import FanucConfigResolver


def emit_ls(
    composite: CompositeInstruction,
    profiles: Mapping[str, LsProfile],
    *,
    frames: FrameBinding[int, int],
    config: FanucConfigResolver | None = None,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Fanuc LS (TP ASCII) `.ls` from a tesseract `CompositeInstruction`.

    Args:
        composite: Planned or authored program (Cartesian or joint moves).
        profiles: tesseract profile name to `LsProfile`; every move's profile must
            be present.
        frames: Binding of working frames to UF numbers and TCP frames to UT
            numbers (build with `ls_frames`).
        config: Resolver of the FLIP/UP/FRONT letters for this robot model.
            Required as soon as the program has a Cartesian move.
        program_name: `/PROG` name (upper-cased); defaults to the composite
            description.

    Returns:
        One `<NAME>.ls` file with the `/MN` logic and the `/POS` table; the
        identity header is the first numbered `!` remark lines of `/MN`.

    Raises:
        EmptyProgramError: no leaf instructions.
        MissingProfileError: a move's profile is absent from `profiles`.
        UnboundFrameError: a move's frame is not in `frames`.
        MissingConfigurationError: a Cartesian move without seed joints or without
            a `config` resolver.
        ExternalAxisError: a move carries more than six axes.
        UnsupportedInstructionError: CIRCULAR moves, tool changes, named signals.
        IdentifierError: the program name is not a legal TP program name.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(
        ir,
        profiles_repr=repr(
            (
                sorted(profiles.items()),
                sorted(frames.work_frames.items()),
                sorted(frames.tools.items()),
                stable_repr(config),
            )
        ),  # fmt: skip
    )
    backend = LsBackend(
        profiles=profiles, frames=frames, config=config, program_name=name, identity=identity
    )
    return EmittedProgram(identity=identity, files=drive(ir, backend))
