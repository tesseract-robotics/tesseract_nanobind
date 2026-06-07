"""Public Fanuc LS emission: ``CompositeInstruction`` → ``EmittedProgram``."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.tesseract_command_language import CompositeInstruction

from ..core.backend import drive
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import LsBackend
from .profile import LsProfile


def emit_ls(
    composite: CompositeInstruction,
    profiles: Mapping[str, LsProfile],
    *,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Fanuc LS (TP ASCII) ``.ls`` from a tesseract ``CompositeInstruction``.

    Args:
        composite: planned/authored program (sparse Cartesian or joint moves).
        profiles: tesseract profile name → ``LsProfile``; every move's profile must
            be present, else ``MissingProfileError``.
        program_name: Fanuc ``/PROG`` name (uppercased); defaults to the composite
            description.

    Returns:
        An ``EmittedProgram`` with one ``<NAME>.ls`` file holding both the ``/MN``
        logic section and the ``/POS`` position table; the content-addressed
        identity is emitted as numbered ``!`` comment lines at the top of ``/MN``.

    Raises:
        EmptyProgramError: composite has no leaf instructions.
        MissingProfileError: a move references a profile absent from ``profiles``.
        UnsupportedInstructionError: an instruction the IR cannot represent.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(ir, profiles_repr=repr(sorted(profiles.items())))
    backend = LsBackend(profiles=profiles, program_name=name, identity=identity)
    return EmittedProgram(files=drive(ir, backend))
