"""Public KRL emission: ``CompositeInstruction`` → ``EmittedProgram``."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.tesseract_command_language import CompositeInstruction

from ..core.backend import drive
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import KrlBackend
from .profile import KrlProfile


def emit_krl(
    composite: CompositeInstruction,
    profiles: Mapping[str, KrlProfile],
    *,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a KUKA KRL ``.src`` from a tesseract ``CompositeInstruction``.

    Args:
        composite: planned/authored program (sparse Cartesian or joint moves).
        profiles: tesseract profile name → ``KrlProfile``; every move's profile
            must be present, else ``MissingProfileError``.
        program_name: KRL ``DEF`` name; defaults to the composite description.

    Returns:
        An ``EmittedProgram`` with one ``<name>.src`` file; the content-addressed
        identity is emitted as KRL comment lines at the top of the ``DEF`` body.

    Raises:
        EmptyProgramError, MissingProfileError, UnsupportedInstructionError.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(ir, profiles_repr=repr(sorted(profiles.items())))
    backend = KrlBackend(profiles=profiles, program_name=name, identity=identity)
    return EmittedProgram(files=drive(ir, backend))
