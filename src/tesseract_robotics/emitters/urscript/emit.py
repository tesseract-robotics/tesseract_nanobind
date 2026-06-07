"""Public URScript emission: ``CompositeInstruction`` → ``EmittedProgram``."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.tesseract_command_language import CompositeInstruction

from ..core.backend import drive
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import UrScriptBackend
from .profile import UrScriptProfile


def emit_urscript(
    composite: CompositeInstruction,
    profiles: Mapping[str, UrScriptProfile],
    *,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Universal Robots ``.script`` from a tesseract ``CompositeInstruction``.

    Args:
        composite: planned/authored program (sparse Cartesian or joint moves).
        profiles: tesseract profile name → ``UrScriptProfile``; every move's
            profile must be present, else ``MissingProfileError``.
        program_name: URScript ``def`` name; defaults to the composite description.

    Returns:
        An ``EmittedProgram`` with one ``<name>.script`` file; the content-addressed
        identity is emitted as ``#`` comment lines at the top of the ``def`` body.

    Raises:
        EmptyProgramError, MissingProfileError, UnsupportedInstructionError.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(ir, profiles_repr=repr(sorted(profiles.items())))
    backend = UrScriptBackend(profiles=profiles, program_name=name, identity=identity)
    return EmittedProgram(files=drive(ir, backend))
