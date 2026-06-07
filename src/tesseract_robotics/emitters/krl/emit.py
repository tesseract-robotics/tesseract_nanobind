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
        An ``EmittedProgram`` with one ``<name>.src`` file; identity lines are
        inserted as KRL comments right after the ``DEF`` line.

    Raises:
        EmptyProgramError, MissingProfileError, UnsupportedInstructionError.
    """
    ir = lower(composite)
    name = program_name or ir.name
    backend = KrlBackend(profiles=profiles, program_name=name)
    files = drive(ir, backend)

    identity = EmitIdentity.build(ir, profiles_repr=repr(sorted(profiles.items())))
    header = "\n".join(f"; {ln}" for ln in identity.header_lines())
    out: dict[str, str] = {}
    for filename, body in files.items():
        def_line, _, rest = body.partition("\n")
        out[filename] = f"{def_line}\n{header}\n{rest}"
    return EmittedProgram(files=out)
