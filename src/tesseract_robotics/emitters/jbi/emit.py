"""Public JBI emission: ``CompositeInstruction`` → ``EmittedProgram``."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.tesseract_command_language import CompositeInstruction

from ..core.backend import drive
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import JbiBackend
from .profile import JbiProfile


def emit_jbi(
    composite: CompositeInstruction,
    profiles: Mapping[str, JbiProfile],
    *,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Yaskawa Motoman INFORM ``.jbi`` from a tesseract ``CompositeInstruction``.

    Args:
        composite: planned/authored program (sparse Cartesian or joint moves).
        profiles: tesseract profile name → ``JbiProfile``; every move's profile
            must be present, else ``MissingProfileError``.
        program_name: JBI job name (``//NAME``); defaults to the composite
            description.

    Returns:
        An ``EmittedProgram`` with one ``<name>.jbi`` file: a ``//POS`` position
        table (``///POSTYPE PULSE``, joint degrees) and a ``//INST`` instruction
        body. The content-addressed identity is emitted as ``'`` comment lines
        immediately after ``NOP``.

    Raises:
        EmptyProgramError: the composite has no leaf instructions.
        MissingProfileError: a move references a profile not in ``profiles``.
        MissingSeedError: a Cartesian move has no seed joints (PULSE positions
            are joint angles, so seed joints are required).
        UnsupportedInstructionError: an instruction the IR cannot represent.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(ir, profiles_repr=repr(sorted(profiles.items())))
    backend = JbiBackend(profiles=profiles, program_name=name, identity=identity)
    return EmittedProgram(files=drive(ir, backend))
