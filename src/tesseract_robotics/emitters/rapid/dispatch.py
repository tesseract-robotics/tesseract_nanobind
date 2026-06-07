"""Tesseract ``CompositeInstruction`` → RAPID.

Thin orchestration over the brand-independent core: ``lower()`` produces the
typed event IR and ``drive()`` runs it through ``RapidBackend``. The walk and
per-instruction recognition now live in ``emitters.core.lowering``; this module
keeps the public ``emit_rapid`` signature and re-exports the error names that
``emitters.rapid`` has always exposed (now the shared core classes).
"""

from __future__ import annotations

from tesseract_robotics.tesseract_command_language import CompositeInstruction

from ..core.backend import drive
from ..core.errors import EmitterError as EmitterError
from ..core.errors import EmptyProgramError as EmptyProgramError
from ..core.errors import MissingProfileError as MissingProfileError
from ..core.errors import UnsupportedInstructionError as UnsupportedInstructionError
from ..core.lowering import lower
from .backend import RapidBackend
from .rapid_writer import RapidProfile


class RapidEmitterError(EmitterError):
    """Backward-compatible alias base (was the rapid-local error root)."""


def emit_rapid(
    composite: CompositeInstruction,
    profiles: dict[str, RapidProfile],
    *,
    module_name: str = "main_program",
    proc_name: str = "main",
) -> str:
    """Walk a tesseract ``CompositeInstruction`` and emit a RAPID ``MODULE`` string.

    Args:
        composite: top-level CompositeInstruction (planner ``raw_results`` or a
            ``MotionProgram.to_composite_instruction()`` output).
        profiles: tesseract profile name → ``RapidProfile``. Every move's profile
            must be present, else ``MissingProfileError``.
        module_name: RAPID ``MODULE`` name.
        proc_name: RAPID ``PROC`` name.

    Raises:
        EmptyProgramError: composite has no leaf instructions.
        UnsupportedInstructionError: CIRCULAR moves, unrecognized polymorphism.
        MissingProfileError: profile referenced but absent from ``profiles``.
    """
    ir = lower(composite)
    backend = RapidBackend(profiles=profiles, module_name=module_name, proc_name=proc_name)
    files = drive(ir, backend)
    return next(iter(files.values()))
