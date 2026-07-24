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
from ..core.errors import UncoordinatedTargetError as UncoordinatedTargetError
from ..core.errors import UnsupportedInstructionError as UnsupportedInstructionError
from ..core.external_axes import ExternalAxisKind as ExternalAxisKind
from ..core.external_axes import ExternalAxisLayout as ExternalAxisLayout
from ..core.external_axes import ExternalAxisSpec as ExternalAxisSpec
from ..core.lowering import lower
from .backend import RapidBackend
from .profile import RapidProfile


class RapidEmitterError(EmitterError):
    """Backward-compatible alias base (was the rapid-local error root)."""


def emit_rapid(
    composite: CompositeInstruction,
    profiles: dict[str, RapidProfile],
    *,
    external_axes: ExternalAxisLayout | None = None,
    module_name: str = "main_program",
    proc_name: str = "main",
) -> str:
    """Walk a tesseract ``CompositeInstruction`` and emit a RAPID ``MODULE`` string.

    Args:
        composite: top-level CompositeInstruction (planner ``raw_results`` or a
            ``MotionProgram.to_composite_instruction()`` output).
        profiles: tesseract profile name → ``RapidProfile``. Every move's profile
            must be present, else ``MissingProfileError``.
        external_axes: arm-vs-external joint classification for a coordinated
            (track/positioner) group. When given, each move's joint vector is split
            by joint name so ``jointtarget``/``robtarget`` keep six robax and carry
            the external DOF in ``eax`` (mm for LINEAR, deg for ROTARY). ``None``
            emits a plain uncoupled program (``eax`` all ``9E+09``).
        module_name: RAPID ``MODULE`` name.
        proc_name: RAPID ``PROC`` name.

    Raises:
        EmptyProgramError: composite has no leaf instructions.
        UnsupportedInstructionError: CIRCULAR moves, unrecognized polymorphism.
        MissingProfileError: profile referenced but absent from ``profiles``.
        UncoordinatedTargetError: a move's joints do not match ``external_axes``.
    """
    ir = lower(composite, external_axes=external_axes)
    backend = RapidBackend(profiles=profiles, module_name=module_name, proc_name=proc_name)
    files = drive(ir, backend)
    return next(iter(files.values()))
