"""Public JBI emission: `CompositeInstruction` to `EmittedProgram`."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.command_language import CompositeInstruction

from ..core.backend import drive
from ..core.frames import FrameBinding
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import JbiBackend
from .profile import JbiProfile
from .syntax import JbiPulseCalibration


def emit_jbi(
    composite: CompositeInstruction,
    profiles: Mapping[str, JbiProfile],
    *,
    frames: FrameBinding[int, int],
    calibration: JbiPulseCalibration,
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Yaskawa Motoman INFORM `.jbi` from a tesseract `CompositeInstruction`.

    Args:
        composite: Program of joint moves (Cartesian moves raise).
        profiles: tesseract profile name to `JbiProfile`; every move's profile
            must be present.
        frames: Binding of TCP frames to tool numbers (build with `jbi_frames`).
        calibration: Per-axis pulse calibration of the robot.
        program_name: JBI job name (`//NAME`); defaults to the composite description.

    Returns:
        One `<name>.jbi` file. The identity is on `EmittedProgram.identity` only:
        its lines exceed INFORM's 32-character comment limit.

    Raises:
        EmptyProgramError: no leaf instructions.
        MissingProfileError: a move's profile is absent from `profiles`.
        UnboundFrameError: a move's TCP frame is not in `frames`.
        ToolConflictError: moves bind to more than one tool.
        ExternalAxisError: a move carries more than six axes.
        UnsupportedInstructionError: Cartesian or CIRCULAR moves, tool changes,
            named signals, analog values outside the AOUT range.
        IdentifierError: the job name is not legal.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(
        ir,
        profiles_repr=repr((sorted(profiles.items()), sorted(frames.tools.items()), calibration)),
    )
    backend = JbiBackend(
        profiles=profiles, frames=frames, calibration=calibration, program_name=name
    )
    return EmittedProgram(identity=identity, files=drive(ir, backend))
