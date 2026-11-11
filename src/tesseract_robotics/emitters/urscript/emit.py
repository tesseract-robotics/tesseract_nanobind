"""Public URScript emission: `CompositeInstruction` to `EmittedProgram`."""

from __future__ import annotations

from collections.abc import Mapping

from tesseract_robotics.command_language import CompositeInstruction
from tesseract_robotics.planning.transforms import Pose

from ..core.backend import drive
from ..core.frames import FrameBinding
from ..core.identity import EmitIdentity
from ..core.lowering import lower
from ..core.program import EmittedProgram
from .backend import UrScriptBackend
from .profile import UrScriptProfile


def _pose_items(mapping: Mapping[str, Pose]) -> list[tuple[str, list[list[float]]]]:
    return sorted((k, v.matrix.round(9).tolist()) for k, v in mapping.items())


def emit_urscript(
    composite: CompositeInstruction,
    profiles: Mapping[str, UrScriptProfile],
    *,
    frames: FrameBinding[Pose, Pose],
    program_name: str | None = None,
) -> EmittedProgram:
    """Emit a Universal Robots `.script` from a tesseract `CompositeInstruction`.

    Args:
        composite: Planned or authored program (Cartesian or joint moves).
        profiles: tesseract profile name to `UrScriptProfile`; every move's
            profile must be present.
        frames: Binding of working frames to feature poses and TCP frames to
            flange-to-TCP poses (build with `ur_frames`).
        program_name: URScript `def` name; defaults to the composite description.

    Returns:
        One `<name>.script` file; the identity header is the first `#` comment
        lines of the `def` body.

    Raises:
        EmptyProgramError: no leaf instructions.
        MissingProfileError: a move's profile is absent from `profiles`.
        UnboundFrameError: a move's frame is not in `frames`.
        MissingConfigurationError: a FREESPACE Cartesian move without seed joints.
        ExternalAxisError: a move carries more than six axes.
        UnsupportedInstructionError: CIRCULAR moves, tool changes, named signals,
            analog values outside `set_standard_analog_out`'s range.
        IdentifierError: the program name is not a legal URScript identifier.
    """
    ir = lower(composite)
    name = program_name or ir.name
    identity = EmitIdentity.build(
        ir,
        profiles_repr=repr(
            (sorted(profiles.items()), _pose_items(frames.work_frames), _pose_items(frames.tools))
        ),
    )
    backend = UrScriptBackend(
        profiles=profiles, frames=frames, program_name=name, identity=identity
    )
    return EmittedProgram(identity=identity, files=drive(ir, backend))
