"""tesseract_command_language Python bindings (nanobind)"""

from tesseract_robotics import ensure_configured

ensure_configured()

from tesseract_robotics.tesseract_command_language._tesseract_command_language import *  # noqa: E402

# Re-export AnyPoly wrappers from tesseract_task_composer for convenience
# (examples import these from command_language for SWIG compatibility)
_HAS_TASK_COMPOSER = False
try:
    from tesseract_robotics.tesseract_task_composer import (
        AnyPoly_as_CompositeInstruction,
        AnyPoly_wrap_CompositeInstruction,
        AnyPoly_wrap_ProfileDictionary,
    )

    _HAS_TASK_COMPOSER = True
except ImportError:
    pass  # task_composer may not be available

__all__ = [
    # Concrete Waypoints
    "JointWaypoint",
    "CartesianWaypoint",
    "StateWaypoint",
    # Poly Waypoints (type-erased)
    "WaypointPoly",
    "CartesianWaypointPoly",
    "JointWaypointPoly",
    "StateWaypointPoly",
    # Poly wrap helpers (SWIG compatibility)
    "CartesianWaypointPoly_wrap_CartesianWaypoint",
    "JointWaypointPoly_wrap_JointWaypoint",
    "StateWaypointPoly_wrap_StateWaypoint",
    # Poly cast helpers (SWIG compatibility)
    "WaypointPoly_as_StateWaypointPoly",
    "WaypointPoly_as_CartesianWaypointPoly",
    "WaypointPoly_as_JointWaypointPoly",
    # Instructions
    "InstructionPoly",
    "MoveInstructionPoly",
    "MoveInstruction",
    "CompositeInstruction",
    # Instruction helpers (SWIG compatibility)
    "MoveInstructionPoly_wrap_MoveInstruction",
    "InstructionPoly_as_MoveInstructionPoly",
    # Enums
    "MoveInstructionType",
    "MoveInstructionType_LINEAR",
    "MoveInstructionType_FREESPACE",
    "MoveInstructionType_CIRCULAR",
    "CompositeInstructionOrder",
    # Profile
    "Profile",
    "ProfileDictionary",
    # Constants
    "DEFAULT_PROFILE_KEY",
]

if _HAS_TASK_COMPOSER:
    __all__ += [
        "AnyPoly_wrap_CompositeInstruction",
        "AnyPoly_wrap_ProfileDictionary",
        "AnyPoly_as_CompositeInstruction",
    ]
