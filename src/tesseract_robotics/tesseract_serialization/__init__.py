"""tesseract_serialization - persist motion programs and environments

Exposes Boost.Serialization for root types. Nested types (waypoints, instructions,
geometry) are automatically handled by C++ recursive serialization.

Example:
    from tesseract_robotics.tesseract_serialization import (
        composite_instruction_to_file,
        composite_instruction_from_file,
    )

    # Save planned trajectory
    composite_instruction_to_file(program, "trajectory.xml")

    # Load next session
    program = composite_instruction_from_file("trajectory.xml")
"""

from tesseract_robotics.tesseract_serialization._tesseract_serialization import *

__all__ = [
    # CompositeInstruction - motion programs
    "composite_instruction_to_xml",
    "composite_instruction_from_xml",
    "composite_instruction_to_file",
    "composite_instruction_from_file",
    "composite_instruction_to_binary",
    "composite_instruction_from_binary",
    # Environment - full scene state
    "environment_to_xml",
    "environment_from_xml",
    "environment_to_file",
    "environment_from_file",
    "environment_to_binary",
    "environment_from_binary",
    # SceneState - state snapshot
    "scene_state_to_xml",
    "scene_state_from_xml",
    "scene_state_to_file",
    "scene_state_from_file",
    "scene_state_to_binary",
    "scene_state_from_binary",
]
