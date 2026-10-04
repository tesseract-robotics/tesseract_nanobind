"""Tesseract serialization bindings (XML/binary via Cereal)"""

from collections.abc import Sequence


def composite_instruction_to_xml(instruction: "tesseract::command_language::CompositeInstruction") -> str:
    """
    Serialize CompositeInstruction to XML string. Recursively captures all waypoints, instructions, profiles, and nested composites.
    """

def composite_instruction_from_xml(xml: str) -> "tesseract::command_language::CompositeInstruction":
    """Deserialize CompositeInstruction from XML string."""

def composite_instruction_to_file(instruction: "tesseract::command_language::CompositeInstruction", path: str) -> bool:
    """Save CompositeInstruction to XML file."""

def composite_instruction_from_file(path: str) -> "tesseract::command_language::CompositeInstruction":
    """Load CompositeInstruction from XML file."""

def composite_instruction_to_binary(instruction: "tesseract::command_language::CompositeInstruction") -> list[int]:
    """
    Serialize CompositeInstruction to binary data (faster, smaller than XML).
    """

def composite_instruction_from_binary(data: Sequence[int]) -> "tesseract::command_language::CompositeInstruction":
    """Deserialize CompositeInstruction from binary data."""

def environment_to_xml(environment: "tesseract::environment::Environment") -> str:
    """
    Serialize Environment to XML string. Includes command history - on deserialize, commands are replayed to rebuild scene graph.
    """

def environment_from_xml(xml: str) -> "tesseract::environment::Environment":
    """Deserialize Environment from XML string. Returns shared_ptr."""

def environment_to_file(environment: "tesseract::environment::Environment", path: str) -> bool:
    """Save Environment to XML file."""

def environment_from_file(path: str) -> "tesseract::environment::Environment":
    """Load Environment from XML file. Returns shared_ptr."""

def environment_to_binary(environment: "tesseract::environment::Environment") -> list[int]:
    """Serialize Environment to binary data."""

def environment_from_binary(data: Sequence[int]) -> "tesseract::environment::Environment":
    """Deserialize Environment from binary data. Returns shared_ptr."""

def scene_state_to_xml(state: "tesseract::scene_graph::SceneState") -> str:
    """
    Serialize SceneState to XML string. Captures joint positions and link transforms.
    """

def scene_state_from_xml(xml: str) -> "tesseract::scene_graph::SceneState":
    """Deserialize SceneState from XML string."""

def scene_state_to_file(state: "tesseract::scene_graph::SceneState", path: str) -> bool:
    """Save SceneState to XML file."""

def scene_state_from_file(path: str) -> "tesseract::scene_graph::SceneState":
    """Load SceneState from XML file."""

def scene_state_to_binary(state: "tesseract::scene_graph::SceneState") -> list[int]:
    """Serialize SceneState to binary data."""

def scene_state_from_binary(data: Sequence[int]) -> "tesseract::scene_graph::SceneState":
    """Deserialize SceneState from binary data."""
