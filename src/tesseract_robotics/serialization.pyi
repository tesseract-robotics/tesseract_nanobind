"""Tesseract serialization bindings (XML/binary via Cereal)"""

from collections.abc import Sequence
import os

import tesseract_robotics.command_language
import tesseract_robotics.environment
import tesseract_robotics.state_solver
import tesseract_robotics.task_composer


def composite_instruction_to_xml(instruction: tesseract_robotics.command_language.CompositeInstruction) -> str:
    """
    Serialize CompositeInstruction to XML string. Recursively captures all waypoints, instructions, profiles, and nested composites.
    """

def composite_instruction_from_xml(xml: str) -> tesseract_robotics.command_language.CompositeInstruction:
    """Deserialize CompositeInstruction from XML string."""

def composite_instruction_to_file(instruction: tesseract_robotics.command_language.CompositeInstruction, path: str) -> bool:
    """Save CompositeInstruction to XML file."""

def composite_instruction_from_file(path: str) -> tesseract_robotics.command_language.CompositeInstruction:
    """Load CompositeInstruction from XML file."""

def composite_instruction_to_binary(instruction: tesseract_robotics.command_language.CompositeInstruction) -> list[int]:
    """
    Serialize CompositeInstruction to binary data (faster, smaller than XML).
    """

def composite_instruction_from_binary(data: Sequence[int]) -> tesseract_robotics.command_language.CompositeInstruction:
    """Deserialize CompositeInstruction from binary data."""

def environment_to_xml(environment: tesseract_robotics.environment.Environment) -> str:
    """
    Serialize Environment to XML string. Includes command history - on deserialize, commands are replayed to rebuild scene graph.
    """

def environment_from_xml(xml: str) -> tesseract_robotics.environment.Environment:
    """Deserialize Environment from XML string. Returns shared_ptr."""

def environment_to_file(environment: tesseract_robotics.environment.Environment, path: str) -> bool:
    """Save Environment to XML file."""

def environment_from_file(path: str) -> tesseract_robotics.environment.Environment:
    """Load Environment from XML file. Returns shared_ptr."""

def environment_to_binary(environment: tesseract_robotics.environment.Environment) -> list[int]:
    """Serialize Environment to binary data."""

def environment_from_binary(data: Sequence[int]) -> tesseract_robotics.environment.Environment:
    """Deserialize Environment from binary data. Returns shared_ptr."""

def scene_state_to_xml(state: tesseract_robotics.state_solver.SceneState) -> str:
    """
    Serialize SceneState to XML string. Captures joint positions and link transforms.
    """

def scene_state_from_xml(xml: str) -> tesseract_robotics.state_solver.SceneState:
    """Deserialize SceneState from XML string."""

def scene_state_to_file(state: tesseract_robotics.state_solver.SceneState, path: str) -> bool:
    """Save SceneState to XML file."""

def scene_state_from_file(path: str) -> tesseract_robotics.state_solver.SceneState:
    """Load SceneState from XML file."""

def scene_state_to_binary(state: tesseract_robotics.state_solver.SceneState) -> list[int]:
    """Serialize SceneState to binary data."""

def scene_state_from_binary(data: Sequence[int]) -> tesseract_robotics.state_solver.SceneState:
    """Deserialize SceneState from binary data."""

def task_composer_log_to_xml(log: tesseract_robotics.task_composer.TaskComposerLog) -> str:
    """
    Serialize TaskComposerLog to XML string: description, initial data, context and dot graph.
    """

def task_composer_log_from_xml(xml: str) -> tesseract_robotics.task_composer.TaskComposerLog:
    """Deserialize TaskComposerLog from XML string."""

def task_composer_log_to_file(log: tesseract_robotics.task_composer.TaskComposerLog, path: str | os.PathLike) -> None:
    """
    Save TaskComposerLog as XML to `path`, as named. Raises OSError if it cannot be written.
    """

def task_composer_log_from_file(path: str | os.PathLike) -> tesseract_robotics.task_composer.TaskComposerLog:
    """
    Load TaskComposerLog from the XML file `path`. Raises OSError if it cannot be read.
    """

def task_composer_log_to_binary(log: tesseract_robotics.task_composer.TaskComposerLog) -> list[int]:
    """Serialize TaskComposerLog to binary data."""

def task_composer_log_from_binary(data: Sequence[int]) -> tesseract_robotics.task_composer.TaskComposerLog:
    """Deserialize TaskComposerLog from binary data."""
