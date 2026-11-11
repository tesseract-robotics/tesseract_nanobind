"""Tesseract serialization bindings (XML/binary via Cereal)"""

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

def composite_instruction_to_file(instruction: tesseract_robotics.command_language.CompositeInstruction, path: str | os.PathLike) -> None:
    """
    Save CompositeInstruction as XML to `path`, as named. Raises OSError if it cannot be written.
    """

def composite_instruction_from_file(path: str | os.PathLike) -> tesseract_robotics.command_language.CompositeInstruction:
    """
    Load CompositeInstruction from the XML file `path`. Raises OSError if it cannot be read.
    """

def composite_instruction_to_binary(instruction: tesseract_robotics.command_language.CompositeInstruction) -> bytes:
    """
    Serialize CompositeInstruction to binary data (bytes; faster, smaller than XML).
    """

def composite_instruction_from_binary(data: bytes) -> tesseract_robotics.command_language.CompositeInstruction:
    """Deserialize CompositeInstruction from binary data (bytes)."""

def environment_to_xml(environment: tesseract_robotics.environment.Environment) -> str:
    """
    Serialize Environment to XML string. Includes command history - on deserialize, commands are replayed to rebuild scene graph.
    """

def environment_from_xml(xml: str) -> tesseract_robotics.environment.Environment:
    """Deserialize Environment from XML string. Returns shared_ptr."""

def environment_to_file(environment: tesseract_robotics.environment.Environment, path: str | os.PathLike) -> None:
    """
    Save Environment as XML to `path`, as named. Raises OSError if it cannot be written.
    """

def environment_from_file(path: str | os.PathLike) -> tesseract_robotics.environment.Environment:
    """
    Load Environment from the XML file `path`. Raises OSError if it cannot be read.
    """

def environment_to_binary(environment: tesseract_robotics.environment.Environment) -> bytes:
    """Serialize Environment to binary data (bytes)."""

def environment_from_binary(data: bytes) -> tesseract_robotics.environment.Environment:
    """Deserialize Environment from binary data (bytes)."""

def scene_state_to_xml(state: tesseract_robotics.state_solver.SceneState) -> str:
    """
    Serialize SceneState to XML string. Captures joint positions and link transforms.
    """

def scene_state_from_xml(xml: str) -> tesseract_robotics.state_solver.SceneState:
    """Deserialize SceneState from XML string."""

def scene_state_to_file(state: tesseract_robotics.state_solver.SceneState, path: str | os.PathLike) -> None:
    """
    Save SceneState as XML to `path`, as named. Raises OSError if it cannot be written.
    """

def scene_state_from_file(path: str | os.PathLike) -> tesseract_robotics.state_solver.SceneState:
    """
    Load SceneState from the XML file `path`. Raises OSError if it cannot be read.
    """

def scene_state_to_binary(state: tesseract_robotics.state_solver.SceneState) -> bytes:
    """Serialize SceneState to binary data (bytes)."""

def scene_state_from_binary(data: bytes) -> tesseract_robotics.state_solver.SceneState:
    """Deserialize SceneState from binary data (bytes)."""

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

def task_composer_log_to_binary(log: tesseract_robotics.task_composer.TaskComposerLog) -> bytes:
    """Serialize TaskComposerLog to binary data (bytes)."""

def task_composer_log_from_binary(data: bytes) -> tesseract_robotics.task_composer.TaskComposerLog:
    """Deserialize TaskComposerLog from binary data (bytes)."""
