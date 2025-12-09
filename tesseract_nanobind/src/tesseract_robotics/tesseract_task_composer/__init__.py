"""tesseract_task_composer Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401
from tesseract_robotics import tesseract_environment  # noqa: F401
from tesseract_robotics import tesseract_command_language  # noqa: F401

from tesseract_robotics.tesseract_task_composer._tesseract_task_composer import *

__all__ = [
    "TaskComposerKeys",
    "TaskComposerDataStorage",
    "TaskComposerNodeInfo",
    "TaskComposerNodeInfoContainer",
    "TaskComposerContext",
    "TaskComposerNode",
    "TaskComposerFuture",
    "TaskComposerExecutor",
    "TaskflowTaskComposerExecutor",
    "TaskComposerPluginFactory",
    # Factory function (backwards compat)
    "createTaskComposerPluginFactory",
    # AnyPoly and wrapper functions
    "AnyPoly",
    "AnyPoly_wrap_CompositeInstruction",
    "AnyPoly_wrap_ProfileDictionary",
    "AnyPoly_wrap_EnvironmentConst",
    "AnyPoly_as_CompositeInstruction",
]
