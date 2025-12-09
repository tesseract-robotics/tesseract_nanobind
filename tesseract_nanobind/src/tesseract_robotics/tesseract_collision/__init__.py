"""tesseract_collision Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401
from tesseract_robotics import tesseract_geometry  # noqa: F401

from tesseract_robotics.tesseract_collision._tesseract_collision import *

__all__ = [
    # Enums
    "ContinuousCollisionType",
    "ContactTestType",
    "CollisionEvaluatorType",
    "CollisionCheckProgramType",
    "ACMOverrideType",

    # SWIG-compatible enum constants
    "ContactTestType_FIRST",
    "ContactTestType_CLOSEST",
    "ContactTestType_ALL",
    "ContactTestType_LIMITED",

    # Contact results
    "ContactResult",
    "ContactResultVector",
    "ContactResultMap",
    "ContactRequest",

    # Config
    "ContactManagerConfig",
    "CollisionCheckConfig",

    # Contact managers
    "DiscreteContactManager",
    "ContinuousContactManager",
    "ContactManagersPluginFactory",
]
