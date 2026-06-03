"""tesseract_collision Python bindings (nanobind)"""
# Import dependencies first to register their types for cross-module access
import tesseract_robotics.tesseract_common  # noqa: F401 - needed for CollisionMarginData, ACM
from tesseract_robotics.tesseract_collision._tesseract_collision import *

__all__ = [
    # Enums
    "ContinuousCollisionType",
    "ContactTestType",
    "CollisionEvaluatorType",
    "CollisionCheckProgramType",
    "CollisionCheckExitType",
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
