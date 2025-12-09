"""tesseract_common Python bindings (nanobind)"""

from tesseract_robotics.tesseract_common._tesseract_common import *


# FilesystemPath - use the C++ binding directly
# Note: SWIG compatibility wrapper removed as it doesn't work with nanobind's
# type-strict overload resolution. The C++ FilesystemPath binding supports
# construction from Python strings directly.
# Alias for backwards compatibility with code that imported _FilesystemPath
from tesseract_robotics.tesseract_common._tesseract_common import FilesystemPath as _FilesystemPath

# TransformMap is a dict wrapper for SWIG compatibility
# In nanobind, plain Python dicts with string keys and Isometry3d values work automatically
class TransformMap(dict):
    """A dict wrapper for SWIG compatibility with TransformMap.

    In nanobind, Python dicts automatically convert to std::map<string, Isometry3d>.
    This class provides compatibility with code written for SWIG bindings.
    """
    pass

__all__ = [
    # Core types
    "ResourceLocator",
    "Resource",
    "GeneralResourceLocator",
    "SimpleLocatedResource",
    "BytesResource",

    # Manipulator
    "ManipulatorInfo",
    "JointState",

    # Collision
    "AllowedCollisionMatrix",
    "CollisionMarginData",

    # Profile (moved from tesseract_planning in 0.33.x)
    "Profile",
    "ProfileDictionary",

    # Kinematics
    "KinematicLimits",

    # Console bridge
    "OutputHandler",
    "LogLevel",
    "setLogLevel",
    "getLogLevel",
    "useOutputHandler",
    "restorePreviousOutputHandler",

    # Logging levels
    "CONSOLE_BRIDGE_LOG_DEBUG",
    "CONSOLE_BRIDGE_LOG_INFO",
    "CONSOLE_BRIDGE_LOG_WARN",
    "CONSOLE_BRIDGE_LOG_ERROR",
    "CONSOLE_BRIDGE_LOG_NONE",

    # Eigen helper types
    "Isometry3d",
    "Translation3d",
    "Quaterniond",
    "AngleAxisd",

    # Console bridge log function
    "log",

    # Plugin
    "PluginInfo",

    # Container types (SWIG compatibility)
    "VectorVector3d",
    "VectorIsometry3d",

    # Filesystem (SWIG compatibility)
    "FilesystemPath",

    # Transform map (SWIG compatibility)
    "TransformMap",
]
