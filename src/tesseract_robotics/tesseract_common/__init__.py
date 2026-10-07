"""tesseract_common Python bindings (nanobind)"""

import numpy as np

from tesseract_robotics.tesseract_common._tesseract_common import *

# Standard unit axes — Vector3d-compatible, frozen to guard against accidental
# in-place mutation when the same constant is reused across call sites.
X_AXIS: np.ndarray = np.array([1.0, 0.0, 0.0])
Y_AXIS: np.ndarray = np.array([0.0, 1.0, 0.0])
Z_AXIS: np.ndarray = np.array([0.0, 0.0, 1.0])
X_AXIS.flags.writeable = False
Y_AXIS.flags.writeable = False
Z_AXIS.flags.writeable = False


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
    "JointTrajectory",
    # Collision
    "AllowedCollisionMatrix",
    "makeOrderedLinkPair",
    "getAllowedCollisions",
    "ContactAllowedValidator",
    "ACMContactAllowedValidator",
    "CombinedContactAllowedValidator",
    "CombinedContactAllowedValidatorType",
    "CollisionMarginData",
    "CollisionMarginPairData",
    "CollisionMarginPairOverrideType",
    "CollisionMarginOverrideType",
    # Kinematics
    "KinematicLimits",
    "satisfiesLimits",
    "isWithinLimits",
    "enforceLimits",
    "LimitsSizeMismatchError",
    # Frame and error math
    "twistChangeRefPoint",
    "twistChangeBase",
    "jacobianChangeBase",
    "jacobianChangeRefPoint",
    "calcRotationalError",
    "calcTransformError",
    "calcJacobianTransformErrorDiff",
    "applyTolerances",
    "ToleranceSizeMismatchError",
    # Console bridge
    "OutputHandler",
    "LogLevel",
    "setLogLevel",
    "getLogLevel",
    "useOutputHandler",
    "restorePreviousOutputHandler",
    # Eigen helper types
    "Isometry3d",
    "Translation3d",
    "Quaterniond",
    "AngleAxisd",
    "Hyperplane3d",
    "ParametrizedLine3d",
    # Unit axes (Vector3d-compatible numpy arrays)
    "X_AXIS",
    "Y_AXIS",
    "Z_AXIS",
    # Console bridge log function
    "log",
    # Plugin
    "PluginInfo",
    "PluginInfoContainer",
    "KinematicsPluginInfo",
    "ContactManagersPluginInfo",
    "ProfilesPluginInfo",
    "TaskComposerPluginInfo",
    # Container types (SWIG compatibility)
    "VectorVector3d",
    "VectorIsometry3d",
]
