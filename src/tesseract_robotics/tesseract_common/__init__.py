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
    # Identity types (upstream LinkId / JointId; str converts implicitly)
    "LinkId",
    "JointId",
    "LinkIdPair",
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
    "getAllowedCollisions",
    "ContactAllowedValidator",
    "ACMContactAllowedValidator",
    "CombinedContactAllowedValidator",
    "CombinedContactAllowedValidatorType",
    "CollisionMarginData",
    "CollisionMarginPairData",
    "CollisionMarginPairOverrideType",
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
    # Logging (spdlog)
    "LogLevel",
    "Logger",
    "LogSink",
    "StderrSink",
    "PythonLoggingSink",
    "LogRecord",
    "getLogger",
    "isLogLevelEnabled",
    "addLogRecordHandler",
    "removeLogRecordHandler",
    # Eigen helper types
    "Isometry3d",
    "Translation3d",
    "Quaterniond",
    "AngleAxisd",
    "Hyperplane3d",
    "ParametrizedLine3d",
    "nearest_rotation",
    "NonFiniteMatrixError",
    "orthonormality_residual",
    "orthonormality_residual_each",
    "determinant",
    "rotation_determinant_each",
    "norm_each",
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
    # Calibration (SRDF calibration_config)
    "CalibrationInfo",
    # Container types (SWIG compatibility)
    "VectorVector3d",
    "VectorIsometry3d",
]
