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
    # Contact results
    "ContactResult",
    "ContactResultVector",
    "ContactResultMap",
    "UnorderedLinkPairError",
    "EmptyContactResultsError",
    "ContactRequest",
    "ContactResultValidator",
    "ContactTrajectorySubstepResults",
    "ContactTrajectoryStepResults",
    "ContactTrajectoryResults",
    # Config
    "ContactManagerConfig",
    "CollisionCheckConfig",
    # Contact managers
    "DiscreteContactManager",
    "ContinuousContactManager",
    "ContactManagersPluginFactory",
    # Convex hulls
    "makeConvexMesh",
    "createConvexHull",
    "ConvexHullError",
    # Convex decomposition
    "FillMode",
    "VHACDParameters",
    "ConvexDecomposition",
    "ConvexDecompositionVHACD",
    "MalformedFacesError",
    "NonTriangleFaceError",
]
