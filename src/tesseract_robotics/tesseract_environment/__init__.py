"""tesseract_environment Python bindings (nanobind)"""

from tesseract_robotics import ensure_configured

ensure_configured()

# Import dependencies first to register their types for cross-module access
import tesseract_robotics.tesseract_common  # noqa: E402, F401 - needed for CollisionMarginData, ACM
import tesseract_robotics.tesseract_kinematics  # noqa: E402, F401 - needed for getKinematicGroup
import tesseract_robotics.tesseract_scene_graph  # noqa: E402, F401
import tesseract_robotics.tesseract_srdf  # noqa: E402, F401 - needed for getKinematicsInformation
from tesseract_robotics.tesseract_environment._tesseract_environment import *  # noqa: E402

# Re-export AnyPoly_wrap_EnvironmentConst from tesseract_task_composer for convenience
try:
    from tesseract_robotics.tesseract_task_composer import AnyPoly_wrap_EnvironmentConst
except ImportError:
    pass  # task_composer may not be available

__all__ = [
    # Environment
    "Environment",
    # Base command class
    "Command",
    # Link/Joint manipulation commands
    "AddLinkCommand",
    "RemoveLinkCommand",
    "AddSceneGraphCommand",
    "RemoveJointCommand",
    "ReplaceJointCommand",
    "MoveJointCommand",
    "MoveLinkCommand",
    # Joint limits commands
    "ChangeJointPositionLimitsCommand",
    "ChangeJointVelocityLimitsCommand",
    "ChangeJointAccelerationLimitsCommand",
    # Origin/transform commands
    "ChangeJointOriginCommand",
    "ChangeLinkOriginCommand",
    # Collision commands
    "ModifyAllowedCollisionsCommand",
    "ModifyAllowedCollisionsType",
    "ModifyAllowedCollisionsType_ADD",
    "ModifyAllowedCollisionsType_REMOVE",
    "ModifyAllowedCollisionsType_REPLACE",
    "RemoveAllowedCollisionLinkCommand",
    "ChangeCollisionMarginsCommand",
    "ChangeLinkCollisionEnabledCommand",
    # Visibility commands
    "ChangeLinkVisibilityCommand",
    # Events
    "Events",
    "Event",
    "CommandAppliedEvent",
    "SceneStateChangedEvent",
    # AnyPoly wrapper (re-exported from task_composer for SWIG compatibility)
    "AnyPoly_wrap_EnvironmentConst",
]
