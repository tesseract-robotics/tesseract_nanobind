"""tesseract_environment Python bindings (nanobind)"""

# Import dependencies first to register their types for cross-module access
import tesseract_robotics.tesseract_common  # noqa: F401 - needed for CollisionMarginData, ACM
import tesseract_robotics.tesseract_kinematics  # noqa: F401 - needed for getKinematicGroup
import tesseract_robotics.tesseract_scene_graph  # noqa: F401
import tesseract_robotics.tesseract_srdf  # noqa: F401 - needed for getKinematicsInformation
from tesseract_robotics.tesseract_environment._tesseract_environment import *

__all__ = [
    # Environment
    "Environment",
    "EnvironmentContactAllowedValidator",
    # Base command class
    "Command",
    "CommandType",
    # Link/Joint manipulation commands
    "AddLinkCommand",
    "RemoveLinkCommand",
    "AddSceneGraphCommand",
    "AddKinematicsInformationCommand",
    "AddContactManagersPluginInfoCommand",
    "AddTrajectoryLinkCommand",
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
    # Contact manager commands
    "SetActiveDiscreteContactManagerCommand",
    "SetActiveContinuousContactManagerCommand",
    # Visibility commands
    "ChangeLinkVisibilityCommand",
    # Events
    "Events",
    "Event",
    "CommandAppliedEvent",
    "SceneStateChangedEvent",
    "EventTypeError",
    # Utils
    "checkTrajectory",
    "checkTrajectorySegment",
    "checkTrajectoryState",
]
