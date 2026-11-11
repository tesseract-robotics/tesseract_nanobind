"""tesseract_environment Python bindings"""

from collections.abc import Callable, Mapping, Sequence
import datetime
import enum
import os
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.collision
import tesseract_robotics.common
import tesseract_robotics.kinematics
import tesseract_robotics.scene_graph
import tesseract_robotics.srdf
import tesseract_robotics.state_solver


class Events(enum.Enum):
    COMMAND_APPLIED = 0

    SCENE_STATE_CHANGED = 1

class Event:
    """An environment event; `type` names its derived class."""

    @property
    def type(self) -> Events:
        """Which event this is."""

class CommandAppliedEvent(Event):
    """Fired after commands were applied (applyCommand(s), init, reset)."""

    @property
    def commands(self) -> list[Command]:
        """The environment's command history at the event (a copy)."""

    @property
    def revision(self) -> int:
        """The environment's revision after the commands."""

class SceneStateChangedEvent(Event):
    """
    Fired after the current state changed (setState, and every applied command).
    """

    @property
    def state(self) -> tesseract_robotics.state_solver.SceneState:
        """The new current state."""

class EventTypeError(TypeError):
    """
    cast_CommandAppliedEvent / cast_SceneStateChangedEvent got an event of the other type.
    """

def cast_CommandAppliedEvent(evt: Event) -> CommandAppliedEvent:
    """
    Downcast an Event to CommandAppliedEvent. Raises EventTypeError if evt.type is not COMMAND_APPLIED.
    """

def cast_SceneStateChangedEvent(evt: Event) -> SceneStateChangedEvent:
    """
    Downcast an Event to SceneStateChangedEvent. Raises EventTypeError if evt.type is not SCENE_STATE_CHANGED.
    """

@overload
def checkTrajectory(manager: tesseract_robotics.collision.DiscreteContactManager, state_solver: tesseract_robotics.state_solver.StateSolver, joint_ids: Sequence[tesseract_robotics.common.JointId | str], traj: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='C')] | Sequence[Sequence[float]], config: tesseract_robotics.collision.CollisionCheckConfig) -> tuple[tesseract_robotics.collision.ContactTrajectoryResults, list[tesseract_robotics.collision.ContactResultMap]]: ...

@overload
def checkTrajectory(manager: tesseract_robotics.collision.DiscreteContactManager, manip: tesseract_robotics.kinematics.JointGroup, traj: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='C')] | Sequence[Sequence[float]], config: tesseract_robotics.collision.CollisionCheckConfig) -> tuple[tesseract_robotics.collision.ContactTrajectoryResults, list[tesseract_robotics.collision.ContactResultMap]]: ...

@overload
def checkTrajectory(manager: tesseract_robotics.collision.ContinuousContactManager, state_solver: tesseract_robotics.state_solver.StateSolver, joint_ids: Sequence[tesseract_robotics.common.JointId | str], traj: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='C')] | Sequence[Sequence[float]], config: tesseract_robotics.collision.CollisionCheckConfig) -> tuple[tesseract_robotics.collision.ContactTrajectoryResults, list[tesseract_robotics.collision.ContactResultMap]]: ...

@overload
def checkTrajectory(manager: tesseract_robotics.collision.ContinuousContactManager, manip: tesseract_robotics.kinematics.JointGroup, traj: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='C')] | Sequence[Sequence[float]], config: tesseract_robotics.collision.CollisionCheckConfig) -> tuple[tesseract_robotics.collision.ContactTrajectoryResults, list[tesseract_robotics.collision.ContactResultMap]]: ...

def checkTrajectorySegment(manager: tesseract_robotics.collision.ContinuousContactManager, state0: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], state1: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], contact_request: tesseract_robotics.collision.ContactRequest) -> tesseract_robotics.collision.ContactResultMap: ...

@overload
def checkTrajectoryState(manager: tesseract_robotics.collision.DiscreteContactManager, state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], contact_request: tesseract_robotics.collision.ContactRequest) -> tesseract_robotics.collision.ContactResultMap: ...

@overload
def checkTrajectoryState(manager: tesseract_robotics.collision.ContinuousContactManager, state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], contact_request: tesseract_robotics.collision.ContactRequest) -> tesseract_robotics.collision.ContactResultMap: ...

class CommandType(enum.Enum):
    UNINITIALIZED = -1

    ADD_LINK = 0

    MOVE_LINK = 1

    MOVE_JOINT = 2

    REMOVE_LINK = 3

    REMOVE_JOINT = 4

    CHANGE_LINK_ORIGIN = 5

    CHANGE_JOINT_ORIGIN = 6

    CHANGE_LINK_COLLISION_ENABLED = 7

    CHANGE_LINK_VISIBILITY = 8

    MODIFY_ALLOWED_COLLISIONS = 9

    REMOVE_ALLOWED_COLLISION_LINK = 10

    ADD_SCENE_GRAPH = 11

    CHANGE_JOINT_POSITION_LIMITS = 12

    CHANGE_JOINT_VELOCITY_LIMITS = 13

    CHANGE_JOINT_ACCELERATION_LIMITS = 14

    ADD_KINEMATICS_INFORMATION = 15

    REPLACE_JOINT = 16

    CHANGE_COLLISION_MARGINS = 17

    ADD_CONTACT_MANAGERS_PLUGIN_INFO = 18

    SET_ACTIVE_DISCRETE_CONTACT_MANAGER = 19

    SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER = 20

    ADD_TRAJECTORY_LINK = 21

class Command:
    def getType(self) -> CommandType: ...

    def __eq__(self, arg: Command, /) -> bool: ...

    def __ne__(self, arg: Command, /) -> bool: ...

    __hash__: None = None

class RemoveJointCommand(Command):
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str) -> None: ...

    def getJointId(self) -> tesseract_robotics.common.JointId: ...

    def __eq__(self, arg: RemoveJointCommand, /) -> bool: ...

    def __ne__(self, arg: RemoveJointCommand, /) -> bool: ...

    __hash__: None = None

class AddLinkCommand(Command):
    @overload
    def __init__(self, link: tesseract_robotics.scene_graph.Link, replace_allowed: bool = False) -> None: ...

    @overload
    def __init__(self, link: tesseract_robotics.scene_graph.Link, joint: tesseract_robotics.scene_graph.Joint, replace_allowed: bool = False) -> None: ...

    def getLink(self) -> tesseract_robotics.scene_graph.Link: ...

    def getJoint(self) -> tesseract_robotics.scene_graph.Joint: ...

    def replaceAllowed(self) -> bool: ...

    def __eq__(self, arg: AddLinkCommand, /) -> bool: ...

    def __ne__(self, arg: AddLinkCommand, /) -> bool: ...

    __hash__: None = None

class RemoveLinkCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str) -> None: ...

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def __eq__(self, arg: RemoveLinkCommand, /) -> bool: ...

    def __ne__(self, arg: RemoveLinkCommand, /) -> bool: ...

    __hash__: None = None

class AddSceneGraphCommand(Command):
    @overload
    def __init__(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, prefix: str = '') -> None: ...

    @overload
    def __init__(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, joint: tesseract_robotics.scene_graph.Joint, prefix: str = '') -> None: ...

    def getSceneGraph(self) -> tesseract_robotics.scene_graph.SceneGraph: ...

    def getJoint(self) -> tesseract_robotics.scene_graph.Joint: ...

    def getPrefix(self) -> str: ...

    def __eq__(self, arg: AddSceneGraphCommand, /) -> bool: ...

    def __ne__(self, arg: AddSceneGraphCommand, /) -> bool: ...

    __hash__: None = None

class AddTrajectoryLinkCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str, parent_link_id: tesseract_robotics.common.LinkId | str, trajectory: tesseract_robotics.common.JointTrajectory, replace_allowed: bool = False, method: AddTrajectoryLinkCommand.Method = AddTrajectoryLinkCommand.Method.PER_STATE_OBJECTS) -> None: ...

    class Method(enum.Enum):
        PER_STATE_OBJECTS = 0

        PER_STATE_CONVEX_HULL = 1

        GLOBAL_PER_LINK_CONVEX_HULL = 2

        GLOBAL_CONVEX_HULL = 3

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def getParentLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def getTrajectory(self) -> tesseract_robotics.common.JointTrajectory: ...

    def replaceAllowed(self) -> bool: ...

    def getMethod(self) -> AddTrajectoryLinkCommand.Method: ...

    def __eq__(self, arg: AddTrajectoryLinkCommand, /) -> bool: ...

    def __ne__(self, arg: AddTrajectoryLinkCommand, /) -> bool: ...

    __hash__: None = None

class AddKinematicsInformationCommand(Command):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, kinematics_information: tesseract_robotics.srdf.KinematicsInformation) -> None: ...

    def getKinematicsInformation(self) -> tesseract_robotics.srdf.KinematicsInformation: ...

    def __eq__(self, arg: AddKinematicsInformationCommand, /) -> bool: ...

    def __ne__(self, arg: AddKinematicsInformationCommand, /) -> bool: ...

    __hash__: None = None

class AddContactManagersPluginInfoCommand(Command):
    def __init__(self, contact_managers_plugin_info: tesseract_robotics.common.ContactManagersPluginInfo) -> None: ...

    def getContactManagersPluginInfo(self) -> tesseract_robotics.common.ContactManagersPluginInfo: ...

    def __eq__(self, arg: AddContactManagersPluginInfoCommand, /) -> bool: ...

    def __ne__(self, arg: AddContactManagersPluginInfoCommand, /) -> bool: ...

    __hash__: None = None

class ModifyAllowedCollisionsType(enum.Enum):
    ADD = 0

    REMOVE = 1

    REPLACE = 2

class ModifyAllowedCollisionsCommand(Command):
    def __init__(self, acm: tesseract_robotics.common.AllowedCollisionMatrix, type: ModifyAllowedCollisionsType) -> None: ...

    def getModifyType(self) -> ModifyAllowedCollisionsType: ...

    def getAllowedCollisionMatrix(self) -> tesseract_robotics.common.AllowedCollisionMatrix: ...

    def __eq__(self, arg: ModifyAllowedCollisionsCommand, /) -> bool: ...

    def __ne__(self, arg: ModifyAllowedCollisionsCommand, /) -> bool: ...

    __hash__: None = None

class RemoveAllowedCollisionLinkCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str) -> None: ...

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def __eq__(self, arg: RemoveAllowedCollisionLinkCommand, /) -> bool: ...

    def __ne__(self, arg: RemoveAllowedCollisionLinkCommand, /) -> bool: ...

    __hash__: None = None

class ChangeJointPositionLimitsCommand(Command):
    @overload
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str, lower: float, upper: float) -> None: ...

    @overload
    def __init__(self, limits: Mapping[tesseract_robotics.common.JointId, tuple[float, float]] | Mapping[str, tuple[float, float]]) -> None: ...

    def getLimits(self) -> dict[tesseract_robotics.common.JointId, tuple[float, float]]: ...

    def __eq__(self, arg: ChangeJointPositionLimitsCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeJointPositionLimitsCommand, /) -> bool: ...

    __hash__: None = None

class ChangeJointVelocityLimitsCommand(Command):
    @overload
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str, limit: float) -> None: ...

    @overload
    def __init__(self, limits: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float]) -> None: ...

    def getLimits(self) -> dict[tesseract_robotics.common.JointId, float]: ...

    def __eq__(self, arg: ChangeJointVelocityLimitsCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeJointVelocityLimitsCommand, /) -> bool: ...

    __hash__: None = None

class ChangeJointAccelerationLimitsCommand(Command):
    @overload
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str, limit: float) -> None: ...

    @overload
    def __init__(self, limits: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float]) -> None: ...

    def getLimits(self) -> dict[tesseract_robotics.common.JointId, float]: ...

    def __eq__(self, arg: ChangeJointAccelerationLimitsCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeJointAccelerationLimitsCommand, /) -> bool: ...

    __hash__: None = None

class ChangeCollisionMarginsCommand(Command):
    @overload
    def __init__(self, default_margin: float) -> None: ...

    @overload
    def __init__(self, pair_margin_data: tesseract_robotics.common.CollisionMarginPairData, override_type: tesseract_robotics.common.CollisionMarginPairOverrideType = tesseract_robotics.common.CollisionMarginPairOverrideType.MODIFY) -> None: ...

    @overload
    def __init__(self, default_margin: float, pair_margin_data: tesseract_robotics.common.CollisionMarginPairData, override_type: tesseract_robotics.common.CollisionMarginPairOverrideType = tesseract_robotics.common.CollisionMarginPairOverrideType.MODIFY) -> None: ...

    def getDefaultCollisionMargin(self) -> float | None: ...

    def getCollisionMarginPairData(self) -> tesseract_robotics.common.CollisionMarginPairData: ...

    def getCollisionMarginPairOverrideType(self) -> tesseract_robotics.common.CollisionMarginPairOverrideType: ...

    def __eq__(self, arg: ChangeCollisionMarginsCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeCollisionMarginsCommand, /) -> bool: ...

    __hash__: None = None

class ChangeLinkCollisionEnabledCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str, enabled: bool) -> None: ...

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def getEnabled(self) -> bool: ...

    def __eq__(self, arg: ChangeLinkCollisionEnabledCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeLinkCollisionEnabledCommand, /) -> bool: ...

    __hash__: None = None

class ChangeLinkVisibilityCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str, visible: bool) -> None: ...

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def getEnabled(self) -> bool: ...

    def __eq__(self, arg: ChangeLinkVisibilityCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeLinkVisibilityCommand, /) -> bool: ...

    __hash__: None = None

class ChangeJointOriginCommand(Command):
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str, origin: tesseract_robotics.common.Isometry3d) -> None: ...

    def getJointId(self) -> tesseract_robotics.common.JointId: ...

    def getOrigin(self) -> tesseract_robotics.common.Isometry3d: ...

    def __eq__(self, arg: ChangeJointOriginCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeJointOriginCommand, /) -> bool: ...

    __hash__: None = None

class ChangeLinkOriginCommand(Command):
    def __init__(self, link_id: tesseract_robotics.common.LinkId | str, origin: tesseract_robotics.common.Isometry3d) -> None: ...

    def getLinkId(self) -> tesseract_robotics.common.LinkId: ...

    def getOrigin(self) -> tesseract_robotics.common.Isometry3d: ...

    def __eq__(self, arg: ChangeLinkOriginCommand, /) -> bool: ...

    def __ne__(self, arg: ChangeLinkOriginCommand, /) -> bool: ...

    __hash__: None = None

class MoveJointCommand(Command):
    def __init__(self, joint_id: tesseract_robotics.common.JointId | str, parent_link: tesseract_robotics.common.LinkId | str) -> None: ...

    def getJointId(self) -> tesseract_robotics.common.JointId: ...

    def getParentLink(self) -> tesseract_robotics.common.LinkId: ...

    def __eq__(self, arg: MoveJointCommand, /) -> bool: ...

    def __ne__(self, arg: MoveJointCommand, /) -> bool: ...

    __hash__: None = None

class MoveLinkCommand(Command):
    def __init__(self, joint: tesseract_robotics.scene_graph.Joint) -> None: ...

    def getJoint(self) -> tesseract_robotics.scene_graph.Joint: ...

    def __eq__(self, arg: MoveLinkCommand, /) -> bool: ...

    def __ne__(self, arg: MoveLinkCommand, /) -> bool: ...

    __hash__: None = None

class ReplaceJointCommand(Command):
    def __init__(self, joint: tesseract_robotics.scene_graph.Joint) -> None: ...

    def getJoint(self) -> tesseract_robotics.scene_graph.Joint: ...

    def __eq__(self, arg: ReplaceJointCommand, /) -> bool: ...

    def __ne__(self, arg: ReplaceJointCommand, /) -> bool: ...

    __hash__: None = None

class SetActiveDiscreteContactManagerCommand(Command):
    def __init__(self, active_contact_manager: str) -> None: ...

    def getName(self) -> str: ...

    def __eq__(self, arg: SetActiveDiscreteContactManagerCommand, /) -> bool: ...

    def __ne__(self, arg: SetActiveDiscreteContactManagerCommand, /) -> bool: ...

    __hash__: None = None

class SetActiveContinuousContactManagerCommand(Command):
    def __init__(self, active_contact_manager: str) -> None: ...

    def getName(self) -> str: ...

    def __eq__(self, arg: SetActiveContinuousContactManagerCommand, /) -> bool: ...

    def __ne__(self, arg: SetActiveContinuousContactManagerCommand, /) -> bool: ...

    __hash__: None = None

class EnvironmentContactAllowedValidator(tesseract_robotics.common.ContactAllowedValidator):
    """
    The contact validator an Environment installs on its contact managers: a link pair may collide unless the scene graph's allowed collision matrix allows it.
    """

    def __init__(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph) -> None:
        """
        Validate against `scene_graph`'s ACM, kept alive and read at each call.
        """

class Environment:
    """
    A scene graph with its current state, kinematic groups, contact managers and command history.

    Thread-safe: every method takes the environment's lock without holding the GIL, so Python threads may share one environment. Event and find-TCP callbacks run on the thread that changed or queried the environment, under its lock: they must not change it.
    """

    def __init__(self) -> None:
        """An empty, uninitialized environment; call `init`."""

    @overload
    def init(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph) -> bool:
        """
        Initialize from a copy of `scene_graph`, with no SRDF; False on failure.
        """

    @overload
    def init(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, srdf: tesseract_robotics.srdf.SRDFModel) -> bool:
        """Initialize from copies of `scene_graph` and `srdf`; False on failure."""

    @overload
    def init(self, urdf_string: str, locator: tesseract_robotics.common.ResourceLocator) -> bool:
        """
        Initialize from URDF text, resolving package:// and file:// URLs with `locator`; False on failure.
        """

    @overload
    def init(self, urdf_path: os.PathLike, locator: tesseract_robotics.common.ResourceLocator) -> bool:
        """Initialize from a URDF file; False on failure."""

    @overload
    def init(self, urdf_string: str, srdf_string: str, locator: tesseract_robotics.common.ResourceLocator) -> bool:
        """Initialize from URDF and SRDF text; False on failure."""

    @overload
    def init(self, urdf_path: os.PathLike, srdf_path: os.PathLike, locator: tesseract_robotics.common.ResourceLocator) -> bool:
        """Initialize from URDF and SRDF files; False on failure."""

    @overload
    def init(self, commands: Sequence[Command]) -> bool:
        """
        Initialize by replaying `commands`, the inverse of getCommandHistory: the list must start with an AddSceneGraphCommand. False on failure.
        """

    def isInitialized(self) -> bool:
        """True once an init call succeeded."""

    def reset(self) -> bool:
        """
        Re-initialize from the init commands, dropping every later command; False on failure.
        """

    def clear(self) -> None:
        """Drop everything: the environment is uninitialized."""

    def getRevision(self) -> int:
        """The number of commands applied, init's own included."""

    def getInitRevision(self) -> int:
        """The revision right after init: what reset() returns to."""

    def getTimestamp(self) -> datetime.datetime:
        """
        Last update time (any change to the environment), as a naive local `datetime.datetime`.
        """

    def getCurrentStateTimestamp(self) -> datetime.datetime:
        """
        Last update time of the current state, as a naive local `datetime.datetime`.
        """

    def getName(self) -> str:
        """The environment's name (the URDF robot name)."""

    def setName(self, name: str) -> None:
        """Rename the environment."""

    def getSceneGraph(self) -> tesseract_robotics.scene_graph.SceneGraph:
        """
        The scene graph, shared with the environment: read-only, changed only through commands.
        """

    @overload
    def getState(self) -> tesseract_robotics.state_solver.SceneState:
        """The current state: joint values and every link's world transform."""

    @overload
    def getState(self, joints: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float], floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> tesseract_robotics.state_solver.SceneState:
        """
        The state at the active-joint values `joints` (the rest current), without changing the environment. Unknown or non-active joints raise ValueError.
        """

    @overload
    def getState(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> tesseract_robotics.state_solver.SceneState:
        """
        The state at `joint_values` for `joint_ids` (the rest current), without changing the environment.
        """

    @overload
    def getState(self, floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> tesseract_robotics.state_solver.SceneState:
        """
        The state at the floating-joint transforms `floating_joints` (the rest current).
        """

    def link_transforms_batch(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None, None), writable=False)] | Sequence[Sequence[float]], link_ids: Sequence[tesseract_robotics.common.LinkId | str]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None, 4, 4), order='C')]:
        """
        World transforms of `link_ids` at each row of `joint_values` ((M, N) for the N `joint_ids`), as an (M, L, 4, 4) array; the environment's state is not changed.
        """

    @overload
    def setState(self, joints: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float], floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> None:
        """
        Set the active-joint values `joints` (and floating-joint transforms). Unknown or non-active joints raise ValueError before anything changes.
        """

    @overload
    def setState(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> None:
        """Set `joint_values` for `joint_ids` (and floating-joint transforms)."""

    @overload
    def setState(self, floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """Set floating-joint transforms."""

    def addEventCallback(self, fn: Callable[[Event], None]) -> int:
        """
        Register `fn(event)`, called after each change with a CommandAppliedEvent or a SceneStateChangedEvent; returns the key for removeEventCallback.

        `fn` runs on the thread that changed the environment, under its lock: it must not change the environment. The environment keeps `fn` alive; `clone()` does not copy it.
        """

    def removeEventCallback(self, key: int) -> None:
        """
        Unregister the callback addEventCallback returned `key` for; an unknown key is ignored.
        """

    def clearEventCallbacks(self) -> None:
        """Unregister every event callback."""

    def applyCommand(self, command: Command) -> bool:
        """
        Apply one command; True on success. The command joins the history (getCommandHistory).
        """

    def applyCommands(self, commands: Sequence[Command]) -> bool:
        """Apply the commands in order; False, and stop, at the first that fails."""

    def getCommandHistory(self) -> list[Command]:
        """
        Every command applied since init, init's own included, each as its derived class.
        """

    def getStateSolver(self) -> tesseract_robotics.state_solver.StateSolver:
        """
        A new state solver at the current state, independent of the environment.
        """

    def getJointIds(self) -> list[tesseract_robotics.common.JointId]:
        """Every joint id."""

    def getActiveJointIds(self) -> list[tesseract_robotics.common.JointId]:
        """The active (non-fixed, non-mimic) joint ids, in state-vector order."""

    def getLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Every link id."""

    @overload
    def getActiveLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Links moved by an active joint."""

    @overload
    def getActiveLinkIds(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> list[tesseract_robotics.common.LinkId]:
        """Links moved by any of `joint_ids`."""

    @overload
    def getStaticLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Links no active joint moves."""

    @overload
    def getStaticLinkIds(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> list[tesseract_robotics.common.LinkId]:
        """Links none of `joint_ids` moves."""

    def getRootLinkId(self) -> tesseract_robotics.common.LinkId:
        """The scene graph's root link."""

    @overload
    def getCurrentJointValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The active joints' current values, in getActiveJointIds order."""

    @overload
    def getCurrentJointValues(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The current values of `joint_ids`; a non-active id raises KeyError."""

    def getLinkTransform(self, link_id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """The link's current world transform; an unknown link raises KeyError."""

    def getRelativeLinkTransform(self, from_link_id: tesseract_robotics.common.LinkId | str, to_link_id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """
        The transform of `to_link_id` in the frame of `from_link_id`; an unknown link raises KeyError.
        """

    @overload
    def getLinkTransforms(self) -> list[tesseract_robotics.common.Isometry3d]:
        """Every link's current world transform."""

    @overload
    def getLinkTransforms(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joints: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> dict[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d]:
        """
        Every link's world transform at `joint_values` for `joint_ids`, without changing the environment.
        """

    @overload
    def getCurrentFloatingJointValues(self) -> dict[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d]:
        """Every floating joint's current transform."""

    @overload
    def getCurrentFloatingJointValues(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> dict[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d]:
        """
        The current transforms of `joint_ids`; a non-floating id raises KeyError.
        """

    def getJointLimits(self, joint_id: tesseract_robotics.common.JointId | str) -> tesseract_robotics.scene_graph.JointLimits:
        """A copy of the joint's limits; an unknown joint raises KeyError."""

    def getLinkCollisionEnabled(self, link_id: tesseract_robotics.common.LinkId | str) -> bool:
        """
        Whether the link takes part in collision checks; KeyError for an unknown link.
        """

    def getLinkVisibility(self, link_id: tesseract_robotics.common.LinkId | str) -> bool:
        """Whether the link is visible; KeyError for an unknown link."""

    def getContactManagersPluginInfo(self) -> tesseract_robotics.common.ContactManagersPluginInfo:
        """The contact-manager plugin configuration."""

    def getLink(self, link_id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.scene_graph.Link:
        """The link (shared, read-only); an unknown link raises KeyError."""

    def getJoint(self, joint_id: tesseract_robotics.common.JointId | str) -> tesseract_robotics.scene_graph.Joint:
        """The joint (shared, read-only); an unknown joint raises KeyError."""

    def getGroupNames(self) -> set[str]:
        """The SRDF group names."""

    def getGroupJointIds(self, group_name: str) -> list[tesseract_robotics.common.JointId]:
        """The group's joint ids; an unknown group raises KeyError."""

    @overload
    def getJointGroup(self, group_name: str) -> tesseract_robotics.kinematics.JointGroup:
        """
        The SRDF group as a JointGroup at the current state; an unknown group raises KeyError.
        """

    @overload
    def getJointGroup(self, group_name: str, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> tesseract_robotics.kinematics.JointGroup:
        """
        A new JointGroup named `group_name` over `joint_ids`; unknown joints raise ValueError.
        """

    def getKinematicGroup(self, group_name: str, ik_solver_name: str = '') -> tesseract_robotics.kinematics.KinematicGroup:
        """
        The SRDF group with its IK solver (the group's default when `ik_solver_name` is empty). An unknown group or solver raises KeyError; a configured solver that fails to load, RuntimeError.
        """

    def findTCPOffset(self, manip_info: tesseract_robotics.common.ManipulatorInfo) -> tesseract_robotics.common.Isometry3d:
        """
        The TCP offset `manip_info.tcp_offset` names: a transform, a link, a group TCP, or what a find-TCP callback returns; RuntimeError when none locates it.
        """

    def addFindTCPOffsetCallback(self, fn: Callable[[tesseract_robotics.common.ManipulatorInfo], tesseract_robotics.common.Isometry3d]) -> None:
        """
        Register `fn(manip_info) -> Isometry3d` as a source of TCP offsets for `findTCPOffset`.

        `findTCPOffset` asks the callbacks, in registration order, only for a `tcp_offset` name that is neither an `Isometry3d`, a link, nor a group TCP of `manip_info.manipulator`. `clone()` copies the callbacks, and the environment keeps `fn` alive.

        A callback that raises, or returns anything but an `Isometry3d` (`None` included), counts as "not located": `findTCPOffset` tries the next one and raises `RuntimeError` ("Could not find tcp by name ...") when none returns a transform. The callback's own exception is not reported (upstream catches it with `catch (...)`).

        `fn` may run on any thread, under this environment's lock: it must not change this environment, and must not keep it alive itself (a closure over it, or an attribute holding it): the environment keeps `fn` alive from C++, so the garbage collector cannot see that cycle.

        Args:
            fn: Callable taking the `ManipulatorInfo` and returning the TCP offset.
        """

    @overload
    def getDiscreteContactManager(self) -> tesseract_robotics.collision.DiscreteContactManager:
        """A clone of the active discrete contact manager, at the current state."""

    @overload
    def getDiscreteContactManager(self, name: str) -> tesseract_robotics.collision.DiscreteContactManager:
        """
        A new discrete contact manager from plugin `name`; an unregistered name raises KeyError.
        """

    @overload
    def getContinuousContactManager(self) -> tesseract_robotics.collision.ContinuousContactManager:
        """
        A clone of the active continuous contact manager, at the current state.
        """

    @overload
    def getContinuousContactManager(self, name: str) -> tesseract_robotics.collision.ContinuousContactManager:
        """
        A new continuous contact manager from plugin `name`; an unregistered name raises KeyError.
        """

    def setActiveDiscreteContactManager(self, name: str) -> bool:
        """
        Make plugin `name` the active discrete contact manager; False if it is not registered.
        """

    def setActiveContinuousContactManager(self, name: str) -> bool:
        """
        Make plugin `name` the active continuous contact manager; False if it is not registered.
        """

    def clearCachedDiscreteContactManager(self) -> None:
        """Drop the cached discrete manager: the next call rebuilds it."""

    def clearCachedContinuousContactManager(self) -> None:
        """Drop the cached continuous manager: the next call rebuilds it."""

    def getAllowedCollisionMatrix(self) -> tesseract_robotics.common.AllowedCollisionMatrix:
        """The allowed collision matrix (shared, read-only)."""

    def getCollisionMarginData(self) -> tesseract_robotics.common.CollisionMarginData:
        """The collision margins (shared, read-only)."""

    def setResourceLocator(self, locator: tesseract_robotics.common.ResourceLocator) -> None:
        """Replace the resource locator commands resolve URLs with."""

    def getResourceLocator(self) -> tesseract_robotics.common.ResourceLocator:
        """The resource locator."""

    def getKinematicsInformation(self) -> tesseract_robotics.srdf.KinematicsInformation:
        """
        The SRDF kinematics information: groups, group states, TCPs and IK plugin configuration.
        """

    def clone(self) -> Environment:
        """
        A deep copy, find-TCP callbacks included and event callbacks not; it compares equal.
        """

    def __eq__(self, arg: Environment, /) -> bool: ...

    def __ne__(self, arg: Environment, /) -> bool: ...

    __hash__: None = None
