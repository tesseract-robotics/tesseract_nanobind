"""tesseract_state_solver Python bindings"""

from collections.abc import Mapping, Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.common
import tesseract_robotics.scene_graph


class SceneState:
    def __init__(self) -> None: ...

    @property
    def joints(self) -> dict[tesseract_robotics.common.JointId, float]:
        """Joint values: joint id -> value [rad or m]."""

    @joints.setter
    def joints(self, arg: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float], /) -> None: ...

    @property
    def floating_joints(self) -> dict[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d]:
        """Floating-joint transforms: joint id -> Isometry3d."""

    @floating_joints.setter
    def floating_joints(self, arg: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], /) -> None: ...

    @property
    def link_transforms(self) -> dict[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d]:
        """World transform of every link: link id -> Isometry3d (a copy)."""

    @link_transforms.setter
    def link_transforms(self, arg: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], /) -> None: ...

    @property
    def joint_transforms(self) -> dict[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d]:
        """World transform of every joint: joint id -> Isometry3d (a copy)."""

    @joint_transforms.setter
    def joint_transforms(self, arg: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], /) -> None: ...

    def getJointValues(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        The values of `joint_ids`, in that order; an unknown id raises KeyError.
        """

    def getFloatingJointValues(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str]) -> dict[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d]: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    __hash__: None = None

class StateSolver:
    @overload
    def setState(self, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> None: ...

    @overload
    def setState(self, joint_values: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> None:
        """
        Set joint values (a vector in getActiveJointIds order, a {joint: value} map, or ids and values) and optional floating-joint transforms. Unknown joints raise ValueError before anything changes.
        """

    @overload
    def setState(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> None:
        """
        Set joint values (a vector in getActiveJointIds order, a {joint: value} map, or ids and values) and optional floating-joint transforms. Unknown joints raise ValueError before anything changes.
        """

    @overload
    def setState(self, floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def getState(self) -> SceneState:
        """
        The state at the given joint values (forms as setState), or the current state with no arguments, without changing the solver.
        """

    @overload
    def getState(self, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> SceneState: ...

    @overload
    def getState(self, joint_values: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> SceneState: ...

    @overload
    def getState(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> SceneState: ...

    @overload
    def getState(self, floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> SceneState: ...

    @overload
    def getLinkTransforms(self) -> list[tesseract_robotics.common.Isometry3d]:
        """
        Every link's world transform, at the current state or at `joint_values` for `joint_ids`.
        """

    @overload
    def getLinkTransforms(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> dict[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d]: ...

    def getRandomState(self) -> SceneState:
        """A state at uniformly random joint values within the joint limits."""

    @overload
    def getJacobian(self, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], link_id: tesseract_robotics.common.LinkId | str, floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @overload
    def getJacobian(self, joint_ids: Sequence[tesseract_robotics.common.JointId | str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], link_id: tesseract_robotics.common.LinkId | str, floating_joint_values: Mapping[tesseract_robotics.common.JointId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d] = {}) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    def getJointIds(self) -> list[tesseract_robotics.common.JointId]:
        """Every joint id of the scene graph."""

    def getFloatingJointIds(self) -> list[tesseract_robotics.common.JointId]:
        """The floating joint ids."""

    def getActiveJointIds(self) -> list[tesseract_robotics.common.JointId]:
        """The active (non-fixed, non-mimic) joint ids, in state-vector order."""

    def getBaseLinkId(self) -> tesseract_robotics.common.LinkId:
        """The root link id."""

    def getLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Every link id."""

    def getActiveLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Links moved by an active joint."""

    def getStaticLinkIds(self) -> list[tesseract_robotics.common.LinkId]:
        """Links no active joint moves."""

    def isActiveLinkId(self, link_id: tesseract_robotics.common.LinkId | str) -> bool:
        """True if an active joint moves `link_id`."""

    def hasLinkId(self, link_id: tesseract_robotics.common.LinkId | str) -> bool:
        """True if `link_id` is a link of the scene graph."""

    def getLinkTransform(self, link_id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """The current world transform of `link_id`."""

    def getRelativeLinkTransform(self, from_link_id: tesseract_robotics.common.LinkId | str, to_link_id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """
        The transform of `to_link_id` in the frame of `from_link_id`, at the current state.
        """

    def getLimits(self) -> tesseract_robotics.common.KinematicLimits:
        """
        The active joints' position, velocity, acceleration and jerk limits, in getActiveJointIds order.
        """

    def clone(self) -> StateSolver:
        """An independent copy at the same state."""

class MutableStateSolver(StateSolver):
    def setRevision(self, revision: int) -> None:
        """Set the revision this solver tracks (the environment's command count)."""

    def getRevision(self) -> int:
        """The revision this solver tracks."""

    def addLink(self, link: tesseract_robotics.scene_graph.Link, joint: tesseract_robotics.scene_graph.Joint) -> bool:
        """Add `link` attached by `joint`; False if the link or joint exists."""

    def moveLink(self, joint: tesseract_robotics.scene_graph.Joint) -> bool:
        """
        Re-attach a link by replacing its parent joint with `joint`; False on failure.
        """

    def removeLink(self, name: tesseract_robotics.common.LinkId | str) -> bool:
        """Remove the link `name` and its subtree; False if unknown."""

    def replaceJoint(self, joint: tesseract_robotics.scene_graph.Joint) -> bool:
        """
        Replace the joint of the same name (same child link); False on failure.
        """

    def removeJoint(self, name: tesseract_robotics.common.JointId | str) -> bool:
        """Remove the joint `name` and its child subtree; False if unknown."""

    def moveJoint(self, name: tesseract_robotics.common.JointId | str, parent_link: tesseract_robotics.common.LinkId | str) -> bool:
        """Re-parent the joint `name` onto `parent_link`; False on failure."""

    def changeJointOrigin(self, name: tesseract_robotics.common.JointId | str, new_origin: tesseract_robotics.common.Isometry3d) -> bool:
        """Set the origin of the joint `name`; False if unknown."""

    def changeJointPositionLimits(self, name: tesseract_robotics.common.JointId | str, lower: float, upper: float) -> bool:
        """Set the position limits of the joint `name`; False if unknown."""

    def changeJointVelocityLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the velocity limit of the joint `name`; False if unknown."""

    def changeJointAccelerationLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the acceleration limit of the joint `name`; False if unknown."""

    def changeJointJerkLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the jerk limit of the joint `name`; False if unknown."""

    def insertSceneGraph(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, joint: tesseract_robotics.scene_graph.Joint, prefix: str = '') -> bool: ...

class KDLStateSolver(StateSolver):
    """
    State solver on a KDL tree. Floating joint values are ignored, and setState / getState with only floating_joint_values raise RuntimeError (upstream: not supported).
    """

    def __init__(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph) -> None: ...

    def clone(self) -> StateSolver:
        """An independent copy at the same state."""

class OFKTStateSolver(MutableStateSolver):
    @overload
    def __init__(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph) -> None: ...

    @overload
    def __init__(self, root_link_id: tesseract_robotics.common.LinkId | str) -> None: ...

    def clone(self) -> StateSolver:
        """An independent copy at the same state."""
