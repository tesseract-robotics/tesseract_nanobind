"""tesseract_scene_graph Python bindings"""

from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.common
import tesseract_robotics.geometry


class JointType(enum.Enum):
    def __str__(self) -> str: ...

    UNKNOWN = 0

    REVOLUTE = 1

    CONTINUOUS = 2

    PRISMATIC = 3

    FLOATING = 4

    PLANAR = 5

    FIXED = 6

class JointDynamics:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, damping: float, friction: float) -> None: ...

    @property
    def damping(self) -> float:
        """Viscous damping [N·s/m or N·m·s/rad]."""

    @damping.setter
    def damping(self, arg: float, /) -> None: ...

    @property
    def friction(self) -> float:
        """Static friction [N or N·m]."""

    @friction.setter
    def friction(self, arg: float, /) -> None: ...

    def clear(self) -> None:
        """Reset to zero."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __str__(self) -> str: ...

    def __repr__(self) -> str: ...

class JointLimits:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, lower: float, upper: float, effort: float, velocity: float, acceleration: float, jerk: float) -> None: ...

    @property
    def lower(self) -> float:
        """Lower position limit [m or rad]."""

    @lower.setter
    def lower(self, arg: float, /) -> None: ...

    @property
    def upper(self) -> float:
        """Upper position limit [m or rad]."""

    @upper.setter
    def upper(self, arg: float, /) -> None: ...

    @property
    def effort(self) -> float:
        """Effort limit [N or N·m]."""

    @effort.setter
    def effort(self, arg: float, /) -> None: ...

    @property
    def velocity(self) -> float:
        """Velocity limit [m/s or rad/s]."""

    @velocity.setter
    def velocity(self, arg: float, /) -> None: ...

    @property
    def acceleration(self) -> float:
        """Acceleration limit [m/s² or rad/s²]."""

    @acceleration.setter
    def acceleration(self, arg: float, /) -> None: ...

    @property
    def jerk(self) -> float:
        """Jerk limit [m/s³ or rad/s³]."""

    @jerk.setter
    def jerk(self, arg: float, /) -> None: ...

    def clear(self) -> None:
        """Reset every limit to zero."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __str__(self) -> str: ...

    def __repr__(self) -> str: ...

class JointSafety:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, soft_upper_limit: float, soft_lower_limit: float, k_position: float, k_velocity: float) -> None: ...

    @property
    def soft_upper_limit(self) -> float:
        """Upper soft limit [m or rad]."""

    @soft_upper_limit.setter
    def soft_upper_limit(self, arg: float, /) -> None: ...

    @property
    def soft_lower_limit(self) -> float:
        """Lower soft limit [m or rad]."""

    @soft_lower_limit.setter
    def soft_lower_limit(self, arg: float, /) -> None: ...

    @property
    def k_position(self) -> float:
        """Position gain of the safety controller."""

    @k_position.setter
    def k_position(self, arg: float, /) -> None: ...

    @property
    def k_velocity(self) -> float:
        """Velocity gain of the safety controller."""

    @k_velocity.setter
    def k_velocity(self, arg: float, /) -> None: ...

    def clear(self) -> None:
        """Reset to zero."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __str__(self) -> str: ...

class JointCalibration:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, reference_position: float, rising: float, falling: float) -> None: ...

    @property
    def reference_position(self) -> float:
        """Reference position [m or rad]."""

    @reference_position.setter
    def reference_position(self, arg: float, /) -> None: ...

    @property
    def rising(self) -> float:
        """Rising-edge position [m or rad]."""

    @rising.setter
    def rising(self, arg: float, /) -> None: ...

    @property
    def falling(self) -> float:
        """Falling-edge position [m or rad]."""

    @falling.setter
    def falling(self, arg: float, /) -> None: ...

    def clear(self) -> None:
        """Reset to zero."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __str__(self) -> str: ...

class JointMimic:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, offset: float, multiplier: float, joint_id: tesseract_robotics.common.JointId | str) -> None: ...

    @property
    def offset(self) -> float:
        """Offset added after the multiplier [m or rad]."""

    @offset.setter
    def offset(self, arg: float, /) -> None: ...

    @property
    def multiplier(self) -> float:
        """Factor applied to the mimicked joint's value."""

    @multiplier.setter
    def multiplier(self, arg: float, /) -> None: ...

    @property
    def joint_id(self) -> tesseract_robotics.common.JointId:
        """The mimicked joint."""

    @joint_id.setter
    def joint_id(self, arg: tesseract_robotics.common.JointId | str, /) -> None: ...

    def clear(self) -> None:
        """Reset to offset 0, multiplier 1, no joint."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __str__(self) -> str: ...

class Joint:
    @overload
    def __init__(self, id: tesseract_robotics.common.JointId | str) -> None: ...

    @overload
    def __init__(self) -> None: ...

    def getName(self) -> str:
        """The joint name."""

    def getId(self) -> tesseract_robotics.common.JointId:
        """The joint id."""

    @property
    def type(self) -> JointType:
        """The joint type."""

    @type.setter
    def type(self, arg: JointType, /) -> None: ...

    @property
    def axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """The joint axis, in the joint frame (unit vector)."""

    @axis.setter
    def axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def child_link_id(self) -> tesseract_robotics.common.LinkId:
        """The link this joint moves."""

    @child_link_id.setter
    def child_link_id(self, arg: tesseract_robotics.common.LinkId | str, /) -> None: ...

    @property
    def parent_link_id(self) -> tesseract_robotics.common.LinkId:
        """The link this joint is attached to."""

    @parent_link_id.setter
    def parent_link_id(self, arg: tesseract_robotics.common.LinkId | str, /) -> None: ...

    @property
    def parent_to_joint_origin_transform(self) -> tesseract_robotics.common.Isometry3d:
        """The joint frame in the parent link's frame."""

    @parent_to_joint_origin_transform.setter
    def parent_to_joint_origin_transform(self, arg: tesseract_robotics.common.Isometry3d, /) -> None: ...

    @property
    def dynamics(self) -> JointDynamics:
        """Damping and friction, or None."""

    @dynamics.setter
    def dynamics(self, arg: JointDynamics, /) -> None: ...

    @property
    def limits(self) -> JointLimits:
        """Position, velocity, acceleration, jerk and effort limits, or None."""

    @limits.setter
    def limits(self, arg: JointLimits, /) -> None: ...

    @property
    def safety(self) -> JointSafety:
        """Soft limits, or None."""

    @safety.setter
    def safety(self, arg: JointSafety, /) -> None: ...

    @property
    def calibration(self) -> JointCalibration:
        """Calibration edges, or None."""

    @calibration.setter
    def calibration(self, arg: JointCalibration, /) -> None: ...

    @property
    def mimic(self) -> JointMimic:
        """The joint this one mimics, or None."""

    @mimic.setter
    def mimic(self, arg: JointMimic, /) -> None: ...

    def clear(self) -> None:
        """Reset every field."""

    @overload
    def clone(self) -> Joint:
        """A copy, optionally under a new id."""

    @overload
    def clone(self, id: tesseract_robotics.common.JointId | str) -> Joint: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __repr__(self) -> str: ...

class Material:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, name: str) -> None: ...

    def getName(self) -> str:
        """The material name."""

    @property
    def texture_filename(self) -> str:
        """Texture image path, or empty."""

    @texture_filename.setter
    def texture_filename(self, arg: str, /) -> None: ...

    @property
    def color(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]:
        """RGBA color, each in [0, 1]."""

    @color.setter
    def color(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')] | Sequence[float], /) -> None: ...

    @staticmethod
    def getDefaultMaterial() -> Material:
        """The material links without one are drawn with."""

    def clear(self) -> None:
        """Reset the color and texture."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

class Inertial:
    def __init__(self) -> None: ...

    @property
    def origin(self) -> tesseract_robotics.common.Isometry3d:
        """The center of mass frame, in the link frame."""

    @origin.setter
    def origin(self, arg: tesseract_robotics.common.Isometry3d, /) -> None: ...

    @property
    def mass(self) -> float:
        """Mass [kg]."""

    @mass.setter
    def mass(self, arg: float, /) -> None: ...

    @property
    def ixx(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @ixx.setter
    def ixx(self, arg: float, /) -> None: ...

    @property
    def ixy(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @ixy.setter
    def ixy(self, arg: float, /) -> None: ...

    @property
    def ixz(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @ixz.setter
    def ixz(self, arg: float, /) -> None: ...

    @property
    def iyy(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @iyy.setter
    def iyy(self, arg: float, /) -> None: ...

    @property
    def iyz(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @iyz.setter
    def iyz(self, arg: float, /) -> None: ...

    @property
    def izz(self) -> float:
        """Inertia tensor entry [kg·m²], in the origin frame."""

    @izz.setter
    def izz(self, arg: float, /) -> None: ...

    def clear(self) -> None:
        """Reset to zero mass and inertia."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

class Visual:
    def __init__(self) -> None: ...

    @property
    def origin(self) -> tesseract_robotics.common.Isometry3d:
        """The geometry's frame, in the link frame."""

    @origin.setter
    def origin(self, arg: tesseract_robotics.common.Isometry3d, /) -> None: ...

    @property
    def geometry(self) -> tesseract_robotics.geometry.Geometry:
        """The geometry drawn."""

    @geometry.setter
    def geometry(self, arg: tesseract_robotics.geometry.Geometry, /) -> None: ...

    @property
    def material(self) -> Material:
        """The material, or None for the default."""

    @material.setter
    def material(self, arg: Material, /) -> None: ...

    @property
    def name(self) -> str:
        """The visual's name."""

    @name.setter
    def name(self, arg: str, /) -> None: ...

    def clear(self) -> None:
        """Reset every field."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

class Collision:
    def __init__(self) -> None: ...

    @property
    def origin(self) -> tesseract_robotics.common.Isometry3d:
        """The geometry's frame, in the link frame."""

    @origin.setter
    def origin(self, arg: tesseract_robotics.common.Isometry3d, /) -> None: ...

    @property
    def geometry(self) -> tesseract_robotics.geometry.Geometry:
        """The collision geometry."""

    @geometry.setter
    def geometry(self, arg: tesseract_robotics.geometry.Geometry, /) -> None: ...

    @property
    def name(self) -> str:
        """The collision's name."""

    @name.setter
    def name(self, arg: str, /) -> None: ...

    def clear(self) -> None:
        """Reset every field."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

class Link:
    @overload
    def __init__(self, id: tesseract_robotics.common.LinkId | str) -> None: ...

    @overload
    def __init__(self) -> None: ...

    def getName(self) -> str:
        """The link name."""

    def getId(self) -> tesseract_robotics.common.LinkId:
        """The link id."""

    @property
    def inertial(self) -> Inertial:
        """Mass properties, or None."""

    @inertial.setter
    def inertial(self, arg: Inertial, /) -> None: ...

    @property
    def visual(self) -> list[Visual]:
        """The visual elements."""

    @visual.setter
    def visual(self, arg: Sequence[Visual], /) -> None: ...

    @property
    def collision(self) -> list[Collision]:
        """The collision elements."""

    @collision.setter
    def collision(self, arg: Sequence[Collision], /) -> None: ...

    @property
    def visible(self) -> bool:
        """Whether the link is visible (link.h:210)"""

    @visible.setter
    def visible(self, arg: bool, /) -> None: ...

    @property
    def collision_enabled(self) -> bool:
        """Whether the link takes part in collision checking (link.h:213)"""

    @collision_enabled.setter
    def collision_enabled(self, arg: bool, /) -> None: ...

    def addVisual(self, visual: Visual) -> None:
        """Add a Visual to this link"""

    def addCollision(self, collision: Collision) -> None:
        """Add a Collision to this link"""

    def clearVisual(self) -> None:
        """Clear all Visual elements from this link"""

    def clearCollision(self) -> None:
        """Clear all Collision elements from this link"""

    def clear(self) -> None:
        """Reset every field."""

    @overload
    def clone(self) -> Link:
        """A copy, optionally under a new id."""

    @overload
    def clone(self, id: tesseract_robotics.common.LinkId | str) -> Link: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __repr__(self) -> str: ...

class ShortestPath:
    def __init__(self) -> None: ...

    @property
    def links(self) -> list[tesseract_robotics.common.LinkId]:
        """The links on the path, root to tip."""

    @links.setter
    def links(self, arg: Sequence[tesseract_robotics.common.LinkId | str], /) -> None: ...

    @property
    def joints(self) -> list[tesseract_robotics.common.JointId]:
        """The joints on the path, root to tip."""

    @joints.setter
    def joints(self, arg: Sequence[tesseract_robotics.common.JointId | str], /) -> None: ...

    @property
    def active_joints(self) -> list[tesseract_robotics.common.JointId]:
        """The active joints on the path, root to tip."""

    @active_joints.setter
    def active_joints(self, arg: Sequence[tesseract_robotics.common.JointId | str], /) -> None: ...

    def __str__(self) -> str: ...

class SceneGraph:
    def __init__(self, name: str = '') -> None: ...

    def clone(self) -> SceneGraph:
        """A deep copy."""

    def clear(self) -> None:
        """Remove every link, joint and allowed collision."""

    def setName(self, name: str) -> None:
        """Set the graph's name."""

    def getName(self) -> str:
        """The graph's name."""

    def setRoot(self, name: tesseract_robotics.common.LinkId | str) -> bool:
        """Make the link `name` the root; False if unknown."""

    def getRoot(self) -> tesseract_robotics.common.LinkId:
        """The root link id."""

    @overload
    def addLink(self, link: Link, replace_allowed: bool = False) -> bool:
        """
        Add a link (with the joint attaching it); False if the name exists and replacement is not allowed.
        """

    @overload
    def addLink(self, link: Link, joint: Joint) -> bool: ...

    def getLink(self, name: tesseract_robotics.common.LinkId | str) -> Link:
        """The link `name` (shared), or None."""

    def getLinks(self) -> list[Link]:
        """Every link (shared)."""

    def getLeafLinks(self) -> list[Link]:
        """The links with no child joint."""

    def removeLink(self, name: tesseract_robotics.common.LinkId | str, recursive: bool = False) -> bool:
        """
        Remove the link `name` (and its subtree when `recursive`); False if unknown.
        """

    def moveLink(self, joint: Joint) -> bool:
        """
        Re-attach a link by replacing its parent joint with `joint`; False on failure.
        """

    def setLinkVisibility(self, name: tesseract_robotics.common.LinkId | str, visibility: bool) -> None:
        """Show or hide the link `name`."""

    def getLinkVisibility(self, name: tesseract_robotics.common.LinkId | str) -> bool:
        """Whether the link `name` is visible."""

    def setLinkCollisionEnabled(self, name: tesseract_robotics.common.LinkId | str, enabled: bool) -> None:
        """Enable or disable collision checking of the link `name`."""

    def getLinkCollisionEnabled(self, name: tesseract_robotics.common.LinkId | str) -> bool:
        """Whether collision checking of the link `name` is enabled."""

    def addJoint(self, joint: Joint) -> bool:
        """Add `joint` between two existing links; False on failure."""

    def getJoint(self, name: tesseract_robotics.common.JointId | str) -> Joint:
        """The joint `name` (shared), or None."""

    def removeJoint(self, name: tesseract_robotics.common.JointId | str, recursive: bool = False) -> bool:
        """
        Remove the joint `name` (and its child subtree when `recursive`); False if unknown.
        """

    def moveJoint(self, name: tesseract_robotics.common.JointId | str, parent_link: tesseract_robotics.common.LinkId | str) -> bool:
        """Re-parent the joint `name` onto `parent_link`; False on failure."""

    def getJoints(self) -> list[Joint]:
        """Every joint (shared)."""

    def getActiveJoints(self) -> list[Joint]:
        """The active (non-fixed, non-mimic) joints."""

    def changeJointOrigin(self, name: tesseract_robotics.common.JointId | str, new_origin: tesseract_robotics.common.Isometry3d) -> bool:
        """Set the origin of the joint `name`; False if unknown."""

    def changeJointLimits(self, name: tesseract_robotics.common.JointId | str, limits: JointLimits) -> bool:
        """Replace the limits of the joint `name`; False if unknown."""

    def changeJointPositionLimits(self, name: tesseract_robotics.common.JointId | str, lower: float, upper: float) -> bool:
        """Set the position limits of the joint `name`; False if unknown."""

    def changeJointVelocityLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the velocity limit of the joint `name`; False if unknown."""

    def changeJointAccelerationLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the acceleration limit of the joint `name`; False if unknown."""

    def changeJointJerkLimits(self, name: tesseract_robotics.common.JointId | str, limit: float) -> bool:
        """Set the jerk limit of the joint `name`; False if unknown."""

    def getJointLimits(self, name: tesseract_robotics.common.JointId | str) -> JointLimits:
        """The limits of the joint `name` (shared), or None."""

    @overload
    def addAllowedCollision(self, link_id1: tesseract_robotics.common.LinkId | str, link_id2: tesseract_robotics.common.LinkId | str, reason: str) -> None:
        """Allow the link pair to collide, with a reason."""

    @overload
    def addAllowedCollision(self, pair: tesseract_robotics.common.LinkIdPair | tuple[str, str], reason: str) -> None: ...

    @overload
    def removeAllowedCollision(self, link_id1: tesseract_robotics.common.LinkId | str, link_id2: tesseract_robotics.common.LinkId | str) -> None:
        """Stop allowing the link pair (or every pair of a link) to collide."""

    @overload
    def removeAllowedCollision(self, pair: tesseract_robotics.common.LinkIdPair | tuple[str, str]) -> None: ...

    @overload
    def removeAllowedCollision(self, link_id: tesseract_robotics.common.LinkId | str) -> None: ...

    def clearAllowedCollisions(self) -> None:
        """Remove every allowed collision."""

    def isCollisionAllowed(self, pair: tesseract_robotics.common.LinkIdPair | tuple[str, str]) -> bool:
        """True if the link pair may collide."""

    def getAllowedCollisionMatrix(self) -> tesseract_robotics.common.AllowedCollisionMatrix:
        """Get the allowed collision matrix"""

    def setAllowedCollisionMatrix(self, acm: tesseract_robotics.common.AllowedCollisionMatrix) -> None:
        """
        Replace the allowed collision matrix. The graph keeps this object: later edits to `acm` are edits to the graph's matrix, as with getAllowedCollisionMatrix.
        """

    def getSourceLink(self, joint_name: tesseract_robotics.common.JointId | str) -> Link:
        """The parent link of the joint `joint_name`."""

    def getTargetLink(self, joint_name: tesseract_robotics.common.JointId | str) -> Link:
        """The child link of the joint `joint_name`."""

    def getInboundJoints(self, link_name: tesseract_robotics.common.LinkId | str) -> list[Joint]:
        """The joints whose child is the link `link_name`."""

    def getOutboundJoints(self, link_name: tesseract_robotics.common.LinkId | str) -> list[Joint]:
        """The joints whose parent is the link `link_name`."""

    def isAcyclic(self) -> bool:
        """True if the graph has no cycle."""

    def isTree(self) -> bool:
        """True if the graph is a tree."""

    def isEmpty(self) -> bool:
        """True if the graph has no link."""

    def getAdjacentLinkIds(self, id: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.common.LinkId]:
        """The links one joint below `id`."""

    def getInvAdjacentLinkIds(self, id: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.common.LinkId]:
        """The links one joint above `id`."""

    def getLinkChildrenIds(self, id: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.common.LinkId]:
        """Every link below `id`."""

    @overload
    def getJointChildrenIds(self, id: tesseract_robotics.common.JointId | str) -> list[tesseract_robotics.common.LinkId]:
        """Every link below the given joint(s)."""

    @overload
    def getJointChildrenIds(self, ids: Sequence[tesseract_robotics.common.JointId | str]) -> list[tesseract_robotics.common.LinkId]: ...

    def getShortestPath(self, root: tesseract_robotics.common.LinkId | str, tip: tesseract_robotics.common.LinkId | str) -> ShortestPath:
        """The links and joints from `root` to `tip`."""

    def getAdjacencyMap(self, link_ids: Sequence[tesseract_robotics.common.LinkId | str]) -> dict[tesseract_robotics.common.LinkId, tesseract_robotics.common.LinkId]:
        """
        Map each link that moves with one of `link_ids` to that link's id (graph.h:499). Each listed link maps itself and every link below it, stopping at another listed link. An unknown id raises KeyError.
        """

    def saveDOT(self, path: str) -> None:
        """Write the graph as Graphviz DOT to `path`."""

    @overload
    def insertSceneGraph(self, scene_graph: SceneGraph, prefix: str = '') -> bool: ...

    @overload
    def insertSceneGraph(self, scene_graph: SceneGraph, joint: Joint, prefix: str = '') -> bool: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __repr__(self) -> str: ...
