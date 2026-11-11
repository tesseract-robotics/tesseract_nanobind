"""tesseract_srdf Python bindings"""

from collections.abc import Mapping, Sequence, Set

import tesseract_robotics.common
import tesseract_robotics.scene_graph


class KinematicsInformation:
    def __init__(self) -> None: ...

    @property
    def group_names(self) -> set[str]:
        """The names of every group (chain, joint and link groups)."""

    @group_names.setter
    def group_names(self, arg: Set[str], /) -> None: ...

    @property
    def chain_groups(self) -> dict[str, list[tuple[tesseract_robotics.common.LinkId, tesseract_robotics.common.LinkId]]]:
        """Chain groups: group name -> list of (base link, tip link)."""

    @chain_groups.setter
    def chain_groups(self, arg: Mapping[str, Sequence[tuple[tesseract_robotics.common.LinkId | str, tesseract_robotics.common.LinkId | str]]], /) -> None: ...

    @property
    def joint_groups(self) -> dict[str, list[tesseract_robotics.common.JointId]]:
        """Joint groups: group name -> joint names."""

    @joint_groups.setter
    def joint_groups(self, arg: Mapping[str, Sequence[tesseract_robotics.common.JointId | str]], /) -> None: ...

    @property
    def link_groups(self) -> dict[str, list[tesseract_robotics.common.LinkId]]:
        """Link groups: group name -> link names."""

    @link_groups.setter
    def link_groups(self, arg: Mapping[str, Sequence[tesseract_robotics.common.LinkId | str]], /) -> None: ...

    @property
    def kinematics_plugin_info(self) -> tesseract_robotics.common.KinematicsPluginInfo:
        """The forward / inverse kinematics solver plugins per group."""

    @kinematics_plugin_info.setter
    def kinematics_plugin_info(self, arg: tesseract_robotics.common.KinematicsPluginInfo, /) -> None: ...

    @property
    def group_states(self) -> dict[str, dict[str, dict[str, float]]]:
        """
        Named joint states per group: group name -> state name -> joint values.
        """

    @group_states.setter
    def group_states(self, arg: Mapping[str, Mapping[str, Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float]]], /) -> None: ...

    @property
    def group_tcps(self) -> dict[str, dict[str, tesseract_robotics.common.Isometry3d]]:
        """
        Named TCP offsets per group: group name -> TCP link name -> Isometry3d.
        """

    @group_tcps.setter
    def group_tcps(self, arg: Mapping[str, Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]], /) -> None: ...

    def insert(self, other: KinematicsInformation) -> None:
        """Merge `other` into this; on a name both define, `other`'s entry wins."""

    def clear(self) -> None:
        """Remove every group, state, TCP and plugin entry."""

    def hasGroup(self, group_name: str) -> bool:
        """True if `group_name` is a chain, joint or link group."""

    def addChainGroup(self, group_name: str, chain_group: Sequence[tuple[tesseract_robotics.common.LinkId | str, tesseract_robotics.common.LinkId | str]]) -> None:
        """Add (or replace) the chain group `group_name`."""

    def removeChainGroup(self, group_name: str) -> None:
        """Remove the chain group `group_name`, if present."""

    def hasChainGroup(self, group_name: str) -> bool:
        """True if `group_name` is a chain group."""

    def addJointGroup(self, group_name: str, joint_group: Sequence[tesseract_robotics.common.JointId | str]) -> None:
        """Add (or replace) the joint group `group_name`."""

    def removeJointGroup(self, group_name: str) -> None:
        """Remove the joint group `group_name`, if present."""

    def hasJointGroup(self, group_name: str) -> bool:
        """True if `group_name` is a joint group."""

    def addLinkGroup(self, group_name: str, link_group: Sequence[tesseract_robotics.common.LinkId | str]) -> None:
        """Add (or replace) the link group `group_name`."""

    def removeLinkGroup(self, group_name: str) -> None:
        """Remove the link group `group_name`, if present."""

    def hasLinkGroup(self, group_name: str) -> bool:
        """True if `group_name` is a link group."""

    def addGroupJointState(self, group_name: str, state_name: str, joint_state: Mapping[tesseract_robotics.common.JointId, float] | Mapping[str, float]) -> None:
        """Add (or replace) the named joint state `state_name` of `group_name`."""

    def removeGroupJointState(self, group_name: str, state_name: str) -> None:
        """Remove the named joint state, if present."""

    def hasGroupJointState(self, group_name: str, state_name: str) -> bool:
        """True if `group_name` has the named joint state `state_name`."""

    def addGroupTCP(self, group_name: str, tcp_name: tesseract_robotics.common.LinkId | str, tcp: tesseract_robotics.common.Isometry3d) -> None:
        """Add (or replace) the named TCP offset `tcp_name` of `group_name`."""

    def removeGroupTCP(self, group_name: str, tcp_name: tesseract_robotics.common.LinkId | str) -> None:
        """Remove the named TCP offset, if present."""

    def hasGroupTCP(self, group_name: str, tcp_name: tesseract_robotics.common.LinkId | str) -> bool:
        """True if `group_name` has the named TCP offset `tcp_name`."""

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

class SRDFModel:
    def __init__(self) -> None: ...

    def initFile(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, filename: str, locator: tesseract_robotics.common.ResourceLocator) -> None:
        """
        Parse the SRDF file `filename` against `scene_graph`, resolving URLs with `locator`; raises on invalid SRDF.
        """

    def initString(self, scene_graph: tesseract_robotics.scene_graph.SceneGraph, xmlstring: str, locator: tesseract_robotics.common.ResourceLocator) -> None:
        """
        Parse SRDF text against `scene_graph`, resolving URLs with `locator`; raises on invalid SRDF.
        """

    def saveToFile(self, file_path: str) -> bool:
        """
        Write the model as SRDF XML to `file_path`; False if the file cannot be written.
        """

    def clear(self) -> None:
        """Reset the model to empty."""

    @property
    def name(self) -> str:
        """The robot name."""

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def version(self) -> list[int]:
        """The SRDF format version, (major, minor, patch)."""

    @version.setter
    def version(self, arg: Sequence[int], /) -> None: ...

    @property
    def kinematics_information(self) -> KinematicsInformation:
        """Groups, group states, TCPs and kinematics plugin configuration."""

    @kinematics_information.setter
    def kinematics_information(self, arg: KinematicsInformation, /) -> None: ...

    @property
    def acm(self) -> tesseract_robotics.common.AllowedCollisionMatrix:
        """The allowed collision matrix (disable_collisions entries)."""

    @acm.setter
    def acm(self, arg: tesseract_robotics.common.AllowedCollisionMatrix, /) -> None: ...

    @property
    def collision_margin_data(self) -> tesseract_robotics.common.CollisionMarginData:
        """Collision margins (default and per link pair)."""

    @collision_margin_data.setter
    def collision_margin_data(self, arg: tesseract_robotics.common.CollisionMarginData, /) -> None: ...

    @property
    def contact_managers_plugin_info(self) -> tesseract_robotics.common.ContactManagersPluginInfo:
        """The contact manager plugin configuration."""

    @contact_managers_plugin_info.setter
    def contact_managers_plugin_info(self, arg: tesseract_robotics.common.ContactManagersPluginInfo, /) -> None: ...

    @property
    def calibration_info(self) -> tesseract_robotics.common.CalibrationInfo:
        """Joint origin calibrations (calibration_config)."""

    @calibration_info.setter
    def calibration_info(self, arg: tesseract_robotics.common.CalibrationInfo, /) -> None: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    def __repr__(self) -> str: ...

def processSRDFAllowedCollisions(scene_graph: tesseract_robotics.scene_graph.SceneGraph, srdf_model: SRDFModel) -> None:
    """Process SRDF allowed collisions and add to scene graph ACM"""
