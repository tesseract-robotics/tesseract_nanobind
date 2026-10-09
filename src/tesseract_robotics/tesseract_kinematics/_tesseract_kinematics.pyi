"""tesseract_kinematics Python bindings"""

from collections.abc import Mapping, Sequence
import os
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph
import tesseract_robotics.tesseract_state_solver._tesseract_state_solver


class URParameters:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, d1: float, a2: float, a3: float, d4: float, d5: float, d6: float) -> None: ...

    @property
    def d1(self) -> float: ...

    @d1.setter
    def d1(self, arg: float, /) -> None: ...

    @property
    def a2(self) -> float: ...

    @a2.setter
    def a2(self, arg: float, /) -> None: ...

    @property
    def a3(self) -> float: ...

    @a3.setter
    def a3(self, arg: float, /) -> None: ...

    @property
    def d4(self) -> float: ...

    @d4.setter
    def d4(self, arg: float, /) -> None: ...

    @property
    def d5(self) -> float: ...

    @d5.setter
    def d5(self, arg: float, /) -> None: ...

    @property
    def d6(self) -> float: ...

    @d6.setter
    def d6(self, arg: float, /) -> None: ...

UR10Parameters: URParameters = ...

UR5Parameters: URParameters = ...

UR3Parameters: URParameters = ...

UR10eParameters: URParameters = ...

UR5eParameters: URParameters = ...

UR3eParameters: URParameters = ...

class KinGroupIKInput:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, pose: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, working_frame: str, tip_link_name: str) -> None: ...

    @property
    def pose(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d: ...

    @pose.setter
    def pose(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def working_frame(self) -> str: ...

    @working_frame.setter
    def working_frame(self, arg: str, /) -> None: ...

    @property
    def tip_link_name(self) -> str: ...

    @tip_link_name.setter
    def tip_link_name(self, arg: str, /) -> None: ...

class KinGroupIKInputs:
    def __init__(self) -> None: ...

    def __len__(self) -> int: ...

    def __getitem__(self, arg: int, /) -> KinGroupIKInput: ...

    def append(self, arg: KinGroupIKInput, /) -> None: ...

    def clear(self) -> None: ...

class ForwardKinematics:
    def calcFwdKin(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.Isometry3d]: ...

    def calcJacobian(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    def getBaseLinkName(self) -> str: ...

    def getJointNames(self) -> list[str]: ...

    def getTipLinkNames(self) -> list[str]: ...

    def numJoints(self) -> int: ...

    def getSolverName(self) -> str: ...

    def clone(self) -> ForwardKinematics: ...

class InverseKinematics:
    def calcInvKin(self, tip_link_poses: Mapping[str, tesseract_robotics.tesseract_common._tesseract_common.Isometry3d], seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]: ...

    def getJointNames(self) -> list[str]: ...

    def numJoints(self) -> int: ...

    def getBaseLinkName(self) -> str: ...

    def getWorkingFrame(self) -> str: ...

    def getTipLinkNames(self) -> list[str]: ...

    def getSolverName(self) -> str: ...

    def clone(self) -> InverseKinematics: ...

class JointGroup:
    def __init__(self, name: str, joint_names: Sequence[str], scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> None: ...

    def calcFwdKin(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.Isometry3d]: ...

    @overload
    def calcJacobian(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @overload
    def calcJacobian(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str, link_point: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @overload
    def calcJacobian(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], base_link_name: str, link_name: str) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @overload
    def calcJacobian(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], base_link_name: str, link_name: str, link_point: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    def calcJacobianWithPoint(self, joint_angles: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str, link_point: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
        """
        Alias of `calcJacobian(joint_angles, link_name, link_point)`, the native form.
        """

    def getJointNames(self) -> list[str]: ...

    def getLinkNames(self) -> list[str]: ...

    def getActiveLinkNames(self) -> list[str]: ...

    def getStaticLinkNames(self) -> list[str]: ...

    def isActiveLinkName(self, link_name: str) -> bool: ...

    def hasLinkName(self, link_name: str) -> bool: ...

    def getLimits(self) -> tesseract_robotics.tesseract_common._tesseract_common.KinematicLimits: ...

    def setLimits(self, limits: tesseract_robotics.tesseract_common._tesseract_common.KinematicLimits) -> None: ...

    def getRedundancyCapableJointIndices(self) -> list[int]: ...

    def numJoints(self) -> int: ...

    def getBaseLinkName(self) -> str: ...

    def getName(self) -> str: ...

    def checkJoints(self, vec: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> bool: ...

class KinematicGroup(JointGroup):
    def __init__(self, name: str, joint_names: Sequence[str], inv_kin: InverseKinematics, scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> None: ...

    @overload
    def calcInvKin(self, tip_link_poses: KinGroupIKInputs, seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]: ...

    @overload
    def calcInvKin(self, tip_link_pose: KinGroupIKInput, seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]: ...

    @overload
    def calcInvKin(self, tip_link_poses: Sequence[KinGroupIKInput], seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]: ...

    def calcInvKinMultiple(self, tip_link_poses: Sequence[KinGroupIKInput], seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """
        Alias of `calcInvKin(tip_link_poses, seed)` with a list, the native form.
        """

    def getAllValidWorkingFrames(self) -> list[str]: ...

    def getAllPossibleTipLinkNames(self) -> list[str]: ...

    def getInverseKinematics(self) -> InverseKinematics: ...

class KinematicsPluginFactory:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, config_path: os.PathLike, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    @overload
    def __init__(self, config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    def addSearchPath(self, path: str) -> None: ...

    def getSearchPaths(self) -> list[str]: ...

    def addSearchLibrary(self, library_name: str) -> None: ...

    def getSearchLibraries(self) -> list[str]: ...

    def addFwdKinPlugin(self, group_name: str, solver_name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> None: ...

    def getFwdKinPlugins(self) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.PluginInfoContainer]: ...

    def removeFwdKinPlugin(self, group_name: str, solver_name: str) -> None:
        """
        Remove a forward kinematics solver from a group.

        Raises:
            KeyError: the group or the solver is unknown.
            KinematicsPluginRemovalError: it is the group's last solver.
        """

    def setDefaultFwdKinPlugin(self, group_name: str, solver_name: str) -> None:
        """
        Set a group's default forward kinematics solver.

        Raises:
            KeyError: the group or the solver is unknown.
        """

    def getDefaultFwdKinPlugin(self, group_name: str) -> str: ...

    def addInvKinPlugin(self, group_name: str, solver_name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> None: ...

    def getInvKinPlugins(self) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.PluginInfoContainer]: ...

    def removeInvKinPlugin(self, group_name: str, solver_name: str) -> None:
        """
        Remove an inverse kinematics solver from a group.

        Raises:
            KeyError: the group or the solver is unknown.
            KinematicsPluginRemovalError: it is the group's last solver.
        """

    def setDefaultInvKinPlugin(self, group_name: str, solver_name: str) -> None:
        """
        Set a group's default inverse kinematics solver.

        Raises:
            KeyError: the group or the solver is unknown.
        """

    def getDefaultInvKinPlugin(self, group_name: str) -> str: ...

    @overload
    def createFwdKin(self, group_name: str, solver_name: str, scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> ForwardKinematics: ...

    @overload
    def createFwdKin(self, solver_name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo, scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> ForwardKinematics: ...

    @overload
    def createInvKin(self, group_name: str, solver_name: str, scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> InverseKinematics: ...

    @overload
    def createInvKin(self, solver_name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo, scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> InverseKinematics: ...

    def saveConfig(self, file_path: str | os.PathLike) -> None: ...

    def getConfig(self) -> str:
        """The factory configuration as a YAML document string."""

class KinematicsPluginRemovalError(RuntimeError):
    """
    Refused to remove a group's last kinematics solver: tesseract 0.35.0 then reads and writes through an erased map iterator (kinematics_plugin_factory.cpp:150-154, tesseract-robotics/tesseract#1381).
    """

def getRedundantSolutions(sol: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)], redundancy_capable_joints: Sequence[int]) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
    """
    Get redundant solutions for a joint configuration by adding +/- 2*pi to redundancy capable joints
    """

@overload
def numericalJacobian(change_base: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, kin: ForwardKinematics, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str, link_point: Annotated[NDArray[numpy.float64], dict(shape=(3), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
    """
    Finite-difference Jacobian of a tip link (step 1e-8 rad per joint, utils.cpp:44).

    Raises:
        ValueError: joint_values does not have kin.numJoints() entries.
        KeyError: link_name is not a tip link of kin.
    """

@overload
def numericalJacobian(change_base: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, joint_group: JointGroup, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], link_name: str, link_point: Annotated[NDArray[numpy.float64], dict(shape=(3), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
    """
    Finite-difference Jacobian of a link of joint_group (step 1e-8 rad per joint, utils.cpp:80).

    Raises:
        ValueError: joint_values does not have joint_group.numJoints() entries.
        KeyError: link_name is not a link of joint_group.
    """

@overload
def numericalJacobian(joint_group: JointGroup, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], base_link_name: str, base_link_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, link_name: str, link_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
    """
    Finite-difference Jacobian of link_name relative to base_link_name, in the base link frame.

    Raises:
        ValueError: joint_values does not have joint_group.numJoints() entries.
        KeyError: base_link_name or link_name is not a link of joint_group.
    """

def isNearSingularity(jacobian: Annotated[NDArray[numpy.float64], dict(shape=(None, None), writable=False)], threshold: float = 0.01) -> bool:
    """
    True when the smallest singular value of jacobian is below threshold.

    Raises:
        ValueError: jacobian is empty.
    """

class ManipulabilityEllipsoid:
    def __init__(self) -> None: ...

    @property
    def eigen_values(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @property
    def measure(self) -> float: ...

    @property
    def condition(self) -> float: ...

    @property
    def volume(self) -> float: ...

    def __repr__(self) -> str: ...

class Manipulability:
    def __init__(self) -> None: ...

    @property
    def m(self) -> ManipulabilityEllipsoid: ...

    @property
    def m_linear(self) -> ManipulabilityEllipsoid: ...

    @property
    def m_angular(self) -> ManipulabilityEllipsoid: ...

    @property
    def f(self) -> ManipulabilityEllipsoid: ...

    @property
    def f_linear(self) -> ManipulabilityEllipsoid: ...

    @property
    def f_angular(self) -> ManipulabilityEllipsoid: ...

    def __repr__(self) -> str: ...

def calcManipulability(jacobian: Annotated[NDArray[numpy.float64], dict(shape=(6, None), writable=False)]) -> Manipulability:
    """
    Manipulability and force ellipsoids of a 6-row jacobian.

    When an ellipsoid's smallest eigenvalue is ~0 (a singular jacobian), its measure and condition
    are the largest double (sys.float_info.max), as upstream returns them (utils.cpp:240-244).
    """

def harmonizeTowardZero(qs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], redundancy_capable_joints: Sequence[int]) -> None:
    """
    Wrap qs[i] into [-pi, pi) in place for each i in redundancy_capable_joints.

    Raises:
        IndexError: an index is outside qs; qs is left unchanged.
    """

def harmonizeTowardMedian(qs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], redundancy_capable_joints: Sequence[int], position_limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)]) -> None:
    """
    Wrap qs[i] into [median - pi, median + pi) in place, median the midpoint of row i of position_limits.

    Raises:
        IndexError: an index is outside qs or position_limits; qs is left unchanged.
    """

def isValid(qs: Sequence[float]) -> bool:
    """True when all six values are finite."""

def checkKinematics(manip: KinematicGroup, tol: float = 0.001) -> bool:
    """
    Round-trip self-check: FK then IK for every working frame and tip link of manip.

    False when an IK solution's translation or angle distance from the FK pose exceeds tol
    (validate.cpp:38-175). True also when IK found no solution at all: only failed solutions
    make it False.
    """
