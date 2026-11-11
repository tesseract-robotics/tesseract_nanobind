"""Reads of every name-keyed map binding, written as user code: keyed by `str`, exactly typed.

`test_name_keyed_maps.py` runs these functions and type-checks this module with ty; each
annotation below is the stub's promise for that binding, so a return keyed by
`LinkId` / `JointId` fails both checks.
"""

import numpy as np
from numpy.typing import NDArray

from tesseract_robotics.collision import ContactManagerConfig
from tesseract_robotics.common import CalibrationInfo, Isometry3d
from tesseract_robotics.environment import (
    ChangeJointAccelerationLimitsCommand,
    ChangeJointPositionLimitsCommand,
    ChangeJointVelocityLimitsCommand,
    Environment,
)
from tesseract_robotics.scene_graph import SceneGraph
from tesseract_robotics.srdf import KinematicsInformation
from tesseract_robotics.state_solver import SceneState

TIP_LINK = "tool0"
FIRST_JOINT = "joint_a1"


def kinematic_group_fwd_kin(env: Environment, q: NDArray[np.float64]) -> dict[str, Isometry3d]:
    poses = env.getKinematicGroup("manipulator").calcFwdKin(q)
    tip: Isometry3d = poses[TIP_LINK]
    assert tip is not None
    return poses


def joint_group_fwd_kin(env: Environment, q: NDArray[np.float64]) -> dict[str, Isometry3d]:
    poses = env.getJointGroup("manipulator").calcFwdKin(q)
    tip: Isometry3d = poses[TIP_LINK]
    assert tip is not None
    return poses


def scene_state_maps(state: SceneState) -> list[dict[str, float] | dict[str, Isometry3d]]:
    joints: dict[str, float] = state.joints
    floating: dict[str, Isometry3d] = state.floating_joints
    links: dict[str, Isometry3d] = state.link_transforms
    joint_tfs: dict[str, Isometry3d] = state.joint_transforms
    value: float = joints[FIRST_JOINT]
    tip: Isometry3d = links[TIP_LINK]
    origin: Isometry3d = joint_tfs[FIRST_JOINT]
    assert value is not None and tip is not None and origin is not None
    return [joints, floating, links, joint_tfs]


def floating_joint_values(state: SceneState, joint: str) -> dict[str, Isometry3d]:
    return state.getFloatingJointValues([joint])


def environment_maps(env: Environment, q: NDArray[np.float64]) -> list[dict[str, Isometry3d]]:
    joint_ids = env.getJointGroup("manipulator").getJointIds()
    links: dict[str, Isometry3d] = env.getLinkTransforms(joint_ids, q)
    solver_links: dict[str, Isometry3d] = env.getStateSolver().getLinkTransforms(joint_ids, q)
    floating: dict[str, Isometry3d] = env.getCurrentFloatingJointValues()
    tip: Isometry3d = links[TIP_LINK]
    assert tip is not None
    return [links, solver_links, floating]


def command_limits(joint: str) -> list[dict[str, tuple[float, float]] | dict[str, float]]:
    position: dict[str, tuple[float, float]] = ChangeJointPositionLimitsCommand(
        {joint: (-1.0, 1.0)}
    ).getLimits()
    velocity: dict[str, float] = ChangeJointVelocityLimitsCommand({joint: 1.0}).getLimits()
    acceleration: dict[str, float] = ChangeJointAccelerationLimitsCommand({joint: 1.0}).getLimits()
    return [position, velocity, acceleration]


def contact_manager_config(link: str) -> dict[str, bool]:
    config = ContactManagerConfig()
    config.modify_object_enabled = {link: True}
    enabled: dict[str, bool] = config.modify_object_enabled
    return enabled


def kinematics_information(joint: str, link: str) -> list[dict[str, float] | dict[str, Isometry3d]]:
    info = KinematicsInformation()
    info.group_states = {"group": {"state": {joint: 0.5}}}
    info.group_tcps = {"group": {link: Isometry3d.Identity()}}
    state: dict[str, float] = info.group_states["group"]["state"]
    tcps: dict[str, Isometry3d] = info.group_tcps["group"]
    return [state, tcps]


def calibration_joints(joint: str) -> dict[str, Isometry3d]:
    info = CalibrationInfo()
    info.joints = {joint: Isometry3d.Identity()}
    joints: dict[str, Isometry3d] = info.joints
    return joints


def adjacency_map(scene_graph: SceneGraph, link: str) -> list[str]:
    adjacency = scene_graph.getAdjacencyMap([link])
    return list(adjacency)
