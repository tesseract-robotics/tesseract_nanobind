"""The TCP path a trajectory commands, as line segments, by move type."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from tesseract_robotics.command_language import MoveInstructionType
from tesseract_robotics.common import Isometry3d, LinkId, ManipulatorInfo
from tesseract_robotics.environment import Environment
from tesseract_robotics.kinematics import JointGroup
from tesseract_robotics.scene_graph import JointType

from .errors import (
    MovingWorkingFrameError,
    NamedTcpOffsetError,
    UnknownJointError,
    UnsupportedMoveTypeError,
)
from .poses import isometry_to_pose
from .trajectory import Trajectory

# Largest revolute motion one drawn MoveJ piece covers [rad]. A piece leaves the arc by
# R * step^2 / 8: 0.06 mm for one joint at R = 1.5 m; measured with all joints moving,
# 0.102 mm worst of 300 random full-range moves (IRB2400, rail gantry).
MAX_JOINT_STEP_RAD = float(np.radians(1.0))
# Largest prismatic motion per piece while a revolute joint also moves [m]. A rail below a
# rotating joint leaves the piece by step_rad * step_m / 4: 0.2 mm at these two steps.
MAX_PRISMATIC_STEP_M = 0.05

_REVOLUTE_TYPES = {JointType.REVOLUTE, JointType.CONTINUOUS}


@dataclass(frozen=True)
class TcpPath:
    """A trajectory's TCP in the world frame.

    Attributes:
        positions: `(N, 3)` TCP position at each waypoint [m].
        wxyzs: `(N, 4)` TCP orientation at each waypoint, scalar first (viser's order).
        segments: `(M, 2, 3)` start and end of each drawn piece [m].
    """

    positions: NDArray[np.float64]
    wxyzs: NDArray[np.float64]
    segments: NDArray[np.float64]


def sample_count(delta: NDArray[np.float64], revolute: NDArray[np.bool_]) -> int:
    """Pieces that draw a MoveJ of joint motion `delta` within the step constants.

    Prismatic motion alone is exactly straight, so it counts only beside a rotation.

    Args:
        delta: `(J,)` joint motion [rad or m].
        revolute: `(J,)` whether each joint rotates.

    Returns:
        Number of pieces, at least 1.
    """
    rotation = float(np.abs(delta[revolute]).max(initial=0.0))
    if rotation == 0.0:
        return 1
    translation = float(np.abs(delta[~revolute]).max(initial=0.0))
    return max(
        1,
        int(np.ceil(rotation / MAX_JOINT_STEP_RAD)),
        int(np.ceil(translation / MAX_PRISMATIC_STEP_M)),
    )


def _tcp_offset(manip_info: ManipulatorInfo) -> Isometry3d:
    offset = manip_info.tcp_offset
    if isinstance(offset, LinkId):
        raise NamedTcpOffsetError(f"tcp_offset names link {offset.name()!r}; give a transform")
    return offset


def _check_move_types(trajectory: Trajectory, kin: JointGroup, manip_info: ManipulatorInfo) -> None:
    for index, move_type in enumerate(trajectory.move_types[1:], start=1):
        if move_type == MoveInstructionType.LINEAR:
            if kin.isActiveLinkId(manip_info.working_frame):
                raise MovingWorkingFrameError(
                    f"move {index} is linear in {manip_info.working_frame.name()!r}, which group "
                    f"{manip_info.manipulator!r} moves: not a straight line in world"
                )
        elif move_type != MoveInstructionType.FREESPACE:
            raise UnsupportedMoveTypeError(f"move {index} is {move_type}: no exact TCP path")


def tcp_path(env: Environment, trajectory: Trajectory, manip_info: ManipulatorInfo) -> TcpPath:
    """Compute the TCP path `trajectory` commands.

    A move's own type shapes the piece arriving at its waypoint. `LINEAR`: the straight
    line between the two TCP positions. `FREESPACE`: FK of the synchronized joint
    interpolation between the two waypoints, in pieces within the step constants. This is
    the commanded path: a controller's blending is not in the program.

    Args:
        env: Environment the group's kinematics and joint types come from.
        trajectory: Waypoints and move types.
        manip_info: Group (`manipulator`), TCP (`tcp_frame`, `tcp_offset`), `working_frame`.

    Returns:
        The path.

    Raises:
        UnknownJointError: the group has a joint the trajectory lacks.
        NamedTcpOffsetError: `tcp_offset` is a link name.
        UnsupportedMoveTypeError: a move is neither `LINEAR` nor `FREESPACE`.
        MovingWorkingFrameError: a `LINEAR` move's working frame moves with the group.
    """
    # FK only: a kinematic group would demand an IK solver the path has no use for.
    kin = env.getJointGroup(manip_info.manipulator)
    group_joints = [jid.name() for jid in kin.getJointIds()]
    missing = set(group_joints) - set(trajectory.joint_names)
    if missing:
        raise UnknownJointError(f"trajectory lacks group joints: {sorted(missing)}")
    offset = _tcp_offset(manip_info)
    _check_move_types(trajectory, kin, manip_info)

    tcp_frame = manip_info.tcp_frame.name()

    def tcp(q: NDArray[np.float64]) -> Isometry3d:
        return kin.calcFwdKin(q)[tcp_frame] * offset

    columns = [trajectory.joint_names.index(name) for name in group_joints]
    waypoints = trajectory.positions[:, columns]
    revolute = np.array([env.getJoint(name).type in _REVOLUTE_TYPES for name in group_joints])
    poses = [isometry_to_pose(tcp(q)) for q in waypoints]
    positions = np.stack([p for p, _ in poses])

    pieces = [positions[:1]]
    for index, move_type in enumerate(trajectory.move_types[1:], start=1):
        start, end = waypoints[index - 1], waypoints[index]
        if move_type == MoveInstructionType.FREESPACE:
            count = sample_count(end - start, revolute)
            inner = [tcp(start + (i / count) * (end - start)).translation for i in range(1, count)]
            if inner:
                pieces.append(np.stack(inner))
        pieces.append(positions[index : index + 1])
    points = np.concatenate(pieces)
    return TcpPath(
        positions,
        np.stack([w for _, w in poses]),
        np.stack([points[:-1], points[1:]], axis=1),
    )
