"""Shared builders for the emitter tests: frames, IR events, composites, IRB 2400 kinematics."""

from __future__ import annotations

from collections.abc import Sequence
from functools import cache
from typing import Final

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
)
from tesseract_robotics.common import GeneralResourceLocator, JointState, ManipulatorInfo
from tesseract_robotics.emitters.core.events import CartesianMove, JointMove, MoveKind
from tesseract_robotics.emitters.core.frames import FramedPose, MoveFrames
from tesseract_robotics.emitters.core.posture import OpwParameters, OpwPostureSolver
from tesseract_robotics.planning import Pose, Robot

#: Joint names of the IRB 2400 manipulator group.
JOINT_NAMES: Final[list[str]] = [f"joint_{i}" for i in range(1, 7)]
#: Frames every test program is expressed in (floor robot, flange TCP).
WORKING_FRAME: Final[str] = "base_link"
TCP_FRAME: Final[str] = "tool0"
FRAMES: Final[MoveFrames] = MoveFrames(WORKING_FRAME, TCP_FRAME, Pose())

#: IRB 2400 OPW parameters, verbatim from tesseract_support
#: `urdf/abb_irb2400_plugins.yaml` (`OPWInvKin` config).
IRB2400_OPW: Final[OpwParameters] = OpwParameters.build(
    a1=0.100,
    a2=-0.135,
    b=0.00,
    c1=0.615,
    c2=0.705,
    c3=0.755,
    c4=0.085,
    offsets=[0, 0, -1.57079632679, 0, 0, 0],
    sign_corrections=[1, 1, 1, 1, 1, 1],
)
IRB2400_POSTURE: Final[OpwPostureSolver] = OpwPostureSolver(IRB2400_OPW)
#: A tool-down, elbow-up, wrist-positive configuration near the IRB 2400 home;
#: the reference for picking one IK branch deterministically.
IRB2400_REFERENCE_SEED: Final[tuple[float, ...]] = tuple(np.deg2rad([0, 20, 0, 0, 70, 0]))


def joint_move(joints_deg: Sequence[float], profile: str = "J") -> JointMove:
    return JointMove(joints=tuple(np.deg2rad(joints_deg)), profile=profile, frames=FRAMES)


def cart_move(
    pose: Pose,
    kind: MoveKind = MoveKind.LINEAR,
    profile: str = "L",
    seed_deg: Sequence[float] | None = None,
) -> CartesianMove:
    seed = None if seed_deg is None else tuple(np.deg2rad(seed_deg))
    return CartesianMove(
        target=FramedPose(pose, FRAMES), kind=kind, profile=profile, seed_joints=seed
    )


def cart_instruction(
    pose: Pose,
    move_type: MoveInstructionType = MoveInstructionType.LINEAR,
    profile: str = "L",
    seed_rad: Sequence[float] | None = None,
) -> MoveInstructionPoly:
    wp = CartesianWaypoint(pose)
    if seed_rad is not None:
        wp.setSeed(JointState(JOINT_NAMES, np.asarray(seed_rad, dtype=np.float64)))
    return MoveInstructionPoly(MoveInstruction(CartesianWaypointPoly(wp), move_type, profile))


def joint_instruction(
    joints_rad: Sequence[float],
    names: Sequence[str] = JOINT_NAMES,
    profile: str = "J",
    move_type: MoveInstructionType = MoveInstructionType.FREESPACE,
) -> MoveInstructionPoly:
    jw = JointWaypoint()
    jw.setJointIds(list(names))
    jw.setPosition(np.asarray(joints_rad, dtype=np.float64))
    return MoveInstructionPoly(MoveInstruction(JointWaypointPoly(jw), move_type, profile))


def composite(*instructions: object, description: str = "test") -> CompositeInstruction:
    """A composite whose ManipulatorInfo carries `FRAMES`, holding `instructions`."""
    c = CompositeInstruction()
    c.setDescription(description)
    c.setManipulatorInfo(ManipulatorInfo("manipulator", WORKING_FRAME, TCP_FRAME))
    for instr in instructions:
        c.push_back(instr)
    return c


@cache
def irb2400() -> Robot:
    """The ABB IRB 2400 from tesseract_support (loaded once per process)."""
    return Robot.from_tesseract_support(
        "abb_irb2400", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )


def irb2400_ik(pose: Pose, near: Sequence[float] = IRB2400_REFERENCE_SEED) -> tuple[float, ...]:
    """The IRB 2400 IK solution of `pose` (tool0 in base_link) closest to `near`."""
    solutions = irb2400().ik(
        "manipulator", pose, TCP_FRAME, seed=np.asarray(near), all_solutions=True
    )
    ref = np.asarray(near)
    best = min(solutions, key=lambda q: float(np.linalg.norm(np.asarray(q) - ref)))
    return tuple(float(v) for v in best)


def irb2400_fk(joints: Sequence[float], link: str) -> np.ndarray:
    """Pose matrix of `link` in base_link at `joints` (IRB 2400)."""
    env = irb2400().env
    state = env.getStateSolver().getState(JOINT_NAMES, np.asarray(joints, dtype=np.float64))
    return np.asarray(state.link_transforms[link].matrix)
