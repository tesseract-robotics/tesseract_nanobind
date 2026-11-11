"""Every IK solution of one tool0 pose on the ABB IRB2400, drawn side by side.

`Robot.ik(..., all_solutions=True)` asks the group's IK solver (OPW, a closed form, on the
IRB2400) for every solution within the joint limits. The pose below has four: the arm reaches
it facing the target or turned half a revolution and leaning back over itself, each with the
wrist flipped or not. `compute` checks that forward kinematics of each solution lands on the
pose.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from numpy.typing import NDArray

from scripts.docs_figures import lock, scene3d
from scripts.docs_figures.style import COLUMN_WIDTH_IN, Theme
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Pose, Robot

GROUP: Final[str] = "manipulator"
TIP: Final[str] = "tool0"
# m and rad: a pose in front of and above the robot, tool0's z axis pointing along +x
# (pitch 90 deg); high enough that the leaning-back branches stay inside joint_2's limits.
TARGET_XYZ_M: Final[tuple[float, float, float]] = (0.7, 0.2, 1.4)
TARGET_RPY_RAD: Final[tuple[float, float, float]] = (0.0, np.pi / 2, 0.0)
# m: FK of a solution must land on the target; OPW is exact to ~1e-12 m, 1 um flags a wrong
# solution and nothing else.
FK_TOL_M: Final[float] = 1e-6
VIEW: Final[scene3d.View] = scene3d.View(azimuth_deg=-20.0, elevation_deg=12.0)
TARGET_AXIS_M: Final[float] = 0.22


class IKSolutionOffTargetError(AssertionError):
    """Forward kinematics of an IK solution does not reach the target."""


@dataclass(frozen=True)
class Branch:
    """One IK solution and the robot drawn in it."""

    joints_rad: NDArray[np.float64]
    triangles: NDArray[np.float64]


@dataclass(frozen=True)
class Branches:
    """All solutions and the target pose."""

    target: NDArray[np.float64]
    branches: list[Branch]


def describe(joints: NDArray[np.float64]) -> str:
    """Name the branch from its joint values: base facing or turned, wrist flip sign."""
    facing = "facing" if abs(joints[0]) < np.pi / 2 else "turned"
    wrist = "wrist +" if joints[4] >= 0.0 else "wrist −"
    return f"{facing}, {wrist}"


def compute() -> Branches:
    """Solve the IK, check each solution with FK, and pose the robot in each."""
    robot = Robot.from_tesseract_support(
        "abb_irb2400", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    names = robot.get_joint_names(GROUP)
    links = list(robot.get_link_names())
    target = Pose.from_xyz_rpy(TARGET_XYZ_M, TARGET_RPY_RAD)
    solutions = robot.ik(GROUP, target, TIP, all_solutions=True)
    branches = []
    for joints in sorted(solutions, key=lambda q: (abs(q[0]) >= np.pi / 2, q[4] < 0)):
        reached = robot.fk(GROUP, joints, TIP).matrix
        error = float(np.linalg.norm(reached[:3, 3] - target.matrix[:3, 3]))
        if error > FK_TOL_M:
            raise IKSolutionOffTargetError(f"{joints}: FK lands {error:.3g} m from the target")
        robot.set_joints(joints, joint_names=names)
        triangles = scene3d.link_triangles(robot.env, scene3d.link_poses(robot.env), links)
        branches.append(Branch(np.asarray(joints), triangles))
    return Branches(target=target.matrix, branches=branches)


def draw(data: Branches, theme: Theme) -> Figure:
    """One panel per solution, same view and scale."""
    count = len(data.branches)
    fig, axes = plt.subplots(1, count, figsize=(COLUMN_WIDTH_IN, 3.1))
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.80, wspace=0.04)
    tips = data.target[:3, 3] + TARGET_AXIS_M * data.target[:3, :3].T
    extent = np.concatenate([*(b.triangles.reshape(-1, 3) for b in data.branches), tips])
    for ax, branch in zip(np.atleast_1d(axes), data.branches, strict=True):
        scene3d.draw_mesh(ax, VIEW, branch.triangles, theme.surface)
        scene3d.draw_frame(ax, VIEW, data.target, TARGET_AXIS_M, label_color=theme.ink, zorder=6)
        scene3d.frame_extent(ax, VIEW, extent, margin_m=0.03)
        q = branch.joints_rad
        rows = "\n".join(
            "  ".join(f"q{i + 1} {q[i]:+.2f}" for i in range(start, start + 3)) for start in (0, 3)
        )
        ax.set_title(
            describe(q), fontsize=8.5, loc="center", fontweight="bold", color=theme.ink, pad=26
        )
        ax.text(
            0.5,
            1.0,
            rows,
            transform=ax.transAxes,
            ha="center",
            va="bottom",
            fontsize=7.0,
            color=theme.muted,
            linespacing=1.3,
        )
    return fig


def inputs() -> list[Path]:
    """The IRB2400 model, its kinematics plugin config (OPW parameters) and meshes."""
    return lock.support_files(
        [
            "support/urdf/abb_irb2400.urdf",
            "support/urdf/abb_irb2400.srdf",
            "support/urdf/abb_irb2400_plugins.yaml",
            "support/meshes/abb_irb2400/**/*",
        ]
    )
