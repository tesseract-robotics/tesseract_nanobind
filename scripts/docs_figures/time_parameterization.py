"""Time-optimal parameterisation (TOTG) of a joint path on the ABB IRB2400, with its limits.

The path runs through four joint configurations, each segment sampled at evenly spaced
waypoints. `TimeOptimalTrajectoryGeneration` times it against the URDF velocity and
acceleration limits (scaling 1.0, upstream's default path tolerance). Velocity and acceleration
are plotted per joint as a fraction of that joint's limit. On this path no joint reaches its
velocity limit; outside the blends at the two interior configurations, one joint is always on its
acceleration limit, which is what time-optimal means here.

Acceleration is drawn as the change of TOTG's waypoint velocities over each interval. The
waypoints' own acceleration field agrees with it on decelerating phases but reads exactly 0 on
accelerating ones: upstream TOTG lands a forward-integration step exactly on every waypoint
between collinear segments and then takes a backward difference over a zero time step
(tesseract_planning `totg/src/time_optimal_trajectory_generation.cpp`, `Trajectory::getAcceleration`).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from numpy.typing import NDArray

from scripts.docs_figures import lock
from scripts.docs_figures.style import CATEGORICAL, COLUMN_WIDTH_IN, Theme
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.planning import Robot
from tesseract_robotics.time_parameterization import (
    InstructionsTrajectory,
    TimeOptimalTrajectoryGeneration,
    TimeOptimalTrajectoryGenerationCompositeProfile,
)

GROUP: Final[str] = "manipulator"
# rad: the four configurations the path passes through (within the IRB2400's joint limits).
KEY_CONFIGS_RAD: Final[NDArray[np.float64]] = np.array(
    [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.8, 0.4, -0.2, 0.5, 0.6, 0.0],
        [1.2, 0.1, 0.3, -0.4, 0.2, 1.0],
        [0.4, -0.3, 0.1, 0.0, -0.5, 0.5],
    ]
)
# Waypoints per segment: dense enough that the plotted samples trace the profile.
WAYPOINTS_PER_SEGMENT: Final[int] = 40
# Limits may be met, never exceeded: the fraction of a limit that counts as a violation is
# 1 + 1e-6, far above TOTG's round-off and far below anything the plot can show.
LIMIT_SLACK: Final[float] = 1e-6
# rad: vertical spacing of the joint labels at the end of the position panel, one line of
# 8 pt type on its 1.7 in tall axes.
LABEL_GAP_RAD: Final[float] = 0.17


class LimitExceededError(AssertionError):
    """The parameterised trajectory exceeds a velocity or acceleration limit."""


@dataclass(frozen=True)
class TimedPath:
    """TOTG's output at every waypoint, and the limits it was timed against."""

    joint_names: list[str]
    time_s: NDArray[np.float64]
    position_rad: NDArray[np.float64]
    velocity: NDArray[np.float64]  # rad/s
    # rad/s^2, per interval between waypoints: the change of TOTG's waypoint velocities over
    # the interval's duration. Not the waypoints' own acceleration field, which upstream TOTG
    # leaves at 0 on accelerating phases (see the module docstring).
    acceleration: NDArray[np.float64]
    velocity_limit: NDArray[np.float64]  # rad/s, symmetric
    acceleration_limit: NDArray[np.float64]  # rad/s^2, symmetric
    key_times_s: NDArray[np.float64]


def path_waypoints() -> NDArray[np.float64]:
    """The joint path: each segment between key configurations, evenly sampled."""
    segments = [
        np.linspace(a, b, WAYPOINTS_PER_SEGMENT, endpoint=False)
        for a, b in zip(KEY_CONFIGS_RAD[:-1], KEY_CONFIGS_RAD[1:], strict=True)
    ]
    return np.vstack([*segments, KEY_CONFIGS_RAD[-1:]])


def compute() -> TimedPath:
    """Time the path with TOTG and read the result back."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400", locator)
    names = robot.get_joint_names(GROUP)
    info = ManipulatorInfo()
    info.manipulator = GROUP
    info.working_frame = "base_link"
    info.tcp_frame = "tool0"
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(info)
    for q in path_waypoints():
        waypoint = StateWaypointPoly(StateWaypoint(names, q))
        program.push_back(
            MoveInstructionPoly(MoveInstruction(waypoint, MoveInstructionType.FREESPACE, "DEFAULT"))
        )
    profiles = ProfileDictionary()
    profiles.addProfile("TOTG", "DEFAULT", TimeOptimalTrajectoryGenerationCompositeProfile())
    if not TimeOptimalTrajectoryGeneration().compute(program, robot.env, profiles):
        raise RuntimeError("TOTG failed on the figure's path")

    trajectory = InstructionsTrajectory(program)
    n = trajectory.size()
    limits = robot.env.getKinematicGroup(GROUP).getLimits()
    time_s = np.array([trajectory.getTimeFromStart(i) for i in range(n)])
    velocity = np.array([trajectory.getVelocity(i) for i in range(n)])
    path = TimedPath(
        joint_names=names,
        time_s=time_s,
        position_rad=np.array([trajectory.getPosition(i) for i in range(n)]),
        velocity=velocity,
        acceleration=np.diff(velocity, axis=0) / np.diff(time_s)[:, None],
        velocity_limit=np.asarray(limits.velocity_limits)[:, 1],
        acceleration_limit=np.asarray(limits.acceleration_limits)[:, 1],
        key_times_s=np.array(
            [
                trajectory.getTimeFromStart(i * WAYPOINTS_PER_SEGMENT)
                for i in range(len(KEY_CONFIGS_RAD))
            ]
        ),
    )
    for name, values, limit in (
        ("velocity", path.velocity, path.velocity_limit),
        ("acceleration", path.acceleration, path.acceleration_limit),
    ):
        worst = float(np.max(np.abs(values) / limit))
        if worst > 1.0 + LIMIT_SLACK:
            raise LimitExceededError(f"{name} reaches {worst:.6f} of its limit")
    return path


def draw(path: TimedPath, theme: Theme) -> Figure:
    """Position, and velocity and acceleration as fractions of their limits, against time."""
    fig, axes = plt.subplots(3, 1, figsize=(COLUMN_WIDTH_IN, 5.4), sharex=True)
    fig.subplots_adjust(left=0.11, right=0.88, bottom=0.09, top=0.97, hspace=0.18)
    rows = (
        (path.position_rad, None, "position (rad)"),
        (path.velocity / path.velocity_limit, 1.0, "velocity\n(fraction of limit)"),
        (
            path.acceleration / path.acceleration_limit,
            1.0,
            "acceleration, Δv/Δt\n(fraction of limit)",
        ),
    )
    for ax, (values, bound, label) in zip(axes, rows, strict=True):
        for t in path.key_times_s[1:-1]:
            ax.axvline(t, color=theme.grid, lw=0.9, ls=(0, (2, 2)))
        style = "default"
        if len(values) == len(path.time_s) - 1:
            # Per-interval values: repeat the last so the step reaches the end time.
            values, style = np.vstack([values, values[-1:]]), "steps-post"
        times = path.time_s
        for j, name in enumerate(path.joint_names):
            ax.plot(times, values[:, j], color=CATEGORICAL[j], lw=1.3, label=name, drawstyle=style)
        if bound is not None:
            for sign in (-1, 1):
                ax.axhline(sign * bound, color=theme.ink, lw=0.9)
            ax.set_ylim(-1.18, 1.18)
            ax.set_yticks([-1, -0.5, 0, 0.5, 1])
            ax.text(
                path.time_s[-1], 1.03, "limit", ha="right", va="bottom", color=theme.ink, fontsize=8
            )
        ax.set_ylabel(label)
        ax.grid(axis="x", visible=False)
    label_y = spread(path.position_rad[-1], LABEL_GAP_RAD)
    for j, name in enumerate(path.joint_names):
        axes[0].annotate(
            name,
            (path.time_s[-1], label_y[j]),
            xytext=(6, 0),
            textcoords="offset points",
            va="center",
            color=CATEGORICAL[j],
            fontsize=8,
            annotation_clip=False,
        )
    axes[-1].set_xlabel("time (s)")
    axes[-1].set_xlim(path.time_s[0], path.time_s[-1])
    return fig


def spread(values: NDArray[np.float64], gap: float) -> NDArray[np.float64]:
    """Label positions near `values`, in the same order, at least `gap` apart.

    Sorts, pushes each label up to `gap` above the one below it, then shifts the stack so
    its mean matches the mean of `values`.
    """
    order = np.argsort(values, kind="stable")
    placed = np.array(values, dtype=np.float64)[order]
    for i in range(1, len(placed)):
        placed[i] = max(placed[i], placed[i - 1] + gap)
    placed += np.mean(values) - np.mean(placed)
    result = np.empty_like(placed)
    result[order] = placed
    return result


def inputs() -> list[Path]:
    """The IRB2400 model files (its URDF holds the limits)."""
    return lock.support_files(["support/urdf/abb_irb2400.urdf", "support/urdf/abb_irb2400.srdf"])
