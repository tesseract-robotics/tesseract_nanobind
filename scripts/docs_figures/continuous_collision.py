"""What each collision evaluator sees of one swept motion past a thin wall.

A 0.1 m cube on an x-y gantry (two prismatic joints) moves through a 3 cm wall in a single
trajectory step. `checkTrajectory` runs on the same three-state trajectory with each
`CollisionEvaluatorType`: DISCRETE checks the states only and misses the wall; LVS_DISCRETE
checks interpolated states and misses it too when they are coarser than the wall; LVS_CONTINUOUS
casts the cube between those states and catches it at any resolution.

The checked sub-states follow upstream's interpolation (`environment/src/utils.cpp`:
`ceil(|q1 - q0| / lvs) + 1` evenly spaced states per step); `compute` asserts that every sub-step
the library reports in contact is one of them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle
from numpy.typing import NDArray
from scipy.spatial import ConvexHull

from scripts.docs_figures import managers
from scripts.docs_figures.style import COLUMN_WIDTH_IN, GREEN, RED, Theme
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    CollisionCheckConfig,
    CollisionCheckExitType,
    CollisionCheckProgramType,
    CollisionEvaluatorType,
    ContactRequest,
    ContactTestType,
    ContactTrajectoryResults,
    ContactTrajectoryStepResults,
)
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.environment import Environment, checkTrajectory

CUBE_EDGE_M: Final[float] = 0.10
# m: wall thickness (x), length (y) and height (z), centred on the origin.
WALL_SIZE_M: Final[tuple[float, float, float]] = (0.03, 0.50, 0.20)
JOINTS: Final[tuple[str, str]] = ("gantry_x", "gantry_y")
# Joint states (x, y), m: through the wall in one step, then along it.
TRAJECTORY_M: Final[NDArray[np.float64]] = np.array([[-0.32, -0.16], [0.32, 0.08], [0.32, 0.30]])
# m: interpolation lengths. COARSE is wider than wall + cube (0.13 m), so a sub-state can
# land on each side of the wall; FINE is under the wall thickness, so one lands in it.
LVS_COARSE_M: Final[float] = 0.25
LVS_FINE_M: Final[float] = 0.025
# m: contact distance threshold of the request (only touching or penetrating counts).
CONTACT_MARGIN_M: Final[float] = 0.0

URDF: Final[
    str
] = f"""<robot name="gantry" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">
  <link name="world"/>
  <link name="carriage"/>
  <link name="cube">
    <visual><geometry><box size="{CUBE_EDGE_M} {CUBE_EDGE_M} {CUBE_EDGE_M}"/></geometry></visual>
    <collision><geometry><box size="{CUBE_EDGE_M} {CUBE_EDGE_M} {CUBE_EDGE_M}"/></geometry></collision>
  </link>
  <link name="wall">
    <visual><geometry><box size="{" ".join(map(str, WALL_SIZE_M))}"/></geometry></visual>
    <collision><geometry><box size="{" ".join(map(str, WALL_SIZE_M))}"/></geometry></collision>
  </link>
  <joint name="{JOINTS[0]}" type="prismatic">
    <parent link="world"/><child link="carriage"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="0" velocity="1"/>
  </joint>
  <joint name="{JOINTS[1]}" type="prismatic">
    <parent link="carriage"/><child link="cube"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="0" velocity="1"/>
  </joint>
  <joint name="wall_joint" type="fixed"><parent link="world"/><child link="wall"/></joint>
</robot>
"""

# The SRDF only names the contact manager plugins (Bullet, upstream's support config).
SRDF: Final[str] = f"""<robot name="gantry">
  <contact_managers_plugin_config filename="package://tesseract/{managers.PLUGINS}"/>
</robot>
"""


class SubstateMismatchError(AssertionError):
    """A sub-step the library reported is not one of the interpolated sub-states."""


@dataclass(frozen=True)
class EvaluatorRun:
    """One evaluator's checked states and which of them (or which sweeps) hit the wall."""

    title: str
    continuous: bool
    lvs_m: float | None
    # Per checked state (discrete) or per swept sub-segment (continuous): (start, end) xy, m.
    checked: list[tuple[NDArray[np.float64], NDArray[np.float64]]]
    in_contact: list[bool]
    contacts: int


def substates(trajectory: NDArray[np.float64], lvs_m: float) -> list[NDArray[np.float64]]:
    """Upstream's LVS sub-states of each step (utils.cpp), without repeating step ends."""
    states = [trajectory[0]]
    for start, end in zip(trajectory[:-1], trajectory[1:], strict=True):
        count = math.ceil(np.linalg.norm(end - start) / lvs_m) + 1
        states.extend(np.linspace(start, end, count)[1:])
    return states


def _environment() -> Environment:
    env = Environment()
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    if not env.init(URDF, SRDF, locator):
        raise RuntimeError("the gantry environment failed to initialise")
    return env


def _run(
    env: Environment, kind: CollisionEvaluatorType, lvs_m: float | None
) -> tuple[ContactTrajectoryResults, list[ContactTrajectoryStepResults]]:
    config = CollisionCheckConfig()
    config.type = kind
    config.contact_request = ContactRequest(ContactTestType.ALL)
    config.check_program_mode = CollisionCheckProgramType.ALL
    config.exit_condition = CollisionCheckExitType.ALL
    if lvs_m is not None:
        config.longest_valid_segment_length = lvs_m
    continuous = kind in (CollisionEvaluatorType.LVS_CONTINUOUS, CollisionEvaluatorType.CONTINUOUS)
    manager = env.getContinuousContactManager() if continuous else env.getDiscreteContactManager()
    manager.setDefaultCollisionMargin(CONTACT_MARGIN_M)
    results, _ = checkTrajectory(manager, env.getStateSolver(), list(JOINTS), TRAJECTORY_M, config)
    return results, [results.steps[i] for i in range(results.numSteps())]


def _contact_substates(steps: list) -> list[NDArray[np.float64]]:
    """Start states of every sub-step the library reports in contact."""
    found = []
    for step in steps:
        for substep in step.substeps:
            if substep.numContacts() > 0:
                found.append(np.asarray(substep.state0))
    return found


def compute() -> list[EvaluatorRun]:
    """Run the four evaluators on the trajectory."""
    env = _environment()
    runs = []
    cases = [
        ("DISCRETE", CollisionEvaluatorType.DISCRETE, None),
        (
            f"LVS_DISCRETE, lvs {LVS_COARSE_M * 100:.0f} cm",
            CollisionEvaluatorType.LVS_DISCRETE,
            LVS_COARSE_M,
        ),
        (
            f"LVS_DISCRETE, lvs {LVS_FINE_M * 100:.1f} cm",
            CollisionEvaluatorType.LVS_DISCRETE,
            LVS_FINE_M,
        ),
        (
            f"LVS_CONTINUOUS, lvs {LVS_COARSE_M * 100:.0f} cm",
            CollisionEvaluatorType.LVS_CONTINUOUS,
            LVS_COARSE_M,
        ),
    ]
    for title, kind, lvs in cases:
        results, steps = _run(env, kind, lvs)
        hit = _contact_substates(steps)
        continuous = kind == CollisionEvaluatorType.LVS_CONTINUOUS
        states = (
            [np.asarray(q) for q in TRAJECTORY_M] if lvs is None else substates(TRAJECTORY_M, lvs)
        )
        if continuous:
            checked = list(zip(states[:-1], states[1:], strict=True))
        else:
            checked = [(s, s) for s in states]
        in_contact = [any(np.allclose(start, h) for h in hit) for start, _ in checked]
        if sum(in_contact) != len(hit):
            raise SubstateMismatchError(
                f"{title}: library reports contact at {hit}, not among the sub-states {states}"
            )
        runs.append(
            EvaluatorRun(title, continuous, lvs, checked, in_contact, results.numContacts())
        )
    return runs


def _square(center: NDArray[np.float64]) -> NDArray[np.float64]:
    h = CUBE_EDGE_M / 2
    return center + np.array([[-h, -h], [h, -h], [h, h], [-h, h]])


def _panel(ax: Axes, run: EvaluatorRun, theme: Theme) -> None:
    wx, wy, _ = WALL_SIZE_M
    ax.add_patch(Rectangle((-wx / 2, -wy / 2), wx, wy, fc=theme.surface, ec="none", zorder=1))
    ax.plot(*TRAJECTORY_M.T, color=theme.muted, lw=0.9, ls=(0, (3, 2)), zorder=2)
    ax.plot(*TRAJECTORY_M.T, "o", color=theme.ink, ms=3.2, zorder=6)
    for (start, end), hit in zip(run.checked, run.in_contact, strict=True):
        color = RED if hit else GREEN
        if run.continuous:
            hull_points = np.vstack([_square(start), _square(end)])
            hull = hull_points[ConvexHull(hull_points).vertices]
            ax.add_patch(
                Polygon(hull, fc=color, alpha=0.30 if hit else 0.16, ec=color, lw=0.8, zorder=3)
            )
        else:
            ax.add_patch(
                Polygon(
                    _square(start),
                    fill=hit,
                    fc=color,
                    ec=color,
                    lw=1.0 if hit else 0.7,
                    alpha=0.55 if hit else 1.0,
                    zorder=4,
                )
            )
    verdict = "hits the wall" if run.contacts else "misses the wall"
    unit = "sweeps" if run.continuous else "states"
    ax.set_title(run.title, loc="left", fontsize=8.5, pad=15)
    ax.text(
        0.0,
        1.0,
        f"{len(run.checked)} {unit} checked: {verdict}",
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        fontsize=8.0,
        color=RED if run.contacts else theme.muted,
    )
    ax.set_aspect("equal")
    ax.set_xlim(-0.42, 0.42)
    ax.set_ylim(-0.27, 0.40)
    ax.set_xticks([-0.3, 0.0, 0.3])
    ax.set_yticks([-0.2, 0.0, 0.2])
    ax.grid(False)


def draw(runs: list[EvaluatorRun], theme: Theme) -> Figure:
    """One panel per evaluator, top view (x right, y up, m)."""
    fig, axes = plt.subplots(2, 2, figsize=(COLUMN_WIDTH_IN, 5.6), sharex=True, sharey=True)
    fig.subplots_adjust(left=0.09, right=0.99, bottom=0.08, top=0.93, wspace=0.10, hspace=0.30)
    for ax, run in zip(axes.flat, runs, strict=True):
        _panel(ax, run, theme)
    for ax in axes[:, 0]:
        ax.set_ylabel("y (m)")
    for ax in axes[1, :]:
        ax.set_xlabel("x (m)")
    return fig


def inputs() -> list[Path]:
    """The contact manager plugin config; the scene is the URDF string in this module."""
    return managers.inputs()
