"""Contact distance, margin and penetration of a sphere against a box, from the contact manager.

A 0.2 m cube sits at the origin; a 0.1 m sphere moves along +x past its face. At every sphere
position the default discrete manager (Bullet) runs a CLOSEST contact test with a 0.1 m
margin. The left panel draws three of those states in the z = 0 section with the nearest
points the manager returned; the right panel plots every reported distance against the
analytic signed distance. Positions beyond the margin return no contact at all.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from numpy.typing import NDArray

from scripts.docs_figures import managers
from scripts.docs_figures.style import BLUE, COLUMN_WIDTH_IN, ORANGE, RED, Theme
from tesseract_robotics.geometry import Box, Sphere

# m: the cube's edge and the sphere's radius.
BOX_EDGE_M: Final[float] = 0.2
SPHERE_RADIUS_M: Final[float] = 0.1
# m: the collision margin, the docs' running example (`CollisionMarginData(0.1)`).
MARGIN_M: Final[float] = 0.1
# m: sphere centre positions swept along x: from 6 cm penetration to 4 cm beyond the margin,
# every 5 mm.
SWEEP_X_M: Final[NDArray[np.float64]] = np.round(np.arange(0.14, 0.3401, 0.005), 6)
# Sphere centres (x, y) drawn in the section, m: penetrating the +x face, within the margin
# above the +y face, beyond the margin off the corner.
STATES_XY_M: Final[tuple[tuple[float, float], ...]] = ((0.17, -0.06), (-0.06, 0.235), (0.34, 0.21))
# Label anchor of each drawn state, offset from its centre (pt) and alignment.
STATE_LABELS: Final[tuple[tuple[str, tuple[float, float], str], ...]] = (
    ("penetration", (34, -26), "left"),
    ("within margin", (0, 40), "center"),
    ("beyond margin:\nno contact", (0, 40), "center"),
)
# m: reported distances must match the analytic one to Bullet's sphere-box precision; the
# sweep's worst residual is ~1e-16, so 1 um only flags a wrong contact, never round-off.
ANALYTIC_TOL_M: Final[float] = 1e-6
M_TO_MM: Final[float] = 1e3

BOX: Final[str] = "box"
SPHERE: Final[str] = "sphere"


class ContactMismatchError(AssertionError):
    """The manager's distance disagrees with the analytic sphere-box distance."""


@dataclass(frozen=True)
class SectionState:
    """One drawn sphere position and what the manager reported for it."""

    center_m: NDArray[np.float64]
    distance_m: float | None
    nearest_points_m: tuple[NDArray[np.float64], NDArray[np.float64]] | None


@dataclass(frozen=True)
class DistanceData:
    """The sweep and the drawn states."""

    sweep_x_m: NDArray[np.float64]
    analytic_m: NDArray[np.float64]
    reported_m: NDArray[np.float64]  # NaN where the manager returned no contact
    states: tuple[SectionState, ...]
    manager_name: str


def analytic_distance(center: NDArray[np.float64]) -> float:
    """Signed distance between the sphere at `center` and the cube at the origin, m."""
    half = BOX_EDGE_M / 2
    q = np.abs(center) - half
    outside = float(np.linalg.norm(np.maximum(q, 0.0)))
    inside = float(min(np.max(q), 0.0))
    return outside + inside - SPHERE_RADIUS_M


def _query(manager, center: NDArray[np.float64]) -> SectionState:
    manager.setCollisionObjectsTransform(SPHERE, managers.translation(*center))
    contact = managers.closest(manager)
    if contact is None:
        return SectionState(center, None, None)
    names = [link.name() for link in contact.link_ids]
    sphere_point = contact.nearest_points[names.index(SPHERE)]
    box_point = contact.nearest_points[names.index(BOX)]
    return SectionState(center, contact.distance, (np.asarray(box_point), np.asarray(sphere_point)))


def compute() -> DistanceData:
    """Sweep the sphere and query the manager at every position."""
    manager = managers.discrete()
    manager.addCollisionObject(
        BOX, 0, [Box(BOX_EDGE_M, BOX_EDGE_M, BOX_EDGE_M)], [managers.translation(0)]
    )
    manager.addCollisionObject(SPHERE, 0, [Sphere(SPHERE_RADIUS_M)], [managers.translation(0)])
    manager.setActiveCollisionObjects([BOX, SPHERE])
    manager.setDefaultCollisionMargin(MARGIN_M)

    sweep = [_query(manager, np.array([x, 0.0, 0.0])) for x in SWEEP_X_M]
    states = tuple(_query(manager, np.array([x, y, 0.0])) for x, y in STATES_XY_M)
    analytic = np.array([analytic_distance(s.center_m) for s in sweep])
    reported = np.array([np.nan if s.distance_m is None else s.distance_m for s in sweep])
    for state in (*sweep, *states):
        expected = analytic_distance(state.center_m)
        if (state.distance_m is None) != (expected >= MARGIN_M):
            raise ContactMismatchError(
                f"at {state.center_m}: contact {state.distance_m}, analytic {expected:.4f} m, "
                f"margin {MARGIN_M} m"
            )
        if state.distance_m is not None and abs(state.distance_m - expected) > ANALYTIC_TOL_M:
            raise ContactMismatchError(
                f"at {state.center_m}: reported {state.distance_m:.6f} m, analytic {expected:.6f} m"
            )
    return DistanceData(SWEEP_X_M, analytic, reported, states, manager.getName())


def _regime_color(distance: float | None) -> str:
    if distance is None:
        return BLUE
    return RED if distance < 0.0 else ORANGE


def _section(ax: Axes, data: DistanceData, theme: Theme) -> None:
    half = BOX_EDGE_M / 2
    # The margin-inflated box: every point within MARGIN_M of the cube (a rounded square).
    ax.add_patch(
        FancyBboxPatch(
            (-half, -half),
            BOX_EDGE_M,
            BOX_EDGE_M,
            boxstyle=f"round,pad={MARGIN_M}",
            fill=False,
            ec=theme.muted,
            lw=0.9,
            ls=(0, (4, 3)),
        )
    )
    ax.add_patch(Rectangle((-half, -half), BOX_EDGE_M, BOX_EDGE_M, fc=theme.surface, ec="none"))
    ax.annotate(
        f"margin {MARGIN_M * M_TO_MM:.0f} mm",
        (-half, -half - MARGIN_M),
        xytext=(0, -4),
        textcoords="offset points",
        ha="center",
        va="top",
        color=theme.muted,
    )
    for state, (label, offset, ha) in zip(data.states, STATE_LABELS, strict=True):
        color = _regime_color(state.distance_m)
        cx, cy = state.center_m[:2]
        ax.add_patch(Circle((cx, cy), SPHERE_RADIUS_M, fc=color, alpha=0.18, ec=color, lw=1.4))
        ax.plot(cx, cy, "+", color=color, ms=5, mew=1.0)
        if state.nearest_points_m is not None and state.distance_m is not None:
            box_point, sphere_point = state.nearest_points_m
            ax.plot(
                [box_point[0], sphere_point[0]],
                [box_point[1], sphere_point[1]],
                color=theme.ink,
                lw=1.2,
                marker="o",
                ms=2.5,
            )
            text = f"{label}\nd = {state.distance_m * M_TO_MM:+.0f} mm"
        else:
            text = label
        ax.annotate(
            text,
            (cx, cy),
            xytext=offset,
            textcoords="offset points",
            ha=ha,
            va="center",
            color=color,
            fontsize=8,
            fontweight="bold",
        )
    ax.set_aspect("equal")
    ax.set_xlim(-0.30, 0.48)
    ax.set_ylim(-0.32, 0.46)
    ax.set_axis_off()
    ax.set_title("Section z = 0, nearest points")


def _sweep(ax: Axes, data: DistanceData, theme: Theme) -> None:
    gap_mm = data.analytic_m * M_TO_MM
    ax.axhspan(gap_mm.min() - 10, 0, color=RED, alpha=0.08, lw=0)
    ax.axhspan(0, MARGIN_M * M_TO_MM, color=ORANGE, alpha=0.08, lw=0)
    ax.axhline(MARGIN_M * M_TO_MM, color=theme.muted, lw=0.9, ls=(0, (4, 3)))
    ax.axhline(0, color=theme.muted, lw=0.8)
    ax.plot(gap_mm, gap_mm, color=theme.muted, lw=0.8, zorder=2)
    reported = ~np.isnan(data.reported_m)
    ax.plot(
        gap_mm[reported],
        data.reported_m[reported] * M_TO_MM,
        "o",
        ms=3.6,
        color=theme.ink,
        zorder=3,
        label="reported by the manager",
    )
    ax.plot(
        gap_mm[~reported],
        gap_mm[~reported],
        "o",
        ms=3.6,
        mfc="none",
        mec=theme.muted,
        zorder=3,
        label="no contact returned",
    )
    ax.text(20, -12, "penetration: d < 0", color=RED, va="top", fontsize=8.5)
    ax.text(
        gap_mm.min() + 2,
        MARGIN_M * M_TO_MM - 6,
        "within margin: 0 ≤ d < margin",
        color=ORANGE,
        va="top",
        fontsize=8.5,
    )
    ax.text(
        gap_mm.min() + 2,
        MARGIN_M * M_TO_MM + 6,
        "beyond margin: not reported",
        color=theme.muted,
        va="bottom",
        fontsize=8.5,
    )
    ax.set_xlabel("true signed distance (mm)")
    ax.set_ylabel("ContactResult.distance (mm)")
    ax.set_xlim(gap_mm.min() - 5, gap_mm.max() + 5)
    ax.set_ylim(gap_mm.min() - 10, gap_mm.max() + 10)
    ax.legend(loc="lower right")
    ax.set_title("Sphere swept along +x")
    ax.title.set_position((0.0, 1.0))


def draw(data: DistanceData, theme: Theme) -> Figure:
    """Section and sweep side by side."""
    fig, (left, right) = plt.subplots(
        1, 2, figsize=(COLUMN_WIDTH_IN, 3.4), gridspec_kw={"width_ratios": [1.0, 1.15]}
    )
    fig.subplots_adjust(left=0.01, right=0.98, bottom=0.14, top=0.91, wspace=0.18)
    _section(left, data, theme)
    _sweep(right, data, theme)
    return fig


def inputs() -> list[Path]:
    """The contact manager plugin config."""
    return managers.inputs()
