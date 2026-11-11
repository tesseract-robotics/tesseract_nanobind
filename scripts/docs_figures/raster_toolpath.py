"""A zigzag raster over a plate on the ABB IRB2400, planned with TrajOpt, drawn from FK.

Five strokes along y, tool down, joined by short linear step-overs, with a freespace approach
from home. `CartesianPipeline` (Descartes, then TrajOpt) plans it with the library's
`create_cartesian_pipeline_profiles`; the drawn
TCP path is the forward kinematics of every planned joint state, and the glyphs are the TCP
z axis (blue, the tool direction) and x axis (red) at each of them. `compute` asserts the
process strokes stay on the plate's plane.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from numpy.typing import NDArray

from scripts.docs_figures import lock, scene3d
from scripts.docs_figures.style import (
    AXIS_X,
    AXIS_Z,
    BLUE,
    COLUMN_WIDTH_IN,
    ORANGE,
    REPO_ROOT,
    Theme,
)
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator, LinkId
from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    box,
)
from tesseract_robotics.planning.geometry import create_obstacle
from tesseract_robotics.planning.profiles import create_cartesian_pipeline_profiles
from tesseract_robotics.planning.toolpaths import TOOL_DOWN_QUAT_XYZW

GROUP: Final[str] = "manipulator"
TCP_FRAME: Final[str] = "tool0"
# calcFwdKin keys its result by LinkId.
TCP_LINK: Final[LinkId] = LinkId(TCP_FRAME)
PIPELINE: Final[str] = "CartesianPipeline"
# One Descartes thread: the ladder-graph search is then a single deterministic pass.
DESCARTES_THREADS: Final[int] = 1
# m: the raster: strokes along y over [-0.25, 0.25], at five x between 0.70 and 0.90, on a
# plane 0.35 m above base_link; a waypoint every 5 cm along a stroke.
STROKE_X_M: Final[NDArray[np.float64]] = np.linspace(0.70, 0.90, 5)
STROKE_Y_M: Final[NDArray[np.float64]] = np.round(np.arange(-0.25, 0.2501, 0.05), 6)
PLANE_Z_M: Final[float] = 0.35
# m: the plate under the raster, 1 cm below the tool plane so the tool never touches it.
PLATE_SIZE_M: Final[tuple[float, float, float]] = (0.36, 0.64, 0.02)
PLATE_GAP_M: Final[float] = 0.01
# m: TCP height deviation allowed on a process stroke: TrajOpt's Cartesian constraints hold to
# its convergence tolerance (1e-4 by default), 1 mm only catches a stroke that left the plane.
PLANE_TOL_M: Final[float] = 1e-3
# m: glyph lengths of the TCP z and x axes.
Z_GLYPH_M: Final[float] = 0.035
X_GLYPH_M: Final[float] = 0.018
VIEW: Final[scene3d.View] = scene3d.View(azimuth_deg=-55.0, elevation_deg=28.0)
# Samples of the joint-interpolated approach drawn as the TCP's curve.
APPROACH_SAMPLES: Final[int] = 60
# The joint state the robot is drawn in: this fraction along the trajectory (mid-raster).
ROBOT_DRAWN_AT: Final[float] = 0.55

# Move kinds. The profile names are the library's standard ones (`STANDARD_PROFILE_NAMES`):
# strokes and step-overs plan with "RASTER", the approach with "FREESPACE".
PROCESS: Final[str] = "RASTER"
TRANSITION: Final[str] = "TRANSITION"
FREESPACE: Final[str] = "FREESPACE"


class OffPlaneError(AssertionError):
    """A process stroke's TCP left the raster plane."""


@dataclass(frozen=True)
class RasterPlan:
    """The planned trajectory's TCP poses, which move each belongs to, and the scene."""

    tcp: NDArray[np.float64]  # (n, 4, 4), base_link
    kind: list[str]  # PROCESS / TRANSITION / FREESPACE per trajectory point
    approach: NDArray[np.float64]  # (k, 3) TCP along the approach, joint-interpolated
    robot_triangles: NDArray[np.float64]
    plate_triangles: NDArray[np.float64]
    max_plane_error_m: float


def raster_targets() -> list[tuple[Pose, str]]:
    """The zigzag's Cartesian targets and their move kind, in order."""
    targets = []
    for i, x in enumerate(STROKE_X_M):
        ys = STROKE_Y_M if i % 2 == 0 else STROKE_Y_M[::-1]
        for j, y in enumerate(ys):
            # A stroke's first target is reached by a step-over (or the approach).
            kind = PROCESS if j > 0 else (TRANSITION if i > 0 else FREESPACE)
            targets.append((Pose.from_xyz_quat([x, y, PLANE_Z_M], TOOL_DOWN_QUAT_XYZW), kind))
    return targets


def compute() -> RasterPlan:
    """Plan the raster and evaluate the TCP along the result."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400", locator)
    robot_links = list(robot.get_link_names())
    sx, sy, sz = PLATE_SIZE_M
    plate_center = Pose.from_xyz(STROKE_X_M.mean(), 0.0, PLANE_Z_M - PLATE_GAP_M - sz / 2)
    create_obstacle(robot, "plate", box(sx, sy, sz), plate_center)
    names = robot.get_joint_names(GROUP)
    home = np.zeros(len(names))
    robot.set_joints(home, joint_names=names)

    program = MotionProgram(
        GROUP, tcp_frame=TCP_FRAME, working_frame="base_link", profile=PROCESS
    ).set_joint_names(names)
    program.move_to(StateTarget(home, names=names, profile=FREESPACE))
    kinds = [FREESPACE]
    for pose, kind in raster_targets():
        if kind == FREESPACE:
            program.move_to(CartesianTarget(pose, profile=FREESPACE))
        else:
            program.linear_to(CartesianTarget(pose, profile=PROCESS))
        kinds.append(kind)

    composer = TaskComposer.from_config()
    result = composer.plan(
        robot,
        program,
        pipeline=PIPELINE,
        profiles=create_cartesian_pipeline_profiles(num_threads=DESCARTES_THREADS),
    )
    if not result.successful:
        raise RuntimeError(f"{PIPELINE} failed on the raster: {result.message}")
    if len(result) != len(kinds):
        raise RuntimeError(f"{len(result)} trajectory points for {len(kinds)} program targets")

    group = robot.env.getKinematicGroup(GROUP)
    tcp = np.array([group.calcFwdKin(p.positions)[TCP_LINK].matrix for p in result])
    process = np.array([k == PROCESS for k in kinds])
    plane_error = float(np.max(np.abs(tcp[process, 2, 3] - PLANE_Z_M)))
    if plane_error > PLANE_TOL_M:
        raise OffPlaneError(f"process TCP up to {plane_error:.4g} m off the plane")

    # The approach is one FREESPACE step: its TCP path is the FK of the joint-space
    # interpolation between the two states, not the straight line between their TCPs.
    q0, q1 = result[0].positions, result[1].positions
    approach = np.array(
        [
            group.calcFwdKin(q0 + s * (q1 - q0))[TCP_LINK].matrix[:3, 3]
            for s in np.linspace(0.0, 1.0, APPROACH_SAMPLES)
        ]
    )

    shown = result[int(ROBOT_DRAWN_AT * (len(result) - 1))].positions
    robot.set_joints(shown, joint_names=names)
    poses = scene3d.link_poses(robot.env)
    return RasterPlan(
        tcp=tcp,
        kind=kinds,
        approach=approach,
        robot_triangles=scene3d.link_triangles(robot.env, poses, robot_links),
        plate_triangles=scene3d.link_triangles(robot.env, poses, ["plate"]),
        max_plane_error_m=plane_error,
    )


def _path(ax: Axes, plan: RasterPlan, *, glyphs: bool, lw: float) -> None:
    points = plan.tcp[:, :3, 3]
    for i in range(1, len(points)):
        kind = plan.kind[i]
        if kind == FREESPACE:
            if not glyphs:
                scene3d.polyline(
                    ax, VIEW, plan.approach, color=ORANGE, lw=lw, ls=(0, (3, 2)), zorder=3
                )
            continue
        color = BLUE if kind == PROCESS else ORANGE
        scene3d.polyline(ax, VIEW, points[i - 1 : i + 1], color=color, lw=lw, zorder=3)
    if not glyphs:
        return
    for pose, kind in zip(plan.tcp, plan.kind, strict=True):
        if kind == FREESPACE:
            continue
        origin = np.asarray(pose)[:3, 3]
        for axis, length, color in ((2, Z_GLYPH_M, AXIS_Z), (0, X_GLYPH_M, AXIS_X)):
            scene3d.polyline(
                ax,
                VIEW,
                [origin, origin + length * np.asarray(pose)[:3, axis]],
                color=color,
                lw=1.0,
                zorder=4,
            )


def draw(plan: RasterPlan, theme: Theme) -> Figure:
    """The whole cell, and the raster zoomed with its tool glyphs."""
    fig = plt.figure(figsize=(COLUMN_WIDTH_IN, 3.6))
    overview = fig.add_axes((0.0, 0.0, 0.36, 1.0))
    zoom = fig.add_axes((0.38, 0.17, 0.62, 0.83))

    scene3d.draw_mesh(overview, VIEW, plan.plate_triangles, theme.surface, zorder=1.0)
    scene3d.draw_mesh(overview, VIEW, plan.robot_triangles, theme.surface, zorder=1.1)
    _path(overview, plan, glyphs=False, lw=1.4)
    extent = np.concatenate(
        [plan.robot_triangles.reshape(-1, 3), plan.plate_triangles.reshape(-1, 3), plan.approach]
    )
    scene3d.frame_extent(overview, VIEW, extent, margin_m=0.04)

    scene3d.draw_mesh(zoom, VIEW, plan.plate_triangles, theme.surface, zorder=1.0)
    _path(zoom, plan, glyphs=True, lw=2.0)
    raster = plan.tcp[1:, :3, 3]
    first = raster[0]
    last = raster[-1]
    for point, text, offset in ((first, "start", (-8, -12)), (last, "end", (8, 8))):
        u, v = VIEW.project(point)[:2]
        zoom.annotate(
            text,
            (float(u), float(v)),
            xytext=offset,
            textcoords="offset points",
            color=theme.ink,
            ha="right" if offset[0] < 0 else "left",
            fontsize=8.5,
        )
    scene3d.frame_extent(
        zoom, VIEW, np.concatenate([plan.plate_triangles.reshape(-1, 3), raster]), margin_m=0.02
    )
    fig.legend(
        handles=[
            Line2D([], [], color=BLUE, lw=2.0, label="stroke (LINEAR)"),
            Line2D([], [], color=ORANGE, lw=2.0, label="step-over (LINEAR)"),
            Line2D(
                [],
                [],
                color=ORANGE,
                lw=2.0,
                ls=(0, (3, 2)),
                label="approach (FREESPACE, joint-interpolated)",
            ),
            Line2D([], [], color=AXIS_Z, lw=1.0, label="TCP z (tool axis)"),
            Line2D([], [], color=AXIS_X, lw=1.0, label="TCP x"),
        ],
        loc="lower right",
        bbox_to_anchor=(1.0, 0.0),
        ncols=3,
        frameon=False,
        fontsize=8,
        handlelength=1.6,
        columnspacing=1.2,
    )
    return fig


def inputs() -> list[Path]:
    """The IRB2400 model files, the composer config and the profile factories."""
    return [
        *lock.composer_config_files(),
        *(
            REPO_ROOT / "src/tesseract_robotics/planning/profiles" / name
            for name in ("pipelines.py", "descartes.py", "trajopt.py", "names.py")
        ),
        *lock.support_files(
            [
                "support/urdf/abb_irb2400.urdf",
                "support/urdf/abb_irb2400.srdf",
                "support/urdf/abb_irb2400_plugins.yaml",
                "support/meshes/abb_irb2400/**/*",
            ]
        ),
    ]
