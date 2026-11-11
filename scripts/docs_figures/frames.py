"""Frames of a Cartesian move on the ABB IRB2400: root, base, working frame, tool0, TCP.

A workpiece link is added to the environment at a fixed pose (the working frame), a target
is given in it, and the joint solution is the IK of `tool0` at `target * tcp_offset^-1`, as
the planners do. The figure draws the meshes at that solution and every frame from forward
kinematics; `compute` asserts the TCP lands on the target.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from numpy.typing import NDArray

from scripts.docs_figures import lock, scene3d
from scripts.docs_figures.style import COLUMN_WIDTH_IN, ORANGE, Theme
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import (
    Pose,
    Robot,
    box,
    create_fixed_joint,
)
from tesseract_robotics.planning.geometry import create_link_with_geometry

GROUP: Final[str] = "manipulator"
TCP_FRAME: Final[str] = "tool0"
WORKING_FRAME: Final[str] = "workpiece"
# m: the working frame's pose in base_link: the near corner of a part in front of the robot,
# yawed 25 deg so its axes are distinguishable from the root's.
WORKPIECE_XYZ_M: Final[tuple[float, float, float]] = (0.80, -0.35, 0.30)
WORKPIECE_YAW_RAD: Final[float] = np.radians(25.0)
# m: the part plate: 0.40 x 0.30, 0.04 thick. The working frame sits on its top face at a
# corner, x and y along the edges, as a part frame usually does.
PLATE_SIZE_M: Final[tuple[float, float, float]] = (0.40, 0.30, 0.04)
# m: the docs' example tool, 10 cm along tool0's z axis (planning.md "Frames and the TCP").
TCP_OFFSET_Z_M: Final[float] = 0.10
# m: tool body radius, drawn between tool0 and the TCP.
TOOL_RADIUS_M: Final[float] = 0.012
TOOL_SECTIONS: Final[int] = 24
# Target in the working frame, m: on the top face, 30 cm along the part's x and 7 cm along
# its y; tool z into the part (180 deg about x).
TARGET_XYZ_M: Final[tuple[float, float, float]] = (0.30, 0.07, 0.0)
TARGET_RPY_RAD: Final[tuple[float, float, float]] = (np.pi, 0.0, 0.0)
# m: the TCP must land on the target to the IK solver's precision; 1 um is far looser than
# OPW's closed form (~1e-12) and far tighter than anything the figure could show.
TCP_TARGET_TOL_M: Final[float] = 1e-6
# The solution drawn: the IK branch closest to this configuration (elbow up, wrist not
# flipped), rad.
PREFERRED_CONFIG_RAD: Final[NDArray[np.float64]] = np.array([0.0, 0.3, 0.2, 0.0, 1.0, 0.0])
VIEW: Final[scene3d.View] = scene3d.View(azimuth_deg=-32.0, elevation_deg=18.0)
# m: frame axis lengths: the root large, the small tool frames short enough not to overlap.
ROOT_AXIS_M: Final[float] = 0.30
WORK_AXIS_M: Final[float] = 0.16
ZOOM_AXIS_M: Final[float] = 0.07
# m: margin around the plate and the tool in the zoomed panel.
ZOOM_MARGIN_M: Final[float] = 0.05


class TcpOffTargetError(AssertionError):
    """The TCP from forward kinematics is not on the target the IK solved for."""


@dataclass(frozen=True)
class FramesScene:
    """Everything the frames figure draws, in base_link (world) coordinates."""

    robot_triangles: NDArray[np.float64]
    part_triangles: NDArray[np.float64]
    tool_triangles: NDArray[np.float64]
    frames: dict[str, NDArray[np.float64]]
    joints_rad: NDArray[np.float64]
    tcp_error_m: float


def compute() -> FramesScene:
    """Build the scene, solve the IK and return the frames from forward kinematics."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400", locator)
    robot_links = set(robot.get_link_names())
    workpiece = Pose.from_xyz_rpy(WORKPIECE_XYZ_M, (0.0, 0.0, WORKPIECE_YAW_RAD))
    sx, sy, sz = PLATE_SIZE_M
    plate = create_link_with_geometry(
        WORKING_FRAME, box(sx, sy, sz), origin=Pose.from_xyz(sx / 2, sy / 2, -sz / 2)
    )
    robot.add_link(
        plate, create_fixed_joint(f"{WORKING_FRAME}_joint", "base_link", WORKING_FRAME, workpiece)
    )
    tool_offset = Pose.from_xyz(0.0, 0.0, TCP_OFFSET_Z_M)

    target_in_work = Pose.from_xyz_rpy(TARGET_XYZ_M, TARGET_RPY_RAD)
    root = robot.env.getSceneGraph().getRoot()
    base_link = robot.env.getKinematicGroup(GROUP).getBaseLinkId().name()
    if base_link != root:
        raise TcpOffTargetError(f"group base {base_link!r} is not the root {root!r}")
    target = workpiece @ target_in_work
    flange_target = target @ tool_offset.inverse()
    solutions = robot.ik(GROUP, flange_target, TCP_FRAME, all_solutions=True)
    joints = min(solutions, key=lambda q: float(np.linalg.norm(q - PREFERRED_CONFIG_RAD)))
    robot.set_joints(joints, joint_names=robot.get_joint_names(GROUP))

    poses = scene3d.link_poses(robot.env)
    tcp = poses[TCP_FRAME] @ tool_offset.matrix
    error = float(np.linalg.norm(tcp[:3, 3] - target.matrix[:3, 3]))
    if error > TCP_TARGET_TOL_M:
        raise TcpOffTargetError(f"TCP is {error:.3g} m from the target (tol {TCP_TARGET_TOL_M})")

    robot_triangles = scene3d.link_triangles(robot.env, poses, robot_links)
    part_triangles = scene3d.link_triangles(robot.env, poses, [WORKING_FRAME])
    # The tool body: a cylinder from tool0 to the TCP. Drawn only; the planners see the
    # offset, not a tool link.
    tool = trimesh.creation.cylinder(TOOL_RADIUS_M, TCP_OFFSET_Z_M, sections=TOOL_SECTIONS)
    tool_pose = poses[TCP_FRAME] @ Pose.from_xyz(0.0, 0.0, TCP_OFFSET_Z_M / 2).matrix
    tool_triangles = scene3d.transformed(
        np.asarray(tool.vertices)[np.asarray(tool.faces)], tool_pose
    )

    return FramesScene(
        robot_triangles=robot_triangles,
        part_triangles=part_triangles,
        tool_triangles=tool_triangles,
        frames={
            "root": poses[str(root)],
            "base": poses["base"],
            "workpiece": poses[WORKING_FRAME],
            "tool0": poses[TCP_FRAME],
            "tcp": tcp,
        },
        joints_rad=np.asarray(joints),
        tcp_error_m=error,
    )


def _draw_scene(ax: Axes, scene: FramesScene, theme: Theme) -> None:
    scene3d.draw_mesh(ax, VIEW, scene.part_triangles, theme.surface, zorder=1.0)
    scene3d.draw_mesh(ax, VIEW, scene.robot_triangles, theme.surface, zorder=1.1)
    scene3d.draw_mesh(ax, VIEW, scene.tool_triangles, ORANGE, zorder=1.2)


def _zoom_box(scene: FramesScene) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """View-plane (u, v) corners of the zoomed region: the plate and the tool."""
    points = np.concatenate(
        [scene.part_triangles.reshape(-1, 3), scene.frames["tool0"][None, :3, 3]]
    )
    uv = VIEW.project(points)[:, :2]
    return uv.min(axis=0) - ZOOM_MARGIN_M, uv.max(axis=0) + ZOOM_MARGIN_M


def draw(scene: FramesScene, theme: Theme) -> Figure:
    """The frames figure for one theme: the whole cell, and the tool and part zoomed."""
    fig = plt.figure(figsize=(COLUMN_WIDTH_IN, 3.5))
    overview = fig.add_axes((0.0, 0.0, 0.47, 1.0))
    zoom = fig.add_axes((0.50, 0.04, 0.50, 0.92))

    _draw_scene(overview, scene, theme)
    scene3d.draw_frame(
        overview,
        VIEW,
        scene.frames["root"],
        ROOT_AXIS_M,
        label="base_link\n(root, IK base)",
        label_offset_pt=(-58, -26),
        label_color=theme.ink,
    )
    scene3d.draw_frame(overview, VIEW, scene.frames["workpiece"], WORK_AXIS_M)
    extent = np.concatenate(
        [scene.robot_triangles.reshape(-1, 3), scene.part_triangles.reshape(-1, 3)]
    )
    scene3d.frame_extent(overview, VIEW, extent, margin_m=0.05)
    lo, hi = _zoom_box(scene)
    overview.add_patch(
        Rectangle(
            (float(lo[0]), float(lo[1])),
            *(hi - lo),
            fill=False,
            ec=theme.muted,
            lw=0.8,
            ls=(0, (3, 2)),
            zorder=6,
        )
    )

    _draw_scene(zoom, scene, theme)
    zoom_frames = {
        "workpiece": ("workpiece (working_frame)", ZOOM_AXIS_M, (-8, -16), "right"),
        "tool0": ("tool0 (tcp_frame)", ZOOM_AXIS_M, (-10, 2), "right"),
        "tcp": ("TCP = tool0 · tcp_offset\n= workpiece · target", ZOOM_AXIS_M, (-12, -24), "right"),
    }
    for key, (text, length, offset, ha) in zoom_frames.items():
        scene3d.draw_frame(
            zoom,
            VIEW,
            scene.frames[key],
            length,
            label=text,
            label_offset_pt=offset,
            label_color=theme.ink,
            label_ha=ha,
        )
    # The target vector, workpiece origin to TCP, dashed: what a CartesianTarget holds.
    scene3d.polyline(
        zoom,
        VIEW,
        [scene.frames["workpiece"][:3, 3], scene.frames["tcp"][:3, 3]],
        color=theme.muted,
        lw=1.0,
        ls=(0, (3, 2)),
        zorder=4.5,
    )
    zoom.set_xlim(lo[0], hi[0])
    zoom.set_ylim(lo[1], hi[1])
    zoom.set_aspect("equal", adjustable="box", anchor="C")
    zoom.set_axis_off()
    zoom.add_patch(
        Rectangle(
            (0, 0),
            1,
            1,
            transform=zoom.transAxes,
            fill=False,
            ec=theme.muted,
            lw=0.8,
            ls=(0, (3, 2)),
            zorder=6,
        )
    )
    return fig


def inputs() -> list[Path]:
    """The IRB2400 model files the figure loads."""
    return lock.support_files(
        [
            "support/urdf/abb_irb2400.urdf",
            "support/urdf/abb_irb2400.srdf",
            "support/urdf/abb_irb2400_plugins.yaml",
            "support/meshes/abb_irb2400/**/*",
        ]
    )
