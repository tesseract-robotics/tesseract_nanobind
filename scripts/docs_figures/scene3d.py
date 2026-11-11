"""Orthographic 3D drawing on a 2D matplotlib axes: shaded robot meshes, frames, polylines.

matplotlib's own 3D axes sorts whole artists, not triangles, and draws perspective panes;
here every triangle of every link is depth-sorted (painter's algorithm) and flat-shaded, and
the axes stay a plain 2D axes with equal aspect, so lengths read true in the view plane.

Geometry comes from the environment: each link's visual meshes (through the viewer's
`visual_to_trimesh`, the same conversion the 3D viewer uses) placed at the link's transform
in the scene state times the visual origin.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Final

import numpy as np
from matplotlib.axes import Axes
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb
from numpy.typing import ArrayLike, NDArray

from scripts.docs_figures.style import AXIS_COLORS, BASE_FONT_PT
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.environment import Environment
from tesseract_robotics.viewer.geometry import visual_to_trimesh

# Fraction of the base colour a face facing away from the light keeps (ambient term).
AMBIENT: Final[float] = 0.45
# Light direction in view coordinates (x right, y up, z towards the viewer): upper left, front.
LIGHT_DIR_VIEW: Final[NDArray[np.float64]] = np.array([-0.45, 0.6, 0.66]) / np.linalg.norm(
    [-0.45, 0.6, 0.66]
)


@dataclass(frozen=True)
class View:
    """An orthographic camera: azimuth about world +Z, elevation above the XY plane.

    Azimuth 0 looks along world -Y onto the XZ plane (x right, z up); positive azimuth
    orbits counter-clockwise seen from above.
    """

    azimuth_deg: float
    elevation_deg: float

    @property
    def rotation(self) -> NDArray[np.float64]:
        """Rows are the view axes (right, up, towards viewer) in world coordinates."""
        az, el = np.radians(self.azimuth_deg), np.radians(self.elevation_deg)
        right = np.array([np.cos(az), np.sin(az), 0.0])
        towards = np.array([np.sin(az) * np.cos(el), -np.cos(az) * np.cos(el), np.sin(el)])
        up = np.cross(towards, right)
        return np.vstack([right, up, towards])

    def project(self, points: ArrayLike) -> NDArray[np.float64]:
        """World points (..., 3) to view coordinates (..., 3): u right, v up, depth to viewer."""
        return np.asarray(points, dtype=np.float64) @ self.rotation.T


def link_poses(env: Environment) -> dict[str, NDArray[np.float64]]:
    """Link name to 4x4 world transform, from the environment's current state."""
    return {link: pose.matrix for link, pose in env.getState().link_transforms.items()}


def link_triangles(
    env: Environment, poses: Mapping[str, NDArray[np.float64]], links: Iterable[str]
) -> NDArray[np.float64]:
    """Every visual triangle of `links`, in world coordinates, shape (n, 3, 3).

    Args:
        env: Environment whose scene graph supplies the visuals.
        poses: Link name to 4x4 world transform (`link_poses`).
        links: Names of the links to include.
    """
    triangles = []
    for name in links:
        for visual in env.getLink(name).visual:
            pose = poses[name] @ visual.origin.matrix
            for mesh in visual_to_trimesh(visual, name):
                vertices = np.asarray(mesh.vertices) @ pose[:3, :3].T + pose[:3, 3]
                triangles.append(vertices[np.asarray(mesh.faces)])
    return np.concatenate(triangles)


def draw_mesh(
    ax: Axes,
    view: View,
    triangles: NDArray[np.float64],
    color: str,
    *,
    alpha: float = 1.0,
    zorder: float = 1.0,
) -> None:
    """Flat-shaded, depth-sorted triangles; rasterised inside a vector figure."""
    projected = view.project(triangles)
    normals = np.cross(projected[:, 1] - projected[:, 0], projected[:, 2] - projected[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    keep = norms > 0.0
    projected, normals = projected[keep], normals[keep] / norms[keep, None]
    # Two-sided lighting: meshes are not guaranteed to be consistently wound.
    lambert = np.abs(normals @ LIGHT_DIR_VIEW)
    shade = AMBIENT + (1.0 - AMBIENT) * lambert
    order = np.argsort(projected[:, :, 2].mean(axis=1))
    base = np.array(to_rgb(color))
    facecolors = np.clip(shade[order, None] * base[None, :], 0.0, 1.0)
    collection = PolyCollection(
        list(projected[order, :, :2]),
        facecolors=np.column_stack([facecolors, np.full(len(order), alpha)]),
        edgecolors=np.column_stack([facecolors, np.full(len(order), alpha)]),
        linewidths=0.15,
        zorder=zorder,
        rasterized=True,
    )
    ax.add_collection(collection)


def draw_frame(
    ax: Axes,
    view: View,
    transform: NDArray[np.float64] | Isometry3d,
    length_m: float,
    *,
    label: str | None = None,
    label_offset_pt: tuple[float, float] = (6.0, 6.0),
    label_color: str | None = None,
    label_ha: str = "left",
    linewidth: float = 1.8,
    zorder: float = 5.0,
) -> None:
    """A frame's x, y, z axes as red, green, blue arrows of `length_m`, labelled at its origin."""
    matrix = transform.matrix if isinstance(transform, Isometry3d) else np.asarray(transform)
    origin = matrix[:3, 3]
    o = view.project(origin)
    # Draw the axis pointing most away from the viewer first, so nearer axes overlap it.
    tips = [view.project(origin + length_m * matrix[:3, i]) for i in range(3)]
    for i in sorted(range(3), key=lambda k: tips[k][2]):
        ax.annotate(
            "",
            xy=(float(tips[i][0]), float(tips[i][1])),
            xytext=(float(o[0]), float(o[1])),
            arrowprops={
                "arrowstyle": "-|>,head_length=0.45,head_width=0.22",
                "color": AXIS_COLORS[i],
                "lw": linewidth,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=zorder,
        )
    ax.plot(*o[:2], "o", ms=2.8, color=label_color or "#808080", zorder=zorder + 0.1)
    if label is not None:
        ax.annotate(
            label,
            (float(o[0]), float(o[1])),
            xytext=label_offset_pt,
            textcoords="offset points",
            ha=label_ha,
            fontsize=BASE_FONT_PT - 0.5,
            color=label_color,
            zorder=zorder + 0.2,
        )


def polyline(
    ax: Axes, view: View, points: Sequence[ArrayLike] | NDArray[np.float64], **kwargs: Any
) -> None:
    """A 3D polyline projected into the view."""
    uv = view.project(np.asarray(points))[:, :2]
    ax.plot(uv[:, 0], uv[:, 1], **kwargs)


def frame_extent(ax: Axes, view: View, points: NDArray[np.float64], margin_m: float) -> None:
    """Equal-aspect limits that hold the projections of `points` plus a margin."""
    uv = view.project(points.reshape(-1, 3))[:, :2]
    lo, hi = uv.min(axis=0) - margin_m, uv.max(axis=0) + margin_m
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.set_axis_off()


def transformed(triangles: NDArray[np.float64], pose: NDArray[np.float64]) -> NDArray[np.float64]:
    """Triangles (n, 3, 3) moved by a 4x4 transform."""
    return triangles @ pose[:3, :3].T + pose[:3, 3]
