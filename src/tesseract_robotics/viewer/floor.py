"""The floor grid under a scene, as the docs' demo players draw it."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
import trimesh
from numpy.typing import NDArray

# Smallest floor [m]: a few reaches of an industrial arm.
MIN_FLOOR_SIZE_M = 6.0
# Floor grid pitch [m]: fine cells for scale, heavier lines every metre.
FLOOR_CELL_M = 0.25
FLOOR_SECTION_M = 1.0
# Floor beyond the scene's footprint on every side [m]: one section.
FLOOR_MARGIN_M = 1.0
# Soft contact shadow under the robot [-]; 0.3 lifts it off the grid without hiding the lines.
FLOOR_SHADOW_OPACITY = 0.3

# Corners of a unit box, as signs per axis.
_CORNER_SIGNS = np.array(
    [[sx, sy, sz] for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)], dtype=np.float64
)


@dataclass(frozen=True)
class Floor:
    """A square floor parallel to the world xy plane.

    Attributes:
        center_m: Centre (x, y) [m].
        size_m: Side length [m].
        z_m: Height [m]: the lowest point of the scene, so nothing sinks through it.
    """

    center_m: tuple[float, float]
    size_m: float
    z_m: float

    @classmethod
    def under(cls, visuals: Iterable[tuple[NDArray[np.float64], trimesh.Trimesh]]) -> Floor:
        """The floor covering every visual's footprint, with lines on whole sections.

        Args:
            visuals: `(world_from_mesh 4x4, mesh)` per visual.

        Returns:
            The floor; `MIN_FLOOR_SIZE_M` at the origin, at z = 0, for an empty scene.
        """
        corners = [_world_corners(pose, mesh) for pose, mesh in visuals]
        if not corners:
            return cls((0.0, 0.0), MIN_FLOOR_SIZE_M, 0.0)
        points = np.concatenate(corners)
        xy = points[:, :2]
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        # On a section line, so the grid's lines fall on whole metres of the world.
        center = np.round((lo + hi) / 2 / FLOOR_SECTION_M) * FLOOR_SECTION_M
        reach = float(np.max(np.maximum(hi - center, center - lo))) + FLOOR_MARGIN_M
        # Whole sections each side of the centre.
        half = math.ceil(reach / FLOOR_SECTION_M) * FLOOR_SECTION_M
        size = max(MIN_FLOOR_SIZE_M, 2 * half)
        return cls((float(center[0]), float(center[1])), size, float(points[:, 2].min()))


def _world_corners(pose: NDArray[np.float64], mesh: trimesh.Trimesh) -> NDArray[np.float64]:
    # The mesh's own bounding box, carried to the world: covers the mesh, cheap on big meshes.
    lo, hi = mesh.bounds
    local = lo + _CORNER_SIGNS * (hi - lo)
    return local @ pose[:3, :3].T + pose[:3, 3]
