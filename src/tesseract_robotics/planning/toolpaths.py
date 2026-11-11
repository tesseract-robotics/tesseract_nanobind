"""Standard toolpaths: fixed, parametric pose sequences for tests, docs and benchmarks.

What `lena` is to image processing, these are to toolpath planning: one known
shape, so a new solver, constraint or emitter is judged on the same path every
time (#84). Each factory returns plain `Pose` lists in the working frame; turning
them into a program (profiles, move types, TCP) stays with the caller.

Example:
    ```python
    from tesseract_robotics.planning import CartesianTarget, MotionProgram
    from tesseract_robotics.planning.toolpaths import circle

    program = MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    for pose in circle(0.25, 84, center=(0.8, 0.0), z_m=0.6):
        program.linear_to(CartesianTarget(pose))
    ```
"""

from __future__ import annotations

import math
from typing import Final

import numpy as np

from tesseract_robotics.planning.transforms import Pose

#: Workpiece-plane height of the zigzag square above the robot base (m). The
#: default 0.8 m x 0.8 m square is placed where the ABB IRB 2400 solves IK for
#: every pose with the tool pointing down; being inside the ~1.55 m reach
#: envelope is not enough (closer in, or higher up, some tool-down poses have no
#: solution).
Z_HEIGHT_M: Final[float] = 0.4

#: X extent of the zigzag square (m).
X_RANGE_M: Final[tuple[float, float]] = (0.6, 1.4)

#: Y extent of the zigzag square (m).
Y_RANGE_M: Final[tuple[float, float]] = (-0.4, 0.4)

#: Zigzag scan lines: Y width / 0.08 m spacing + 1.
ZIGZAG_LINE_COUNT: Final[int] = 11

#: Square outline poses: 4 corners plus the closing return to the first.
OUTLINE_POSE_COUNT: Final[int] = 5

#: Orientation of every zigzag pose, scalar-last [qx, qy, qz, qw]: 180 degrees
#: about world Y, so the TCP z-axis points along world -Z, into the workpiece
#: (tool down), with the TCP x-axis along world -X.
ZIGZAG_QUAT_XYZW: Final[tuple[float, float, float, float]] = (0.0, 1.0, 0.0, 0.0)

#: Tool down, scalar-last [qx, qy, qz, qw]: 180 degrees about world X, so the
#: TCP z-axis points along world -Z, into the workpiece.
TOOL_DOWN_QUAT_XYZW: Final[tuple[float, float, float, float]] = (1.0, 0.0, 0.0, 0.0)

#: Fewest waypoints that span a circle rather than a line.
MIN_CIRCLE_WAYPOINTS: Final[int] = 3


class DegenerateToolpathError(ValueError):
    """The parameters describe no toolpath (too few waypoints, a non-positive radius)."""


def zigzag_square(
    *,
    x_range_m: tuple[float, float] = X_RANGE_M,
    y_range_m: tuple[float, float] = Y_RANGE_M,
    z_m: float = Z_HEIGHT_M,
    line_count: int = ZIGZAG_LINE_COUNT,
    quat_xyzw: tuple[float, float, float, float] = ZIGZAG_QUAT_XYZW,
) -> list[Pose]:
    """Square outline, then a zigzag fill across it.

    The first `OUTLINE_POSE_COUNT` poses trace the outline (four corners and the
    return to the first); then each of `line_count` scan lines, evenly spaced
    over `y_range_m`, adds its two endpoints, alternating direction.

    Args:
        x_range_m: X extent `(lo, hi)` of the square (m).
        y_range_m: Y extent `(lo, hi)` of the square (m).
        z_m: Plane height (m).
        line_count: Number of zigzag scan lines.
        quat_xyzw: Orientation of every pose, scalar-last.

    Returns:
        `OUTLINE_POSE_COUNT + 2 * line_count` poses.
    """
    x_lo, x_hi = x_range_m
    y_lo, y_hi = y_range_m
    corners = [(x_lo, y_lo), (x_hi, y_lo), (x_hi, y_hi), (x_lo, y_hi)]
    xy: list[tuple[float, float]] = [*corners, corners[0]]
    for i, y in enumerate(np.linspace(y_lo, y_hi, line_count)):
        x_start, x_end = (x_lo, x_hi) if i % 2 == 0 else (x_hi, x_lo)
        xy += [(x_start, float(y)), (x_end, float(y))]
    return [Pose.from_xyz_quat([x, y, z_m], quat_xyzw) for x, y in xy]


def circle(
    radius_m: float,
    n_waypoints: int,
    *,
    center: tuple[float, float] = (0.8, 0.0),
    z_m: float = Z_HEIGHT_M,
    quat_xyzw: tuple[float, float, float, float] = TOOL_DOWN_QUAT_XYZW,
) -> list[Pose]:
    """Closed circle in a horizontal plane, tool down, evenly spaced.

    The contour of #140 (84 to 419 waypoints on the ABB IRB 2400). The first
    pose is at angle 0 (`+X` of `center`), the path runs counter-clockwise, and
    the start is not repeated at the end.

    Args:
        radius_m: Circle radius (m).
        n_waypoints: Number of poses.
        center: Circle center `(x, y)` (m).
        z_m: Plane height (m).
        quat_xyzw: Orientation of every pose, scalar-last.

    Returns:
        `n_waypoints` poses.

    Raises:
        DegenerateToolpathError: `n_waypoints` below `MIN_CIRCLE_WAYPOINTS`, or
            `radius_m` not positive.
    """
    if n_waypoints < MIN_CIRCLE_WAYPOINTS:
        raise DegenerateToolpathError(
            f"n_waypoints={n_waypoints}: a circle needs at least {MIN_CIRCLE_WAYPOINTS}"
        )
    if not radius_m > 0.0:
        raise DegenerateToolpathError(f"radius_m={radius_m}: must be positive")
    cx, cy = center
    step = 2.0 * math.pi / n_waypoints
    return [
        Pose.from_xyz_quat(
            [cx + radius_m * math.cos(i * step), cy + radius_m * math.sin(i * step), z_m],
            quat_xyzw,
        )
        for i in range(n_waypoints)
    ]
