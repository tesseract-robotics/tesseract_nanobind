"""Standard toolpaths: shape contracts of the `planning.toolpaths` factories (#84)."""

import numpy as np
import pytest

from tesseract_robotics.planning import Pose
from tesseract_robotics.planning.toolpaths import (
    OUTLINE_POSE_COUNT,
    X_RANGE_M,
    Y_RANGE_M,
    ZIGZAG_LINE_COUNT,
    DegenerateToolpathError,
    circle,
    zigzag_square,
)

# Positions are built from float64 trig and sums of a few terms; 1 nm is far
# below any toolpath feature and far above float64 roundoff at metre scale.
POSITION_ATOL_M = 1e-9
# Unit-vector components of a rotation built from one quaternion: float64 roundoff.
AXIS_ATOL = 1e-12

TOOL_DOWN_Z_AXIS = np.array([0.0, 0.0, -1.0])


def test_zigzag_pose_count_and_bounds() -> None:
    poses = zigzag_square()

    assert len(poses) == OUTLINE_POSE_COUNT + 2 * ZIGZAG_LINE_COUNT
    assert all(isinstance(p, Pose) for p in poses)
    xyz = np.array([p.translation for p in poses])
    assert xyz[:, 0].min() >= X_RANGE_M[0] - POSITION_ATOL_M
    assert xyz[:, 0].max() <= X_RANGE_M[1] + POSITION_ATOL_M
    assert xyz[:, 1].min() >= Y_RANGE_M[0] - POSITION_ATOL_M
    assert xyz[:, 1].max() <= Y_RANGE_M[1] + POSITION_ATOL_M
    # The outline closes on its first corner.
    np.testing.assert_allclose(xyz[OUTLINE_POSE_COUNT - 1], xyz[0], atol=POSITION_ATOL_M)


@pytest.mark.parametrize("n_waypoints", [84, 210, 419])
def test_circle_on_radius_tool_down(n_waypoints: int) -> None:
    radius_m, center, z_m = 0.25, (0.8, 0.1), 0.6
    poses = circle(radius_m, n_waypoints, center=center, z_m=z_m)

    assert len(poses) == n_waypoints
    xyz = np.array([p.translation for p in poses])
    np.testing.assert_allclose(np.hypot(xyz[:, 0] - center[0], xyz[:, 1] - center[1]), radius_m)
    np.testing.assert_allclose(xyz[:, 2], z_m, atol=POSITION_ATOL_M)
    for p in poses:
        np.testing.assert_allclose(p.rotation_matrix[:, 2], TOOL_DOWN_Z_AXIS, atol=AXIS_ATOL)
    # Evenly spaced, no duplicate closing pose.
    chords = np.linalg.norm(np.diff(np.vstack([xyz, xyz[:1]]), axis=0), axis=1)
    np.testing.assert_allclose(chords, chords[0])


@pytest.mark.parametrize("bad", [0, 2])
def test_circle_rejects_degenerate_waypoint_count(bad: int) -> None:
    with pytest.raises(DegenerateToolpathError, match="n_waypoints"):
        circle(0.25, bad)
