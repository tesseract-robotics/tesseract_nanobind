"""OctoMap `.bt` decoder: occupied voxels of a tesseract octree."""

import numpy as np
import pytest

from tesseract_robotics.examples.pointcloud_octree_collision_example import build_octree_obstacle
from tesseract_robotics.viewer.errors import MalformedOctreeStreamError
from tesseract_robotics.viewer.octree import _iter_octomap_bt_leaves, octree_voxels

RESOLUTION_M = 0.1


def test_voxels_contain_the_points():
    points = np.array([[0.02, 0.03, 0.04], [0.52, 0.53, 0.54]])
    centers, sizes = octree_voxels(build_octree_obstacle(points, resolution=RESOLUTION_M))
    np.testing.assert_allclose(sizes, RESOLUTION_M)
    for point in points:
        assert (np.abs(centers - point) <= sizes[:, None] / 2).all(axis=1).any()
    assert len(centers) == len(points)


@pytest.mark.parametrize(
    ("stream", "message"),
    [
        (b"res 0.1\n", "no 'data' line"),
        (b"id OcTree\ndata\n\x00\x00", "no 'res' line"),
        (b"res 0.1\ndata\n\x00", "truncated"),
    ],
    ids=["no-data-line", "no-resolution", "truncated"],
)
def test_malformed_stream_raises(stream, message):
    with pytest.raises(MalformedOctreeStreamError, match=message):
        list(_iter_octomap_bt_leaves(stream))
