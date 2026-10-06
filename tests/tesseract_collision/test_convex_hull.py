"""gh-175: createConvexHull bound as (n_faces, vertices, faces); failure raises ConvexHullError.

Upstream convex_hull_utils.cpp (0.35.0) returns -1 only when Bullet's
btConvexHullComputer::compute returns < 0, which happens only when a positive
`shrink` cannot be applied (btConvexHullInternal::shrink -> shiftFace fails).
Empty input returns 0 faces, not an error.
"""

import itertools

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import ConvexHullError, createConvexHull

# Unit cube corners (m), plus its centre, which is not a hull vertex.
CUBE_CORNERS = [np.array(c, dtype=float) for c in itertools.product((0.0, 1.0), repeat=3)]
CUBE_CENTRE = np.array([0.5, 0.5, 0.5])
# m: shrink well inside the 0.5 m half-width.
SMALL_SHRINK = 0.1
# m: shrink ten times the cube's edge, unclamped: the faces cross and Bullet gives up.
IMPOSSIBLE_SHRINK = 10.0


def _walk_faces(faces):
    """Split the flat `[count, i0, ..., i{count-1}, ...]` array into index lists."""
    out, i = [], 0
    while i < len(faces):
        count = int(faces[i])
        out.append([int(x) for x in faces[i + 1 : i + 1 + count]])
        i += 1 + count
    assert i == len(faces), "face counts do not consume the array exactly"
    return out


def test_cube_hull():
    n, vertices, faces = createConvexHull(CUBE_CORNERS + [CUBE_CENTRE])
    assert len(vertices) == len(CUBE_CORNERS)
    walked = _walk_faces(faces)
    assert len(walked) == n
    assert all(0 <= idx < len(vertices) for face in walked for idx in face)
    hull = {tuple(np.round(v, 9)) for v in vertices}
    assert hull == {tuple(c) for c in CUBE_CORNERS}


def test_shrink_moves_vertices_inside():
    _, vertices, _ = createConvexHull(CUBE_CORNERS, shrink=SMALL_SHRINK)
    assert vertices
    for v in vertices:
        assert np.all(v > 0.0) and np.all(v < 1.0), v


def test_empty_input_is_zero_faces():
    """Bullet returns 0 for no points: no hull, but not a failure."""
    n, vertices, faces = createConvexHull([])
    assert n == 0
    assert vertices == []
    assert len(faces) == 0


def test_failed_shrink_raises_convex_hull_error():
    with pytest.raises(ConvexHullError, match="createConvexHull"):
        createConvexHull(CUBE_CORNERS, shrink=IMPOSSIBLE_SHRINK)


def test_convex_hull_error_is_runtime_error():
    assert issubclass(ConvexHullError, RuntimeError)
