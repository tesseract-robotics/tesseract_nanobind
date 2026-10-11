"""gh-175: createConvexHull bound as (n_faces, vertices, faces); failure raises ConvexHullError.

Upstream convex_hull_utils.cpp (0.35.0) returns -1 only when Bullet's
btConvexHullComputer::compute returns < 0, which happens only when a positive
`shrink` cannot be applied (btConvexHullInternal::shrink -> shiftFace fails).
Empty input returns 0 faces, not an error.

gh-233: upstream createConvexHull (convex_hull_utils.cpp:75-99) never shrinks its
reused `face` vector, so face i lists the 4th+ vertices of every earlier face; the
binding strips that prefix (ConvexHullFaceRepairError if it is not there).
"""

import itertools

import numpy as np
import pytest

import tesseract_robotics.tesseract_collision as tesseract_collision
from tesseract_robotics.tesseract_collision import ConvexHullError, createConvexHull, makeConvexMesh
from tesseract_robotics.tesseract_common import BytesResource
from tesseract_robotics.tesseract_geometry import ConvexMesh, Mesh

# Unit cube corners (m), plus its centre, which is not a hull vertex.
CUBE_CORNERS = [np.array(c, dtype=float) for c in itertools.product((0.0, 1.0), repeat=3)]
CUBE_CENTRE = np.array([0.5, 0.5, 0.5])
# m: shrink well inside the 0.5 m half-width.
SMALL_SHRINK = 0.1
# m: shrink ten times the cube's edge, unclamped: the faces cross and Bullet gives up.
IMPOSSIBLE_SHRINK = 10.0
# Box faces as (axis, side), side 0 = the minimum on that axis, 1 = the maximum.
BOX_SIDES = [(axis, side) for axis in range(3) for side in (0, 1)]
# A non-uniform scale, so a dropped scale cannot pass as the default (1, 1, 1).
MESH_SCALE = np.array([2.0, 3.0, 4.0])


def _walk_faces(faces):
    """Split the flat `[count, i0, ..., i{count-1}, ...]` array into index lists."""
    out, i = [], 0
    while i < len(faces):
        count = int(faces[i])
        out.append([int(x) for x in faces[i + 1 : i + 1 + count]])
        i += 1 + count
    assert i == len(faces), "face counts do not consume the array exactly"
    return out


def _assert_box_hull(vertices, faces):
    """The hull of an axis-aligned box: six faces of four distinct vertices, each face lying
    on its own one of the box's six planes.

    Exact, no tolerance: Bullet computes the hull on an integer grid (rational after a
    shrink) and maps each axis back on its own, so vertices that share a grid coordinate
    share the float. A face's four vertices sharing the extreme coordinate on an axis are
    coplanar, and every hull vertex is on or behind that plane.
    """
    points = np.array([np.asarray(v) for v in vertices])
    bounds = (points.min(axis=0), points.max(axis=0))
    walked = _walk_faces(faces)
    assert [len(face) for face in walked] == [4] * len(BOX_SIDES), walked
    sides = []
    for face in walked:
        assert len(set(face)) == len(face), face
        on = [
            (axis, side)
            for axis, side in BOX_SIDES
            if np.all(points[face, axis] == bounds[side][axis])
        ]
        assert len(on) == 1, (face, points[face])
        sides.append(on[0])
    assert sorted(sides) == BOX_SIDES


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


@pytest.mark.parametrize("shrink", [-1.0, SMALL_SHRINK])
def test_cube_hull_faces_are_the_cube_faces(shrink):
    """gh-233: upstream returns face sizes 4, 5, 6, 7, 8, 9 here."""
    n, vertices, faces = createConvexHull(CUBE_CORNERS, shrink=shrink)
    assert n == len(BOX_SIDES)
    _assert_box_hull(vertices, faces)


def test_make_convex_mesh_faces_are_the_cube_faces():
    """gh-233: makeConvexMesh goes through createConvexHull; the repaired mesh keeps the
    vertices, face count, resource, scale and creation method."""
    # makeConvexMesh hulls the vertices only (convex_hull_utils.cpp:114): one triangle
    # makes a valid Mesh.
    resource = BytesResource("cube.stl", b"")
    mesh = Mesh(CUBE_CORNERS, np.array([3, 0, 1, 3], dtype=np.int32), resource, MESH_SCALE)
    hull = makeConvexMesh(mesh)
    assert isinstance(hull, ConvexMesh)
    _assert_box_hull(hull.getVertices(), hull.getFaces())
    assert hull.getFaceCount() == len(BOX_SIDES)
    _, vertices, _ = createConvexHull(CUBE_CORNERS)
    np.testing.assert_array_equal(np.asarray(hull.getVertices()), np.asarray(vertices))
    assert hull.getResource().getUrl() == resource.getUrl()
    np.testing.assert_array_equal(hull.getScale(), MESH_SCALE)
    assert hull.getCreationMethod() == ConvexMesh.CreationMethod.CONVERTED


def test_convex_hull_face_repair_error_is_runtime_error():
    # Looked up here, not imported: before the repair, a missing name fails this test alone.
    assert issubclass(tesseract_collision.ConvexHullFaceRepairError, RuntimeError)


# The cube hull's faces as upstream 0.35.0 returns them (gh-233) and as the repair must
# return them.
UPSTREAM_CUBE_FACES = [
    4, 0, 1, 5, 2,
    5, 0, 2, 6, 2, 3,
    6, 0, 3, 4, 2, 3, 1,
    7, 1, 4, 7, 2, 3, 1, 5,
    8, 2, 5, 7, 2, 3, 1, 5, 6,
    9, 3, 6, 7, 2, 3, 1, 5, 6, 4,
]  # fmt: skip
REPAIRED_CUBE_FACES = [
    4, 0, 1, 5, 2,
    4, 0, 2, 6, 3,
    4, 0, 3, 4, 1,
    4, 1, 4, 7, 5,
    4, 2, 5, 7, 6,
    4, 3, 6, 7, 4,
]  # fmt: skip


def _repair(faces):
    # Private oracle for the repair helper: upstream output cannot reach its error paths.
    from tesseract_robotics.tesseract_collision._tesseract_collision import (
        _repair_convex_hull_faces,
    )

    return _repair_convex_hull_faces(np.array(faces, dtype=np.int32)).tolist()


def test_repair_strips_the_accumulated_prefix():
    assert _repair(UPSTREAM_CUBE_FACES) == REPAIRED_CUBE_FACES


def test_repair_leaves_triangles_alone():
    triangles = [3, 0, 1, 2, 3, 0, 2, 3]
    assert _repair(triangles) == triangles


def test_repair_of_correct_faces_raises():
    """A fixed upstream: face 1 does not carry face 0's extra vertex. Fail loud, never cut."""
    with pytest.raises(tesseract_collision.ConvexHullFaceRepairError, match="face 1"):
        _repair(REPAIRED_CUBE_FACES)


def test_repair_of_face_shorter_than_prefix_raises():
    with pytest.raises(tesseract_collision.ConvexHullFaceRepairError, match="face 1"):
        _repair([4, 0, 1, 2, 3, 3, 0, 1, 2])


def test_repair_of_count_past_end_raises():
    with pytest.raises(tesseract_collision.ConvexHullFaceRepairError, match="face 0"):
        _repair([4, 0, 1, 2])
