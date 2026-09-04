import os

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import tesseract_common, tesseract_geometry


def test_geometry_instantiation():
    # Test that all basic geometry types can be instantiated
    assert tesseract_geometry.Box(1, 1, 1) is not None
    assert tesseract_geometry.Cone(1, 1) is not None
    assert tesseract_geometry.Cylinder(1, 1) is not None
    assert tesseract_geometry.Capsule(1, 1) is not None
    assert tesseract_geometry.Plane(1, 1, 1, 1) is not None
    assert tesseract_geometry.Sphere(1) is not None
    # Mesh types require vertices/faces - see test_mesh, test_convex_mesh


def test_geometry_box():
    geom = tesseract_geometry.Box(1, 1, 1)

    nptest.assert_almost_equal(geom.getX(), 1)
    nptest.assert_almost_equal(geom.getY(), 1)
    nptest.assert_almost_equal(geom.getZ(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getX(), 1)
    nptest.assert_almost_equal(geom_clone.getY(), 1)
    nptest.assert_almost_equal(geom_clone.getZ(), 1)


def test_geometry_cone():
    geom = tesseract_geometry.Cone(1, 1)

    nptest.assert_almost_equal(geom.getRadius(), 1)
    nptest.assert_almost_equal(geom.getLength(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getRadius(), 1)
    nptest.assert_almost_equal(geom_clone.getLength(), 1)


def test_geometry_cylinder():
    geom = tesseract_geometry.Cylinder(1, 1)

    nptest.assert_almost_equal(geom.getRadius(), 1)
    nptest.assert_almost_equal(geom.getLength(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getRadius(), 1)
    nptest.assert_almost_equal(geom_clone.getLength(), 1)


def test_geometry_capsule():
    geom = tesseract_geometry.Capsule(1, 1)

    nptest.assert_almost_equal(geom.getRadius(), 1)
    nptest.assert_almost_equal(geom.getLength(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getRadius(), 1)
    nptest.assert_almost_equal(geom_clone.getLength(), 1)


def test_geometry_sphere():
    geom = tesseract_geometry.Sphere(1)

    nptest.assert_almost_equal(geom.getRadius(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getRadius(), 1)


def test_geometry_plane():
    geom = tesseract_geometry.Plane(1, 1, 1, 1)

    nptest.assert_almost_equal(geom.getA(), 1)
    nptest.assert_almost_equal(geom.getB(), 1)
    nptest.assert_almost_equal(geom.getC(), 1)
    nptest.assert_almost_equal(geom.getD(), 1)

    geom_clone = geom.clone()
    nptest.assert_almost_equal(geom_clone.getA(), 1)
    nptest.assert_almost_equal(geom_clone.getB(), 1)
    nptest.assert_almost_equal(geom_clone.getC(), 1)
    nptest.assert_almost_equal(geom_clone.getD(), 1)


def test_geometry_load_mesh():
    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/sphere_p25m.stl")
    meshes = tesseract_geometry.createMeshFromPath(mesh_file)
    assert len(meshes) == 1
    assert meshes[0].getFaceCount() == 80
    assert meshes[0].getVertexCount() == 42

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/sphere_p25m.ply")
    meshes = tesseract_geometry.createMeshFromPath(mesh_file)
    assert len(meshes) == 1
    assert meshes[0].getFaceCount() == 80
    assert meshes[0].getVertexCount() == 42

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/sphere_p25m.dae")
    meshes = tesseract_geometry.createMeshFromPath(mesh_file)
    assert len(meshes) == 2
    assert meshes[0].getFaceCount() == 80
    assert meshes[0].getVertexCount() == 42
    assert meshes[1].getFaceCount() == 80
    assert meshes[1].getVertexCount() == 42

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/sphere_p25m.dae")
    meshes = tesseract_geometry.createMeshFromPath(
        mesh_file, np.array((1, 1, 1), dtype=np.float64), False, True
    )
    assert len(meshes) == 1
    assert meshes[0].getFaceCount() == 2 * 80
    assert meshes[0].getVertexCount() == 2 * 42

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/box_2m.ply")
    meshes = tesseract_geometry.createMeshFromPath(
        mesh_file, np.array((1, 1, 1), dtype=np.float64), True, True
    )
    assert len(meshes) == 1
    assert meshes[0].getFaceCount() == 12
    assert meshes[0].getVertexCount() == 8

    mesh_file = os.path.join(TESSERACT_SUPPORT_DIR, "meshes/box_2m.ply")
    meshes = tesseract_geometry.createConvexMeshFromPath(
        mesh_file, np.array((1, 1, 1), dtype=np.float64), False, False
    )
    assert len(meshes) == 1
    assert meshes[0].getFaceCount() == 6
    assert meshes[0].getVertexCount() == 8


def test_mesh():
    vertices = tesseract_common.VectorVector3d()
    vertices.append(np.array([1, 1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))
    vertices.append(np.array([-1, -1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))

    faces = np.array([3, 0, 1, 2, 3, 0, 2, 3], np.int32)

    geom = tesseract_geometry.Mesh(vertices, faces)
    assert len(geom.getVertices()) > 0
    assert len(geom.getFaces()) > 0
    assert geom.getVertexCount() == 4
    assert geom.getFaceCount() == 2

    geom_clone = geom.clone()
    assert len(geom_clone.getVertices()) > 0
    assert len(geom_clone.getFaces()) > 0
    assert geom_clone.getVertexCount() == 4
    assert geom_clone.getFaceCount() == 2


def test_convex_mesh():
    vertices = tesseract_common.VectorVector3d()
    vertices.append(np.array([1, 1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))
    vertices.append(np.array([-1, -1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))

    faces = np.array([4, 0, 1, 2, 3], np.int32)

    geom = tesseract_geometry.ConvexMesh(vertices, faces)
    assert len(geom.getVertices()) > 0
    assert len(geom.getFaces()) > 0
    assert geom.getVertexCount() == 4
    assert geom.getFaceCount() == 1

    geom_clone = geom.clone()
    assert len(geom_clone.getVertices()) > 0
    assert len(geom_clone.getFaces()) > 0
    assert geom_clone.getVertexCount() == 4
    assert geom_clone.getFaceCount() == 1


def test_octree():
    pc = tesseract_geometry.PointCloud()
    pc.addPoint(0.0, 0.0, 0.0)
    pc.addPoint(0.1, 0.0, 0.0)
    pc.addPoint(0.0, 0.1, 0.0)

    ot = tesseract_geometry.createOctree(pc, 0.05, True, True)
    nptest.assert_almost_equal(ot.getResolution(), 0.05)
    assert ot.getNumLeafNodes() == 3

    geom = tesseract_geometry.Octree(
        ot, tesseract_geometry.OctreeSubType.BOX, True, True
    )
    assert geom.getType() == tesseract_geometry.GeometryType.OCTREE
    assert geom.getSubType() == tesseract_geometry.OctreeSubType.BOX
    assert geom.getPruned() is True
    assert geom.calcNumSubShapes() == 3

    geom_clone = geom.clone()
    assert geom_clone.getType() == tesseract_geometry.GeometryType.OCTREE


def test_geometry_utils_and_conversions():
    box = tesseract_geometry.Box(1, 1, 1)
    sphere = tesseract_geometry.Sphere(1)

    assert tesseract_geometry.isIdentical(box, tesseract_geometry.Box(1, 1, 1))
    assert not tesseract_geometry.isIdentical(box, sphere)

    origin = tesseract_common.Isometry3d()
    verts = tesseract_geometry.extractVertices(box, origin)
    assert len(verts) == 8

    mesh = tesseract_geometry.toTriangleMesh(box, 0.01, origin)
    assert mesh.getVertexCount() == 8
    assert mesh.getFaceCount() == 12

    sphere_mesh = tesseract_geometry.toTriangleMesh(sphere, 0.05, origin)
    assert sphere_mesh.getVertexCount() > 0
    assert sphere_mesh.getFaceCount() > 0


def test_octree_direct_construction():
    ot = tesseract_geometry.OcTree(0.05)
    ot.updateNode(0.0, 0.0, 0.0, True)
    ot.updateNode(0.1, 0.0, 0.0, True)
    ot.updateInnerOccupancy()
    ot.toMaxLikelihood()
    assert ot.size() > 0

    geom = tesseract_geometry.Octree(
        ot, tesseract_geometry.OctreeSubType.SPHERE_INSIDE
    )
    assert geom.getSubType() == tesseract_geometry.OctreeSubType.SPHERE_INSIDE
    assert geom.getPruned() is False


# --------------------------------------------------------------------------------------------
# SignedDistanceField
# --------------------------------------------------------------------------------------------

SDF_DIMS = np.array([9, 9, 9], dtype=np.int32)
SDF_MIN = np.array([-1.0, -1.0, -1.0])
SDF_MAX = np.array([1.0, 1.0, 1.0])
SDF_RADIUS = 0.5


def _sphere(point):
    """Signed distance to a sphere at the origin, negative inside."""
    return float(np.linalg.norm(point) - SDF_RADIUS)


def _sphere_batched(points):
    """Batched form of :func:`_sphere` - one call for all (N, 3) sample points."""
    return np.linalg.norm(points, axis=1) - SDF_RADIUS


def _sphere_grid():
    """The sphere field sampled on the lattice, as a 3D array indexed [i, j, k]."""
    axes = [np.linspace(SDF_MIN[i], SDF_MAX[i], SDF_DIMS[i]) for i in range(3)]
    x, y, z = np.meshgrid(*axes, indexing="ij")
    return np.sqrt(x**2 + y**2 + z**2) - SDF_RADIUS


def test_signed_distance_field_from_grid():
    grid = _sphere_grid()
    geom = tesseract_geometry.SignedDistanceField(
        SDF_MIN, SDF_MAX, SDF_DIMS, grid.ravel(order="F")
    )

    assert geom.getType() == tesseract_geometry.GeometryType.SIGNED_DISTANCE_FIELD
    nptest.assert_allclose(geom.getDomainMin(), SDF_MIN)
    nptest.assert_allclose(geom.getDomainMax(), SDF_MAX)
    nptest.assert_array_equal(geom.getDimensions(), SDF_DIMS)
    nptest.assert_allclose(geom.getScale(), [1, 1, 1])
    assert geom.isDiscretized()

    # distances are flat and x-fastest, so they reshape back with Fortran order
    assert geom.getDistances().shape == (int(np.prod(SDF_DIMS)),)
    nptest.assert_allclose(geom.getDistances().reshape(SDF_DIMS, order="F"), grid)

    # centre of the sphere is one radius inside the surface
    nptest.assert_almost_equal(geom.getDistance(np.zeros(3)), -SDF_RADIUS)

    geom_clone = geom.clone()
    assert geom_clone.getType() == tesseract_geometry.GeometryType.SIGNED_DISTANCE_FIELD
    nptest.assert_array_equal(geom_clone.getDistances(), geom.getDistances())
    assert tesseract_geometry.isIdentical(geom, geom_clone)


def test_signed_distance_field_scale():
    grid = _sphere_grid().ravel(order="F")
    geom = tesseract_geometry.SignedDistanceField(
        SDF_MIN, SDF_MAX, SDF_DIMS, grid, np.array([2.0, 2.0, 2.0])
    )
    nptest.assert_allclose(geom.getScale(), [2, 2, 2])


def test_signed_distance_field_invalid():
    grid = _sphere_grid().ravel(order="F")

    # distances must cover the whole grid
    with pytest.raises(RuntimeError):
        tesseract_geometry.SignedDistanceField(SDF_MIN, SDF_MAX, SDF_DIMS, np.zeros(5))

    # every axis needs at least two samples
    with pytest.raises(RuntimeError):
        tesseract_geometry.SignedDistanceField(
            SDF_MIN, SDF_MAX, np.array([1, 9, 9], dtype=np.int32), np.zeros(81)
        )

    # domain max must exceed domain min
    with pytest.raises(RuntimeError):
        tesseract_geometry.SignedDistanceField(SDF_MAX, SDF_MIN, SDF_DIMS, grid)


def test_create_discrete_signed_distance_field():
    expected = _sphere_grid().ravel(order="F")

    per_point = tesseract_geometry.createDiscreteSignedDistanceField(
        _sphere, SDF_MIN, SDF_MAX, SDF_DIMS
    )
    batched = tesseract_geometry.createDiscreteSignedDistanceField(
        _sphere_batched, SDF_MIN, SDF_MAX, SDF_DIMS, batched=True
    )

    for geom in (per_point, batched):
        assert geom.getType() == tesseract_geometry.GeometryType.SIGNED_DISTANCE_FIELD
        assert geom.isDiscretized()
        nptest.assert_allclose(geom.getDistances(), expected)

    # both paths visit exactly the same lattice points
    nptest.assert_array_equal(per_point.getDistances(), batched.getDistances())
    assert tesseract_geometry.isIdentical(per_point, batched)


def test_create_discrete_signed_distance_field_drops_the_sampler():
    """The eager field must be pure data: no reference back to the Python callable.

    A retained callable would take the GIL on every collision query and, via the callable's
    __globals__, put the geometry in a reference cycle the GC cannot see through.
    """
    sentinel = []

    def sampler(point):
        sentinel.append(1)
        return _sphere(point)

    geom = tesseract_geometry.createDiscreteSignedDistanceField(
        sampler, SDF_MIN, SDF_MAX, SDF_DIMS
    )
    called_during_construction = len(sentinel)
    assert called_during_construction == int(np.prod(SDF_DIMS))

    # querying the finished field interpolates the grid instead of calling back into Python
    geom.getDistance(np.array([0.137, -0.42, 0.061]))
    geom.getDistances()
    assert len(sentinel) == called_during_construction


def test_create_signed_distance_field_lazy():
    lazy = tesseract_geometry.createSignedDistanceField(
        _sphere_batched, SDF_MIN, SDF_MAX, SDF_DIMS, batched=True
    )
    assert not lazy.isDiscretized()

    # a lazy field evaluates the sampler exactly, rather than interpolating the lattice
    off_grid = np.array([0.137, -0.42, 0.061])
    nptest.assert_almost_equal(lazy.getDistance(off_grid), _sphere(off_grid))

    grid_backed = tesseract_geometry.createDiscreteSignedDistanceField(
        _sphere_batched, SDF_MIN, SDF_MAX, SDF_DIMS, batched=True
    )
    assert abs(grid_backed.getDistance(off_grid) - _sphere(off_grid)) > 1e-6

    lazy.discretize()
    assert lazy.isDiscretized()
    nptest.assert_allclose(lazy.getDistances(), _sphere_grid().ravel(order="F"))


def test_create_signed_distance_field_discretizes_on_access():
    lazy = tesseract_geometry.createSignedDistanceField(
        _sphere, SDF_MIN, SDF_MAX, SDF_DIMS
    )
    assert not lazy.isDiscretized()
    nptest.assert_allclose(lazy.getDistances(), _sphere_grid().ravel(order="F"))
    assert lazy.isDiscretized()


def test_signed_distance_field_sampler_errors():
    # a batched sampler must return one distance per point
    with pytest.raises(RuntimeError):
        tesseract_geometry.createDiscreteSignedDistanceField(
            lambda points: np.zeros(3), SDF_MIN, SDF_MAX, SDF_DIMS, batched=True
        )

    # an exception raised inside the sampler reaches the caller unchanged
    def boom(point):
        raise ValueError("sampler exploded")

    with pytest.raises(ValueError, match="sampler exploded"):
        tesseract_geometry.createDiscreteSignedDistanceField(
            boom, SDF_MIN, SDF_MAX, SDF_DIMS
        )


@pytest.mark.parametrize(
    ("write", "read"),
    [
        (
            tesseract_geometry.writeSignedDistanceFieldVDB,
            tesseract_geometry.readSignedDistanceFieldVDB,
        ),
        (
            tesseract_geometry.writeSignedDistanceFieldNVDB,
            tesseract_geometry.readSignedDistanceFieldNVDB,
        ),
    ],
    ids=["vdb", "nvdb"],
)
def test_signed_distance_field_vdb_round_trip(write, read):
    grid = _sphere_grid().ravel(order="F")
    geom = tesseract_geometry.SignedDistanceField(SDF_MIN, SDF_MAX, SDF_DIMS, grid)

    blob = write(geom)
    assert isinstance(blob, bytes)
    assert len(blob) > 0

    restored = read(blob)
    nptest.assert_array_equal(restored.getDimensions(), SDF_DIMS)
    nptest.assert_allclose(restored.getDomainMin(), SDF_MIN, atol=1e-6)
    nptest.assert_allclose(restored.getDomainMax(), SDF_MAX, atol=1e-6)
    nptest.assert_allclose(restored.getDistances(), grid, atol=1e-5)

    # scale is not carried in the grid and is supplied on read
    scaled = read(blob, np.array([3.0, 3.0, 3.0]))
    nptest.assert_allclose(scaled.getScale(), [3, 3, 3])
