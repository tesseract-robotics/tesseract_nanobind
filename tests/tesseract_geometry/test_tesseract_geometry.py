import os

import numpy as np
import numpy.testing as nptest

from tesseract_robotics import tesseract_common, tesseract_geometry


def test_geometry_instantiation():
    # Test that all basic geometry types can be instantiated
    assert tesseract_geometry.Box(1, 1, 1) is not None
    assert tesseract_geometry.Cone(1, 1) is not None
    assert tesseract_geometry.Cylinder(1, 1) is not None
    assert tesseract_geometry.Capsule(1, 1) is not None
    assert tesseract_geometry.Plane(1, 1, 1, 1) is not None
    assert tesseract_geometry.Sphere(1) is not None
    # Mesh types require vertices/faces - see test_mesh, test_convex_mesh, test_sdf_mesh


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


def test_sdf_mesh():
    vertices = tesseract_common.VectorVector3d()
    vertices.append(np.array([1, 1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))
    vertices.append(np.array([-1, -1, 0], dtype=np.float64))
    vertices.append(np.array([1, -1, 0], dtype=np.float64))

    faces = np.array([3, 0, 1, 2, 3, 0, 2, 3], np.int32)

    geom = tesseract_geometry.SDFMesh(vertices, faces)
    assert len(geom.getVertices()) > 0
    assert len(geom.getFaces()) > 0
    assert geom.getVertexCount() == 4
    assert geom.getFaceCount() == 2

    geom_clone = geom.clone()
    assert len(geom_clone.getVertices()) > 0
    assert len(geom_clone.getFaces()) > 0
    assert geom_clone.getVertexCount() == 4
    assert geom_clone.getFaceCount() == 2
