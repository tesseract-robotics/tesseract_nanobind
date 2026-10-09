import gc
import os
import uuid
from pathlib import Path

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import tesseract_common, tesseract_geometry, tesseract_urdf


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

    geom = tesseract_geometry.Octree(ot, tesseract_geometry.OctreeSubType.BOX, True, True)
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

    geom = tesseract_geometry.Octree(ot, tesseract_geometry.OctreeSubType.SPHERE_INSIDE)
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


def test_geometry_uuid_is_canonical_string():
    s = tesseract_geometry.Box(1, 1, 1).getUUID()
    assert isinstance(s, str)
    assert str(uuid.UUID(s)) == s


def test_geometry_equality_is_type_and_uuid():
    a = tesseract_geometry.Box(1, 1, 1)
    b = tesseract_geometry.Box(1, 1, 1)
    assert a != b
    b.setUUID(a.getUUID())
    assert a == b
    assert b.getUUID() == a.getUUID()

    clone = a.clone()
    assert a != clone
    clone.setUUID(a.getUUID())
    assert a == clone


def test_geometry_set_uuid_refuses_malformed_string():
    geom = tesseract_geometry.Box(1, 1, 1)
    before = geom.getUUID()
    with pytest.raises(ValueError, match="not-a-uuid"):
        geom.setUUID("not-a-uuid")
    assert geom.getUUID() == before


# Two triangles over a unit square: faces is [count, i0, i1, i2] per face.
SQUARE_VERTICES = [
    np.array([0.0, 0.0, 0.0]),
    np.array([1.0, 0.0, 0.0]),
    np.array([1.0, 1.0, 0.0]),
    np.array([0.0, 1.0, 0.0]),
]
SQUARE_FACES = np.array([3, 0, 1, 2, 3, 0, 2, 3], np.int32)
SQUARE_FACE_COUNT = 2
# The 8-byte PNG signature: MeshTexture stores its image resource without decoding it.
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

MESH_CLASSES = [
    tesseract_geometry.Mesh,
    tesseract_geometry.SDFMesh,
    tesseract_geometry.PolygonMesh,
    tesseract_geometry.ConvexMesh,
]


def _mesh_texture():
    uvs = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0]), np.array([0.0, 1.0])]
    return tesseract_geometry.MeshTexture(
        tesseract_common.BytesResource("tex.png", PNG_SIGNATURE), uvs
    )


def _full_mesh_kwargs():
    return dict(
        resource=tesseract_common.BytesResource("square.stl", b"solid square"),
        scale=np.array([2.0, 3.0, 4.0]),
        normals=[np.array([0.0, 0.0, 1.0])] * 4,
        vertex_colors=[np.array([1.0, 0.0, 0.0, 1.0])] * 4,
        mesh_material=tesseract_geometry.MeshMaterial(
            np.array([0.1, 0.2, 0.3, 1.0]), 0.4, 0.5, np.array([0.0, 0.0, 0.0, 1.0])
        ),
        mesh_textures=[_mesh_texture()],
    )


def _assert_full_mesh_round_trip(geom, kwargs):
    assert geom.getResource().getUrl() == "square.stl"
    nptest.assert_array_equal(geom.getScale(), kwargs["scale"])
    nptest.assert_array_equal(np.array(geom.getNormals()), np.array(kwargs["normals"]))
    nptest.assert_array_equal(np.array(geom.getVertexColors()), np.array(kwargs["vertex_colors"]))
    nptest.assert_array_equal(geom.getMaterial().getBaseColorFactor(), [0.1, 0.2, 0.3, 1.0])
    assert geom.getMaterial().getMetallicFactor() == 0.4
    (texture,) = geom.getTextures()
    assert texture.getTextureImage().getUrl() == "tex.png"
    assert len(texture.getUVs()) == 4


@pytest.mark.parametrize("cls", MESH_CLASSES, ids=lambda c: c.__name__)
def test_mesh_full_constructor_round_trips(cls):
    kwargs = _full_mesh_kwargs()
    geom = cls(SQUARE_VERTICES, SQUARE_FACES, **kwargs)
    assert geom.getFaceCount() == SQUARE_FACE_COUNT
    _assert_full_mesh_round_trip(geom, kwargs)


@pytest.mark.parametrize("cls", MESH_CLASSES, ids=lambda c: c.__name__)
def test_mesh_face_count_constructor_round_trips(cls):
    kwargs = _full_mesh_kwargs()
    geom = cls(SQUARE_VERTICES, SQUARE_FACES, SQUARE_FACE_COUNT, **kwargs)
    assert geom.getFaceCount() == cls(SQUARE_VERTICES, SQUARE_FACES).getFaceCount()
    _assert_full_mesh_round_trip(geom, kwargs)


@pytest.mark.parametrize("cls", MESH_CLASSES, ids=lambda c: c.__name__)
def test_mesh_minimal_constructor_defaults(cls):
    geom = cls(SQUARE_VERTICES, SQUARE_FACES)
    assert geom.getResource() is None
    nptest.assert_array_equal(geom.getScale(), [1.0, 1.0, 1.0])
    assert geom.getNormals() is None
    assert geom.getVertexColors() is None
    assert geom.getMaterial() is None
    assert geom.getTextures() is None


@pytest.mark.parametrize("cls", MESH_CLASSES, ids=lambda c: c.__name__)
@pytest.mark.parametrize("face_count", [SQUARE_FACE_COUNT - 1, SQUARE_FACE_COUNT + 1])
def test_mesh_wrong_face_count_raises(cls, face_count):
    with pytest.raises(tesseract_geometry.MeshFaceCountError, match=f"face_count {face_count}"):
        cls(SQUARE_VERTICES, SQUARE_FACES, face_count)
    assert issubclass(tesseract_geometry.MeshFaceCountError, ValueError)


def test_polygon_mesh_type_argument():
    G = tesseract_geometry.GeometryType
    assert tesseract_geometry.PolygonMesh(SQUARE_VERTICES, SQUARE_FACES).getType() == G.POLYGON_MESH
    geom = tesseract_geometry.PolygonMesh(SQUARE_VERTICES, SQUARE_FACES, type=G.MESH)
    assert geom.getType() == G.MESH
    geom = tesseract_geometry.PolygonMesh(
        SQUARE_VERTICES, SQUARE_FACES, SQUARE_FACE_COUNT, type=G.MESH
    )
    assert geom.getType() == G.MESH


def test_mesh_texture_constructor():
    uvs = [np.array([0.0, 0.0]), np.array([1.0, 0.5])]
    texture = tesseract_geometry.MeshTexture(
        tesseract_common.BytesResource("tex.png", PNG_SIGNATURE), uvs
    )
    assert texture.getTextureImage().getUrl() == "tex.png"
    nptest.assert_array_equal(np.array(texture.getUVs()), np.array(uvs))
    with pytest.raises(TypeError):
        tesseract_geometry.MeshTexture(None, uvs)
    with pytest.raises(TypeError):
        tesseract_geometry.MeshTexture(
            tesseract_common.BytesResource("tex.png", PNG_SIGNATURE), None
        )


def test_convex_mesh_creation_method():
    CreationMethod = tesseract_geometry.ConvexMesh.CreationMethod
    assert {m.name for m in CreationMethod} == {"DEFAULT", "MESH", "CONVERTED"}
    geom = tesseract_geometry.ConvexMesh(SQUARE_VERTICES, SQUARE_FACES)
    assert geom.getCreationMethod() == CreationMethod.DEFAULT
    geom.setCreationMethod(CreationMethod.MESH)
    assert geom.getCreationMethod() == CreationMethod.MESH
    assert not hasattr(tesseract_geometry, "CreationMethod_MESH")


def test_convex_mesh_equality_compares_creation_method():
    CreationMethod = tesseract_geometry.ConvexMesh.CreationMethod
    a = tesseract_geometry.ConvexMesh(SQUARE_VERTICES, SQUARE_FACES)
    b = tesseract_geometry.ConvexMesh(SQUARE_VERTICES, SQUARE_FACES)
    b.setUUID(a.getUUID())
    assert a == b
    b.setCreationMethod(CreationMethod.CONVERTED)
    assert a != b


def test_convex_mesh_clone_resets_creation_method():
    """Pins upstream: ConvexMesh::clone() calls a constructor, so the clone is DEFAULT (convex_mesh.cpp:77 @ 0.35.0)."""
    CreationMethod = tesseract_geometry.ConvexMesh.CreationMethod
    geom = tesseract_geometry.ConvexMesh(SQUARE_VERTICES, SQUARE_FACES)
    geom.setCreationMethod(CreationMethod.MESH)
    assert geom.clone().getCreationMethod() == CreationMethod.DEFAULT


def test_urdf_make_convex_reports_converted():
    stl = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "meshes/sphere_p25m.stl"
    urdf = f"""
<robot name="convex" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <link name="base">
    <collision>
      <geometry>
        <mesh filename="file://{stl}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
    scene = tesseract_urdf.parseURDFString(urdf, tesseract_common.GeneralResourceLocator())
    (collision,) = scene.getLink("base").collision
    geom = collision.geometry
    assert geom.getType() == tesseract_geometry.GeometryType.CONVEX_MESH
    assert geom.getCreationMethod() == tesseract_geometry.ConvexMesh.CreationMethod.CONVERTED


# Whether the class's own operator== compares scale: PolygonMesh, Mesh and ConvexMesh do through
# PolygonMesh::operator== (polygon_mesh.cpp:111-120); SDFMesh::operator== is Geometry's alone
# (sdf_mesh.cpp:80-85 @ 0.35.0).
COMPARES_SCALE = {
    tesseract_geometry.Mesh: True,
    tesseract_geometry.SDFMesh: False,
    tesseract_geometry.PolygonMesh: True,
    tesseract_geometry.ConvexMesh: True,
}


@pytest.mark.parametrize("cls", MESH_CLASSES, ids=lambda c: c.__name__)
def test_mesh_equality_is_the_class_operator(cls):
    """Each mesh class binds its own operator==, not Geometry's type-and-UUID comparison."""
    a = cls(SQUARE_VERTICES, SQUARE_FACES)
    b = cls(SQUARE_VERTICES, SQUARE_FACES)
    b.setUUID(a.getUUID())
    assert a == b
    c = cls(SQUARE_VERTICES, SQUARE_FACES, scale=np.array([2.0, 2.0, 2.0]))
    c.setUUID(a.getUUID())
    assert (a != c) is COMPARES_SCALE[cls]


SPHERE_STL = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "meshes/sphere_p25m.stl"


@pytest.mark.parametrize("triangulate", [True, False])
def test_create_mesh_from_bytes_matches_path(triangulate):
    data = SPHERE_STL.read_bytes()
    from_bytes = tesseract_geometry.createMeshFromBytes(
        "sphere_p25m.stl", data, triangulate=triangulate
    )
    from_path = tesseract_geometry.createMeshFromPath(str(SPHERE_STL), triangulate=triangulate)
    assert [(m.getVertexCount(), m.getFaceCount()) for m in from_bytes] == [
        (m.getVertexCount(), m.getFaceCount()) for m in from_path
    ]


def test_create_mesh_from_bytes_defaults_match_path():
    """triangulate defaults to True like the Path / Resource siblings (plan Q5b), not the header's false."""
    data = SPHERE_STL.read_bytes()
    from_bytes = tesseract_geometry.createMeshFromBytes("sphere_p25m.stl", data)
    from_path = tesseract_geometry.createMeshFromPath(str(SPHERE_STL))
    assert [m.getFaceCount() for m in from_bytes] == [m.getFaceCount() for m in from_path]


@pytest.mark.parametrize(
    ("fn", "cls"),
    [
        ("createMeshFromBytes", tesseract_geometry.Mesh),
        ("createConvexMeshFromBytes", tesseract_geometry.ConvexMesh),
        ("createSDFMeshFromBytes", tesseract_geometry.SDFMesh),
    ],
)
def test_create_mesh_from_bytes_instance_types(fn, cls):
    meshes = getattr(tesseract_geometry, fn)("sphere_p25m.stl", SPHERE_STL.read_bytes())
    assert len(meshes) == 1
    assert type(meshes[0]) is cls
    assert fn in tesseract_geometry.__all__


def test_create_mesh_from_bytes_outlives_the_buffer():
    data = SPHERE_STL.read_bytes()
    meshes = tesseract_geometry.createMeshFromBytes("sphere_p25m.stl", data)
    expected = np.array(meshes[0].getVertices())
    del data
    gc.collect()
    nptest.assert_array_equal(np.array(meshes[0].getVertices()), expected)


MATERIAL_DAE = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "meshes/tesseract_material_mesh.dae"
# The flags the URDF parser passes for a visual mesh (urdf/src/mesh.cpp:84-85 @ 0.35.0).
URDF_VISUAL_FLAGS = dict(flatten=True, normals=True, vertex_colors=True, material_and_texture=True)


def _urdf_visual_meshes(path):
    urdf = f"""
<robot name="visual" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <joint name="mesh_joint" type="fixed">
    <parent link="world"/>
    <child link="mesh_link"/>
  </joint>
  <link name="mesh_link">
    <visual>
      <geometry>
        <mesh filename="file://{path}"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
    scene = tesseract_urdf.parseURDFString(urdf, tesseract_common.GeneralResourceLocator())
    (visual,) = scene.getLink("mesh_link").visual
    return visual.geometry.getMeshes()


def _mesh_summary(meshes):
    return [
        (
            m.getFaceCount(),
            m.getVertexCount(),
            len(m.getNormals()),
            tuple(np.round(m.getMaterial().getBaseColorFactor(), 6)),
        )
        for m in meshes
    ]


def test_create_mesh_from_path_flags_match_urdf_visual():
    meshes = tesseract_geometry.createMeshFromPath(str(MATERIAL_DAE), **URDF_VISUAL_FLAGS)
    assert _mesh_summary(meshes) == _mesh_summary(_urdf_visual_meshes(MATERIAL_DAE))


def test_create_mesh_flags_default_false():
    for m in tesseract_geometry.createMeshFromPath(str(MATERIAL_DAE)):
        assert m.getNormals() is None
        assert m.getVertexColors() is None
        assert m.getMaterial() is None
        assert m.getTextures() is None


def test_create_mesh_from_resource_and_bytes_flags_match_path():
    expected = _mesh_summary(
        tesseract_geometry.createMeshFromPath(str(MATERIAL_DAE), **URDF_VISUAL_FLAGS)
    )
    resource = tesseract_common.BytesResource(MATERIAL_DAE.name, MATERIAL_DAE.read_bytes())
    from_resource = tesseract_geometry.createMeshFromResource(resource, **URDF_VISUAL_FLAGS)
    assert _mesh_summary(from_resource) == expected
    from_bytes = tesseract_geometry.createMeshFromBytes(
        MATERIAL_DAE.name, MATERIAL_DAE.read_bytes(), **URDF_VISUAL_FLAGS
    )
    assert _mesh_summary(from_bytes) == expected


@pytest.mark.parametrize(
    "fn",
    [
        "createConvexMeshFromPath",
        "createSDFMeshFromPath",
        "createConvexMeshFromResource",
        "createSDFMeshFromResource",
    ],
)
def test_create_mesh_instances_accept_flags(fn):
    source = (
        str(SPHERE_STL)
        if fn.endswith("Path")
        else tesseract_common.BytesResource(SPHERE_STL.name, SPHERE_STL.read_bytes())
    )
    meshes = getattr(tesseract_geometry, fn)(
        source, normals=True, vertex_colors=False, material_and_texture=False
    )
    assert len(meshes) == 1
