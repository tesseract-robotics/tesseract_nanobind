"""visual_to_trimesh: tesseract geometry in the visual frame."""

import json
import struct
from pathlib import Path

import numpy as np
import pytest
from trimesh.visual.texture import TextureVisuals

from tesseract_robotics import tesseract_geometry as tg
from tesseract_robotics.examples.pointcloud_octree_collision_example import build_octree_obstacle
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_scene_graph import Material, Visual
from tesseract_robotics.viewer.errors import UnsupportedGeometryError
from tesseract_robotics.viewer.geometry import RGBA_BYTE_MAX, visual_to_trimesh

MESH_URDF = """
<robot name="mesh_viewer" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="dae_link">
    <visual><geometry>
      <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.dae"/>
    </geometry></visual>
  </link>
  <link name="stl_link">
    <visual><geometry>
      <mesh filename="package://tesseract/support/meshes/sphere_p25m.stl" scale="{scale}"/>
    </geometry></visual>
  </link>
  <joint name="j_dae" type="fixed"><parent link="world"/><child link="dae_link"/></joint>
  <joint name="j_stl" type="fixed"><parent link="world"/><child link="stl_link"/></joint>
</robot>
"""


# Unit-normal agreement [-]: float32 storage in tesseract (~6e-8) with headroom.
NORMAL_ATOL = 1e-6


def _visual(geometry, material=None):
    visual = Visual()
    visual.geometry = geometry
    if material is not None:
        visual.material = material
    return visual


def _mesh_env(scale=(1.0, 1.0, 1.0)):
    env = Environment()
    urdf = MESH_URDF.format(scale=" ".join(str(s) for s in scale))
    assert env.init(urdf, GeneralResourceLocator())
    return env


def _base_color(mesh):
    return np.asarray(mesh.visual.material.baseColorFactor)


def test_box_bounds():
    (mesh,) = visual_to_trimesh(_visual(tg.Box(1.0, 2.0, 3.0)), "l")
    np.testing.assert_allclose(mesh.bounds, [[-0.5, -1.0, -1.5], [0.5, 1.0, 1.5]])


def test_cylinder_z_aligned_and_centred():
    (mesh,) = visual_to_trimesh(_visual(tg.Cylinder(0.1, 2.0)), "l")
    np.testing.assert_allclose(mesh.bounds[:, 2], [-1.0, 1.0])


def test_cone_centred_on_origin():
    (mesh,) = visual_to_trimesh(_visual(tg.Cone(0.1, 2.0)), "l")
    np.testing.assert_allclose(mesh.bounds[:, 2], [-1.0, 1.0])


def test_capsule_length_is_cylinder_part():
    (mesh,) = visual_to_trimesh(_visual(tg.Capsule(0.1, 1.0)), "l")
    np.testing.assert_allclose(mesh.bounds[:, 2], [-0.6, 0.6])


def test_sphere_radius():
    (mesh,) = visual_to_trimesh(_visual(tg.Sphere(0.25)), "l")
    np.testing.assert_allclose(np.linalg.norm(mesh.vertices, axis=1), 0.25)


def test_mesh_scale_applied():
    def extents(scale):
        (visual,) = _mesh_env(scale).getLink("stl_link").visual
        (mesh,) = visual_to_trimesh(visual, "stl_link")
        return mesh.extents

    np.testing.assert_allclose(extents((2.0, 2.0, 2.0)), 2.0 * extents((1.0, 1.0, 1.0)))


def _stl_mesh(scale):
    (visual,) = _mesh_env(scale).getLink("stl_link").visual
    (mesh,) = visual_to_trimesh(visual, "stl_link")
    return visual.geometry, mesh


def test_mesh_keeps_tesseract_normals():
    """Authored normals, not face-averaged ones: on IRB2400 link_6 they differ by ~0.07."""
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), locator)
    (visual,) = env.getLink("link_6").visual
    (mesh,) = visual_to_trimesh(visual, "link_6")
    np.testing.assert_allclose(
        mesh.vertex_normals,
        np.asarray(visual.geometry.getNormals()).reshape(-1, 3),
        atol=NORMAL_ATOL,
    )


# Two triangles; the second is degenerate (zero area), so its lone vertex has no normal.
TWO_TRIANGLES = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [2.0, 0.0, 0.0]]
TWO_TRIANGLE_FACES = np.array([3, 0, 1, 2, 3, 1, 3, 1], dtype=np.int32)


SCALED_DAE_URDF = """
<robot name="scaled_dae" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="dae_link">
    <visual><geometry>
      <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.dae" scale="{scale}"/>
    </geometry></visual>
  </link>
  <joint name="j_dae" type="fixed"><parent link="world"/><child link="dae_link"/></joint>
</robot>
"""


def _raw_dae_normals(scale):
    env = Environment()
    assert env.init(SCALED_DAE_URDF.format(scale=scale), GeneralResourceLocator())
    (visual,) = env.getLink("dae_link").visual
    return np.vstack(
        [np.asarray(m.getNormals()).reshape(-1, 3) for m in visual.geometry.getMeshes()]
    )


def test_parser_stores_normals_scaled_like_vertices():
    """The invariant _scaled_normals undoes: tesseract stores S n, not n."""
    scale = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(_raw_dae_normals("1 2 3"), _raw_dae_normals("1 1 1") * scale)


def test_mesh_normals_follow_non_uniform_scale():
    """Normals transform by the inverse scale: n' = normalize(S^-1 n)."""
    scale = np.array([1.0, 2.0, 3.0])
    # Authored smooth normals: one per vertex, not per face, so they are kept.
    n = np.array([[1.0, 1.0, 1.0], [1.0, -1.0, 2.0], [-1.0, 2.0, 1.0], [2.0, 1.0, -1.0]])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    mesh = tg.Mesh(TWO_TRIANGLES, TWO_TRIANGLE_FACES, scale=scale, normals=list(n * scale))
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    expected = n / scale
    expected /= np.linalg.norm(expected, axis=1, keepdims=True)
    np.testing.assert_allclose(result.vertex_normals, expected, atol=NORMAL_ATOL)


def _glb_attributes(mesh):
    glb = mesh.export(file_type="glb")
    header_len = struct.unpack_from("<I", glb, 12)[0]
    return json.loads(glb[20 : 20 + header_len])["meshes"][0]["primitives"][0]["attributes"]


def test_all_zero_normals_mean_no_normals():
    """A file without normals (car_seat end_effector_simple.stl) comes back all zero; it is
    auto-smoothed like a faceted file."""
    mesh = tg.Mesh(TWO_TRIANGLES, TWO_TRIANGLE_FACES, normals=[[0.0, 0.0, 0.0]] * 4)
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    assert "NORMAL" in _glb_attributes(result)
    assert np.isfinite(result.vertex_normals).all()


def test_zero_normal_on_degenerate_face_stays_finite():
    """NaN normals cost the browser its WebGL context; a zero normal must not become one."""
    normals = [[0.0, 0.0, 1.0]] * 3 + [[0.0, 0.0, 0.0]]
    mesh = tg.Mesh(TWO_TRIANGLES, TWO_TRIANGLE_FACES, normals=normals)
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    assert np.isfinite(result.vertex_normals).all()
    np.testing.assert_allclose(result.vertex_normals[:3], [[0.0, 0.0, 1.0]] * 3)


def test_mesh_normals_reach_the_glb():
    _, mesh = _stl_mesh((1.0, 1.0, 1.0))
    glb = mesh.export(file_type="glb")
    header_len = struct.unpack_from("<I", glb, 12)[0]
    gltf = json.loads(glb[20 : 20 + header_len])
    assert "NORMAL" in gltf["meshes"][0]["primitives"][0]["attributes"]


def test_compound_mesh_one_per_submesh():
    (visual,) = _mesh_env().getLink("dae_link").visual
    meshes = visual_to_trimesh(visual, "dae_link")
    assert len(meshes) == len(visual.geometry.getMeshes())


def test_mesh_keeps_own_material_over_default():
    (visual,) = _mesh_env().getLink("dae_link").visual
    sub = visual.geometry.getMeshes()[1]
    expected = np.round(sub.getMaterial().getBaseColorFactor() * RGBA_BYTE_MAX)
    np.testing.assert_array_equal(_base_color(visual_to_trimesh(visual, "dae_link")[1]), expected)


def test_visual_color_overrides_mesh_material():
    (visual,) = _mesh_env().getLink("dae_link").visual
    visual.material = Material("tint")
    visual.material.color = np.array([1.0, 0.0, 0.0, 1.0])
    for mesh in visual_to_trimesh(visual, "dae_link"):
        np.testing.assert_array_equal(_base_color(mesh), [255, 0, 0, 255])


def test_textured_mesh_keeps_uv():
    (visual,) = _mesh_env().getLink("dae_link").visual
    textured = [
        m
        for m, sub in zip(visual_to_trimesh(visual, "dae_link"), visual.geometry.getMeshes())
        if sub.getTextures()
    ]
    assert textured
    assert isinstance(textured[0].visual, TextureVisuals)
    assert textured[0].visual.uv.shape == (len(textured[0].vertices), 2)
    assert textured[0].visual.material.baseColorTexture is not None


def test_octree_voxel_count():
    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.5, 0.5]])
    octree = build_octree_obstacle(points, resolution=0.1)
    (mesh,) = visual_to_trimesh(_visual(octree), "pcd_octree")
    assert len(mesh.faces) == 12 * octree.calcNumSubShapes()


def test_unsupported_raises():
    with pytest.raises(UnsupportedGeometryError, match="Plane.*'floor'"):
        visual_to_trimesh(_visual(tg.Plane(0.0, 0.0, 1.0, 0.0)), "floor")


def _folded_strip(fold_deg):
    """Two triangles hinged on the x axis, unshared vertices, facet normals (an STL)."""
    t = np.radians(fold_deg)
    a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1.0, 0.0]])
    b = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.5, -np.cos(t), np.sin(t)]])
    vertices = np.vstack([a, b])
    normals = []
    for tri in (a, b):
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        normals += [n / np.linalg.norm(n)] * 3
    faces = np.array([3, 0, 1, 2, 3, 3, 4, 5], dtype=np.int32)
    return tg.Mesh(list(vertices), faces, normals=normals), np.array(normals[::3])


def _normals_at_hinge(result):
    on_hinge = np.isclose(result.vertices[:, 1], 0.0) & np.isclose(result.vertices[:, 2], 0.0)
    return result.vertex_normals[on_hinge]


def test_faceted_mesh_is_smooth_shaded_below_the_crease_angle():
    mesh, (n_a, n_b) = _folded_strip(fold_deg=10.0)
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    hinge = _normals_at_hinge(result)
    expected = (n_a + n_b) / np.linalg.norm(n_a + n_b)
    np.testing.assert_allclose(hinge, np.tile(expected, (len(hinge), 1)), atol=NORMAL_ATOL)


def test_faceted_mesh_keeps_creases():
    mesh, (n_a, n_b) = _folded_strip(fold_deg=90.0)
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    hinge = _normals_at_hinge(result)
    for n in hinge:
        assert np.isclose(abs(n @ n_a), 1.0) or np.isclose(abs(n @ n_b), 1.0)


def test_materials_are_double_sided():
    """Sheet-metal meshes (car_seat car.stl) mix windings; one-sided materials punch holes."""
    mesh, _ = _folded_strip(fold_deg=10.0)
    (result,) = visual_to_trimesh(_visual(mesh), "l")
    assert result.visual.material.doubleSided


# Angle-weighted normals vs the true radial on a level-3 icosphere [-]: 4.6e-3 measured
# (0.26 deg), the facet averaging itself.
SPHERE_NORMAL_ATOL = 5e-3


def test_sphere_has_smooth_radial_normals():
    (mesh,) = visual_to_trimesh(_visual(tg.Sphere(0.25)), "l")
    assert "NORMAL" in _glb_attributes(mesh)
    np.testing.assert_allclose(mesh.vertex_normals, mesh.vertices / 0.25, atol=SPHERE_NORMAL_ATOL)


def test_cylinder_side_is_smooth_and_cap_edges_stay_creased():
    (mesh,) = visual_to_trimesh(_visual(tg.Cylinder(0.1, 2.0)), "l")
    assert "NORMAL" in _glb_attributes(mesh)
    z_axis_component = np.abs(mesh.vertex_normals[:, 2])
    # Every vertex belongs to a side patch (radial) or a cap patch (+-z), nothing between.
    is_side = np.isclose(z_axis_component, 0.0, atol=NORMAL_ATOL)
    is_cap = np.isclose(z_axis_component, 1.0, atol=NORMAL_ATOL)
    assert (is_side | is_cap).all()
    radial = mesh.vertices[is_side, :2] / 0.1
    np.testing.assert_allclose(mesh.vertex_normals[is_side, :2], radial, atol=NORMAL_ATOL)


@pytest.mark.parametrize(
    "geometry", [tg.Capsule(0.1, 1.0), tg.Cone(0.1, 2.0)], ids=["capsule", "cone"]
)
def test_curved_primitives_carry_normals(geometry):
    (mesh,) = visual_to_trimesh(_visual(geometry), "l")
    assert "NORMAL" in _glb_attributes(mesh)
    assert np.isfinite(mesh.vertex_normals).all()


def test_box_stays_flat_shaded():
    """No normals on the wire: the browser's flat shading is exactly a box's creases."""
    (mesh,) = visual_to_trimesh(_visual(tg.Box(1.0, 2.0, 3.0)), "l")
    assert "NORMAL" not in _glb_attributes(mesh)
