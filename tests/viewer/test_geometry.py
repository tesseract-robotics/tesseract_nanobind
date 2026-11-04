"""visual_to_trimesh: tesseract geometry in the visual frame."""

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
      <mesh filename="package://tesseract/support/meshes/sphere_p25m.stl" scale="{scale} {scale} {scale}"/>
    </geometry></visual>
  </link>
  <joint name="j_dae" type="fixed"><parent link="world"/><child link="dae_link"/></joint>
  <joint name="j_stl" type="fixed"><parent link="world"/><child link="stl_link"/></joint>
</robot>
"""


def _visual(geometry, material=None):
    visual = Visual()
    visual.geometry = geometry
    if material is not None:
        visual.material = material
    return visual


def _mesh_env(scale=1.0):
    env = Environment()
    assert env.init(MESH_URDF.format(scale=scale), GeneralResourceLocator())
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

    np.testing.assert_allclose(extents(2.0), 2.0 * extents(1.0))


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
