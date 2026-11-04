"""Tesseract visual geometry to `trimesh.Trimesh`, in the visual's own frame."""

from __future__ import annotations

import io

import numpy as np
import trimesh
from numpy.typing import NDArray
from PIL import Image
from trimesh.visual.material import PBRMaterial
from trimesh.visual.texture import TextureVisuals

from tesseract_robotics import tesseract_geometry as tg
from tesseract_robotics.tesseract_scene_graph import Material, Visual

from .errors import UnsupportedGeometryError
from .tesseract_env_to_gltf import (
    _octree_voxels as octree_voxels,
)  # moves to octree.py with the old viewer (#367)

# Icosphere refinement level: 3 → 642 vertices, smooth at robot-cell viewing distances.
SPHERE_SUBDIVISIONS = 3

# glTF RGBA as 0-255 bytes, the form trimesh's PBR material stores.
RGBA_BYTE_MAX = 255

_DEFAULT_MATERIAL_NAME = Material.getDefaultMaterial().getName()


def _rgba_bytes(rgba: NDArray[np.float64]) -> NDArray[np.uint8]:
    return np.round(np.clip(rgba, 0.0, 1.0) * RGBA_BYTE_MAX).astype(np.uint8)


def _triangles(faces: NDArray[np.int32]) -> NDArray[np.int64]:
    """Tesseract `[n, i_0, ..., i_{n-1}, ...]` faces as triangles, fanning each polygon.

    A fan is exact for convex polygons: the faces of `ConvexMesh` and the triangles
    assimp emits for `Mesh`.
    """
    triangles: list[tuple[int, int, int]] = []
    i = 0
    while i < len(faces):
        n = int(faces[i])
        polygon = faces[i + 1 : i + 1 + n]
        triangles.extend((polygon[0], polygon[k], polygon[k + 1]) for k in range(1, n - 1))
        i += n + 1
    return np.asarray(triangles, dtype=np.int64).reshape(-1, 3)


def _has_own_material(mesh: tg.PolygonMesh) -> bool:
    # STL carries no material; assimp hands it a placeholder, which must not win.
    resource = mesh.getResource()
    is_stl = resource is not None and resource.getUrl().lower().endswith(".stl")
    return not is_stl and mesh.getMaterial() is not None


def _polygon_mesh(mesh: tg.PolygonMesh, visual_material: Material | None) -> trimesh.Trimesh:
    # The mesh parser already applied the URDF scale to the vertices; getScale() is
    # informational and must not be applied again.
    vertices = np.asarray(mesh.getVertices(), dtype=np.float64)
    result = trimesh.Trimesh(vertices, _triangles(mesh.getFaces()), process=False)
    user_color = visual_material is not None and visual_material.getName() != _DEFAULT_MATERIAL_NAME
    if user_color or not _has_own_material(mesh):
        color = (
            visual_material.color
            if visual_material is not None
            else Material.getDefaultMaterial().color
        )
        result.visual = TextureVisuals(material=PBRMaterial(baseColorFactor=_rgba_bytes(color)))
        return result
    own = mesh.getMaterial()
    material = PBRMaterial(
        baseColorFactor=_rgba_bytes(own.getBaseColorFactor()),
        metallicFactor=own.getMetallicFactor(),
        roughnessFactor=own.getRoughnessFactor(),
    )
    textures = mesh.getTextures()
    if not textures:
        result.visual = TextureVisuals(material=material)
        return result
    texture = textures[0]
    material.baseColorTexture = Image.open(
        io.BytesIO(bytes(texture.getTextureImage().getResourceContents()))
    )
    uv = np.asarray(texture.getUVs(), dtype=np.float64)
    result.visual = TextureVisuals(uv=uv, material=material)
    return result


def _colored(mesh: trimesh.Trimesh, visual_material: Material | None) -> trimesh.Trimesh:
    color = (
        visual_material.color
        if visual_material is not None
        else Material.getDefaultMaterial().color
    )
    mesh.visual = TextureVisuals(material=PBRMaterial(baseColorFactor=_rgba_bytes(color)))
    return mesh


def _octree_mesh(octree: tg.Octree) -> trimesh.Trimesh:
    centers, sizes = octree_voxels(octree)
    unit = trimesh.creation.box((1.0, 1.0, 1.0))
    vertices = unit.vertices[None, :, :] * sizes[:, None, None] + centers[:, None, :]
    faces = unit.faces[None, :, :] + (np.arange(len(centers)) * len(unit.vertices))[:, None, None]
    return trimesh.Trimesh(vertices.reshape(-1, 3), faces.reshape(-1, 3), process=False)


def visual_to_trimesh(visual: Visual, link_name: str) -> list[trimesh.Trimesh]:
    """Convert one link visual to meshes in the visual frame.

    The caller places them at `visual.origin`. Colour: a URDF `<material>` on the
    visual wins; otherwise a mesh's own material (and texture); otherwise the
    tesseract default grey.

    Args:
        visual: Link visual.
        link_name: Owning link, for the error message.

    Returns:
        One mesh per sub-mesh of a `CompoundMesh`, a single mesh otherwise.

    Raises:
        UnsupportedGeometryError: the geometry type has no mesh form (`Plane`, ...).
    """
    geometry = visual.geometry
    material = visual.material
    if isinstance(geometry, tg.CompoundMesh):
        return [_polygon_mesh(m, material) for m in geometry.getMeshes()]
    if isinstance(geometry, tg.PolygonMesh):
        return [_polygon_mesh(geometry, material)]
    if isinstance(geometry, tg.Box):
        mesh = trimesh.creation.box((geometry.getX(), geometry.getY(), geometry.getZ()))
    elif isinstance(geometry, tg.Sphere):
        mesh = trimesh.creation.icosphere(SPHERE_SUBDIVISIONS, geometry.getRadius())
    elif isinstance(geometry, tg.Cylinder):
        mesh = trimesh.creation.cylinder(geometry.getRadius(), geometry.getLength())
    elif isinstance(geometry, tg.Capsule):
        mesh = trimesh.creation.capsule(geometry.getLength(), geometry.getRadius())
    elif isinstance(geometry, tg.Cone):
        # trimesh's cone has its base at z = 0; tesseract centres it on the origin. The
        # shift is applied to the vertex array: apply_translation / transform= on a
        # trimesh cone corrupt its z coordinates (trimesh 4.12).
        cone = trimesh.creation.cone(geometry.getRadius(), geometry.getLength())
        shift = np.array([0.0, 0.0, -0.5 * geometry.getLength()])
        mesh = trimesh.Trimesh(np.asarray(cone.vertices) + shift, cone.faces, process=False)
    elif isinstance(geometry, tg.Octree):
        mesh = _octree_mesh(geometry)
    else:
        raise UnsupportedGeometryError(
            f"{type(geometry).__name__} visual on link {link_name!r} has no viewer representation"
        )
    return [_colored(mesh, material)]
