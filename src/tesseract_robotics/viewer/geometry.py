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
from .octree import octree_voxels

# Icosphere refinement level: 3 → 642 vertices, a round outline at robot-cell viewing
# distances; its shading comes from _auto_smoothed.
SPHERE_SUBDIVISIONS = 3

# glTF RGBA as 0-255 bytes, the form trimesh's PBR material stores.
RGBA_BYTE_MAX = 255

# Auto-smooth crease [rad]: neighbours meeting at less than 30 deg share a normal, the
# Blender auto-smooth default. Applied only to meshes whose file carries no smooth normals.
SMOOTH_CREASE_ANGLE_RAD = float(np.radians(30.0))

# Spread of a face's three vertex normals [-] below which they are one facet normal:
# float32 noise from the mesh parser's vertex joining is 1.8e-5 on car_seat car.stl;
# authored smooth normals (IRB2400 link_6) spread by 1e-2 and more.
FACET_NORMAL_ATOL = 1e-4

_DEFAULT_MATERIAL_NAME = Material.getDefaultMaterial().getName()


def _rgba_bytes(rgba: NDArray[np.float64]) -> NDArray[np.uint8]:
    return np.round(np.clip(rgba, 0.0, 1.0) * RGBA_BYTE_MAX).astype(np.uint8)


def _alpha_mode(rgba: NDArray[np.uint8]) -> str:
    # glTF draws a material opaque whatever its alpha, unless told to blend.
    return "BLEND" if rgba[3] < RGBA_BYTE_MAX else "OPAQUE"


def _color_material(color: NDArray[np.float64]) -> PBRMaterial:
    rgba = _rgba_bytes(color)
    return PBRMaterial(baseColorFactor=rgba, alphaMode=_alpha_mode(rgba), doubleSided=True)


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


def _scaled_normals(mesh: tg.PolygonMesh) -> NDArray[np.float64] | None:
    # The mesh parser multiplied the file normals n by the URDF scale S like the vertices
    # (S n, unnormalized). A normal maps by the inverse transpose, n' = normalize(S^-1 n)
    # = normalize(S^-2 (S n)). Without normals the browser flat-shades the mesh.
    normals = mesh.getNormals()
    if normals is None:
        return None
    scale = np.asarray(mesh.getScale(), dtype=np.float64)
    scaled = np.asarray(normals, dtype=np.float64).reshape(-1, 3) / scale**2
    length = np.linalg.norm(scaled, axis=1, keepdims=True)
    # All zero: the file had no normals (e.g. car_seat end_effector_simple.stl).
    if not length.any():
        return None
    # Some zero: vertices of zero-area faces, which draw no pixels. Keep them zero; NaN
    # vertex data costs the browser its WebGL context.
    return np.divide(scaled, length, out=np.zeros_like(scaled), where=length > 0.0)


def _faceted(faces: NDArray[np.int64], normals: NDArray[np.float64]) -> bool:
    # True when every face's vertex normals are its one facet normal (an STL), i.e. the
    # file says nothing about how the surface curves between faces.
    corners = normals[faces]
    spread = np.maximum(
        np.abs(corners[:, 1] - corners[:, 0]).max(axis=1),
        np.abs(corners[:, 2] - corners[:, 0]).max(axis=1),
    )
    return bool((spread <= FACET_NORMAL_ATOL).all())


def _auto_smoothed(vertices: NDArray[np.float64], faces: NDArray[np.int64]) -> trimesh.Trimesh:
    # Merge coincident vertices, then cut the surface at creases: faces joined by
    # sub-crease edges form a patch, and a vertex is duplicated once per patch it touches.
    # Each patch then gets angle-weighted vertex normals. Untextured only: merging would
    # orphan per-vertex UVs.
    merged = trimesh.Trimesh(vertices, faces, process=True)
    smooth_edges = merged.face_adjacency[merged.face_adjacency_angles < SMOOTH_CREASE_ANGLE_RAD]
    patch = trimesh.graph.connected_component_labels(smooth_edges, node_count=len(merged.faces))
    # One int64 key per (patch, vertex) corner; 1-D unique is ~10x faster than axis=0.
    vertex_count = len(merged.vertices)
    corner_keys = np.repeat(patch.astype(np.int64), 3) * vertex_count + merged.faces.ravel()
    keys, corner_vertex = np.unique(corner_keys, return_inverse=True)
    split = trimesh.Trimesh(
        merged.vertices[keys % vertex_count], corner_vertex.reshape(-1, 3), process=False
    )
    return trimesh.Trimesh(
        split.vertices, split.faces, vertex_normals=split.vertex_normals, process=False
    )


def _polygon_mesh(mesh: tg.PolygonMesh, visual_material: Material | None) -> trimesh.Trimesh:
    # The mesh parser already applied the URDF scale to the vertices; getScale() is
    # informational and must not be applied again.
    vertices = np.asarray(mesh.getVertices(), dtype=np.float64)
    faces = _triangles(mesh.getFaces())
    normals = _scaled_normals(mesh)
    if not mesh.getTextures() and (normals is None or _faceted(faces, normals)):
        result = _auto_smoothed(vertices, faces)
    else:
        result = trimesh.Trimesh(vertices, faces, vertex_normals=normals, process=False)
    user_color = visual_material is not None and visual_material.getName() != _DEFAULT_MATERIAL_NAME
    if user_color or not _has_own_material(mesh):
        color = (
            visual_material.color
            if visual_material is not None
            else Material.getDefaultMaterial().color
        )
        result.visual = TextureVisuals(material=_color_material(color))
        return result
    own = mesh.getMaterial()
    base_color = _rgba_bytes(own.getBaseColorFactor())
    material = PBRMaterial(
        doubleSided=True,
        baseColorFactor=base_color,
        alphaMode=_alpha_mode(base_color),
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


def _smooth(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    return _auto_smoothed(mesh.vertices, mesh.faces)


def _colored(mesh: trimesh.Trimesh, visual_material: Material | None) -> trimesh.Trimesh:
    color = (
        visual_material.color
        if visual_material is not None
        else Material.getDefaultMaterial().color
    )
    mesh.visual = TextureVisuals(material=_color_material(color))
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
        # Boxes and octree voxels are all creases: no normals, so the browser flat-shades them.
        mesh = trimesh.creation.box((geometry.getX(), geometry.getY(), geometry.getZ()))
    elif isinstance(geometry, tg.Sphere):
        mesh = _smooth(trimesh.creation.icosphere(SPHERE_SUBDIVISIONS, geometry.getRadius()))
    elif isinstance(geometry, tg.Cylinder):
        mesh = _smooth(trimesh.creation.cylinder(geometry.getRadius(), geometry.getLength()))
    elif isinstance(geometry, tg.Capsule):
        mesh = _smooth(trimesh.creation.capsule(geometry.getLength(), geometry.getRadius()))
    elif isinstance(geometry, tg.Cone):
        # trimesh's cone has its base at z = 0; tesseract centres it on the origin. The
        # shift is applied to the vertex array: apply_translation / transform= on a
        # trimesh cone corrupt its z coordinates (trimesh 4.12).
        cone = trimesh.creation.cone(geometry.getRadius(), geometry.getLength())
        shift = np.array([0.0, 0.0, -0.5 * geometry.getLength()])
        mesh = _auto_smoothed(np.asarray(cone.vertices) + shift, cone.faces)
    elif isinstance(geometry, tg.Octree):
        mesh = _octree_mesh(geometry)
    else:
        raise UnsupportedGeometryError(
            f"{type(geometry).__name__} visual on link {link_name!r} has no viewer representation"
        )
    return [_colored(mesh, material)]
