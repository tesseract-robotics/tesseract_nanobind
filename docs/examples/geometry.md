# Geometry Examples

Meshes as `tesseract_geometry` loads, builds and writes them. The example takes
its meshes and parameters from upstream tesseract: the support meshes
`box_2m.ply` and `tesseract_material_mesh.glb`, and the expected counts of
`tesseract_geometry_unit.cpp`, cited in the module docstring. Every value
printed is also asserted in `tests/examples`.

```bash
# Installed console script
tesseract_mesh_roundtrip_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.mesh_roundtrip_example
```

For the primitive shapes (`Box`, `Sphere`, `Cylinder`, ...) see
`geometry_showcase_example.py` under [Basic Examples](basic.md).

## Mesh Round Trip

### Loading from bytes

`createMeshFromBytes`, `createSDFMeshFromBytes` and `createConvexMeshFromBytes`
parse a mesh held in memory; the URL names the resource and its extension tells
Assimp the format. `triangulate` defaults to `True` in the binding, which is
what the URDF parser passes for every mesh (`urdf/src/mesh.cpp:84–88`).
Upstream's C++ default is `false` (`mesh_parser.h:541`).

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:from_bytes"
```

| call | (faces, vertices) |
|---|---|
| `createMeshFromBytes(url, data)` | (12, 8): six quads, triangulated |
| `createSDFMeshFromBytes(url, data, scale, True, ...)` | (12, 8) |
| `createConvexMeshFromBytes(url, data, scale, False, ...)` | (6, 8): the quads kept |

### Normals, vertex colors, materials and textures

The path and resource loaders take three more flags: `normals`,
`vertex_colors` and `material_and_texture`. With all of them on, as the URDF
parser loads a visual mesh, `tesseract_material_mesh.glb` gives four meshes
with one normal per vertex; the last one, a quad, carries a 38212-byte texture
image with four UVs. Without the flags, `getNormals()` and `getTextures()`
return `None`. The file has no vertex colors.

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:flags"
```

| mesh | faces | vertices | normals | textures |
|---|---|---|---|---|
| 0 | 34 | 68 | 68 | 0 |
| 1 | 15 | 17 | 17 | 0 |
| 2 | 15 | 17 | 17 | 0 |
| 3 | 2 | 4 | 4 | 1 |

### Full constructors

`Mesh`, `SDFMesh`, `PolygonMesh` and `ConvexMesh` take every part a loader
fills: resource, scale, normals, vertex colors, material and textures. Each has
two overloads, with and without `face_count` as the third argument; a
`face_count` that disagrees with `faces` raises `MeshFaceCountError`.
`PolygonMesh` also takes its `GeometryType`. A `MeshTexture` is an image
resource plus one UV per vertex.

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:constructors"
```

### UUID and equality

`==` on meshes compares type, UUID, vertex count, face count and scale
(`geometry.cpp:40`, `polygon_mesh.cpp:111–120`), not the vertices themselves.
Each constructor draws a fresh UUID, so two meshes built from the same parts are
unequal until `setUUID` copies one UUID onto the other. `getUUID` returns the
UUID as its canonical string.

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:uuid"
```

`ConvexMesh ==` also compares the creation method (`convex_mesh.cpp:80–86`).
A loaded convex mesh is `CreationMethod.DEFAULT`; after
`setCreationMethod(CreationMethod.CONVERTED)` it no longer equals a copy with
the same UUID.

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:creation_method"
```

### Writing and reloading

`writeMeshToFile` (in `tesseract_urdf`) always writes an ASCII PLY file of
vertex positions and faces, with seven decimals (`ply_io.cpp:102`).

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:roundtrip"
```

!!! note "The round trip keeps positions, not vertices"
    The reader joins identical vertices (`aiProcess_JoinIdenticalVertices`,
    `mesh_parser.h:372`). A mesh that repeats a position, one vertex per normal
    as the glb's first mesh does, comes back with one vertex per distinct
    position. Face counts of triangle meshes are kept.

| mesh | written (faces, vertices) | reloaded | distinct positions |
|---|---|---|---|
| `box_2m.ply` | (12, 8) | (12, 8) | 8 |
| `tesseract_material_mesh.glb`, mesh 0 | (34, 68) | (34, 34) | 34 |

Every reloaded vertex lies within the PLY precision of a written one, and
the other way round: half a unit of the seventh decimal plus half a float32 ulp.

### Transforming vertices and normals

`Isometry3d.apply_points` maps an `(N, 3)` array of points through the
transform; `apply_directions` applies only its rotation, for normals. Each row
equals `tf * p`, bit for bit. The transform is upstream's
`checkTranslationInvariant` (`tesseract_geometry_unit.cpp:914–915`).

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:transform"
```

### Point clouds

`PointCloud.Point` is a point of a `PointCloud`, built empty (the origin) or
from `x, y, z`. `createOctree` puts each point into a voxel of the given
resolution. The eight corners of the 2 m box plus the origin fill nine 1 cm voxels.

```python title="mesh_roundtrip_example.py"
--8<-- "src/tesseract_robotics/examples/mesh_roundtrip_example.py:point_cloud"
```
