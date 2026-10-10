# tesseract_robotics.tesseract_geometry

Geometric primitives for collision and visualization.

## Primitives

### Box

```python
from tesseract_robotics.tesseract_geometry import Box

box = Box(1.0, 0.5, 0.25)  # x, y, z dimensions
print(f"Dimensions: {box.getX()} x {box.getY()} x {box.getZ()}")
```

### Sphere

```python
from tesseract_robotics.tesseract_geometry import Sphere

sphere = Sphere(0.1)  # radius
print(f"Radius: {sphere.getRadius()}")
```

### Cylinder

```python
from tesseract_robotics.tesseract_geometry import Cylinder

cylinder = Cylinder(0.05, 0.2)  # radius, length
print(f"Radius: {cylinder.getRadius()}, Length: {cylinder.getLength()}")
```

### Capsule

```python
from tesseract_robotics.tesseract_geometry import Capsule

capsule = Capsule(0.05, 0.2)  # radius, length
```

### Cone

```python
from tesseract_robotics.tesseract_geometry import Cone

cone = Cone(0.1, 0.3)  # radius, length
```

### Plane

```python
from tesseract_robotics.tesseract_geometry import Plane

# ax + by + cz + d = 0
plane = Plane(0, 0, 1, 0)  # XY plane (z = 0)
```

## Meshes

### Mesh

Triangle mesh from file.

```python
from tesseract_robotics.tesseract_geometry import Mesh, createMeshFromPath
import numpy as np

# Load from file
meshes = createMeshFromPath("model.stl")
mesh = meshes[0]

# Access data
vertices = mesh.getVertices()   # list of 3-vectors
faces = mesh.getFaces()         # [count, i0, i1, ..., count, ...]

# With scale
meshes = createMeshFromPath("model.stl", scale=np.array([0.001, 0.001, 0.001]))
```

`Mesh`, `SDFMesh`, `ConvexMesh` and `PolygonMesh` bind both native constructors. `faces` is flat:
each face is its vertex count followed by that many vertex indices. Every argument after `faces` is
optional and takes the C++ default; the pointer arguments take `None` for "not set".

```python
from tesseract_robotics.tesseract_common import BytesResource
from tesseract_robotics.tesseract_geometry import Mesh, MeshFaceCountError, MeshMaterial
import numpy as np

vertices = [np.array(v, dtype=float) for v in ([0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0])]
faces = np.array([3, 0, 1, 2, 3, 0, 2, 3], dtype=np.int32)

mesh = Mesh(vertices, faces)  # counts the faces
mesh = Mesh(
    vertices, faces,
    resource=BytesResource("square.stl", b"..."),
    scale=np.array([1.0, 1.0, 1.0]),
    normals=[np.array([0.0, 0.0, 1.0])] * 4,
    vertex_colors=[np.array([1.0, 0.0, 0.0, 1.0])] * 4,
    mesh_material=MeshMaterial(np.array([1.0, 0.0, 0.0, 1.0]), 0.0, 0.5, np.zeros(4)),
    mesh_textures=None,
)
mesh = Mesh(vertices, faces, 2)  # face_count given

try:
    Mesh(vertices, faces, 3)
except MeshFaceCountError:  # a ValueError
    pass
```

The `face_count` overload exists in C++ to skip counting. Upstream stores the count without checking
it; the binding counts the faces anyway and raises `MeshFaceCountError` when the two disagree, so
`getFaceCount()` always matches `getFaces()`. `PolygonMesh` also takes a trailing
`type=GeometryType.POLYGON_MESH`. Fixed-size vectors (`scale`, each normal) must be numpy arrays:
nanobind's Eigen caster refuses a plain list.

### ConvexMesh

Convex hull for efficient collision.

```python
from tesseract_robotics.tesseract_geometry import ConvexMesh, createConvexMeshFromPath

meshes = createConvexMeshFromPath("model.stl")
convex = meshes[0]
```

`getCreationMethod()` reports how the mesh was made, as `ConvexMesh.CreationMethod`: `DEFAULT`
(a new mesh), `MESH`, or `CONVERTED`, which the URDF parser sets when `tesseract:make_convex`
turned a mesh into its convex hull. `setCreationMethod` sets it, and `ConvexMesh` equality
compares it. `clone()` returns a `DEFAULT` mesh without normals, colors, material or textures:
upstream's clone passes only the vertices, faces, face count, resource and scale
(convex_mesh.cpp:77 @ 0.35.0).

```python
from tesseract_robotics.tesseract_geometry import ConvexMesh

convex.getCreationMethod() == ConvexMesh.CreationMethod.DEFAULT
convex.setCreationMethod(ConvexMesh.CreationMethod.MESH)
```

### SDFMesh

Signed distance field mesh.

```python
from tesseract_robotics.tesseract_geometry import SDFMesh, createSDFMeshFromPath

meshes = createSDFMeshFromPath("model.stl")
sdf = meshes[0]
```

### CompoundMesh

Multiple mesh parts as single geometry.

```python
from tesseract_robotics.tesseract_geometry import CompoundMesh

# Combine multiple meshes
compound = CompoundMesh(meshes)
```

## Octree

Occupancy octree from 3D sensor data, backed by [octomap](https://octomap.github.io/).

### PointCloud → Octree

```python
from tesseract_robotics.tesseract_geometry import (
    PointCloud, Octree, OctreeSubType, createOctree,
)

# Build a point cloud and convert to an octomap OcTree
pc = PointCloud()
pc.addPoint(0.0, 0.0, 0.0)
pc.addPoint(0.1, 0.0, 0.0)
pc.addPoint(0.0, 0.1, 0.0)

# Or assign whole points: the point type is nested, as in C++ (PointCloud::Point)
pc.points = [PointCloud.Point(0.0, 0.0, 0.0), PointCloud.Point(0.1, 0.0, 0.0)]

ot = createOctree(pc, resolution=0.05, prune=True, binary=True)

# Wrap in a tesseract Octree geometry
octree = Octree(ot, OctreeSubType.BOX, pruned=True, binary_octree=True)
print(octree.calcNumSubShapes())
```

### Building an Octree directly

```python
from tesseract_robotics.tesseract_geometry import OcTree

ot = OcTree(0.05)  # leaf resolution
ot.updateNode(0.0, 0.0, 0.0, True)
ot.updateNode(0.1, 0.0, 0.0, True)
ot.updateInnerOccupancy()
ot.toMaxLikelihood()
ot.writeBinary("/tmp/scene.bt")

# Load back from file
loaded = OcTree("/tmp/scene.bt")
```

| OctreeSubType | Description |
|---------------|-------------|
| `BOX` | Each occupied voxel becomes a box |
| `SPHERE_INSIDE` | Inscribed sphere per voxel |
| `SPHERE_OUTSIDE` | Circumscribed sphere per voxel |

## Utilities & Conversions

```python
from tesseract_robotics.tesseract_geometry import (
    Box, Sphere, isIdentical, extractVertices, toTriangleMesh,
)
from tesseract_robotics.tesseract_common import Isometry3d

origin = Isometry3d()
box = Box(1, 1, 1)

# Compare two geometries structurally
isIdentical(box, Box(1, 1, 1))   # True
isIdentical(box, Sphere(1))      # False

# Extract vertices (primitives are converted to a mesh first)
verts = extractVertices(box, origin)        # 8 verts

# Convert a primitive to a triangle Mesh
mesh = toTriangleMesh(box, tolerance=0.01, origin=origin)
```

## Mesh Materials

### MeshMaterial

Surface material properties.

```python
from tesseract_robotics.tesseract_geometry import MeshMaterial
import numpy as np

material = MeshMaterial(
    np.array([1.0, 0.0, 0.0, 1.0]),  # base_color_factor (RGBA red)
    0.0,                             # metallic_factor
    0.5,                             # roughness_factor
    np.zeros(4),                     # emissive_factor (RGBA)
)
material.getBaseColorFactor()
```

### MeshTexture

A texture image and its per-vertex UV coordinates.

```python
from tesseract_robotics.tesseract_common import BytesResource
from tesseract_robotics.tesseract_geometry import MeshTexture
import numpy as np

png_bytes = open("texture.png", "rb").read()
uvs = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0]), np.array([0.0, 1.0])]
texture = MeshTexture(BytesResource("texture.png", png_bytes), uvs)
texture.getTextureImage().getUrl()  # "texture.png"
```

`texture_image` must be a jpg or png resource. Neither argument takes `None` (`TypeError`): upstream
stores both without checking them, but a texture without an image or UVs is never valid.

## Geometry Base

### Geometry

Base class for all geometry types.

```python
from tesseract_robotics.tesseract_geometry import Geometry, GeometryType

geom = box  # any geometry

# Type checking
geom_type = geom.getType()
if geom_type == GeometryType.BOX:
    print("It's a box")

# Clone
copy = geom.clone()
```

`==` compares the geometry type and its UUID, not its content: every constructor and `clone()` draws a
new random UUID, so `Box(1, 1, 1) == Box(1, 1, 1)` and `geom == geom.clone()` are `False`. Use
`isIdentical` to compare content. Each class adds its own fields to `==`: the mesh classes compare
vertex count, face count and scale (`PolygonMesh`, `Mesh`, `ConvexMesh`, which also compares its
creation method), except `SDFMesh`, whose upstream `operator==` is type and UUID alone. `getUUID()` returns the UUID as its canonical string; `setUUID(str)`
sets it and raises `ValueError` for a malformed string, leaving the UUID unchanged.

```python
from tesseract_robotics.tesseract_geometry import Box, isIdentical

a, b = Box(1, 1, 1), Box(1, 1, 1)
a == b               # False: different UUIDs
isIdentical(a, b)    # True: same content
b.setUUID(a.getUUID())
a == b               # True
```

| GeometryType | Description |
|--------------|-------------|
| `BOX` | Rectangular box |
| `SPHERE` | Sphere |
| `CYLINDER` | Cylinder |
| `CAPSULE` | Capsule (cylinder + hemisphere caps) |
| `CONE` | Cone |
| `PLANE` | Infinite plane |
| `MESH` | Triangle mesh |
| `CONVEX_MESH` | Convex hull |
| `SDF_MESH` | Signed distance field |
| `OCTREE` | Occupancy octree |
| `COMPOUND_MESH` | Multiple meshes |

## Factory Functions

| Function | Description |
|----------|-------------|
| `createMeshFromPath(path, scale)` | Load mesh from file |
| `createMeshFromResource(resource, scale)` | Load mesh from Resource |
| `createConvexMeshFromPath(path, scale)` | Load as convex hull |
| `createConvexMeshFromResource(resource, scale)` | Convex from Resource |
| `createSDFMeshFromPath(path, scale)` | Load as SDF mesh |
| `createSDFMeshFromResource(resource, scale)` | SDF from Resource |
| `createMeshFromBytes(url, data, scale)` | Load mesh from an in-memory file |
| `createConvexMeshFromBytes(url, data, scale)` | Convex from an in-memory file |
| `createSDFMeshFromBytes(url, data, scale)` | SDF from an in-memory file |
| `createOctree(point_cloud, resolution, prune, binary)` | Build an octomap OcTree from a `PointCloud` |

Every loader also takes `flatten`, `normals`, `vertex_colors` and `material_and_texture`, all
`False` by default as in C++. Turn the last three on to load a mesh's normals, vertex colors,
materials and textures. The URDF parser loads visual meshes with `flatten` and all three flags on.
Every mesh loader defaults to `triangulate=True`, where the C++ header says `false`. The URDF
parser always triangulates (urdf/src/mesh.cpp:84–88), and `Mesh` expects triangles.

```python
from tesseract_robotics.tesseract_geometry import createMeshFromPath

meshes = createMeshFromPath(
    "textured.dae", flatten=True, normals=True, vertex_colors=True, material_and_texture=True
)
meshes[0].getMaterial().getBaseColorFactor()
```

The `…FromBytes` loaders take bytes you already hold. The extension of `url` selects the format,
so `url` must end in `.stl`, `.dae`, and so on. The bytes are copied, so the returned meshes do not
depend on `data` afterwards. `createMeshFromAsset` and `extractMeshData` are not bound: they take
assimp's `aiScene`, which Python never sees.

```python
from pathlib import Path
from tesseract_robotics.tesseract_geometry import createMeshFromBytes

data = Path("model.stl").read_bytes()
meshes = createMeshFromBytes("model.stl", data)
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_geometry._tesseract_geometry
    options:
      show_root_heading: false
      show_source: false
      members_order: source
