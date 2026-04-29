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
vertices = mesh.getVertices()   # VectorVector3d
triangles = mesh.getTriangles() # face indices

# With scale
meshes = createMeshFromPath("model.stl", scale=np.array([0.001, 0.001, 0.001]))
```

### ConvexMesh

Convex hull for efficient collision.

```python
from tesseract_robotics.tesseract_geometry import ConvexMesh, createConvexMeshFromPath

meshes = createConvexMeshFromPath("model.stl")
convex = meshes[0]
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

material = MeshMaterial()
material.base_color = np.array([1.0, 0.0, 0.0, 1.0])  # RGBA red
material.metallic = 0.0
material.roughness = 0.5
```

### MeshTexture

Texture for mesh surfaces.

```python
from tesseract_robotics.tesseract_geometry import MeshTexture

texture = MeshTexture()
texture.image = resource  # Resource pointing to image file
texture.uvs = uv_coords   # UV coordinates per vertex
```

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
| `createOctree(point_cloud, resolution, prune, binary)` | Build an octomap OcTree from a `PointCloud` |

## Auto-generated API Reference

::: tesseract_robotics.tesseract_geometry._tesseract_geometry
    options:
      show_root_heading: false
      show_source: false
      members_order: source
