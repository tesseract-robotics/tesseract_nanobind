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

### CompoundMesh

Multiple mesh parts as single geometry.

```python
from tesseract_robotics.tesseract_geometry import CompoundMesh

# Combine multiple meshes
compound = CompoundMesh(meshes)
```

## SignedDistanceField

A volumetric geometry: instead of a surface, it stores the signed distance to a surface —
negative inside, zero on it, positive outside — sampled on a regular axis-aligned grid in the
geometry's local frame. That yields distance and gradient *everywhere* in the volume, including
for concave shapes a convex hull cannot represent, which is what trajectory optimization
consumes. The discrete Bullet and FCL contact managers support it; continuous (cast) managers
do not, because the field is concave.

!!! note
    Unrelated to the Gazebo "SDF" Simulation Description Format. This replaces the old
    `SDFMesh`, which was a *mesh* despite the name.

### From a distance function

The usual entry point. `createDiscreteSignedDistanceField` evaluates your callable once per grid
point and keeps only the samples, so the returned geometry is pure data — it holds no reference
to your Python objects and takes no GIL during collision checking.

```python
import numpy as np
from tesseract_robotics.tesseract_geometry import createDiscreteSignedDistanceField

def sphere(point):            # negative inside, positive outside
    return float(np.linalg.norm(point) - 0.5)

field = createDiscreteSignedDistanceField(
    sphere,
    domain_min=np.array([-1.0, -1.0, -1.0]),
    domain_max=np.array([1.0, 1.0, 1.0]),
    dimensions=np.array([33, 33, 33], dtype=np.int32),  # >= 2 per axis
)

field.getDistance(np.zeros(3))   # -0.5
```

Pass `batched=True` to receive every sample point as one `(N, 3)` float64 array and return `N`
distances — the fast path for a vectorised or GPU evaluator, and one Python call instead of
`dimensions.prod()`:

```python
field = createDiscreteSignedDistanceField(
    lambda points: np.linalg.norm(points, axis=1) - 0.5,
    np.array([-1.0, -1.0, -1.0]),
    np.array([1.0, 1.0, 1.0]),
    np.array([33, 33, 33], dtype=np.int32),
    batched=True,
)
```

### From a grid you already have

`distances` is flat and ordered x-fastest (`index = i + nx*(j + ny*k)`), so a numpy grid built
with `indexing="ij"` transfers with `ravel(order="F")` and comes back with `reshape(..., order="F")`.

```python
import numpy as np
from tesseract_robotics.tesseract_geometry import SignedDistanceField

dims = np.array([33, 33, 33], dtype=np.int32)
lo, hi = np.full(3, -1.0), np.full(3, 1.0)

x, y, z = np.meshgrid(*[np.linspace(lo[i], hi[i], dims[i]) for i in range(3)], indexing="ij")
grid = np.sqrt(x**2 + y**2 + z**2) - 0.5

field = SignedDistanceField(lo, hi, dims, grid.ravel(order="F"))
assert np.allclose(field.getDistances().reshape(dims, order="F"), grid)
```

### From a VDB file

Fields round-trip through the standard OpenVDB and NanoVDB grid formats as raw bytes, so a field
baked offline loads directly. The local `scale` is not stored in the grid and is supplied on read.

```python
from pathlib import Path
from tesseract_robotics.tesseract_geometry import (
    readSignedDistanceFieldVDB, writeSignedDistanceFieldVDB,
)

field = readSignedDistanceFieldVDB(Path("part.vdb").read_bytes())
Path("out.vdb").write_bytes(writeSignedDistanceFieldVDB(field))
```

`readSignedDistanceFieldNVDB` / `writeSignedDistanceFieldNVDB` do the same for NanoVDB. Both
readers accept exactly one `FloatGrid` with an axis-aligned, uniformly scaled transform.

### Lazy fields

`createSignedDistanceField` keeps your callable as the field's source of truth instead of
sampling up front, so queries are exact rather than trilinearly interpolated. `dimensions` then
only sets the resolution used if the grid is ever materialized (on serialization, comparison, or
an explicit `discretize()`).

Prefer the discrete form unless you need that exactness. A lazy field must be thread-safe, and it
holds a reference to your callable — so storing one in a module-level global forms a reference
cycle the garbage collector cannot see through (the callable reaches its `__globals__`, which
reaches the field, and nanobind instances have no `tp_traverse`).

```python
from tesseract_robotics.tesseract_geometry import createSignedDistanceField

field = createSignedDistanceField(sphere, lo, hi, dims)
field.isDiscretized()   # False
field.getDistance(p)    # exact - calls sphere(p)
field.discretize()      # materialize the grid up front
```

!!! warning "Discretize before using a lazy field for collision"
    The collision backends call `getDistance()` **once per sample point**, and each call
    re-acquires the GIL. `contactTest` releases the GIL specifically so trajectory sweeps can run
    in parallel across threads; a lazy field hands that parallelism straight back to the
    interpreter. Bullet also transcodes the field into its own grid when the geometry is added to
    a manager, which is one Python call per lattice point.

    `discretize()` up front fixes both — afterwards the sampler is never invoked again and the
    field behaves like pure data:

    ```python
    field.discretize()          # pay the sampling cost once, under your control
    create_obstacle(robot, name="field", geometry=field)
    ```

    Note `discretize()` holds a *process-wide* static mutex while calling the sampler, so it must
    never run on a thread that has released the GIL: that thread would hold the mutex and block on
    the GIL while a Python thread holding the GIL blocks on the mutex. The bindings avoid this (the
    VDB writers discretize before releasing the GIL); avoid arranging it yourself from a C++
    callback.

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
| `SIGNED_DISTANCE_FIELD` | Volumetric signed distance field |
| `OCTREE` | Occupancy octree |
| `COMPOUND_MESH` | Multiple meshes |

## Factory Functions

| Function | Description |
|----------|-------------|
| `createMeshFromPath(path, scale)` | Load mesh from file |
| `createMeshFromResource(resource, scale)` | Load mesh from Resource |
| `createConvexMeshFromPath(path, scale)` | Load as convex hull |
| `createConvexMeshFromResource(resource, scale)` | Convex from Resource |
| `createOctree(point_cloud, resolution, prune, binary)` | Build an octomap OcTree from a `PointCloud` |
| `createDiscreteSignedDistanceField(sdf, domain_min, domain_max, dimensions, scale, batched)` | Sample a distance function onto a grid |
| `createSignedDistanceField(sdf, domain_min, domain_max, dimensions, scale, batched)` | Lazy field backed by a distance function |
| `readSignedDistanceFieldVDB(data, scale)` / `writeSignedDistanceFieldVDB(sdf)` | OpenVDB `FloatGrid` bytes ↔ field |
| `readSignedDistanceFieldNVDB(data, scale)` / `writeSignedDistanceFieldNVDB(sdf)` | NanoVDB `FloatGrid` bytes ↔ field |

## Auto-generated API Reference

::: tesseract_robotics.tesseract_geometry._tesseract_geometry
    options:
      show_root_heading: false
      show_source: false
      members_order: source
