"""tesseract_geometry Python bindings"""

from collections.abc import Callable, Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray


class GeometryType(enum.Enum):
    UNINITIALIZED = 0

    SPHERE = 1

    CYLINDER = 2

    CAPSULE = 3

    CONE = 4

    BOX = 5

    PLANE = 6

    MESH = 7

    CONVEX_MESH = 8

    SIGNED_DISTANCE_FIELD = 9

    OCTREE = 10

    POLYGON_MESH = 11

    COMPOUND_MESH = 12

class Geometry:
    def getType(self) -> GeometryType:
        """Get the geometry type"""

    def clone(self) -> Geometry:
        """Create a copy of this geometry"""

    def __eq__(self, arg: Geometry, /) -> bool: ...

    def __ne__(self, arg: Geometry, /) -> bool: ...

class GeometriesConst:
    def __init__(self) -> None: ...

    def __len__(self) -> int: ...

    def __getitem__(self, arg: int, /) -> Geometry: ...

    def append(self, arg: Geometry, /) -> None: ...

    def clear(self) -> None: ...

class Box(Geometry):
    @overload
    def __init__(self, x: float, y: float, z: float) -> None:
        """Create a box with dimensions x, y, z"""

    @overload
    def __init__(self) -> None:
        """Create a default box"""

    def getX(self) -> float:
        """Get X dimension"""

    def getY(self) -> float:
        """Get Y dimension"""

    def getZ(self) -> float:
        """Get Z dimension"""

    def __eq__(self, arg: Box, /) -> bool: ...

    def __ne__(self, arg: Box, /) -> bool: ...

    def __repr__(self) -> str: ...

class Sphere(Geometry):
    @overload
    def __init__(self, r: float) -> None:
        """Create a sphere with radius r"""

    @overload
    def __init__(self) -> None:
        """Create a default sphere"""

    def getRadius(self) -> float:
        """Get the radius"""

    def __eq__(self, arg: Sphere, /) -> bool: ...

    def __ne__(self, arg: Sphere, /) -> bool: ...

    def __repr__(self) -> str: ...

class Cylinder(Geometry):
    @overload
    def __init__(self, r: float, l: float) -> None:
        """Create a cylinder with radius r and length l"""

    @overload
    def __init__(self) -> None:
        """Create a default cylinder"""

    def getRadius(self) -> float:
        """Get the radius"""

    def getLength(self) -> float:
        """Get the length"""

    def __eq__(self, arg: Cylinder, /) -> bool: ...

    def __ne__(self, arg: Cylinder, /) -> bool: ...

    def __repr__(self) -> str: ...

class Capsule(Geometry):
    @overload
    def __init__(self, r: float, l: float) -> None:
        """Create a capsule with radius r and length l"""

    @overload
    def __init__(self) -> None:
        """Create a default capsule"""

    def getRadius(self) -> float:
        """Get the radius"""

    def getLength(self) -> float:
        """Get the length"""

    def __eq__(self, arg: Capsule, /) -> bool: ...

    def __ne__(self, arg: Capsule, /) -> bool: ...

    def __repr__(self) -> str: ...

class Cone(Geometry):
    @overload
    def __init__(self, r: float, l: float) -> None:
        """Create a cone with radius r and length l"""

    @overload
    def __init__(self) -> None:
        """Create a default cone"""

    def getRadius(self) -> float:
        """Get the radius"""

    def getLength(self) -> float:
        """Get the length"""

    def __eq__(self, arg: Cone, /) -> bool: ...

    def __ne__(self, arg: Cone, /) -> bool: ...

    def __repr__(self) -> str: ...

class Plane(Geometry):
    @overload
    def __init__(self, a: float, b: float, c: float, d: float) -> None:
        """Create a plane with equation ax + by + cz + d = 0"""

    @overload
    def __init__(self) -> None:
        """Create a default plane"""

    def getA(self) -> float:
        """Get coefficient a"""

    def getB(self) -> float:
        """Get coefficient b"""

    def getC(self) -> float:
        """Get coefficient c"""

    def getD(self) -> float:
        """Get coefficient d"""

    def __eq__(self, arg: Plane, /) -> bool: ...

    def __ne__(self, arg: Plane, /) -> bool: ...

    def __repr__(self) -> str: ...

class MeshMaterial:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, base_color_factor: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')], metallic_factor: float, roughness_factor: float, emissive_factor: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None: ...

    def getBaseColorFactor(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]:
        """Get base color (RGBA)"""

    def getMetallicFactor(self) -> float:
        """Get metallic factor (0-1)"""

    def getRoughnessFactor(self) -> float:
        """Get roughness factor (0-1)"""

    def getEmissiveFactor(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]:
        """Get emissive factor (RGBA)"""

class MeshTexture:
    def getTextureImage(self) -> "tesseract_common::Resource":
        """Get the texture image resource"""

    def getUVs(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(2), order='C')]]:
        """Get UV coordinates"""

class PolygonMesh(Geometry):
    def getVertexCount(self) -> int:
        """Get number of vertices"""

    def getFaceCount(self) -> int:
        """Get number of faces"""

    def getScale(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get mesh scale"""

    def getVertices(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]: ...

    def getFaces(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]: ...

    def getNormals(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None:
        """Get vertex normals (optional)"""

    def getVertexColors(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None:
        """Get vertex colors (optional)"""

    def getMaterial(self) -> MeshMaterial:
        """Get mesh material (optional)"""

    def getTextures(self) -> list[MeshTexture] | None:
        """Get mesh textures (optional)"""

    def getResource(self) -> "tesseract_common::Resource":
        """Get mesh resource"""

class Mesh(PolygonMesh):
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]) -> None: ...

class ConvexMesh(PolygonMesh):
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]) -> None: ...

class SignedDistanceField(Geometry):
    def __init__(self, domain_min: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], domain_max: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], dimensions: Annotated[NDArray[numpy.int32], dict(shape=(3), order='C')], distances: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ...) -> None:
        """
        Create a field from signed distances sampled on a regular grid.

        domain_min/domain_max bound the sampled axis-aligned domain in the field's local frame,
        dimensions is the number of samples along each axis (>= 2 per axis), and distances holds
        dimensions.prod() values, negative inside the surface.

        distances is flat and ordered x-fastest: index = i + nx*(j + ny*k). From a numpy grid built
        with np.meshgrid(..., indexing='ij'), pass grid.ravel(order='F').
        """

    def getDomainMin(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the minimum corner of the sampled domain (local frame)"""

    def getDomainMax(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the maximum corner of the sampled domain (local frame)"""

    def getDimensions(self) -> Annotated[NDArray[numpy.int32], dict(shape=(3), order='C')]:
        """Get the number of samples along each axis"""

    def getDistances(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Get the sampled signed distances, flat and x-fastest (index = i + nx*(j + ny*k)).
        Reshape with .reshape(field.getDimensions(), order='F') for a 3D view.
        Discretizes a function-backed field on first call.
        """

    def getScale(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the local scaling applied to the field"""

    def getDistance(self, point: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> float:
        """
        Get the signed distance at a point in the field's local frame.
        A function-backed field evaluates its sampler directly; a grid-backed field
        trilinearly interpolates. The point is clamped to the domain.
        """

    def isDiscretized(self) -> bool:
        """Whether the dense sample grid has been materialized"""

    def discretize(self) -> None:
        """
        Materialize the dense sample grid from the sampler (idempotent, no-op if already discretized).
        A function-backed field must be discretized before it can be serialized or compared; that
        happens automatically at those boundaries, so call this only to pin the snapshot up front.
        """

    def __eq__(self, arg: SignedDistanceField, /) -> bool: ...

    def __ne__(self, arg: SignedDistanceField, /) -> bool: ...

    def __repr__(self) -> str: ...

class CompoundMesh(Geometry):
    def __init__(self, meshes: Sequence[PolygonMesh]) -> None: ...

    def getMeshes(self) -> list[PolygonMesh]:
        """Get the vector of meshes"""

    def getResource(self) -> "tesseract_common::Resource":
        """Get the resource used to create this mesh"""

    def getScale(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the scale applied to the mesh"""

class OcTree:
    @overload
    def __init__(self, resolution: float) -> None:
        """Create an empty octomap OcTree with the given leaf resolution"""

    @overload
    def __init__(self, filename: str) -> None:
        """Load an octomap OcTree from a .bt or .ot file"""

    def getResolution(self) -> float:
        """Get the leaf resolution"""

    def size(self) -> int:
        """Get the total number of nodes"""

    def getNumLeafNodes(self) -> int:
        """Get the number of leaf nodes"""

    def updateNode(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool = False) -> None:
        """Insert/update a node at the given coordinate"""

    def updateInnerOccupancy(self) -> None:
        """Recompute inner occupancies after lazy updates"""

    def toMaxLikelihood(self) -> None:
        """
        Convert occupancy probabilities to a binary maximum-likelihood representation
        """

    def writeBinary(self, filename: str) -> bool:
        """Write the octree to a binary .bt file"""

class OctreeSubType(enum.Enum):
    BOX = 0

    SPHERE_INSIDE = 1

    SPHERE_OUTSIDE = 2

class PointCloudPoint:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, x: float, y: float, z: float) -> None: ...

    @property
    def x(self) -> float: ...

    @x.setter
    def x(self, arg: float, /) -> None: ...

    @property
    def y(self) -> float: ...

    @y.setter
    def y(self, arg: float, /) -> None: ...

    @property
    def z(self) -> float: ...

    @z.setter
    def z(self, arg: float, /) -> None: ...

class PointCloud:
    def __init__(self) -> None: ...

    @property
    def points(self) -> list[PointCloudPoint]: ...

    @points.setter
    def points(self, arg: Sequence[PointCloudPoint], /) -> None: ...

    def addPoint(self, x: float, y: float, z: float) -> None:
        """Add a point to the cloud"""

class Octree(Geometry):
    def __init__(self, octree: OcTree, sub_type: OctreeSubType, pruned: bool = False, binary_octree: bool = False) -> None:
        """Create an Octree geometry wrapping an octomap OcTree"""

    def getOctree(self) -> OcTree:
        """Get the underlying octomap OcTree"""

    def getSubType(self) -> OctreeSubType:
        """Get the sub-shape type"""

    def getPruned(self) -> bool:
        """Whether the octree was pruned"""

    def calcNumSubShapes(self) -> int:
        """Calculate the number of sub-shapes (expensive)"""

    def __eq__(self, arg: Octree, /) -> bool: ...

    def __ne__(self, arg: Octree, /) -> bool: ...

    @staticmethod
    def prune(octree: OcTree) -> None:
        """Prune the octomap OcTree using tesseract's occupancy-threshold rule"""

def createOctree(point_cloud: PointCloud, resolution: float, prune: bool, binary: bool = True) -> OcTree:
    """Build an octomap OcTree from a PointCloud"""

def createMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[Mesh]:
    """Load mesh from file and return vector of Mesh geometries"""

def createConvexMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[ConvexMesh]:
    """Load mesh from file and return vector of ConvexMesh geometries"""

def createMeshFromResource(resource: "tesseract_common::Resource", scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[Mesh]:
    """Load Mesh from resource (e.g., package:// URL)"""

def createConvexMeshFromResource(resource: "tesseract_common::Resource", scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[ConvexMesh]:
    """Load ConvexMesh from resource (e.g., package:// URL)"""

def createDiscreteSignedDistanceField(sdf: Callable, domain_min: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], domain_max: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], dimensions: Annotated[NDArray[numpy.int32], dict(shape=(3), order='C')], scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., batched: bool = False) -> SignedDistanceField:
    """
    Sample a signed distance function onto a dense grid up front and return the resulting field.

    sdf is negative inside the surface and evaluated in the field's local frame. The grid spans
    [domain_min, domain_max] inclusive with dimensions samples per axis (>= 2 per axis), so the
    callable is invoked dimensions.prod() times - pass batched=True to evaluate them in one call.

    Prefer this over createSignedDistanceField: sdf is called only during this call and is not
    retained, so the returned field is pure data - no GIL taken during collision checking and no
    reference back into your Python objects.
    """

def createSignedDistanceField(sdf: Callable, domain_min: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], domain_max: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], dimensions: Annotated[NDArray[numpy.int32], dict(shape=(3), order='C')], scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., batched: bool = False) -> SignedDistanceField:
    """
    Create a lazily-evaluated field that keeps the distance function as its source of truth.

    No grid is sampled up front: queries and collision backends call sdf directly (exact, no
    resampling), and the grid is materialized only for serialization or comparison. dimensions is
    the resolution used when that happens.

    Use createDiscreteSignedDistanceField unless you specifically need exact sampling. The field
    keeps sdf alive and re-enters it from C++, which costs you three things: sdf must be
    thread-safe; every evaluation takes the GIL, so a lazy field in a contact manager serializes
    collision checking against the interpreter; and because the field references sdf (and so its
    __globals__), storing the field in a module-level global forms a reference cycle the garbage
    collector cannot see through - drop the field explicitly or keep it out of module scope.
    """

def writeSignedDistanceFieldVDB(sdf: SignedDistanceField) -> bytes:
    """
    Serialize a field as a standard OpenVDB FloatGrid (raises if the voxel spacing is non-uniform)
    """

def readSignedDistanceFieldVDB(data: bytes, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ...) -> SignedDistanceField:
    """
    Reconstruct a field from OpenVDB FloatGrid bytes (exactly one axis-aligned, uniformly scaled grid)
    """

def writeSignedDistanceFieldNVDB(sdf: SignedDistanceField) -> bytes:
    """Serialize a field as a standard NanoVDB FloatGrid file"""

def readSignedDistanceFieldNVDB(data: bytes, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ...) -> SignedDistanceField:
    """
    Reconstruct a field from NanoVDB FloatGrid bytes (exactly one axis-aligned, uniformly scaled grid)
    """

def isIdentical(geom1: Geometry, geom2: Geometry) -> bool:
    """Check if two geometries are identical"""

def extractVertices(geom: Geometry, origin: "Eigen::Transform<double, 3, 1, 0>") -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]:
    """
    Extract vertices from a geometry, transforming primitives to a mesh first
    """

def toTriangleMesh(geom: Geometry, tolerance: float, origin: "Eigen::Transform<double, 3, 1, 0>") -> Mesh:
    """Convert a primitive geometry to a triangle Mesh"""
