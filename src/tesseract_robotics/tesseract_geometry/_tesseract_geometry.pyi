"""tesseract_geometry Python bindings"""

from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_common._tesseract_common


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

    SDF_MESH = 9

    OCTREE = 10

    POLYGON_MESH = 11

    COMPOUND_MESH = 12

class Geometry:
    def getType(self) -> GeometryType:
        """Get the geometry type"""

    def clone(self) -> Geometry:
        """Create a copy of this geometry"""

    def getUUID(self) -> str:
        """UUID as its canonical string"""

    def setUUID(self, uuid: str) -> None:
        """
        Set the UUID from its canonical string; a malformed string raises ValueError
        """

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
    def __init__(self, texture_image: tesseract_robotics.tesseract_common._tesseract_common.Resource, uvs: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(2), order='C')]]) -> None:
        """Create a texture from a jpg or png image resource and per-vertex UVs"""

    def getTextureImage(self) -> tesseract_robotics.tesseract_common._tesseract_common.Resource:
        """Get the texture image resource"""

    def getUVs(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(2), order='C')]]:
        """Get UV coordinates"""

class MeshFaceCountError(ValueError):
    """
    A mesh constructor's face_count disagrees with the number of faces its faces array describes.
    """

class PolygonMesh(Geometry):
    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None, type: GeometryType = GeometryType.POLYGON_MESH) -> None: ...

    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], face_count: int, resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None, type: GeometryType = GeometryType.POLYGON_MESH) -> None: ...

    def __eq__(self, arg: PolygonMesh, /) -> bool: ...

    def __ne__(self, arg: PolygonMesh, /) -> bool: ...

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

    def getResource(self) -> tesseract_robotics.tesseract_common._tesseract_common.Resource:
        """Get mesh resource"""

class Mesh(PolygonMesh):
    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], face_count: int, resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    def __eq__(self, arg: Mesh, /) -> bool: ...

    def __ne__(self, arg: Mesh, /) -> bool: ...

class ConvexMesh(PolygonMesh):
    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], face_count: int, resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    class CreationMethod(enum.Enum):
        DEFAULT = 0

        MESH = 1

        CONVERTED = 2

    def __eq__(self, arg: ConvexMesh, /) -> bool: ...

    def __ne__(self, arg: ConvexMesh, /) -> bool: ...

    def getCreationMethod(self) -> ConvexMesh.CreationMethod:
        """
        How the mesh was made; the URDF parser sets CONVERTED for tesseract:make_convex
        """

    def setCreationMethod(self, method: ConvexMesh.CreationMethod) -> None:
        """Set how the mesh was made"""

class SDFMesh(PolygonMesh):
    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    @overload
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], face_count: int, resource: tesseract_robotics.tesseract_common._tesseract_common.Resource | None = None, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., normals: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]] | None = None, vertex_colors: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]] | None = None, mesh_material: MeshMaterial | None = None, mesh_textures: Sequence[MeshTexture] | None = None) -> None: ...

    def __eq__(self, arg: SDFMesh, /) -> bool: ...

    def __ne__(self, arg: SDFMesh, /) -> bool: ...

class CompoundMesh(Geometry):
    def __init__(self, meshes: Sequence[PolygonMesh]) -> None: ...

    def getMeshes(self) -> list[PolygonMesh]:
        """Get the vector of meshes"""

    def getResource(self) -> tesseract_robotics.tesseract_common._tesseract_common.Resource:
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

class PointCloud:
    def __init__(self) -> None: ...

    class Point:
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

    @property
    def points(self) -> list[PointCloud.Point]: ...

    @points.setter
    def points(self, arg: Sequence[PointCloud.Point], /) -> None: ...

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

def createMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[Mesh]:
    """Load mesh from file and return vector of Mesh geometries"""

def createConvexMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[ConvexMesh]:
    """Load mesh from file and return vector of ConvexMesh geometries"""

def createSDFMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[SDFMesh]:
    """Load mesh from file and return vector of SDFMesh geometries"""

def createMeshFromResource(resource: tesseract_robotics.tesseract_common._tesseract_common.Resource, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[Mesh]:
    """Load Mesh from resource (e.g., package:// URL)"""

def createConvexMeshFromResource(resource: tesseract_robotics.tesseract_common._tesseract_common.Resource, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[ConvexMesh]:
    """Load ConvexMesh from resource (e.g., package:// URL)"""

def createSDFMeshFromResource(resource: tesseract_robotics.tesseract_common._tesseract_common.Resource, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[SDFMesh]:
    """Load SDFMesh from resource (e.g., package:// URL)"""

def createMeshFromBytes(url: str, data: bytes, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[Mesh]:
    """
    Load Mesh geometries from an in-memory mesh file; url's extension (.stl, .dae, ...) selects the format
    """

def createConvexMeshFromBytes(url: str, data: bytes, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[ConvexMesh]:
    """
    Load ConvexMesh geometries from an in-memory mesh file; url's extension selects the format
    """

def createSDFMeshFromBytes(url: str, data: bytes, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False, normals: bool = False, vertex_colors: bool = False, material_and_texture: bool = False) -> list[SDFMesh]:
    """
    Load SDFMesh geometries from an in-memory mesh file; url's extension selects the format
    """

def isIdentical(geom1: Geometry, geom2: Geometry) -> bool:
    """Check if two geometries are identical"""

def extractVertices(geom: Geometry, origin: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]:
    """
    Extract vertices from a geometry, transforming primitives to a mesh first
    """

def toTriangleMesh(geom: Geometry, tolerance: float, origin: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> Mesh:
    """Convert a primitive geometry to a triangle Mesh"""
