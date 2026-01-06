from collections.abc import Sequence
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

    SDF_MESH = 9

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

class SDFMesh(PolygonMesh):
    def __init__(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]) -> None: ...

class CompoundMesh(Geometry):
    def __init__(self, meshes: Sequence[PolygonMesh]) -> None: ...

    def getMeshes(self) -> list[PolygonMesh]:
        """Get the vector of meshes"""

    def getResource(self) -> "tesseract_common::Resource":
        """Get the resource used to create this mesh"""

    def getScale(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Get the scale applied to the mesh"""

def createMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[Mesh]:
    """Load mesh from file and return vector of Mesh geometries"""

def createConvexMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[ConvexMesh]:
    """Load mesh from file and return vector of ConvexMesh geometries"""

def createSDFMeshFromPath(path: str, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[SDFMesh]:
    """Load mesh from file and return vector of SDFMesh geometries"""

def createMeshFromResource(resource: "tesseract_common::Resource", scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[Mesh]:
    """Load Mesh from resource (e.g., package:// URL)"""

def createConvexMeshFromResource(resource: "tesseract_common::Resource", scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[ConvexMesh]:
    """Load ConvexMesh from resource (e.g., package:// URL)"""

def createSDFMeshFromResource(resource: "tesseract_common::Resource", scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ..., triangulate: bool = True, flatten: bool = False) -> list[SDFMesh]:
    """Load SDFMesh from resource (e.g., package:// URL)"""
