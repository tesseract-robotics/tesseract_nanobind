"""tesseract_geometry Python bindings (nanobind)"""

from tesseract_robotics.tesseract_geometry._tesseract_geometry import *

__all__ = [
    # Enum
    "GeometryType",
    # Base class
    "Geometry",
    # Vector types
    "GeometriesConst",
    # Primitive geometries
    "Box",
    "Sphere",
    "Cylinder",
    "Capsule",
    "Cone",
    "Plane",
    # Material/Texture
    "MeshMaterial",
    "MeshTexture",
    # Mesh base
    "PolygonMesh",
    "MeshFaceCountError",
    # Mesh types
    "Mesh",
    "ConvexMesh",
    "SDFMesh",
    "CompoundMesh",
    # Octree
    "OcTree",
    "OctreeSubType",
    "PointCloud",
    "Octree",
    "createOctree",
    # Mesh loading functions
    "createMeshFromPath",
    "createConvexMeshFromPath",
    "createSDFMeshFromPath",
    "createMeshFromResource",
    "createConvexMeshFromResource",
    "createSDFMeshFromResource",
    "createMeshFromBytes",
    "createConvexMeshFromBytes",
    "createSDFMeshFromBytes",
    # Utilities / conversions
    "isIdentical",
    "extractVertices",
    "toTriangleMesh",
]
