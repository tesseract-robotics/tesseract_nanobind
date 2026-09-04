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
    # Mesh types
    "Mesh",
    "ConvexMesh",
    "CompoundMesh",
    # Signed distance field
    "SignedDistanceField",
    "createDiscreteSignedDistanceField",
    "createSignedDistanceField",
    "readSignedDistanceFieldNVDB",
    "readSignedDistanceFieldVDB",
    "writeSignedDistanceFieldNVDB",
    "writeSignedDistanceFieldVDB",
    # Octree
    "OcTree",
    "OctreeSubType",
    "PointCloud",
    "PointCloudPoint",
    "Octree",
    "createOctree",
    # Mesh loading functions
    "createMeshFromPath",
    "createConvexMeshFromPath",
    # Utilities / conversions
    "isIdentical",
    "extractVertices",
    "toTriangleMesh",
]
