"""tesseract_geometry Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401

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
    "SDFMesh",
    "CompoundMesh",

    # Mesh loading functions
    "createMeshFromPath",
    "createConvexMeshFromPath",
    "createSDFMeshFromPath",
]
