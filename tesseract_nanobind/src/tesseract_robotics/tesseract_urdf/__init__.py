"""tesseract_urdf Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401
from tesseract_robotics import tesseract_scene_graph  # noqa: F401

from tesseract_robotics.tesseract_urdf._tesseract_urdf import *

__all__ = [
    "parseURDFString",
    "parseURDFFile",
    "writeURDFFile",
]
