"""tesseract_urdf Python bindings"""

import pathlib

import tesseract_robotics.common
import tesseract_robotics.geometry
import tesseract_robotics.scene_graph


def parseURDFString(urdf_xml_string: str, locator: tesseract_robotics.common.ResourceLocator) -> tesseract_robotics.scene_graph.SceneGraph:
    """Parse a URDF string into a SceneGraph"""

def parseURDFFile(path: str, locator: tesseract_robotics.common.ResourceLocator) -> tesseract_robotics.scene_graph.SceneGraph:
    """Parse a URDF file into a SceneGraph"""

class URDFWriteError(RuntimeError):
    """
    writeURDFFile could not write the scene graph; the message names the cause.
    """

def writeURDFFile(scene_graph: tesseract_robotics.scene_graph.SceneGraph, package_path: str, urdf_name: str = '') -> pathlib.Path:
    """
    Write `scene_graph` to `package_path`/urdf/`urdf_name`.urdf (`urdf_name` is a stem; default: the graph's name), its meshes as PLY files under `package_path`/visual and /collision. Returns the URDF's path. Raises URDFWriteError naming the cause.
    """

def writeMeshToFile(mesh: tesseract_robotics.geometry.PolygonMesh, filepath: str) -> None:
    """
    Write the mesh as an ASCII PLY file (always PLY, whatever the extension of filepath).
    """
