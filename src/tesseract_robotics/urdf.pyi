"""tesseract_urdf Python bindings"""

import tesseract_robotics.common._common
import tesseract_robotics.geometry
import tesseract_robotics.scene_graph._scene_graph


def parseURDFString(urdf_xml_string: str, locator: tesseract_robotics.common._common.ResourceLocator) -> tesseract_robotics.scene_graph._scene_graph.SceneGraph:
    """Parse a URDF string into a SceneGraph"""

def parseURDFFile(path: str, locator: tesseract_robotics.common._common.ResourceLocator) -> tesseract_robotics.scene_graph._scene_graph.SceneGraph:
    """Parse a URDF file into a SceneGraph"""

def writeURDFFile(scene_graph: tesseract_robotics.scene_graph._scene_graph.SceneGraph, package_path: str, urdf_name: str = '') -> None:
    """Write a SceneGraph to a URDF file"""

def writeMeshToFile(mesh: tesseract_robotics.geometry.PolygonMesh, filepath: str) -> None:
    """
    Write the mesh as an ASCII PLY file (always PLY, whatever the extension of filepath).
    """
