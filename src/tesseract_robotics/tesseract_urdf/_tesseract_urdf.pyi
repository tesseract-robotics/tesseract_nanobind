"""tesseract_urdf Python bindings"""

import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph


def parseURDFString(urdf_xml_string: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph:
    """Parse a URDF string into a SceneGraph"""

def parseURDFFile(path: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph:
    """Parse a URDF file into a SceneGraph"""

def writeURDFFile(scene_graph: tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph.SceneGraph, package_path: str, urdf_name: str = '') -> None:
    """Write a SceneGraph to a URDF file"""
