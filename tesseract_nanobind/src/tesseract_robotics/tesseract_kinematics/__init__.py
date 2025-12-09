"""tesseract_kinematics Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401
from tesseract_robotics import tesseract_scene_graph  # noqa: F401
from tesseract_robotics import tesseract_state_solver  # noqa: F401

from tesseract_robotics.tesseract_kinematics._tesseract_kinematics import *

__all__ = [
    # Types
    "URParameters",
    "UR10Parameters",
    "UR5Parameters",
    "UR3Parameters",
    "UR10eParameters",
    "UR5eParameters",
    "UR3eParameters",

    # IK Input
    "KinGroupIKInput",
    "KinGroupIKInputs",

    # Kinematics classes
    "ForwardKinematics",
    "InverseKinematics",
    "JointGroup",
    "KinematicGroup",

    # Plugin factory
    "KinematicsPluginFactory",

    # Utility functions
    "getRedundantSolutions",
]
