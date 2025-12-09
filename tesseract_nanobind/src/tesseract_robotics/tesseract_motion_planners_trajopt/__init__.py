"""tesseract_motion_planners_trajopt Python bindings (nanobind)"""

# Import dependencies first to ensure C++ types are registered
from tesseract_robotics import tesseract_common  # noqa: F401
from tesseract_robotics import tesseract_environment  # noqa: F401
from tesseract_robotics import tesseract_command_language  # noqa: F401
from tesseract_robotics import tesseract_motion_planners  # noqa: F401

from ._tesseract_motion_planners_trajopt import *
