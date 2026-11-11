"""tesseract_common Python bindings (nanobind)"""

import numpy as np

from tesseract_robotics.common._common import *

# Standard unit axes — Vector3d-compatible, frozen to guard against accidental
# in-place mutation when the same constant is reused across call sites.
X_AXIS: np.ndarray = np.array([1.0, 0.0, 0.0])
Y_AXIS: np.ndarray = np.array([0.0, 1.0, 0.0])
Z_AXIS: np.ndarray = np.array([0.0, 0.0, 1.0])
X_AXIS.flags.writeable = False
Y_AXIS.flags.writeable = False
Z_AXIS.flags.writeable = False
