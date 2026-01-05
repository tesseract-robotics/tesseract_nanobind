# tesseract_motion_planners_trajopt Python bindings
# 0.33 API: TrajOptCollisionConfig is in trajopt_ifopt (trajopt_common)
# Re-export for backwards compatibility
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

from ._tesseract_motion_planners_trajopt import *

__all__ = [
    "ProfileDictionary_addTrajOptCompositeProfile",
    "ProfileDictionary_addTrajOptMoveProfile",
    "ProfileDictionary_addTrajOptPlanProfile",
    "TrajOptCartesianWaypointConfig",
    "TrajOptCollisionConfig",
    "TrajOptCompositeProfile",
    "TrajOptDefaultCompositeProfile",
    "TrajOptDefaultMoveProfile",
    "TrajOptDefaultPlanProfile",
    "TrajOptJointWaypointConfig",
    "TrajOptMotionPlanner",
    "TrajOptMoveProfile",
    "TrajOptPlanProfile",
]
