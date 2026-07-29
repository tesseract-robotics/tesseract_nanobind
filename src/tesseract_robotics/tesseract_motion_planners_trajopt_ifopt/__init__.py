# tesseract_motion_planners_trajopt_ifopt Python bindings
# TrajOptCollisionConfig lives in trajopt_ifopt (trajopt_common) and SQPParameters in
# trajopt_sqp; both are needed to configure TrajOptIfoptDefaultCompositeProfile's
# collision_{cost,constraint}_config and TrajOptIfoptSolverProfile's opt_params, so
# re-export them here for parity with tesseract_motion_planners_trajopt.
# CollisionEvaluatorType lives in tesseract_collision (0.33 API) — re-exported too.
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.trajopt_sqp import SQPParameters

from ._tesseract_motion_planners_trajopt_ifopt import *

__all__ = [
    "CollisionEvaluatorType",
    "ProfileDictionary_addTrajOptIfoptCompositeProfile",
    "ProfileDictionary_addTrajOptIfoptMoveProfile",
    "ProfileDictionary_addTrajOptIfoptPlanProfile",
    "ProfileDictionary_addTrajOptIfoptSolverProfile",
    "SQPParameters",
    "TrajOptCollisionConfig",
    "TrajOptIfoptCartesianWaypointConfig",
    "TrajOptIfoptCompositeProfile",
    "TrajOptIfoptDefaultCompositeProfile",
    "TrajOptIfoptDefaultMoveProfile",
    "TrajOptIfoptDefaultPlanProfile",
    "TrajOptIfoptJointWaypointConfig",
    "TrajOptIfoptMotionPlanner",
    "TrajOptIfoptMoveProfile",
    "TrajOptIfoptOSQPSolverProfile",
    "TrajOptIfoptPlanProfile",
    "TrajOptIfoptSolverProfile",
]
