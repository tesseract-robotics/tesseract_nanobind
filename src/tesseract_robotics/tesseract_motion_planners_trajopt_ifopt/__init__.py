# tesseract_motion_planners_trajopt_ifopt Python bindings
# TrajOptCollisionConfig lives in trajopt_ifopt (trajopt_common) and SQPParameters in
# trajopt_sqp; both are needed to configure TrajOptIfoptDefaultCompositeProfile's
# collision_{cost,constraint}_config and TrajOptIfoptSolverProfile's opt_params, so
# re-export them here for parity with tesseract_motion_planners_trajopt.
# CollisionEvaluatorType lives in tesseract_collision (0.33 API) — re-exported too.
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_motion_planners._profile_overrides import (
    ProfileOverrideNotDispatchedError,
    refuse_hook_overrides,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.trajopt_sqp import SQPParameters

from ._tesseract_motion_planners_trajopt_ifopt import *

refuse_hook_overrides(
    TrajOptIfoptDefaultMoveProfile,
    ("create",),
    "Subclass TrajOptIfoptMoveProfile and hold this profile to delegate to it.",
)
refuse_hook_overrides(
    TrajOptIfoptDefaultCompositeProfile,
    ("create",),
    "Subclass TrajOptIfoptCompositeProfile and hold this profile to delegate to it.",
)
refuse_hook_overrides(
    TrajOptIfoptOSQPSolverProfile,
    ("create", "createOptimizationCallbacks"),
    "Solver profiles cannot be authored in Python: configure the solver through its fields "
    "and add SQP callbacks to `callbacks`.",
)

__all__ = [
    "ProfileOverrideNotDispatchedError",
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
    "TrajOptIfoptProfileReturnTypeError",
    "TrajOptIfoptPureVirtualCallError",
    "TrajOptIfoptSolverProfile",
    "TrajOptIfoptTermInfos",
    "TrajOptIfoptWaypointInfo",
    "TrajOptIfoptWaypointInfoInUseError",
    "TrajOptIfoptWaypointInfoWithoutNodeError",
    "createCollisionConstraints",
    "createJointAccelerationConstraint",
    "createJointJerkConstraint",
    "createJointPositionConstraint",
    "createJointVelocityConstraint",
]
