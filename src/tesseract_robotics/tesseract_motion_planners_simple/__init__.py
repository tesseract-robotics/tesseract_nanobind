from tesseract_robotics.tesseract_motion_planners._profile_overrides import (
    ProfileOverrideNotDispatchedError,
    refuse_hook_overrides,
)
from tesseract_robotics.tesseract_motion_planners_simple._tesseract_motion_planners_simple import *

for _concrete in (
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
):
    refuse_hook_overrides(
        _concrete,
        ("generate",),
        "Subclass SimplePlannerMoveProfile and hold this profile to delegate to it.",
    )
del _concrete

__all__ = [
    "ProfileOverrideNotDispatchedError",
    "SimpleMotionPlanner",
    "generateInterpolatedProgram",
    "interpolate",
    "interpolate_waypoint",
    "getInterpolatedInstructions",
    "JointGroupInstructionInfo",
    "KinematicGroupInstructionInfo",
    "interpolateJointJointWaypoint",
    "interpolateJointCartWaypoint",
    "interpolateCartJointWaypoint",
    "interpolateCartCartWaypoint",
    "getClosestJointSolution",
    "SimplePlannerMoveProfile",
    "SimplePlannerPureVirtualCallError",
    "SimplePlannerCompositeProfile",
    "SimplePlannerFixedSizeMoveProfile",
    "SimplePlannerFixedSizeAssignMoveProfile",
    "SimplePlannerFixedSizeAssignNoIKMoveProfile",
    "SimplePlannerLVSMoveProfile",
    "SimplePlannerLVSNoIKMoveProfile",
    "SimplePlannerLVSAssignMoveProfile",
    "SimplePlannerLVSAssignNoIKMoveProfile",
]
