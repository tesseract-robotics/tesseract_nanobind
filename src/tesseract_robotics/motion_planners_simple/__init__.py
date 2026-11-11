from tesseract_robotics.motion_planners._profile_overrides import (
    ProfileOverrideNotDispatchedError as ProfileOverrideNotDispatchedError,
)
from tesseract_robotics.motion_planners._profile_overrides import (
    refuse_hook_overrides,
)
from tesseract_robotics.motion_planners_simple._motion_planners_simple import *

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
