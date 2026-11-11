# tesseract_motion_planners_descartes Python bindings
from tesseract_robotics.motion_planners._profile_overrides import (
    ProfileOverrideNotDispatchedError as ProfileOverrideNotDispatchedError,
)
from tesseract_robotics.motion_planners._profile_overrides import (
    refuse_hook_overrides,
)

from ._motion_planners_descartes import *

refuse_hook_overrides(
    DescartesDefaultMoveProfileD,
    ("createWaypointSampler", "createEdgeEvaluator", "createStateEvaluator"),
    "Subclass DescartesMoveProfileD and hold this profile to delegate to it.",
)
refuse_hook_overrides(
    DescartesLadderGraphSolverProfileD,
    ("create",),
    "Solver profiles cannot be authored in Python: configure the solver through its fields.",
)
refuse_hook_overrides(
    DescartesDefaultMoveProfileF,
    ("createWaypointSampler", "createEdgeEvaluator", "createStateEvaluator"),
    "Subclass DescartesMoveProfileF and hold this profile to delegate to it.",
)
refuse_hook_overrides(
    DescartesLadderGraphSolverProfileF,
    ("create",),
    "Solver profiles cannot be authored in Python: configure the solver through its fields.",
)
