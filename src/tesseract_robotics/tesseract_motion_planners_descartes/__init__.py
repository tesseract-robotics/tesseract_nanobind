# tesseract_motion_planners_descartes Python bindings
from tesseract_robotics.tesseract_motion_planners._profile_overrides import (
    ProfileOverrideNotDispatchedError,
    refuse_hook_overrides,
)

from ._tesseract_motion_planners_descartes import *

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

__all__ = [
    "ProfileOverrideNotDispatchedError",
    "DescartesDefaultMoveProfileD",
    "DescartesDefaultPlanProfileD",
    "DescartesEdgeEvaluatorD",
    "DescartesLadderGraphSolverProfileD",
    "DescartesMotionPlannerD",
    "DescartesMoveProfileD",
    "DescartesPlanProfileD",
    "DescartesSolverProfileD",
    "DescartesStateD",
    "DescartesStateEvaluatorD",
    "DescartesStateSampleD",
    "DescartesWaypointSamplerD",
    "cast_DescartesMoveProfileD",
    "cast_DescartesPlanProfileD",
    "cast_DescartesSolverProfileD",
    "sampleFixed",
    "sampleToolAxis",
    "sampleToolXAxis",
    "sampleToolYAxis",
    "sampleToolZAxis",
]
