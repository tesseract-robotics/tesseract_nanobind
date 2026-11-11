"""Profile names and namespaces the planning profile helpers register under."""

from __future__ import annotations

from typing import Final

# Standard profile names used across examples and pipelines
STANDARD_PROFILE_NAMES: Final[tuple[str, ...]] = (
    "DEFAULT",
    "FREESPACE",
    "CARTESIAN",
    "RASTER",
    "UPRIGHT",
)

# OMPL: only freespace planning, Cartesian/Raster/Upright don't apply
OMPL_PROFILE_NAMES: Final[tuple[str, ...]] = ("DEFAULT", "FREESPACE")

# Descartes: Cartesian sampling, freespace doesn't apply
DESCARTES_PROFILE_NAMES: Final[tuple[str, ...]] = ("DEFAULT", "CARTESIAN", "RASTER")

# Profile name of the orientation-constrained (glass upright) TrajOpt profiles
UPRIGHT_PROFILE_NAMES: Final[tuple[str, ...]] = ("UPRIGHT",)

TRAJOPT_DEFAULT_NAMESPACE: Final = "TrajOptMotionPlannerTask"
TRAJOPT_IFOPT_DEFAULT_NAMESPACE: Final = "TrajOptIfoptMotionPlannerTask"
OMPL_DEFAULT_NAMESPACE: Final = "OMPLMotionPlannerTask"
DESCARTES_DEFAULT_NAMESPACE: Final = "DescartesMotionPlannerTask"
