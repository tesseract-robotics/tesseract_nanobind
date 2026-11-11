"""Profiles for complete pipelines, which run several planners, and the per-pipeline defaults.

Based on the shipped task composer config:

- `FreespacePipeline`: OMPL, then TrajOpt
- `CartesianPipeline`: Descartes, then TrajOpt
- `TrajOptPipeline`: TrajOpt only
- `Raster*Pipeline`: Descartes and TrajOpt for the raster passes, OMPL and TrajOpt for freespace
  transitions
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Final

from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.planning.errors import NoDefaultProfilesError
from tesseract_robotics.planning.profiles.descartes import (
    create_descartes_default_profiles,
    register_descartes,
)
from tesseract_robotics.planning.profiles.names import STANDARD_PROFILE_NAMES
from tesseract_robotics.planning.profiles.ompl import (
    OMPL_MAX_SOLUTIONS,
    OMPL_PLANNING_TIME_S,
    OMPL_RANGE_AUTO,
    create_ompl_default_profiles,
    register_ompl,
)
from tesseract_robotics.planning.profiles.trajopt import (
    create_trajopt_default_profiles,
    create_trajopt_ifopt_default_profiles,
    register_trajopt,
    register_trajopt_ifopt,
)


def create_freespace_pipeline_profiles(
    profile_names: Sequence[str] | None = None,
    num_planners: int | None = None,
    planning_time: float = OMPL_PLANNING_TIME_S,
    optimize: bool = True,
    max_solutions: int = OMPL_MAX_SOLUTIONS,
    planner_range: float = OMPL_RANGE_AUTO,
) -> ProfileDictionary:
    """Create profiles for FreespacePipeline (OMPL + TrajOpt).

    The FreespacePipeline is a two-stage approach for collision-free motion:
    1. OMPL finds a feasible collision-free path (fast, approximate)
    2. TrajOpt optimizes the path for smoothness and exact collision margins

    This combines the strengths of sampling-based planning (global search)
    and optimization-based planning (solution quality).

    Pipeline Flow:
        Input: Start/goal joint states
        -> OMPL: Parallel RRTConnect planners find collision-free path
        -> TrajOpt: Optimize path for smoothness + collision margins
        -> Output: Smooth, collision-free trajectory

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)
            Standard names: ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]
        num_planners: Number of parallel OMPL planners (default: min(4, CPU count))
            More planners = higher success rate for difficult problems
        planning_time: OMPL planning time in seconds (default: 10.0)
            TrajOpt has no timeout (runs until convergence)
        optimize: Continue planning to find better solutions (default: True)
            If True, runs for full planning_time to improve solution quality
            If False, returns first valid solution (faster, shows parallel speedup)
        max_solutions: Stop early after finding this many solutions (default: 10)
        planner_range: RRTConnect `range`, the longest extension step in rad of joint-space
            norm (default: 0, leaving OMPL's estimate)

    Returns:
        ProfileDictionary with both OMPL and TrajOpt profiles

    Raises:
        InvalidProfileArgumentError: an OMPL argument is out of range.

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

        # Standard freespace planning
        profiles = create_freespace_pipeline_profiles()

        # Difficult environments: more time + parallelism
        profiles = create_freespace_pipeline_profiles(planning_time=10.0, num_planners=16)
        ```
    """
    names = profile_names or STANDARD_PROFILE_NAMES
    profiles = ProfileDictionary()
    register_ompl(
        profiles,
        names,
        planning_time=planning_time,
        max_solutions=max_solutions,
        optimize=optimize,
        num_planners=num_planners,
        planner_range=planner_range,
    )
    register_trajopt(profiles, names)
    return profiles


def create_freespace_ifopt_pipeline_profiles(
    profile_names: Sequence[str] | None = None,
    num_planners: int | None = None,
    planning_time: float = OMPL_PLANNING_TIME_S,
    optimize: bool = True,
    max_solutions: int = OMPL_MAX_SOLUTIONS,
    planner_range: float = OMPL_RANGE_AUTO,
) -> ProfileDictionary:
    """Create profiles for FreespaceIfoptPipeline (OMPL + TrajOptIfopt).

    Same two-stage approach as FreespacePipeline but uses TrajOptIfopt (SQP/OSQP)
    instead of TrajOpt (SCO) for the optimization step.

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)
        num_planners: Number of parallel OMPL planners (default: min(4, CPU count))
        planning_time: OMPL planning time in seconds (default: 10.0)
        optimize: Continue planning to find better solutions (default: True)
        max_solutions: Stop early after finding this many solutions (default: 10)
        planner_range: RRTConnect `range`, the longest extension step in rad of joint-space
            norm (default: 0, leaving OMPL's estimate)

    Returns:
        ProfileDictionary with both OMPL and TrajOptIfopt profiles

    Raises:
        InvalidProfileArgumentError: an OMPL argument is out of range.
    """
    names = profile_names or STANDARD_PROFILE_NAMES
    profiles = ProfileDictionary()
    register_ompl(
        profiles,
        names,
        planning_time=planning_time,
        max_solutions=max_solutions,
        optimize=optimize,
        num_planners=num_planners,
        planner_range=planner_range,
    )
    register_trajopt_ifopt(profiles, names)
    return profiles


def create_cartesian_pipeline_profiles(
    profile_names: Sequence[str] | None = None,
    num_threads: int | None = None,
) -> ProfileDictionary:
    """Create profiles for CartesianPipeline (Descartes + TrajOpt).

    The CartesianPipeline handles motion with Cartesian (tool pose) constraints:
    1. Descartes samples IK solutions and finds minimum-cost joint path
    2. TrajOpt optimizes the path for smoothness and collision margins

    This is ideal for tasks with tool orientation constraints (welding, painting,
    milling, etc.) where the tool must follow a specific Cartesian path.

    Pipeline Flow:
        Input: Cartesian waypoints (tool poses)
        -> Descartes: Sample IK solutions at each waypoint
        -> Descartes: Find minimum-cost path through ladder graph
        -> TrajOpt: Optimize path for smoothness + collision margins
        -> Output: Smooth trajectory following Cartesian path

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)
            Standard names: ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]
        num_threads: Number of Descartes solver threads (default: CPU count)
            More threads = faster graph search for long paths

    Returns:
        ProfileDictionary with both Descartes and TrajOpt profiles

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_cartesian_pipeline_profiles

        # Standard Cartesian planning
        profiles = create_cartesian_pipeline_profiles()

        # Custom thread count for large raster paths
        profiles = create_cartesian_pipeline_profiles(num_threads=8)
        ```
    """
    names = profile_names or STANDARD_PROFILE_NAMES
    profiles = ProfileDictionary()
    register_descartes(profiles, names, num_threads=num_threads)
    register_trajopt(profiles, names)
    return profiles


def create_raster_pipeline_profiles(
    profile_names: Sequence[str] | None = None,
    num_threads: int | None = None,
    num_planners: int | None = None,
) -> ProfileDictionary:
    """Create profiles for the Raster pipelines (Descartes + OMPL + TrajOpt).

    A raster pipeline plans each raster pass as a Cartesian task (Descartes, then TrajOpt) and the
    transitions between passes as freespace tasks (OMPL, then TrajOpt) or, in the `Ct` variants,
    as Cartesian tasks. The dictionary holds all three planners' profiles so every variant finds
    its own.

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)
        num_threads: Number of Descartes solver threads (default: CPU count)
        num_planners: Number of parallel OMPL planners (default: min(4, CPU count))

    Returns:
        ProfileDictionary with Descartes, OMPL and TrajOpt profiles.
    """
    names = profile_names or STANDARD_PROFILE_NAMES
    profiles = ProfileDictionary()
    register_descartes(profiles, names, num_threads=num_threads)
    register_ompl(profiles, names, num_planners=num_planners)
    register_trajopt(profiles, names)
    return profiles


# Pipelines the default profiles are defined for, by their configured base name; each runs as
# `<base>Pipeline` and as its `<base>Task`. Raster pipelines take `Raster{Ft,Ct}[Only][Global]`.
_RASTER_BASES: Final = tuple(
    f"Raster{transition}{only}{scope}"
    for transition in ("Ft", "Ct")
    for only in ("", "Only")
    for scope in ("", "Global")
)

_DEFAULT_PROFILE_FACTORIES_BY_BASE: Final[Mapping[str, Callable[[], ProfileDictionary]]] = {
    "TrajOpt": create_trajopt_default_profiles,
    "TrajOptIfopt": create_trajopt_ifopt_default_profiles,
    "OMPL": create_ompl_default_profiles,
    "Freespace": create_freespace_pipeline_profiles,
    "FreespaceIfopt": create_freespace_ifopt_pipeline_profiles,
    "Cartesian": create_cartesian_pipeline_profiles,
    **{
        f"Descartes{scalar}{variant}": create_descartes_default_profiles
        for scalar in ("F", "D")
        for variant in ("", "NPC")
    },
    **{base: create_raster_pipeline_profiles for base in _RASTER_BASES},
}

DEFAULT_PROFILE_FACTORIES: Final[Mapping[str, Callable[[], ProfileDictionary]]] = {
    f"{base}{suffix}": factory
    for base, factory in _DEFAULT_PROFILE_FACTORIES_BY_BASE.items()
    for suffix in ("Pipeline", "Task")
}


def default_profiles_for(pipeline: str) -> ProfileDictionary:
    """Default profiles of a shipped pipeline or task.

    Args:
        pipeline: A configured node name such as "FreespacePipeline" or "RasterCtTask".

    Returns:
        A new dictionary with the profiles `pipeline` reads.

    Raises:
        NoDefaultProfilesError: `pipeline` is not one of the shipped pipelines; build its
            profiles and pass them as `profiles=`.
    """
    try:
        factory = DEFAULT_PROFILE_FACTORIES[pipeline]
    except KeyError:
        raise NoDefaultProfilesError(
            f"No default profiles for pipeline '{pipeline}'; pass profiles= explicitly. "
            f"Pipelines with defaults: {', '.join(sorted(DEFAULT_PROFILE_FACTORIES))}"
        ) from None
    return factory()
