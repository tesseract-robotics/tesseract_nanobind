"""OMPL profile helpers: parallel RRTConnect move profiles and planner configurators."""

from __future__ import annotations

import inspect
import os
from collections.abc import Sequence
from typing import Final

from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.planning.errors import InvalidProfileArgumentError
from tesseract_robotics.planning.profiles.names import OMPL_DEFAULT_NAMESPACE, OMPL_PROFILE_NAMES


class UnknownPlannerTypeError(ValueError):
    """A planner name is not an `OMPLPlannerType` member."""


class UnknownConfiguratorFieldError(ValueError):
    """A keyword argument is a field of none of the requested OMPL configurators."""


# Collision margin of the OMPL state validity check (m): a state closer than 10 mm to an obstacle
# is invalid. The TrajOpt margins are 0, 5 and 10 mm, so this equals the largest. Enough for OMPL
# to return paths that are clearly collision-free.
OMPL_CONTACT_MARGIN_M: Final = 0.01

# Default wall-clock budget of all OMPL planners together (s): long enough for RRTConnect to find
# and improve a path in the example scenes.
OMPL_PLANNING_TIME_S: Final = 10.0

# Default number of solutions after which OMPL stops early, whatever time is left.
OMPL_MAX_SOLUTIONS: Final = 10

# Upper bound on the default number of parallel RRTConnect planners. A planner per CPU overwhelms
# the machine when test suites run in parallel; four give a good chance of a quick first solution.
OMPL_DEFAULT_MAX_PLANNERS: Final = 4

# `planner_range` that leaves RRTConnect's `range` (maximum extension step, rad of joint-space
# norm) at OMPL's own estimate from the extent of the state space.
OMPL_RANGE_AUTO: Final = 0.0


def _is_configurator_field(cls: type, key: str) -> bool:
    """Whether `key` is a bound data field (a property) of configurator class `cls`.

    Methods such as `getType` are attributes too, so `hasattr` would accept them.
    """
    return isinstance(inspect.getattr_static(cls, key, None), property)


def _validate_ompl_arguments(
    planning_time: float, max_solutions: int, num_planners: int, planner_range: float
) -> None:
    if not planning_time > 0.0:
        raise InvalidProfileArgumentError(f"planning_time must be positive, got {planning_time}")
    if max_solutions < 1:
        raise InvalidProfileArgumentError(f"max_solutions must be at least 1, got {max_solutions}")
    if num_planners < 1:
        raise InvalidProfileArgumentError(f"num_planners must be at least 1, got {num_planners}")
    if planner_range < 0.0:
        raise InvalidProfileArgumentError(
            f"planner_range must not be negative, got {planner_range}"
        )


def register_ompl(
    profiles: ProfileDictionary,
    profile_names: Sequence[str] = OMPL_PROFILE_NAMES,
    planning_time: float = OMPL_PLANNING_TIME_S,
    max_solutions: int = OMPL_MAX_SOLUTIONS,
    optimize: bool = True,
    simplify: bool = False,
    num_planners: int | None = None,
    planner_range: float = OMPL_RANGE_AUTO,
) -> None:
    """Add parallel-RRTConnect OMPL move profiles to `profiles` under every name.

    Each name gets its own profile instance, so editing one name's profile leaves the others.

    Args:
        profiles: Dictionary to add to.
        profile_names: Profile names to register.
        planning_time: Wall-clock budget of all planners together (s).
        max_solutions: Stop once this many solutions are found.
        optimize: Use the whole planning time to improve the solution.
        simplify: Simplify the path after planning (may remove waypoints).
        num_planners: Parallel RRTConnect planners; None is `min(OMPL_DEFAULT_MAX_PLANNERS,
            CPU count)`.
        planner_range: RRTConnect `range`, the longest extension step (rad of joint-space norm);
            `OMPL_RANGE_AUTO` (0) leaves OMPL's estimate.

    Raises:
        InvalidProfileArgumentError: `planning_time`, `max_solutions`, `num_planners` or
            `planner_range` is out of range.
    """
    from tesseract_robotics.motion_planners_ompl import (
        OMPLRealVectorMoveProfile,
        RRTConnectConfigurator,
    )

    if num_planners is None:
        num_planners = min(OMPL_DEFAULT_MAX_PLANNERS, os.cpu_count() or 1)
    _validate_ompl_arguments(planning_time, max_solutions, num_planners, planner_range)

    for name in profile_names:
        profile = OMPLRealVectorMoveProfile()
        profile.solver_config.planning_time = planning_time
        profile.solver_config.max_solutions = max_solutions
        profile.solver_config.optimize = optimize
        profile.solver_config.simplify = simplify
        profile.contact_manager_config.default_margin = OMPL_CONTACT_MARGIN_M

        # Parallel RRTConnect planners (the C++ default is 2)
        profile.solver_config.clearPlanners()
        for _ in range(num_planners):
            cfg = RRTConnectConfigurator()
            if planner_range > OMPL_RANGE_AUTO:
                cfg.range = planner_range
            profile.solver_config.addPlanner(cfg)

        profiles.addProfile(OMPL_DEFAULT_NAMESPACE, name, profile)


def create_ompl_default_profiles(
    profile_names: Sequence[str] | None = None,
    planning_time: float = OMPL_PLANNING_TIME_S,
    max_solutions: int = OMPL_MAX_SOLUTIONS,
    optimize: bool = True,
    simplify: bool = False,
    num_planners: int | None = None,
    planner_range: float = OMPL_RANGE_AUTO,
) -> ProfileDictionary:
    """Create OMPL profiles with sensible defaults.

    OMPL (Open Motion Planning Library) provides sampling-based motion planners
    that search the configuration space for collision-free paths. This function
    creates profiles configured with parallel RRTConnect planners.

    Profile Configuration:
        - Parallel RRTConnect planners (`min(4, CPU count)` by default)
        - Planning runs until timeout OR max_solutions found
        - Optimization enabled to improve solution quality
        - Simplification disabled (preserves waypoints)

    Args:
        profile_names: Profile names to register (default: `OMPL_PROFILE_NAMES`)
            Relevant names: ["DEFAULT", "FREESPACE"]
            (OMPL doesn't apply to Cartesian/Raster/Upright tasks)
        planning_time: Max planning time in seconds (default: 10.0)
            Total time budget for all parallel planners
        max_solutions: Max solutions to find before exiting (default: 10)
            Stops early if this many solutions found
        optimize: Use all planning time to optimize trajectory (default: True)
            If True, continues improving solution until timeout
            If False, stops at first solution
        simplify: Simplify trajectory after planning (default: False)
            If True, removes waypoints (may violate constraints)
            If False, preserves all waypoints
        num_planners: Number of parallel RRTConnect planners (default: min(4, CPU count))
            More planners = higher chance of finding solution quickly
        planner_range: RRTConnect `range`: the longest extension step, in rad of joint-space
            norm (default: 0, leaving OMPL's estimate from the extent of the state space).
            Shorter steps make the tree denser and slower to grow.

    Returns:
        ProfileDictionary with configured OMPL profiles; each name holds its own profile.

    Raises:
        InvalidProfileArgumentError: `planning_time`, `max_solutions`, `num_planners` or
            `planner_range` is out of range.

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_ompl_default_profiles

        # Quick planning with defaults
        profiles = create_ompl_default_profiles()

        # Custom planning time and parallelism
        profiles = create_ompl_default_profiles(planning_time=10.0, num_planners=8)

        # Fast first-solution mode
        profiles = create_ompl_default_profiles(planning_time=2.0, optimize=False, max_solutions=1)
        ```
    """
    profiles = ProfileDictionary()
    register_ompl(
        profiles,
        profile_names or OMPL_PROFILE_NAMES,
        planning_time=planning_time,
        max_solutions=max_solutions,
        optimize=optimize,
        simplify=simplify,
        num_planners=num_planners,
        planner_range=planner_range,
    )
    return profiles


def create_ompl_planner_configurators(
    planners: Sequence[str] | None = None,
    num_planners: int | None = None,
    **kwargs: float | int | bool,
) -> list:
    """Create OMPL planner configurators with custom parameters.

    Every `OMPLPlannerType` name is accepted: SBL, EST, LBKPIECE1, BKPIECE1,
    KPIECE1, BiTRRT, RRT, RRTConnect, RRTstar, TRRT, PRM, PRMstar,
    LazyPRMstar, SPARS. Each maps to its `<name>Configurator` class.

    Args:
        planners: Planner type names (default: ["RRTConnect"]).
        num_planners: Number of instances per planner type (default: 1).
            Use multiple instances for parallel planning.
        **kwargs: Configurator fields, e.g. `range`, `goal_bias`,
            `delay_collision_checking`, `max_nearest_neighbors`. Each is set on
            every requested configurator that has the field.

    Returns:
        List of configured OMPLPlannerConfigurator instances.

    Raises:
        UnknownPlannerTypeError: A planner name is not an `OMPLPlannerType`.
        UnknownConfiguratorFieldError: A keyword argument is a field of none of
            the requested configurators.

    Example:
        ```python
        # Single RRTConnect with default settings
        planners = create_ompl_planner_configurators()

        # Multiple parallel RRTConnect planners
        planners = create_ompl_planner_configurators(planners=["RRTConnect"], num_planners=4)

        # Mix of planner types; goal_bias applies to RRTstar only
        planners = create_ompl_planner_configurators(
            planners=["RRTConnect", "RRTstar"],
            num_planners=2,
            range=0.1,
            goal_bias=0.1,
        )
        ```
    """
    from tesseract_robotics import motion_planners_ompl as ompl

    if planners is None:
        planners = ["RRTConnect"]

    if num_planners is None:
        num_planners = 1

    planner_types = ompl.OMPLPlannerType.__members__
    unknown = [name for name in planners if name not in planner_types]
    if unknown:
        raise UnknownPlannerTypeError(
            f"Unknown planner: {unknown}. Supported: {', '.join(planner_types)}"
        )

    classes = [getattr(ompl, f"{name}Configurator") for name in planners]
    unused = [key for key in kwargs if not any(_is_configurator_field(cls, key) for cls in classes)]
    if unused:
        raise UnknownConfiguratorFieldError(f"{unused} is not a field of any of {list(planners)}")

    configurators = []
    for cls in classes:
        for _ in range(num_planners):
            cfg = cls()
            for key, value in kwargs.items():
                if _is_configurator_field(cls, key):
                    setattr(cfg, key, value)
            configurators.append(cfg)

    return configurators
