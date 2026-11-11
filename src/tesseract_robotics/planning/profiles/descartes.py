"""Descartes profile helpers: move profiles (waypoint sampling) and ladder graph solver profiles."""

from __future__ import annotations

import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.planning.errors import ConflictingProfileArgumentsError
from tesseract_robotics.planning.profiles.names import (
    DESCARTES_DEFAULT_NAMESPACE,
    DESCARTES_PROFILE_NAMES,
)

if TYPE_CHECKING:
    from tesseract_robotics.motion_planners_descartes import (
        DescartesDefaultMoveProfileD,
        DescartesDefaultMoveProfileF,
    )

    type DescartesDefaultMoveProfile = DescartesDefaultMoveProfileD | DescartesDefaultMoveProfileF


def _descartes_move_twin(profile: DescartesDefaultMoveProfile) -> DescartesDefaultMoveProfile:
    """The other scalar type's default move profile, with `profile`'s field values.

    Args:
        profile: A `DescartesDefaultMoveProfileD` or `DescartesDefaultMoveProfileF`.

    Returns:
        A new `DescartesDefaultMoveProfileF` for a `D` profile, or `...D` for an `F` one.

    Raises:
        ProfileOverrideNotDispatchedError: `profile` is not a default move profile; a Python
            subclass of `DescartesMoveProfileD` / `DescartesMoveProfileF` has hooks no
            profile of the other scalar type can call.
    """
    from tesseract_robotics.motion_planners import ProfileOverrideNotDispatchedError
    from tesseract_robotics.motion_planners_descartes import (
        DescartesDefaultMoveProfileD,
        DescartesDefaultMoveProfileF,
        DescartesMoveProfileD,
        DescartesMoveProfileF,
    )

    twins = {
        DescartesDefaultMoveProfileD: DescartesDefaultMoveProfileF,
        DescartesDefaultMoveProfileF: DescartesDefaultMoveProfileD,
    }
    for concrete, twin_type in twins.items():
        if isinstance(profile, concrete):
            twin = twin_type()
            for field, value in vars(concrete).items():
                if isinstance(value, property):
                    setattr(twin, field, getattr(profile, field))
            return twin

    accepted = "pass a DescartesDefaultMoveProfileD or DescartesDefaultMoveProfileF"
    for base, other in (
        (DescartesMoveProfileD, DescartesMoveProfileF),
        (DescartesMoveProfileF, DescartesMoveProfileD),
    ):
        if isinstance(profile, base):
            raise ProfileOverrideNotDispatchedError(
                f"move_profile {type(profile).__qualname__} is a Python subclass of "
                f"{base.__name__}: its hooks have no {other.__name__} twin, so a pipeline whose "
                f"Descartes node reads {other.__name__} would plan with the defaults. Add it to "
                f"a ProfileDictionary yourself, for a pipeline that reads {base.__name__}, or "
                f"{accepted}."
            )
    raise ProfileOverrideNotDispatchedError(
        f"move_profile is a {type(profile).__qualname__}, not a Descartes move profile: {accepted}."
    )


def register_descartes(
    profiles: ProfileDictionary,
    profile_names: Sequence[str] = DESCARTES_PROFILE_NAMES,
    enable_collision: bool | None = None,
    enable_edge_collision: bool | None = None,
    num_threads: int | None = None,
    sample_axis: ArrayLike | None = None,
    sample_resolution: float | None = None,
    sample_min: float | None = None,
    sample_max: float | None = None,
    ik_solver: str | None = None,
    use_redundant_joint_solutions: bool | None = None,
    move_profile: DescartesDefaultMoveProfile | None = None,
) -> None:
    """Add Descartes move and solver profiles, of both scalar types, to `profiles`.

    The arguments are those of `create_descartes_default_profiles`. Every name gets its own
    profile instances (a `move_profile` override is registered under every name as given, and
    its twin is built per name).

    Raises:
        ConflictingProfileArgumentsError: `move_profile` combined with move-profile arguments.
        ProfileOverrideNotDispatchedError: `move_profile` is not a default move profile.
    """
    from tesseract_robotics.motion_planners_descartes import (
        DescartesDefaultMoveProfileD,
        DescartesLadderGraphSolverProfileD,
        DescartesLadderGraphSolverProfileF,
    )

    if num_threads is None:
        num_threads = os.cpu_count() or 1

    sampling = (sample_axis, sample_resolution, sample_min, sample_max)
    if move_profile is not None:
        passed = {
            "enable_collision": enable_collision,
            "enable_edge_collision": enable_edge_collision,
            "sample_axis": sample_axis,
            "sample_resolution": sample_resolution,
            "sample_min": sample_min,
            "sample_max": sample_max,
            "ik_solver": ik_solver,
            "use_redundant_joint_solutions": use_redundant_joint_solutions,
        }
        conflicting = [name for name, value in passed.items() if value is not None]
        if conflicting:
            raise ConflictingProfileArgumentsError(
                f"move_profile is a full profile override: {conflicting} would be "
                f"silently ignored; configure them on the custom profile instead"
            )

    for name in profile_names:
        if move_profile is not None:
            plan_profile = move_profile
        else:
            plan_profile = DescartesDefaultMoveProfileD()
            if enable_collision is not None:
                plan_profile.enable_collision = enable_collision
            if enable_edge_collision is not None:
                plan_profile.enable_edge_collision = enable_edge_collision
            if any(v is not None for v in sampling):
                plan_profile.target_pose_fixed = False
                if sample_axis is not None:
                    plan_profile.target_pose_sample_axis = np.asarray(sample_axis, dtype=float)
                if sample_resolution is not None:
                    plan_profile.target_pose_sample_resolution = sample_resolution
                if sample_min is not None:
                    plan_profile.target_pose_sample_min = sample_min
                if sample_max is not None:
                    plan_profile.target_pose_sample_max = sample_max
            if ik_solver is not None:
                plan_profile.manipulator_ik_solver = ik_solver
            if use_redundant_joint_solutions is not None:
                plan_profile.use_redundant_joint_solutions = use_redundant_joint_solutions

        registered = [plan_profile, _descartes_move_twin(plan_profile)]
        for solver_type in (DescartesLadderGraphSolverProfileD, DescartesLadderGraphSolverProfileF):
            solver_profile = solver_type()
            solver_profile.num_threads = num_threads
            registered.append(solver_profile)
        for profile in registered:
            profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, profile)


def create_descartes_default_profiles(
    profile_names: Sequence[str] | None = None,
    enable_collision: bool | None = None,
    enable_edge_collision: bool | None = None,
    num_threads: int | None = None,
    sample_axis: ArrayLike | None = None,
    sample_resolution: float | None = None,
    sample_min: float | None = None,
    sample_max: float | None = None,
    ik_solver: str | None = None,
    use_redundant_joint_solutions: bool | None = None,
    move_profile: DescartesDefaultMoveProfile | None = None,
) -> ProfileDictionary:
    """Create Descartes profiles with sensible defaults.

    Descartes is a Cartesian path planner that uses a ladder graph approach
    to find joint configurations for Cartesian waypoints. It samples IK solutions
    at each waypoint and finds the minimum-cost path through the graph.

    Creates both plan profiles (waypoint sampling) and solver profiles
    (ladder graph solver configuration), each for both scalar types from the same
    settings: every shipped Cartesian and raster task reads the `float` profiles
    (`DescartesMoveProfileF`, `DescartesSolverProfileF`), `DescartesDPipeline` and
    `DescartesDNPCPipeline` read the `double` ones (`...D`). A profile of the other
    type is never found by a node (#350).

    The profiles serve the standalone Descartes pipelines (`DescartesFPipeline`,
    `DescartesDPipeline` and the NPC variants, which skip the post-plan contact check): the
    auxiliary tasks (min length, contact check, time parameterization) run on their library
    default profiles. For Descartes + TrajOpt refinement use `create_cartesian_pipeline_profiles`.

    Profile Configuration:
        Plan Profile (library defaults unless overridden):
        - enable_collision: vertex (waypoint) collision checking, default on
        - enable_edge_collision: edge (transition) collision checking, default off

        Solver Profile:
        - num_threads: Parallel graph solver threads (default: all CPUs)

    Args:
        profile_names: Profile names to register (default: `DESCARTES_PROFILE_NAMES`)
            Relevant names: ["DEFAULT", "CARTESIAN", "RASTER"]
            (Descartes doesn't apply to freespace tasks)
        enable_collision: Vertex collision checking (default: library default, on)
        enable_edge_collision: Edge collision checking (default: library default, off)
            Warning: Significantly slower, use only if needed
        num_threads: Number of threads for solver (default: CPU count)
            More threads = faster graph search
        sample_axis: Tool axis (3-vector, tool frame) to sample target rotation
            about, for axis-symmetric processes (milling, welding, drilling).
            Setting any `sample_*` argument switches the target pose from fixed
            to sampled (`target_pose_fixed = False`).
        sample_resolution: Rotation sampling step in radians (e.g. `radians(30)`)
        sample_min: Rotation sampling range start in radians (library default: -pi)
        sample_max: Rotation sampling range end in radians (library default: +pi/2,
            i.e. pi minus the default resolution: the header subtracts the resolution
            so that -pi and +pi are not both sampled, and a caller who changes the
            resolution sets the maximum with it)
        ik_solver: Override the kinematic group's IK solver for sampling
            (e.g. "OPWInvKin": analytic solvers make the ladder graph fast)
        use_redundant_joint_solutions: Also sample joint-redundant IK solutions
            (e.g. wrist flips), enlarging the ladder graph
        move_profile: Full move-profile override, a `DescartesDefaultMoveProfileD` or
            `DescartesDefaultMoveProfileF`. It is registered as given, together with a
            profile of the other scalar type holding the same field values (the `float`
            one rounds each double). Mutually exclusive with the move-profile arguments
            above; configure those on the override instead. A Python subclass of
            `DescartesMoveProfileD` / `DescartesMoveProfileF` is refused: its hooks have
            no twin of the other type. Add it to a `ProfileDictionary` yourself, under
            the type the pipeline's Descartes node reads.

    Returns:
        ProfileDictionary with configured Descartes profiles

    Raises:
        ConflictingProfileArgumentsError: `move_profile` combined with move-profile arguments.
        ProfileOverrideNotDispatchedError: `move_profile` is not a default move profile.

    Example:
        ```python
        import math

        from tesseract_robotics.planning.profiles import create_descartes_default_profiles

        # Standard configuration
        profiles = create_descartes_default_profiles()

        # Axis-symmetric tool: sample rotation about tool Z every 30 degrees
        profiles = create_descartes_default_profiles(
            sample_axis=(0, 0, 1),
            sample_resolution=math.radians(30),
        )

        # A configured default move profile; its float twin is made for you
        custom = DescartesDefaultMoveProfileD()
        custom.use_redundant_joint_solutions = True
        profiles = create_descartes_default_profiles(move_profile=custom)

        result = composer.plan(robot, program, pipeline="DescartesFPipeline", profiles=profiles)
        ```
    """
    profiles = ProfileDictionary()
    register_descartes(
        profiles,
        profile_names or DESCARTES_PROFILE_NAMES,
        enable_collision=enable_collision,
        enable_edge_collision=enable_edge_collision,
        num_threads=num_threads,
        sample_axis=sample_axis,
        sample_resolution=sample_resolution,
        sample_min=sample_min,
        sample_max=sample_max,
        ik_solver=ik_solver,
        use_redundant_joint_solutions=use_redundant_joint_solutions,
        move_profile=move_profile,
    )
    return profiles
