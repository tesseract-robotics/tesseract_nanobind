"""Time parameterization constructors: TOTG and iterative spline (ISP)."""

from __future__ import annotations

from tesseract_robotics.time_parameterization import (
    IterativeSplineParameterization,
    TimeOptimalTrajectoryGeneration,
)


def create_time_optimal_parameterization() -> TimeOptimalTrajectoryGeneration:
    """Create Time-Optimal Trajectory Generation (TOTG) parameterization.

    TOTG finds the time-optimal trajectory that respects velocity, acceleration,
    and jerk limits. It uses a numerical integration approach to find the fastest
    trajectory along a given geometric path.

    The path tolerance, minimum angle change and limits are fields of
    `TimeOptimalTrajectoryGenerationCompositeProfile` (`path_tolerance`,
    `min_angle_change`, ...), which `compute` looks up in the profile dictionary.

    Returns:
        TimeOptimalTrajectoryGeneration instance

    Example:
        ```python
        from tesseract_robotics.command_language import ProfileDictionary
        from tesseract_robotics.planning.profiles import create_time_optimal_parameterization
        from tesseract_robotics.time_parameterization import (
            TimeOptimalTrajectoryGenerationCompositeProfile,
        )

        totg = create_time_optimal_parameterization()

        # The limits live in the profile, looked up under the parameterization's
        # name ("TOTG") and the program's profile name
        profiles = ProfileDictionary()
        profiles.addProfile(
            totg.getName(), "DEFAULT", TimeOptimalTrajectoryGenerationCompositeProfile()
        )

        # Edits `program` (a CompositeInstruction of StateWaypoints) in place
        success = totg.compute(program, env, profiles)
        ```

    Reference:
        Kunz & Stilman, "Time-Optimal Trajectory Generation for Path Following
        with Bounded Acceleration and Velocity", RSS 2012
    """
    return TimeOptimalTrajectoryGeneration()


def create_iterative_spline_parameterization() -> IterativeSplineParameterization:
    """Create Iterative Spline Parameterization (ISP).

    ISP iteratively fits a spline to the trajectory while respecting velocity,
    acceleration, and jerk limits. Generally faster than TOTG but may produce
    slower trajectories.

    Whether to add intermediate points, and the limits, are fields of
    `IterativeSplineParameterizationCompositeProfile` (`add_points`, ...), which
    `compute` looks up in the profile dictionary.

    Returns:
        IterativeSplineParameterization instance

    Example:
        ```python
        from tesseract_robotics.command_language import ProfileDictionary
        from tesseract_robotics.planning.profiles import create_iterative_spline_parameterization
        from tesseract_robotics.time_parameterization import (
            IterativeSplineParameterizationCompositeProfile,
        )

        isp = create_iterative_spline_parameterization()

        # The limits live in the profile, looked up under the parameterization's
        # name ("ISP") and the program's profile name
        profiles = ProfileDictionary()
        profiles.addProfile(
            isp.getName(), "DEFAULT", IterativeSplineParameterizationCompositeProfile()
        )

        # Edits `program` (a CompositeInstruction of StateWaypoints) in place
        success = isp.compute(program, env, profiles)
        ```

    Note:
        ISP is typically faster to compute than TOTG, but TOTG produces
        time-optimal trajectories. Use ISP when computation speed matters
        more than trajectory time.
    """
    return IterativeSplineParameterization()
