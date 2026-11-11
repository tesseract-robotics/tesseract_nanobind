"""TrajOpt (SCO) and TrajOptIfopt (SQP/OSQP) profile helpers, matching the C++ example defaults."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Final

import numpy as np

from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.planning.profiles.names import (
    STANDARD_PROFILE_NAMES,
    TRAJOPT_DEFAULT_NAMESPACE,
    TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
    UPRIGHT_PROFILE_NAMES,
)

if TYPE_CHECKING:
    from tesseract_robotics.motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultMoveProfile,
    )

# Collision settings of the default profile, from the C++ car_seat_example. The margin is the
# clearance (m) each term drives toward; the buffer (m) widens contact collection beyond the margin
# and is not used when the error is computed. The coefficient is the unitless weight of the term.
# Hard constraint at zero clearance:
CONSTRAINT_MARGIN_M: Final = 0.0
CONSTRAINT_MARGIN_BUFFER_M: Final = 0.005
CONSTRAINT_COEFF: Final = 10.0
# Soft cost pushing 5 mm clear of obstacles:
COST_MARGIN_M: Final = 0.005
COST_MARGIN_BUFFER_M: Final = 0.01
COST_COEFF: Final = 50.0

# Longest valid segment of the LVS_CONTINUOUS check (rad, joint-space norm): the sweep between two
# waypoints is sampled every 0.1 rad, from the C++ car_seat_example.
LONGEST_VALID_SEGMENT_RAD: Final = 0.1

# Collision settings of the UPRIGHT profile, from the C++ glass_upright_example:
# TrajOptCollisionConfig(0.01, 1) with the DISCRETE evaluator, the config's default type.
UPRIGHT_MARGIN_M: Final = 0.01
UPRIGHT_MARGIN_BUFFER_M: Final = 0.01
UPRIGHT_COLLISION_COEFF: Final = 1.0

# Weight of the velocity and acceleration smoothing costs of the UPRIGHT profile (C++ velocity_coeff = Ones(1)).
UPRIGHT_SMOOTHING_COEFF: Final = 1.0

# Cartesian constraint weights of the UPRIGHT profile, [x, y, z, rx, ry, rz]: position free (0),
# orientation constrained (1), as the C++ glass_upright_example sets them
# (`Eigen::VectorXd::Constant(6, 1)` with the first three zeroed).
UPRIGHT_CARTESIAN_COEFF: Final = (0.0, 0.0, 0.0, 1.0, 1.0, 1.0)


def create_trajopt_profile_pair() -> tuple[
    TrajOptDefaultCompositeProfile, TrajOptDefaultMoveProfile
]:
    """Create the TrajOpt composite and move profiles with the C++ example defaults.

    Collision is checked with `LVS_CONTINUOUS` (critical for attached objects), as a hard
    constraint at zero margin plus a soft cost at 5 mm. The margin is the clearance each term drives
    toward: without the cost's margin the soft term never pushed the path off an obstacle, TrajOpt
    converged onto the zero-distance boundary, and the pipeline's own contact check rejected it
    whenever the solver's ~1e-5 m residual came out negative (#103).

    Returns:
        The composite profile and the move profile; every call returns new instances.
    """
    from tesseract_robotics.collision import CollisionEvaluatorType
    from tesseract_robotics.motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultMoveProfile,
    )

    composite = TrajOptDefaultCompositeProfile()
    for config, margin, buffer, coeff in (
        (
            composite.collision_constraint_config,
            CONSTRAINT_MARGIN_M,
            CONSTRAINT_MARGIN_BUFFER_M,
            CONSTRAINT_COEFF,
        ),
        (
            composite.collision_cost_config,
            COST_MARGIN_M,
            COST_MARGIN_BUFFER_M,
            COST_COEFF,
        ),
    ):
        config.enabled = True
        config.contact_manager_config.default_margin = margin
        config.collision_margin_buffer = buffer
        config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS
        config.collision_check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_RAD
        config.collision_coeff_data.setDefaultCollisionCoeff(coeff)

    move = TrajOptDefaultMoveProfile()
    move.cartesian_cost_config.enabled = False
    move.cartesian_constraint_config.enabled = True
    move.joint_cost_config.enabled = False
    move.joint_constraint_config.enabled = True

    return composite, move


def create_trajopt_upright_profile_pair() -> tuple[
    TrajOptDefaultCompositeProfile, TrajOptDefaultMoveProfile
]:
    """Create the TrajOpt composite and move profiles for glass-upright constraints.

    The UPRIGHT profile constrains orientation while allowing free position movement, as the C++
    glass_upright_example: collision with the `DISCRETE` evaluator, velocity smoothing, and a
    Cartesian constraint weighted on orientation only.

    Returns:
        The composite profile and the move profile; every call returns new instances.
    """
    from tesseract_robotics.collision import CollisionEvaluatorType
    from tesseract_robotics.motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultMoveProfile,
    )

    composite = TrajOptDefaultCompositeProfile()
    for config in (composite.collision_cost_config, composite.collision_constraint_config):
        config.enabled = True
        config.contact_manager_config.default_margin = UPRIGHT_MARGIN_M
        config.collision_margin_buffer = UPRIGHT_MARGIN_BUFFER_M
        config.collision_check_config.type = CollisionEvaluatorType.DISCRETE
        config.collision_coeff_data.setDefaultCollisionCoeff(UPRIGHT_COLLISION_COEFF)
    composite.smooth_velocities = True
    composite.velocity_coeff = np.full(1, UPRIGHT_SMOOTHING_COEFF)
    composite.smooth_accelerations = False
    composite.acceleration_coeff = np.full(1, UPRIGHT_SMOOTHING_COEFF)
    composite.smooth_jerks = False

    move = TrajOptDefaultMoveProfile()
    move.joint_cost_config.enabled = False
    move.cartesian_cost_config.enabled = False
    move.cartesian_constraint_config.enabled = True
    move.cartesian_constraint_config.coeff = np.array(UPRIGHT_CARTESIAN_COEFF)

    return composite, move


def register_trajopt(
    profiles: ProfileDictionary, profile_names: Sequence[str] = STANDARD_PROFILE_NAMES
) -> None:
    """Add the default TrajOpt profiles to `profiles` under every name.

    Each name gets its own profile instances, so editing one name's profile leaves the others.

    Args:
        profiles: Dictionary to add to.
        profile_names: Profile names to register.
    """
    for name in profile_names:
        composite, move = create_trajopt_profile_pair()
        profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, name, composite)
        profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, name, move)


def register_trajopt_ifopt(
    profiles: ProfileDictionary, profile_names: Sequence[str] = STANDARD_PROFILE_NAMES
) -> None:
    """Add the default TrajOptIfopt profiles to `profiles` under every name.

    Composite profile: velocity, acceleration and jerk smoothing off (smoothing needs a coefficient
    sized to the robot's DOF, which is not known here), collision configs left at the C++ defaults
    (both enabled, no margins: the useful margin depends on the robot and scene). Move profile:
    Cartesian and joint targets are constraints, not costs. Solver profile: default OSQP settings,
    with the SQP loop (`opt_params`) at the C++ defaults, `max_iter=50`,
    `max_qp_solver_failures=3`.

    Each name gets its own profile instances, so editing one name's profile leaves the others.

    Args:
        profiles: Dictionary to add to.
        profile_names: Profile names to register.
    """
    from tesseract_robotics.motion_planners_trajopt_ifopt import (
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptDefaultMoveProfile,
        TrajOptIfoptOSQPSolverProfile,
    )

    for name in profile_names:
        composite = TrajOptIfoptDefaultCompositeProfile()
        composite.smooth_velocities = False
        composite.smooth_accelerations = False
        composite.smooth_jerks = False

        move = TrajOptIfoptDefaultMoveProfile()
        move.cartesian_cost_config.enabled = False
        move.cartesian_constraint_config.enabled = True
        move.joint_cost_config.enabled = False
        move.joint_constraint_config.enabled = True

        profiles.addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, name, composite)
        profiles.addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, name, move)
        profiles.addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, name, TrajOptIfoptOSQPSolverProfile())


def create_trajopt_ifopt_default_profiles(
    profile_names: Sequence[str] | None = None,
) -> ProfileDictionary:
    """Create TrajOptIfopt profiles matching C++ example defaults.

    TrajOptIfopt uses Sequential Quadratic Programming (SQP) with OSQP solver
    to optimize trajectories. It provides a modern alternative to the original
    TrajOpt implementation using the IFOPT interface. The profile settings are those of
    `register_trajopt_ifopt`.

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)

    Returns:
        ProfileDictionary with configured TrajOptIfopt profiles; each name holds its own
        profile instances.

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_trajopt_ifopt_default_profiles

        profiles = create_trajopt_ifopt_default_profiles()
        result = composer.plan(robot, program, pipeline="TrajOptIfoptPipeline", profiles=profiles)
        ```

        Both knob sets are reachable from the planner module, which re-exports
        `CollisionEvaluatorType` and `TrajOptCollisionConfig` for this purpose:

        ```python
        from tesseract_robotics.motion_planners_trajopt_ifopt import (
            CollisionEvaluatorType,
            TrajOptIfoptDefaultCompositeProfile,
            TrajOptIfoptOSQPSolverProfile,
        )

        composite = TrajOptIfoptDefaultCompositeProfile()
        composite.collision_constraint_config.contact_manager_config.default_margin = 0.01
        composite.collision_constraint_config.collision_check_config.type = (
            CollisionEvaluatorType.LVS_DISCRETE
        )
        # max_num_cnt is honoured by trajopt_ifopt only: it fixes the number of
        # collision constraint rows per timestep so QP size stops tracking contact count
        composite.collision_constraint_config.max_num_cnt = 3

        solver = TrajOptIfoptOSQPSolverProfile()
        solver.opt_params.max_iter = 100
        ```

        The profiles this helper builds can also be retrieved and edited in
        place; `getProfile` returns the concrete profile type, not the `Profile` base. An
        edit applies to the one name it is retrieved under:

        ```python
        composite = profiles.getProfile(
            TrajOptIfoptDefaultCompositeProfile.createKey(),
            TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
            "DEFAULT",
        )
        composite.collision_constraint_config.max_num_cnt = 3
        ```
    """
    profiles = ProfileDictionary()
    register_trajopt_ifopt(profiles, profile_names or STANDARD_PROFILE_NAMES)
    return profiles


def create_trajopt_default_profiles(
    profile_names: Sequence[str] | None = None,
) -> ProfileDictionary:
    """Create TrajOpt profiles matching C++ example defaults.

    This creates profiles with collision and cost/constraint settings that
    match the tesseract_examples C++ code (car_seat_example.cpp, etc.).

    TrajOpt uses Sequential Convex Optimization (SCO) to optimize trajectories
    by minimizing costs (smoothness, collision distance) while satisfying
    constraints (joint limits, collision avoidance, waypoint targets).

    Profile Configuration (0.33 API):
        Composite Profile (applies to entire trajectory), `LVS_CONTINUOUS` collision checks:
        - collision_constraint: margin=0.00 m, collision_margin_buffer=0.005 m, coeff=10
        - collision_cost: margin=0.005 m, collision_margin_buffer=0.01 m, coeff=50

        Move Profile (applies per waypoint):
        - cartesian_constraint: enabled (enforce Cartesian targets exactly)
        - joint_constraint: enabled (enforce joint targets exactly)
        - cartesian_cost: disabled (no soft Cartesian costs)
        - joint_cost: disabled (no soft joint costs)

    Args:
        profile_names: Profile names to register (default: `STANDARD_PROFILE_NAMES`)

    Returns:
        ProfileDictionary with configured TrajOpt profiles; each name holds its own
        profile instances.

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_trajopt_default_profiles

        # Create profiles with standard names
        profiles = create_trajopt_default_profiles()

        # Or specify custom profile names
        profiles = create_trajopt_default_profiles(profile_names=["MY_PROFILE"])
        ```
    """
    profiles = ProfileDictionary()
    register_trajopt(profiles, profile_names or STANDARD_PROFILE_NAMES)
    return profiles


def create_trajopt_upright_profiles(
    profile_names: Sequence[str] | None = None,
) -> ProfileDictionary:
    """Create TrajOpt profiles for orientation-constrained motion (glass-upright).

    This creates profiles matching the C++ glass_upright_example.cpp:
    - Collision: margin=0.01 m, collision_margin_buffer=0.01 m, coeff=1 (`DISCRETE` evaluator)
    - Cartesian constraint: coeff=[0,0,0,1,1,1] (position free, orientation constrained)

    The UPRIGHT profile keeps tool orientation constant while allowing position to vary.
    Essential for tasks like carrying a glass of water without spilling.

    Args:
        profile_names: Profile names to register (default: `UPRIGHT_PROFILE_NAMES`)

    Returns:
        ProfileDictionary with configured TrajOpt profiles for orientation constraints; each
        name holds its own profile instances.

    Example:
        ```python
        from tesseract_robotics.planning.profiles import create_trajopt_upright_profiles

        profiles = create_trajopt_upright_profiles()
        result = composer.plan(robot, program, pipeline="TrajOptPipeline", profiles=profiles)
        ```
    """
    profiles = ProfileDictionary()
    for name in profile_names or UPRIGHT_PROFILE_NAMES:
        composite, move = create_trajopt_upright_profile_pair()
        profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, name, composite)
        profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, name, move)
    return profiles
