"""Shared validity check for planned trajectories.

A planner that reports `successful` has not shown a usable trajectory: the path can leave the
joint limits, clip an obstacle between two waypoints, or stop short of the goal. Tests that plan
call `assert_trajectory_valid` instead of asserting `result.successful` alone.

Example:
    ```python
    result = plan_freespace(robot, program, composer=composer)
    assert_trajectory_valid(robot, result, goal=JointGoal(goal_positions))
    ```
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike
from scipy.spatial.transform import Rotation

from tesseract_robotics.collision import (
    CollisionCheckConfig,
    CollisionEvaluatorType,
    ContactRequest,
    ContactTestType,
)
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.environment import checkTrajectory
from tesseract_robotics.planning import PlanningResult, Pose, Robot

# Slack on a joint limit (rad). The planners enforce limits as optimizer constraints, which
# trajopt's sco solver satisfies to its constraint tolerance, cnt_tolerance = 1e-4.
JOINT_LIMIT_TOL_RAD = 1e-4

# Distance of the last state from a joint goal (rad, per joint). A joint goal is a hard
# constraint in every pipeline, so the same cnt_tolerance applies.
JOINT_GOAL_TOL_RAD = 1e-4

# Distance of the last state's tip from a Cartesian goal (m): cnt_tolerance of the Cartesian
# constraint, 1e-4, on a unit-weighted position error.
CARTESIAN_POSITION_TOL_M = 1e-4

# Rotation angle between the last state's tip and a Cartesian goal (rad): same constraint, the
# orientation error is a rotation vector whose norm is this angle.
CARTESIAN_ORIENTATION_TOL_RAD = 1e-4

# Longest valid segment of the collision check (rad of joint-space norm). Between two waypoints
# the swept volume is sampled every 0.01 rad; on the ~1.5 m reach of the shipped industrial arms
# that is 1.5 cm of tip travel, below the smallest obstacle in the tests (5 cm).
COLLISION_CHECK_RESOLUTION_RAD = 0.01


@dataclass(frozen=True)
class JointGoal:
    """The last state must equal these joint positions (rad), in the group's joint order."""

    positions: np.ndarray

    @classmethod
    def build(cls, positions: ArrayLike) -> JointGoal:
        """Goal from any 1-D array-like of joint positions (rad)."""
        return cls(np.asarray(positions, dtype=np.float64).ravel())


@dataclass(frozen=True)
class CartesianGoal:
    """The last state's TCP must sit at `pose`, in the frame `Robot.fk` reports (group base).

    Attributes:
        pose: Goal pose of the TCP.
        tip_link: Link the TCP is attached to, as for `Robot.fk`.
        tcp_offset: Offset from `tip_link` to the TCP; None is identity.
    """

    pose: Isometry3d
    tip_link: str
    tcp_offset: Isometry3d | None = None


class TrajectoryInvalidError(AssertionError):
    """A planned trajectory violates one of the checks of `assert_trajectory_valid`."""


def _joint_limit_violation(robot: Robot, group: str, positions: np.ndarray) -> str | None:
    names = robot.get_joint_names(group)
    # The joint group, not the kinematic group: a group without an inverse-kinematics solver
    # (the 9-DOF `manipulator_aux`) has limits too.
    limits = robot.env.getJointGroup(group).getLimits().joint_limits
    lower, upper = limits[:, 0], limits[:, 1]
    below = lower - positions
    above = positions - upper
    worst = max(float(below.max()), float(above.max()))
    if worst <= JOINT_LIMIT_TOL_RAD:
        return None
    step, joint = np.unravel_index(int(np.argmax(np.maximum(below, above))), positions.shape)
    return (
        f"state {step}, joint {names[joint]}: {positions[step, joint]:.6f} rad is outside "
        f"[{lower[joint]:.6f}, {upper[joint]:.6f}] by {worst:.2e} rad "
        f"(tolerance {JOINT_LIMIT_TOL_RAD:.0e})"
    )


def _check_goal(
    robot: Robot, group: str, last: np.ndarray, goal: JointGoal | CartesianGoal
) -> None:
    if isinstance(goal, JointGoal):
        if goal.positions.shape != last.shape:
            raise TrajectoryInvalidError(
                f"joint goal has {goal.positions.shape} positions, the trajectory has {last.shape}"
            )
        error = float(np.max(np.abs(last - goal.positions)))
        if error > JOINT_GOAL_TOL_RAD:
            raise TrajectoryInvalidError(
                f"last state misses the joint goal by {error:.2e} rad "
                f"(tolerance {JOINT_GOAL_TOL_RAD:.0e}): {last} vs {goal.positions}"
            )
        return
    reached = robot.fk(group, last, goal.tip_link)
    if goal.tcp_offset is not None:
        reached = reached * goal.tcp_offset
    position_error = float(np.linalg.norm(np.asarray(reached.translation - goal.pose.translation)))
    relative = np.asarray(reached.linear).T @ np.asarray(goal.pose.linear)
    orientation_error = float(Rotation.from_matrix(relative).magnitude())
    if position_error > CARTESIAN_POSITION_TOL_M:
        raise TrajectoryInvalidError(
            f"last state's TCP misses the goal position by {position_error:.2e} m "
            f"(tolerance {CARTESIAN_POSITION_TOL_M:.0e}): {Pose(reached)} vs {Pose(goal.pose)}"
        )
    if orientation_error > CARTESIAN_ORIENTATION_TOL_RAD:
        raise TrajectoryInvalidError(
            f"last state's TCP misses the goal orientation by {orientation_error:.2e} rad "
            f"(tolerance {CARTESIAN_ORIENTATION_TOL_RAD:.0e}): {Pose(reached)} vs {Pose(goal.pose)}"
        )


def assert_trajectory_valid(
    robot: Robot,
    result: PlanningResult,
    goal: JointGoal | CartesianGoal | None = None,
    *,
    group: str = "manipulator",
    check_collision: bool = True,
) -> None:
    """Assert a planning result is a usable trajectory.

    Checks, in order: the plan succeeded; the trajectory is non-empty and uses the group's joints
    in the group's order; every state is within the joint limits; with `check_collision`, no
    state and no segment between consecutive states collides (`LVS_CONTINUOUS` at
    `COLLISION_CHECK_RESOLUTION_RAD`); the last state reaches `goal`.

    Args:
        robot: The robot the trajectory was planned on, in the state it was planned in.
        result: The planner's result.
        goal: Where the last state must be; None skips the goal check.
        group: Kinematic group the trajectory moves.
        check_collision: Run the continuous collision check.

    Raises:
        TrajectoryInvalidError: a check failed; the message names the first violation.
    """
    if not result.successful:
        raise TrajectoryInvalidError(f"planning failed: {result.message}")
    if len(result.trajectory) == 0:
        raise TrajectoryInvalidError("planning succeeded with an empty trajectory")
    expected_names = robot.get_joint_names(group)
    for index, point in enumerate(result.trajectory):
        if list(point.joint_names) != expected_names:
            raise TrajectoryInvalidError(
                f"state {index} has joints {list(point.joint_names)}, group '{group}' has "
                f"{expected_names}"
            )
    positions = np.array([point.positions for point in result.trajectory])
    if not np.all(np.isfinite(positions)):
        raise TrajectoryInvalidError("trajectory holds non-finite joint positions")

    violation = _joint_limit_violation(robot, group, positions)
    if violation is not None:
        raise TrajectoryInvalidError(violation)

    if check_collision:
        config = CollisionCheckConfig()
        config.type = CollisionEvaluatorType.LVS_CONTINUOUS
        config.longest_valid_segment_length = COLLISION_CHECK_RESOLUTION_RAD
        config.contact_request = ContactRequest(ContactTestType.FIRST)
        contacts, _ = checkTrajectory(
            robot.env.getContinuousContactManager(),
            robot.env.getJointGroup(group),
            positions,
            config,
        )
        if contacts:
            raise TrajectoryInvalidError(f"trajectory collides:\n{contacts.condensedSummary()}")

    if goal is not None:
        _check_goal(robot, group, positions[-1], goal)
