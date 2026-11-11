"""Joint-space trajectory the viewer animates."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.command_language import (
    CompositeInstruction,
    JointWaypointPoly,
    MoveInstructionType,
    StateWaypointPoly,
)

from .errors import (
    EmptyTrajectoryError,
    TrajectoryShapeError,
    UnplannedWaypointError,
    UntimedTrajectoryError,
)


def _names(waypoint: StateWaypointPoly | JointWaypointPoly) -> list[str]:
    """Joint names of a state or joint waypoint: the viewer works in `str`, tesseract in ids."""
    return [jid.name() for jid in waypoint.getJointIds()]


@dataclass(frozen=True)
class Trajectory:
    """Joint positions over time.

    Attributes:
        joint_names: Joint name per column of `positions`.
        positions: `(N, J)` joint values, one row per waypoint.
        times_s: `(N,)` non-decreasing waypoint times [s].
        move_types: Per waypoint, the motion arriving at it; the first arrives from nowhere.
    """

    joint_names: tuple[str, ...]
    positions: NDArray[np.float64]
    times_s: NDArray[np.float64]
    move_types: tuple[MoveInstructionType, ...]

    @classmethod
    def build(
        cls,
        joint_names: Sequence[str],
        positions: ArrayLike,
        times_s: ArrayLike,
        move_types: Sequence[MoveInstructionType] | None = None,
    ) -> Trajectory:
        """Validate and build a trajectory.

        Args:
            joint_names: Joint name per column.
            positions: `(N, J)` joint values.
            times_s: `(N,)` waypoint times [s].
            move_types: `(N,)` motion arriving at each waypoint. Default: joint
                interpolation, which is all joint values alone can say.

        Returns:
            The trajectory.

        Raises:
            EmptyTrajectoryError: no waypoints.
            TrajectoryShapeError: shapes disagree, or times decrease.
        """
        q = np.asarray(positions, dtype=np.float64)
        t = np.asarray(times_s, dtype=np.float64)
        if q.size == 0 or t.size == 0:
            raise EmptyTrajectoryError("trajectory has no waypoints")
        if q.ndim != 2 or q.shape[1] != len(joint_names) or t.shape != (q.shape[0],):
            raise TrajectoryShapeError(
                f"positions {q.shape}, times {t.shape} and {len(joint_names)} joint names disagree"
            )
        if np.any(np.diff(t) < 0.0):
            raise TrajectoryShapeError("waypoint times decrease")
        moves = (MoveInstructionType.FREESPACE,) * len(q) if move_types is None else move_types
        if len(moves) != len(q):
            raise TrajectoryShapeError(f"{len(moves)} move types for {len(q)} waypoints")
        return cls(tuple(joint_names), q, t, tuple(moves))

    @classmethod
    def from_rows(cls, joint_names: Sequence[str], rows: ArrayLike) -> Trajectory:
        """Build from rows of `[q_1, ..., q_J, t]`.

        Args:
            joint_names: Joint name per joint column.
            rows: `(N, J + 1)` joint values with the time [s] as last column.

        Returns:
            The trajectory.

        Raises:
            EmptyTrajectoryError: no rows.
            TrajectoryShapeError: `rows` is not 2-D, or its shape disagrees with `joint_names`.
        """
        a = np.asarray(rows, dtype=np.float64)
        if a.size == 0:
            raise EmptyTrajectoryError("trajectory has no waypoints")
        if a.ndim != 2:
            raise TrajectoryShapeError(f"rows must be 2-D (N, J + 1), got shape {a.shape}")
        return cls.build(joint_names, a[:, :-1], a[:, -1])

    @classmethod
    def from_composite(
        cls, composite: CompositeInstruction, *, untimed_step_s: float | None = None
    ) -> Trajectory:
        """Build from a planned program, descending into child composites.

        State waypoints keep their time. A program without times (joint waypoints, or
        state waypoints that were never time-parameterized, all at zero) has no
        duration of its own; it is placed on a clock only when the caller says how:
        waypoint `i` at `i * untimed_step_s`. Columns follow the first waypoint's
        joint order, whatever order later waypoints name them in.

        Args:
            composite: Planner output, e.g. `PlanningResult.raw_results`.
            untimed_step_s: Time between waypoints [s] for a program without times;
                `None` raises on such a program instead of inventing a clock.

        Returns:
            The trajectory.

        Raises:
            EmptyTrajectoryError: no move instruction.
            UnplannedWaypointError: a move has a Cartesian waypoint.
            UntimedTrajectoryError: the program has no times and `untimed_step_s` is `None`.
            TrajectoryShapeError: waypoints name different joints, times decrease, or
                `untimed_step_s` is not positive.
        """
        if untimed_step_s is not None and not untimed_step_s > 0.0:
            raise TrajectoryShapeError(f"untimed_step_s must be positive, got {untimed_step_s}")
        joint_names: list[str] | None = None
        rows: list[NDArray[np.float64]] = []
        times: list[float | None] = []
        move_types: list[MoveInstructionType] = []
        moves = [i for i in composite.flatten() if i.isMoveInstruction()]
        for index, instruction in enumerate(moves):
            move = instruction.asMoveInstruction()
            wp = move.getWaypoint()
            if wp.isStateWaypoint():
                state = wp.asStateWaypoint()
                names, q, t = _names(state), state.getPosition(), state.getTime()
            elif wp.isJointWaypoint():
                joint = wp.asJointWaypoint()
                names, q, t = _names(joint), joint.getPosition(), None
            else:
                raise UnplannedWaypointError(
                    f"move {index} has a Cartesian waypoint; plan it first"
                )
            if joint_names is None:
                joint_names = list(names)
            row = dict(zip(names, np.asarray(q, dtype=np.float64).ravel()))
            if row.keys() != set(joint_names):
                raise TrajectoryShapeError(
                    f"move {index} names joints {sorted(row)}, the first names {joint_names}"
                )
            rows.append(np.array([row[name] for name in joint_names]))
            times.append(t)
            move_types.append(move.getMoveType())
        if joint_names is None:
            raise EmptyTrajectoryError("no state or joint waypoint in the program")
        timed = [t for t in times if t is not None]
        if len(timed) == len(times) and any(timed):
            return cls.build(joint_names, np.stack(rows), timed, move_types)
        if timed and any(timed):
            raise TrajectoryShapeError("the program mixes timed state and untimed joint waypoints")
        if untimed_step_s is None:
            raise UntimedTrajectoryError(
                "the program carries no waypoint times (not time-parameterized); "
                "time-parameterize it, or pass untimed_step_s to place waypoint i at i * step"
            )
        steps = [index * untimed_step_s for index in range(len(times))]
        return cls.build(joint_names, np.stack(rows), steps, move_types)

    @property
    def duration_s(self) -> float:
        """Time of the last waypoint [s]."""
        return float(self.times_s[-1])

    def sample(self, t_s: float) -> NDArray[np.float64]:
        """Joint values at `t_s`, linearly interpolated and clamped to the trajectory.

        Args:
            t_s: Time [s].

        Returns:
            `(J,)` joint values.
        """
        return np.array([np.interp(t_s, self.times_s, column) for column in self.positions.T])
