"""Joint-space trajectory the viewer animates."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_JointWaypointPoly,
    WaypointPoly_as_StateWaypointPoly,
)

from .errors import EmptyTrajectoryError, TrajectoryShapeError, UnplannedWaypointError


@dataclass(frozen=True)
class Trajectory:
    """Joint positions over time.

    Attributes:
        joint_names: Joint name per column of `positions`.
        positions: `(N, J)` joint values, one row per waypoint.
        times_s: `(N,)` non-decreasing waypoint times [s].
    """

    joint_names: tuple[str, ...]
    positions: NDArray[np.float64]
    times_s: NDArray[np.float64]

    @classmethod
    def build(
        cls, joint_names: Sequence[str], positions: ArrayLike, times_s: ArrayLike
    ) -> Trajectory:
        """Validate and build a trajectory.

        Args:
            joint_names: Joint name per column.
            positions: `(N, J)` joint values.
            times_s: `(N,)` waypoint times [s].

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
        return cls(tuple(joint_names), q, t)

    @classmethod
    def from_rows(cls, joint_names: Sequence[str], rows: ArrayLike) -> Trajectory:
        """Build from rows of `[q_1, ..., q_J, t]`.

        Args:
            joint_names: Joint name per joint column.
            rows: `(N, J + 1)` joint values with the time [s] as last column.

        Returns:
            The trajectory.
        """
        a = np.asarray(rows, dtype=np.float64)
        if a.ndim != 2:
            raise EmptyTrajectoryError("trajectory has no waypoints")
        return cls.build(joint_names, a[:, :-1], a[:, -1])

    @classmethod
    def from_composite(cls, composite: CompositeInstruction) -> Trajectory:
        """Build from a planned program, descending into child composites.

        State waypoints keep their time; joint waypoints carry none, so their index is
        their time. So is a state waypoint's index when every time is zero: the program
        was not time-parameterized and would otherwise have no duration. Columns follow
        the first waypoint's joint order, whatever order later waypoints name them in.

        Args:
            composite: Planner output, e.g. `PlanningResult.raw_results`.

        Returns:
            The trajectory.

        Raises:
            EmptyTrajectoryError: no move instruction.
            UnplannedWaypointError: a move has a Cartesian waypoint.
            TrajectoryShapeError: waypoints name different joints, or times decrease.
        """
        joint_names: list[str] | None = None
        rows: list[NDArray[np.float64]] = []
        times: list[float] = []
        moves = [i for i in composite.flatten() if i.isMoveInstruction()]
        for index, instruction in enumerate(moves):
            wp = InstructionPoly_as_MoveInstructionPoly(instruction).getWaypoint()
            if wp.isStateWaypoint():
                state = WaypointPoly_as_StateWaypointPoly(wp)
                names, q, t = state.getNames(), state.getPosition(), state.getTime()
            elif wp.isJointWaypoint():
                joint = WaypointPoly_as_JointWaypointPoly(wp)
                names, q, t = joint.getNames(), joint.getPosition(), float(index)
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
        if joint_names is None:
            raise EmptyTrajectoryError("no state or joint waypoint in the program")
        if not any(times):
            times = [float(index) for index in range(len(times))]
        return cls.build(joint_names, np.stack(rows), times)

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
