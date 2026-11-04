"""Trajectory: the viewer's playback value type."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
)
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.viewer.errors import (
    EmptyTrajectoryError,
    TrajectoryShapeError,
    UnplannedWaypointError,
)
from tesseract_robotics.viewer.trajectory import Trajectory

NAMES = ["j1", "j2"]


def _append(composite, waypoint_poly):
    move = MoveInstruction(waypoint_poly, MoveInstructionType.FREESPACE)
    composite.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(move))


def test_from_rows_splits_time_column():
    traj = Trajectory.from_rows(NAMES, [[0, 0, 0.0], [1, 2, 0.5]])
    assert traj.joint_names == ("j1", "j2")
    np.testing.assert_array_equal(traj.positions, [[0, 0], [1, 2]])
    np.testing.assert_array_equal(traj.times_s, [0.0, 0.5])
    assert traj.duration_s == 0.5


def test_sample_interpolates_midpoint():
    traj = Trajectory.from_rows(NAMES, [[0, 0, 0.0], [1, 2, 0.5]])
    np.testing.assert_allclose(traj.sample(0.25), [0.5, 1.0])


def test_sample_clamps():
    traj = Trajectory.from_rows(NAMES, [[0, 0, 0.0], [1, 2, 0.5]])
    np.testing.assert_array_equal(traj.sample(-1.0), [0, 0])
    np.testing.assert_array_equal(traj.sample(9.0), [1, 2])


def test_build_rejects_decreasing_times():
    with pytest.raises(TrajectoryShapeError):
        Trajectory.build(NAMES, [[0, 0], [1, 1]], [1.0, 0.0])


def test_build_rejects_width_mismatch():
    with pytest.raises(TrajectoryShapeError):
        Trajectory.build(NAMES, [[0, 0, 0]], [0.0])


def test_empty_raises():
    with pytest.raises(EmptyTrajectoryError):
        Trajectory.build(NAMES, np.zeros((0, 2)), [])
    with pytest.raises(EmptyTrajectoryError):
        Trajectory.from_composite(CompositeInstruction())


def test_from_composite_joint_waypoints_use_index_time():
    composite = CompositeInstruction()
    for q in ([0.0, 0.0], [1.0, 1.0], [2.0, 2.0]):
        wp = JointWaypoint(NAMES, np.array(q))
        _append(composite, JointWaypointPoly_wrap_JointWaypoint(wp))
    traj = Trajectory.from_composite(composite)
    np.testing.assert_array_equal(traj.times_s, [0.0, 1.0, 2.0])
    np.testing.assert_array_equal(traj.positions[2], [2.0, 2.0])


def test_from_composite_state_waypoints_keep_time():
    composite = CompositeInstruction()
    for q, t in (([0.0, 0.0], 0.0), ([1.0, 2.0], 0.75)):
        wp = StateWaypoint(NAMES, np.array(q))
        wp.setTime(t)
        _append(composite, StateWaypointPoly_wrap_StateWaypoint(wp))
    traj = Trajectory.from_composite(composite)
    assert traj.joint_names == ("j1", "j2")
    np.testing.assert_array_equal(traj.times_s, [0.0, 0.75])


def test_from_composite_descends_into_child_composites():
    inner = CompositeInstruction()
    wp = StateWaypoint(NAMES, np.array([1.0, 1.0]))
    _append(inner, StateWaypointPoly_wrap_StateWaypoint(wp))
    outer = CompositeInstruction()
    outer.push_back(inner)
    assert Trajectory.from_composite(outer).positions.shape == (1, 2)


def test_from_composite_rejects_cartesian_waypoint():
    composite = CompositeInstruction()
    _append(
        composite, CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(Isometry3d()))
    )
    with pytest.raises(UnplannedWaypointError):
        Trajectory.from_composite(composite)
