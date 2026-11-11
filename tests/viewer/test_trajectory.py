"""Trajectory: the viewer's playback value type."""

import numpy as np
import pytest

from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.viewer.errors import (
    EmptyTrajectoryError,
    TrajectoryShapeError,
    UnplannedWaypointError,
)
from tesseract_robotics.viewer.trajectory import Trajectory

NAMES = ["j1", "j2"]


def _append(composite, waypoint_poly):
    move = MoveInstruction(waypoint_poly, MoveInstructionType.FREESPACE)
    composite.push_back(MoveInstructionPoly(move))


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
        _append(composite, JointWaypointPoly(wp))
    traj = Trajectory.from_composite(composite)
    np.testing.assert_array_equal(traj.times_s, [0.0, 1.0, 2.0])
    np.testing.assert_array_equal(traj.positions[2], [2.0, 2.0])


def test_from_composite_state_waypoints_keep_time():
    composite = CompositeInstruction()
    for q, t in (([0.0, 0.0], 0.0), ([1.0, 2.0], 0.75)):
        wp = StateWaypoint(NAMES, np.array(q))
        wp.setTime(t)
        _append(composite, StateWaypointPoly(wp))
    traj = Trajectory.from_composite(composite)
    assert traj.joint_names == ("j1", "j2")
    np.testing.assert_array_equal(traj.times_s, [0.0, 0.75])


def test_from_composite_descends_into_child_composites():
    inner = CompositeInstruction()
    wp = StateWaypoint(NAMES, np.array([1.0, 1.0]))
    _append(inner, StateWaypointPoly(wp))
    outer = CompositeInstruction()
    outer.push_back(inner)
    assert Trajectory.from_composite(outer).positions.shape == (1, 2)


def test_from_composite_rejects_cartesian_waypoint():
    composite = CompositeInstruction()
    _append(composite, CartesianWaypointPoly(CartesianWaypoint(Isometry3d())))
    with pytest.raises(UnplannedWaypointError):
        Trajectory.from_composite(composite)


def test_from_composite_unparameterized_state_waypoints_use_index_time():
    """Time parameterization not run: every state waypoint has t = 0 (descartes output)."""
    composite = CompositeInstruction()
    for q in ([0.0, 0.0], [1.0, 1.0], [2.0, 2.0]):
        _append(composite, StateWaypointPoly(StateWaypoint(NAMES, np.array(q))))
    traj = Trajectory.from_composite(composite)
    np.testing.assert_array_equal(traj.times_s, [0.0, 1.0, 2.0])
    assert traj.duration_s == 2.0


def test_from_composite_mixed_times_that_decrease_raise():
    """A timed state waypoint then an index-timed joint waypoint: no consistent clock."""
    composite = CompositeInstruction()
    timed = StateWaypoint(NAMES, np.array([0.0, 0.0]))
    timed.setTime(5.0)
    _append(composite, StateWaypointPoly(timed))
    untimed = JointWaypoint(NAMES, np.array([1.0, 1.0]))
    _append(composite, JointWaypointPoly(untimed))
    with pytest.raises(TrajectoryShapeError):
        Trajectory.from_composite(composite)


def test_from_composite_maps_columns_by_joint_name():
    """The same pose with its names swapped is the same row."""
    composite = CompositeInstruction()
    first = StateWaypoint(["j1", "j2"], np.array([1.0, 2.0]))
    second = StateWaypoint(["j2", "j1"], np.array([2.0, 1.0]))
    for wp in (first, second):
        _append(composite, StateWaypointPoly(wp))
    np.testing.assert_array_equal(
        Trajectory.from_composite(composite).positions, [[1.0, 2.0], [1.0, 2.0]]
    )


@pytest.mark.parametrize(
    ("names", "q"),
    [(["j1", "j3"], [1.0, 2.0]), (["j1", "j2", "j3"], [1.0, 2.0, 3.0]), (["j1"], [1.0])],
    ids=["other-joint", "extra-joint", "missing-joint"],
)
def test_from_composite_rejects_a_different_joint_set(names, q):
    composite = CompositeInstruction()
    _append(composite, StateWaypointPoly(StateWaypoint(NAMES, np.array([0.0, 0.0]))))
    _append(composite, StateWaypointPoly(StateWaypoint(names, np.array(q))))
    with pytest.raises(TrajectoryShapeError, match="j1"):
        Trajectory.from_composite(composite)


def test_build_defaults_to_joint_interpolation():
    traj = Trajectory.build(["a"], [[0.0], [1.0]], [0.0, 1.0])
    assert traj.move_types == (MoveInstructionType.FREESPACE, MoveInstructionType.FREESPACE)


def test_from_rows_is_joint_interpolation():
    traj = Trajectory.from_rows(["a"], [[0.0, 0.0], [1.0, 1.0]])
    assert traj.move_types == (MoveInstructionType.FREESPACE, MoveInstructionType.FREESPACE)


def test_build_rejects_a_move_type_count_that_disagrees():
    with pytest.raises(TrajectoryShapeError):
        Trajectory.build(["a"], [[0.0], [1.0]], [0.0, 1.0], (MoveInstructionType.LINEAR,))


def test_from_composite_keeps_each_move_type():
    composite = CompositeInstruction()
    for move_type in (MoveInstructionType.FREESPACE, MoveInstructionType.LINEAR):
        waypoint = StateWaypointPoly(StateWaypoint(["a"], np.zeros(1)))
        move = MoveInstruction(waypoint, move_type)
        composite.push_back(MoveInstructionPoly(move))
    assert Trajectory.from_composite(composite).move_types == (
        MoveInstructionType.FREESPACE,
        MoveInstructionType.LINEAR,
    )
