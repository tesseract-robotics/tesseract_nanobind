"""Free functions of `tesseract/command_language/utils.h` (#274)."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CompositeInstruction,
    JointWaypoint,
    MoveInstruction,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
    WaitInstruction,
    WaypointPoly,
    checkJointPositionFormat,
    clampToJointLimits,
    formatJointPosition,
    getJointNames,
    getJointPosition,
    isWithinJointLimits,
    makeTimeMonotonicallyIncreasing,
    setJointPosition,
    toDelimitedFile,
    toJointTrajectory,
)
from tesseract_robotics.tesseract_common import Isometry3d

NAMES = ["j1", "j2"]


def _state_move(position, time):
    wp = StateWaypoint(NAMES, position, [0.0, 0.0], [0.0, 0.0], time)
    return MoveInstruction(StateWaypointPoly(wp), MoveInstructionType.FREESPACE)


def _program(times=(0.0, 1.0, 2.0)):
    """A planned-program shape: [child[move, wait, move], move], state waypoints with times."""
    child = CompositeInstruction()
    child.push_back(_state_move([0.0, 0.0], times[0]))
    child.push_back(WaitInstruction(0.5))
    child.push_back(_state_move([0.1, 0.2], times[1]))
    top = CompositeInstruction()
    top.push_back(child)
    top.push_back(_state_move([0.3, 0.4], times[2]))
    return top


def _times(program):
    return [
        ip.asMoveInstruction().getWaypoint().asStateWaypoint().getTime()
        for ip in program.flatten(_is_move)
    ]


def _is_move(ip, parent):
    return ip.isMoveInstruction()


def test_to_joint_trajectory():
    traj = toJointTrajectory(_program())
    assert len(traj) == 3
    np.testing.assert_allclose(
        [s.position for s in traj.states], [[0.0, 0.0], [0.1, 0.2], [0.3, 0.4]]
    )
    assert [s.time for s in traj.states] == [0.0, 1.0, 2.0]
    assert traj[0].joint_names == NAMES


def test_to_joint_trajectory_from_instruction_poly():
    program = _program()
    assert len(toJointTrajectory(program[0])) == 2


def test_joint_names_and_position():
    wp = WaypointPoly(JointWaypoint(NAMES, [1.0, 2.0]))
    assert getJointNames(wp) == NAMES
    assert list(getJointPosition(wp)) == [1.0, 2.0]
    assert list(getJointPosition(["j2", "j1"], wp)) == [2.0, 1.0]


def test_format_joint_position_reorders_in_place():
    wp = WaypointPoly(JointWaypoint(NAMES, [1.0, 2.0]))
    assert not checkJointPositionFormat(["j2", "j1"], wp)
    assert formatJointPosition(["j2", "j1"], wp)
    assert checkJointPositionFormat(["j2", "j1"], wp)
    assert getJointNames(wp) == ["j2", "j1"]
    assert list(getJointPosition(wp)) == [2.0, 1.0]


def test_set_joint_position_through_instruction_waypoint():
    move = _state_move([0.0, 0.0], 0.0)
    assert setJointPosition(move.getWaypoint(), [0.5, 0.6])
    assert list(getJointPosition(move.getWaypoint())) == [0.5, 0.6]


def test_set_joint_position_on_cartesian_returns_false():
    assert not setJointPosition(WaypointPoly(CartesianWaypoint(Isometry3d.Identity())), [0.0, 0.0])


LIMITS = np.array([[-1.0, 1.0], [-1.0, 1.0]])


def test_clamp_to_joint_limits():
    wp = WaypointPoly(JointWaypoint(NAMES, [1.5, 0.0]))
    assert not isWithinJointLimits(wp, LIMITS)
    assert clampToJointLimits(wp, LIMITS)
    assert isWithinJointLimits(wp, LIMITS)
    assert list(getJointPosition(wp)) == [1.0, 0.0]


def test_clamp_respects_max_deviation():
    wp = WaypointPoly(JointWaypoint(NAMES, [1.5, 0.0]))
    assert not clampToJointLimits(wp, LIMITS, 0.1)
    assert not clampToJointLimits(wp, LIMITS, [0.1, 0.1])
    assert clampToJointLimits(wp, LIMITS, [1.0, 1.0])
    assert isWithinJointLimits(wp, LIMITS)


def test_make_time_monotonically_increasing():
    # A drop in time starts a new segment, appended after the previous one: 0, 1 | 0.5 -> 0, 1, 1.5.
    program = _program(times=(0.0, 1.0, 0.5))
    makeTimeMonotonicallyIncreasing(program)
    assert _times(program) == pytest.approx([0.0, 1.0, 1.5])
    # A repeated time is left as it is.
    program = _program(times=(0.0, 1.0, 1.0))
    makeTimeMonotonicallyIncreasing(program)
    assert _times(program) == pytest.approx([0.0, 1.0, 1.0])


def test_to_delimited_file(tmp_path):
    path = tmp_path / "program.csv"
    assert toDelimitedFile(_program(), path)
    lines = path.read_text().strip().splitlines()
    assert lines[1].split(",")[:2] == ["0", "0"]
    path_semicolon = tmp_path / "program_semicolon.csv"
    assert toDelimitedFile(_program(), path_semicolon, ";")
    assert ";" in path_semicolon.read_text()


def test_in_place_functions_reject_typed_polys():
    """A typed Poly is not a WaypointPoly: converting would edit a temporary."""
    with pytest.raises(TypeError):
        setJointPosition(StateWaypointPoly(StateWaypoint(NAMES, [0.0, 0.0])), [1.0, 1.0])
