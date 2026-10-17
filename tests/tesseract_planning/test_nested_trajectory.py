"""#270/#274: trajectory extraction descends into child composites (raster programs)."""

import numpy as np
import pytest

from tesseract_robotics.planning.composer import TaskComposer
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.viewer.util import tesseract_trajectory_to_list

NAMES = ["j1", "j2"]


def _state_move(q, t):
    wp = StateWaypoint(NAMES, np.array(q, dtype=float))
    wp.setTime(t)
    return MoveInstructionPoly(
        MoveInstruction(StateWaypointPoly(wp), MoveInstructionType.FREESPACE, "DEFAULT")
    )


def _nested_program():
    """Top level: one move, then a child composite of two moves whose times restart at 0."""
    program = CompositeInstruction("DEFAULT")
    program.push_back(_state_move([0.0, 0.0], 0.0))
    child = CompositeInstruction("DEFAULT")
    child.push_back(_state_move([0.1, 0.1], 0.0))
    child.push_back(_state_move([0.2, 0.2], 0.5))
    program.push_back(child)
    return program


def test_composer_extraction_reads_child_composites():
    # _extract_trajectory reads no state from the composer: skip the plugin load
    composer = object.__new__(TaskComposer)
    points = composer._extract_trajectory(_nested_program())
    np.testing.assert_allclose([p.positions for p in points], [[0, 0], [0.1, 0.1], [0.2, 0.2]])
    # Each StateWaypoint's own time, as before
    assert [p.time for p in points] == [0.0, 0.0, 0.5]


def test_viewer_list_reads_child_composites():
    joint_names, rows = tesseract_trajectory_to_list(_nested_program())
    assert joint_names == NAMES
    np.testing.assert_allclose([r[:2] for r in rows], [[0, 0], [0.1, 0.1], [0.2, 0.2]])
    # toJointTrajectory: time accumulates across the child composite's restart
    assert [r[2] for r in rows] == pytest.approx([0.0, 0.0, 0.5])


def test_viewer_list_joint_waypoints_count_one_second_each():
    program = CompositeInstruction("DEFAULT")
    for q in ([0.0, 0.0], [0.3, 0.3]):
        program.push_back(
            MoveInstructionPoly(
                MoveInstruction(
                    JointWaypointPoly(JointWaypoint(NAMES, np.array(q))),
                    MoveInstructionType.FREESPACE,
                    "DEFAULT",
                )
            )
        )
    _, rows = tesseract_trajectory_to_list(program)
    assert [r[2] for r in rows] == [1.0, 2.0]


def test_viewer_list_without_joint_states_raises():
    with pytest.raises(ValueError, match="no joint states"):
        tesseract_trajectory_to_list(CompositeInstruction("DEFAULT"))
