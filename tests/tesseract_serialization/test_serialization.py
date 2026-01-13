"""Tests for tesseract_serialization bindings.

Tests XML/binary serialization roundtrip for root types:
- CompositeInstruction (motion programs)
- Environment (full scene via command replay)
- SceneState (joint/link state snapshots)
"""

import tempfile
from pathlib import Path

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType,
    WaypointPoly_as_JointWaypointPoly,
)
from tesseract_robotics.tesseract_serialization import (
    composite_instruction_from_binary,
    composite_instruction_from_file,
    composite_instruction_from_xml,
    composite_instruction_to_binary,
    composite_instruction_to_file,
    composite_instruction_to_xml,
)


def make_test_program():
    """Create a simple test program with 3 waypoints."""
    program = CompositeInstruction("DEFAULT")

    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    positions = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
        np.array([0.2, 0.4, 0.6, 0.8, 1.0, 1.2]),
    ]

    for pos in positions:
        wp = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joint_names, pos))
        mi = MoveInstructionPoly_wrap_MoveInstruction(
            MoveInstruction(wp, MoveInstructionType.FREESPACE)
        )
        program.appendMoveInstruction(mi)

    return program


class TestCompositeInstructionSerialization:
    """Test CompositeInstruction XML/binary serialization."""

    def test_xml_roundtrip(self):
        """Test XML string serialization roundtrip."""
        original = make_test_program()

        xml = composite_instruction_to_xml(original)
        assert isinstance(xml, str)
        assert len(xml) > 0
        assert "<?xml" in xml

        restored = composite_instruction_from_xml(xml)
        assert restored.size() == original.size()
        assert restored.getProfile() == original.getProfile()

    def test_file_roundtrip(self):
        """Test file serialization roundtrip."""
        original = make_test_program()

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "program.xml"

            composite_instruction_to_file(original, str(path))
            assert path.exists()
            assert path.stat().st_size > 0

            restored = composite_instruction_from_file(str(path))
            assert restored.size() == original.size()

    def test_binary_roundtrip(self):
        """Test binary serialization roundtrip."""
        original = make_test_program()

        binary = composite_instruction_to_binary(original)
        assert isinstance(binary, (bytes, list))
        assert len(binary) > 0

        restored = composite_instruction_from_binary(binary)
        assert restored.size() == original.size()

    def test_waypoint_data_preserved(self):
        """Test that waypoint data survives roundtrip."""
        original = make_test_program()

        xml = composite_instruction_to_xml(original)
        restored = composite_instruction_from_xml(xml)

        # Check first waypoint
        orig_inst = original[0].asMoveInstruction()
        rest_inst = restored[0].asMoveInstruction()

        orig_wp = WaypointPoly_as_JointWaypointPoly(orig_inst.getWaypoint())
        rest_wp = WaypointPoly_as_JointWaypointPoly(rest_inst.getWaypoint())

        np.testing.assert_array_almost_equal(orig_wp.getPosition(), rest_wp.getPosition())
        assert orig_wp.getNames() == rest_wp.getNames()

    def test_empty_program(self):
        """Test serialization of empty program."""
        program = CompositeInstruction("EMPTY")

        xml = composite_instruction_to_xml(program)
        restored = composite_instruction_from_xml(xml)

        assert restored.size() == 0
        assert restored.getProfile() == "EMPTY"


# SceneState and Environment serialization tests require environment loading
# These are tested via integration tests in examples/
