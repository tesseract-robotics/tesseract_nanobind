"""Tests for tesseract_serialization bindings.

Tests XML/binary serialization roundtrip for root types:
- CompositeInstruction (motion programs)
- Environment (full scene via command replay)
- SceneState (joint/link state snapshots)
"""

import os
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
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_serialization import (
    composite_instruction_from_binary,
    composite_instruction_from_file,
    composite_instruction_from_xml,
    composite_instruction_to_binary,
    composite_instruction_to_file,
    composite_instruction_to_xml,
    environment_from_binary,
    environment_from_file,
    environment_from_xml,
    environment_to_binary,
    environment_to_file,
    environment_to_xml,
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


def make_test_environment():
    """Build an Environment from the bundled lbr_iiwa support model.

    Located via TESSERACT_RESOURCE_PATH (set by tesseract_robotics for bundled
    data) so the test does not depend on a TESSERACT_SUPPORT_DIR override.
    """
    base = None
    for entry in os.environ.get("TESSERACT_RESOURCE_PATH", "").split(os.pathsep):
        cand = Path(entry) / "tesseract" / "support" / "urdf"
        if (cand / "lbr_iiwa_14_r820.urdf").exists():
            base = cand
            break
    assert base is not None, "lbr_iiwa_14_r820 support model not on TESSERACT_RESOURCE_PATH"

    env = Environment()
    assert env.initFromUrdfSrdf(
        (base / "lbr_iiwa_14_r820.urdf").read_text(),
        (base / "lbr_iiwa_14_r820.srdf").read_text(),
        GeneralResourceLocator(),
    )
    return env


def _env_fingerprint(env):
    return (
        sorted(env.getLinkNames()),
        sorted(env.getJointNames()),
        sorted(env.getActiveLinkNames()),
    )


class TestEnvironmentSerialization:
    """Round-trip the full Environment (scene graph + command history + resource locator).

    Regression for a writer/reader type asymmetry: the writers serialized a *concrete*
    `Environment` while the readers deserialized a *polymorphic* `shared_ptr<Environment>`
    (``is_polymorphic<Environment>`` is true), so the reader misparsed the stream and every
    ``environment_from_*`` failed with "Trying to load a registered polymorphic type with an
    unregistered polymorphic cast". Both sides must use ``shared_ptr<Environment>`` — as
    tesseract's own C++ ``EnvironmentSerializeUnit`` test does.
    """

    def test_binary_roundtrip(self):
        env = make_test_environment()
        restored = environment_from_binary(environment_to_binary(env))
        assert _env_fingerprint(restored) == _env_fingerprint(env)

    def test_xml_roundtrip(self):
        env = make_test_environment()
        restored = environment_from_xml(environment_to_xml(env))
        assert _env_fingerprint(restored) == _env_fingerprint(env)

    def test_file_roundtrip(self):
        env = make_test_environment()
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "env.xml")
            environment_to_file(env, path)
            restored = environment_from_file(path)
        assert _env_fingerprint(restored) == _env_fingerprint(env)

    def test_roundtrip_preserves_contact_manager(self):
        """The deserialized env must still produce a working discrete contact manager —
        the polymorphic ResourceLocator load is exactly what the asymmetry broke."""
        env = make_test_environment()
        restored = environment_from_binary(environment_to_binary(env))
        restored.clearCachedDiscreteContactManager()
        assert restored.getDiscreteContactManager() is not None
