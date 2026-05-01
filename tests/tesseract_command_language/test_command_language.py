"""Tests for tesseract_command_language bindings.

These tests cover the command language types used in motion planning examples,
including waypoints, instructions, and composite instructions.
"""

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    # Poly wrappers
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    # Non-motion instruction casts
    InstructionPoly_as_SetAnalogInstruction,
    InstructionPoly_as_SetDigitalInstruction,
    InstructionPoly_as_SetToolInstruction,
    InstructionPoly_as_TimerInstruction,
    InstructionPoly_as_WaitInstruction,
    # Waypoint types
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    # Instructions
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionPoly_wrap_MoveInstruction,
    # Enums
    MoveInstructionType,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    # Other
    ProfileDictionary,
    SetAnalogInstruction,
    SetDigitalInstruction,
    SetToolInstruction,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    TimerInstruction,
    TimerInstructionType,
    # Non-motion instructions
    WaitInstruction,
    WaitInstructionType,
)
from tesseract_robotics.tesseract_common import (
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)


class TestJointWaypoint:
    """Test JointWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = JointWaypoint()
        assert wp is not None

    def test_constructor_with_names_and_position(self):
        names = ["joint_1", "joint_2", "joint_3"]
        position = np.array([0.1, 0.2, 0.3])
        wp = JointWaypoint(names, position)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), position)

    def test_constructor_with_constrained_flag(self):
        names = ["j1", "j2"]
        position = np.array([1.0, 2.0])
        wp = JointWaypoint(names, position, False)
        assert not wp.isConstrained()

    def test_set_get_methods(self):
        wp = JointWaypoint()
        names = ["a", "b"]
        pos = np.array([0.5, 1.5])
        wp.setNames(names)
        wp.setPosition(pos)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), pos)


class TestCartesianWaypoint:
    """Test CartesianWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = CartesianWaypoint()
        assert wp is not None

    def test_constructor_with_isometry(self):
        transform = Isometry3d.Identity() * Translation3d(1.0, 2.0, 3.0)
        wp = CartesianWaypoint(transform)
        result = wp.getTransform()
        np.testing.assert_array_almost_equal(result.translation(), np.array([1.0, 2.0, 3.0]))

    def test_constructor_with_quaternion(self):
        # Create transform with rotation
        transform = (
            Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5) * Quaterniond(0.707, 0, 0.707, 0)
        )
        wp = CartesianWaypoint(transform)
        assert wp is not None


class TestStateWaypoint:
    """Test StateWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = StateWaypoint()
        assert wp is not None

    def test_constructor_with_names_and_position(self):
        names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        position = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        wp = StateWaypoint(names, position)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), position)


class TestWaypointPoly:
    """Test Poly wrapper types for waypoints."""

    def test_cartesian_waypoint_poly_wrap(self):
        transform = Isometry3d.Identity() * Translation3d(1.0, 0.0, 0.0)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()

    def test_joint_waypoint_poly_wrap(self):
        names = ["j1", "j2"]
        position = np.array([0.5, 1.0])
        wp = JointWaypoint(names, position)
        poly = JointWaypointPoly_wrap_JointWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()

    def test_state_waypoint_poly_wrap(self):
        names = ["j1", "j2"]
        position = np.array([0.5, 1.0])
        wp = StateWaypoint(names, position)
        poly = StateWaypointPoly_wrap_StateWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()


class TestMoveInstruction:
    """Test MoveInstruction creation - the core of motion planning."""

    def test_constructor_cartesian_two_args(self):
        """Test MoveInstruction(waypoint, type) - 2-arg form."""
        transform = Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType.FREESPACE)
        assert mi is not None
        assert mi.getMoveType() == MoveInstructionType.FREESPACE

    def test_constructor_cartesian_three_args_with_profile(self):
        """Test MoveInstruction(waypoint, type, profile) - 3-arg SWIG-compatible form."""
        transform = Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType_FREESPACE, "DEFAULT")
        assert mi is not None
        assert mi.getMoveType() == MoveInstructionType.FREESPACE
        assert mi.getProfile() == "DEFAULT"

    def test_constructor_joint_three_args(self):
        """Test MoveInstruction with JointWaypoint and profile."""
        names = ["j1", "j2", "j3"]
        position = np.array([0.1, 0.2, 0.3])
        wp = JointWaypoint(names, position)
        poly = JointWaypointPoly_wrap_JointWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType_LINEAR, "MY_PROFILE")
        assert mi.getMoveType() == MoveInstructionType.LINEAR
        assert mi.getProfile() == "MY_PROFILE"

    def test_move_instruction_poly_wrap(self):
        """Test wrapping MoveInstruction in MoveInstructionPoly."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")
        mi_poly = MoveInstructionPoly_wrap_MoveInstruction(mi)
        assert mi_poly is not None
        assert not mi_poly.isNull()


class TestCompositeInstruction:
    """Test CompositeInstruction - container for motion planning programs."""

    def test_default_constructor(self):
        program = CompositeInstruction()
        assert program is not None
        assert program.empty()
        assert len(program) == 0

    def test_constructor_with_profile(self):
        """Test CompositeInstruction(profile) - SWIG-compatible constructor."""
        program = CompositeInstruction("DEFAULT")
        assert program is not None
        assert program.getProfile() == "DEFAULT"

    def test_set_manipulator_info(self):
        program = CompositeInstruction("DEFAULT")
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        program.setManipulatorInfo(manip_info)
        result = program.getManipulatorInfo()
        assert result.tcp_frame == "tool0"

    def test_append_move_instruction(self):
        """Test appending MoveInstructionPoly to CompositeInstruction."""
        program = CompositeInstruction("DEFAULT")

        # Create waypoints and instructions
        wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, -0.3, 1.5))
        wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5))

        mi1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        mi2 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi1))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi2))

        assert len(program) == 2
        assert not program.empty()


class TestNestedCompositeInstruction:
    """Test nesting one CompositeInstruction inside another."""

    @staticmethod
    def _make_move_instr(x: float) -> MoveInstructionPoly:
        wp = CartesianWaypoint(Isometry3d.Identity() * Translation3d(x, 0.0, 1.0))
        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        return MoveInstructionPoly_wrap_MoveInstruction(mi)

    def test_push_back_composite(self):
        """push_back accepts a CompositeInstruction as a nested child."""
        outer = CompositeInstruction("DEFAULT")
        inner = CompositeInstruction("DEFAULT")
        inner.setDescription("approach")
        inner.push_back(self._make_move_instr(0.1))

        outer.push_back(inner)

        assert len(outer) == 1
        child = outer[0]
        assert child.isCompositeInstruction()
        assert not child.isMoveInstruction()

    def test_mixed_children(self):
        """push_back can interleave move instructions and nested composites."""
        outer = CompositeInstruction("DEFAULT")
        inner = CompositeInstruction("DEFAULT")
        inner.setDescription("approach")

        outer.push_back(inner)
        outer.push_back(self._make_move_instr(0.5))

        assert len(outer) == 2
        assert outer[0].isCompositeInstruction()
        assert outer[1].isMoveInstruction()

    def test_as_composite_instruction_round_trip(self):
        """asCompositeInstruction extracts the composite payload with its
        description + children intact."""
        outer = CompositeInstruction("DEFAULT")
        inner = CompositeInstruction("DEFAULT")
        inner.setDescription("to_end")
        inner.push_back(self._make_move_instr(1.0))

        outer.push_back(inner)
        recovered = outer[0].asCompositeInstruction()

        assert recovered.getDescription() == "to_end"
        assert len(recovered) == 1

    def test_as_composite_instruction_wrong_type_raises(self):
        """Calling asCompositeInstruction on a move-instruction payload
        raises RuntimeError, matching asMoveInstruction's error style."""
        import pytest

        outer = CompositeInstruction("DEFAULT")
        outer.push_back(self._make_move_instr(0.0))

        move_ip = outer[0]
        assert move_ip.isMoveInstruction()

        with pytest.raises(RuntimeError):
            move_ip.asCompositeInstruction()

    def test_raster_like_dispatch_structure(self):
        """Full RasterMotionTask-style structure: outer composite whose
        children are description-tagged sub-composites that a dispatcher
        routes to different sub-pipelines."""
        outer = CompositeInstruction("DEFAULT")

        approach = CompositeInstruction("DEFAULT")
        approach.setDescription("approach")
        approach.push_back(self._make_move_instr(0.0))

        raster = CompositeInstruction("DEFAULT")
        raster.setDescription("Raster Index 0")
        raster.push_back(self._make_move_instr(0.1))
        raster.push_back(self._make_move_instr(0.2))

        to_end = CompositeInstruction("DEFAULT")
        to_end.setDescription("to_end")
        to_end.push_back(self._make_move_instr(0.3))

        outer.push_back(approach)
        outer.push_back(raster)
        outer.push_back(to_end)

        assert len(outer) == 3
        descriptions = []
        for i in range(len(outer)):
            child_ip = outer[i]
            assert child_ip.isCompositeInstruction()
            descriptions.append(child_ip.asCompositeInstruction().getDescription())
        assert descriptions == ["approach", "Raster Index 0", "to_end"]


class TestProfileDictionary:
    """Test ProfileDictionary for motion planning profiles."""

    def test_default_constructor(self):
        profiles = ProfileDictionary()
        assert profiles is not None


class TestABBExampleWorkflow:
    """Integration test simulating the ABB IRB2400 viewer example workflow.

    This tests the exact sequence of operations used in abb_irb2400_viewer.py
    to ensure users can run examples without errors.
    """

    def test_complete_command_language_workflow(self):
        """Test the command_language portion of the ABB example."""
        # Setup ManipulatorInfo (as in example)
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Define waypoints (as in example)
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, -0.3, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.3, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )
        wp3 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.5, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )

        # Create instructions (as in example - 3-arg form with profile)
        start_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        _plan_f2 = MoveInstruction(  # noqa: F841  # kept for test expansion
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp3),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        # Create program (as in example)
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        assert len(program) == 2
        assert program.getProfile() == "DEFAULT"
        result_info = program.getManipulatorInfo()
        assert result_info.tcp_frame == "tool0"


class TestMoveInstructionDescription:
    """Test MoveInstruction.setDescription and getDescription methods."""

    def test_set_get_description(self):
        """Test setDescription and getDescription on MoveInstruction."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")

        # Set description
        mi.setDescription("test_waypoint_1")
        assert mi.getDescription() == "test_waypoint_1"

        # Change description
        mi.setDescription("updated_waypoint")
        assert mi.getDescription() == "updated_waypoint"

    def test_description_on_move_instruction_poly(self):
        """Test setDescription on MoveInstructionPoly."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")
        mi_poly = MoveInstructionPoly_wrap_MoveInstruction(mi)

        # MoveInstructionPoly also has setDescription
        mi_poly.setDescription("poly_description")
        assert mi_poly.getDescription() == "poly_description"


class TestAnyPolyWrappers:
    """Test AnyPoly wrapper functions for TaskComposerDataStorage."""

    def test_anypoly_wrap_composite_instruction(self):
        """Test wrapping CompositeInstruction in AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_wrap_CompositeInstruction,
        )

        program = CompositeInstruction("DEFAULT")
        # Add some instructions
        wp = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5))
        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        # Wrap in AnyPoly
        any_poly = AnyPoly_wrap_CompositeInstruction(program)
        assert any_poly is not None
        assert not any_poly.isNull()

    def test_anypoly_wrap_profile_dictionary(self):
        """Test wrapping ProfileDictionary in AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_wrap_ProfileDictionary,
        )

        profiles = ProfileDictionary()
        any_poly = AnyPoly_wrap_ProfileDictionary(profiles)
        assert any_poly is not None
        assert not any_poly.isNull()

    def test_anypoly_roundtrip_composite_instruction(self):
        """Test wrapping and unwrapping CompositeInstruction via AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_as_CompositeInstruction,
            AnyPoly_wrap_CompositeInstruction,
        )

        # Create program with specific content
        program = CompositeInstruction("MY_PROFILE")
        wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(1.0, 0.0, 0.0))
        wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.0, 1.0, 0.0))
        mi1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        mi2 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_LINEAR,
            "DEFAULT",
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi1))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi2))

        # Wrap
        any_poly = AnyPoly_wrap_CompositeInstruction(program)

        # Unwrap
        recovered = AnyPoly_as_CompositeInstruction(any_poly)
        assert recovered is not None
        assert len(recovered) == 2
        assert recovered.getProfile() == "MY_PROFILE"


class TestWaitInstruction:
    """Test WaitInstruction: construction, predicates, casts, implicit
    conversion, and a scan-style integration use-case."""

    def test_time_constructor(self):
        wi = WaitInstruction(0.5)
        assert wi.getWaitType() == WaitInstructionType.TIME
        assert wi.getWaitTime() == 0.5

    def test_digital_constructor(self):
        wi = WaitInstruction(WaitInstructionType.DIGITAL_INPUT_HIGH, 3)
        assert wi.getWaitType() == WaitInstructionType.DIGITAL_INPUT_HIGH
        assert wi.getWaitIO() == 3

    def test_setters_round_trip(self):
        wi = WaitInstruction(0.0)
        wi.setWaitType(WaitInstructionType.DIGITAL_OUTPUT_LOW)
        wi.setWaitIO(7)
        wi.setWaitTime(1.25)
        assert wi.getWaitType() == WaitInstructionType.DIGITAL_OUTPUT_LOW
        assert wi.getWaitIO() == 7
        assert wi.getWaitTime() == 1.25

    def test_predicate_and_implicit_conversion(self):
        """push_back(WaitInstruction) goes through implicit conversion to
        InstructionPoly; isWaitInstruction is the only predicate that fires."""
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(WaitInstruction(0.5))

        ip = composite[0]
        assert ip.isWaitInstruction()
        assert not ip.isMoveInstruction()
        assert not ip.isCompositeInstruction()
        assert not ip.isTimerInstruction()
        assert not ip.isSetAnalogInstruction()
        assert not ip.isSetDigitalInstruction()
        assert not ip.isSetToolInstruction()

    def test_round_trip_cast(self):
        """as method and module-level helper both recover the value."""
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(WaitInstruction(0.5))
        ip = composite[0]

        from_method = ip.asWaitInstruction()
        from_helper = InstructionPoly_as_WaitInstruction(ip)
        assert from_method.getWaitType() == WaitInstructionType.TIME
        assert from_method.getWaitTime() == 0.5
        assert from_helper.getWaitTime() == from_method.getWaitTime()

    def test_wrong_type_cast_raises(self):
        import pytest

        composite = CompositeInstruction("DEFAULT")
        composite.push_back(WaitInstruction(0.5))
        wait_ip = composite[0]

        with pytest.raises(RuntimeError):
            wait_ip.asMoveInstruction()
        with pytest.raises(RuntimeError):
            wait_ip.asTimerInstruction()

    def test_scan_program_interleaves_moves_and_waits(self):
        """Concrete usage: a scan program where each move is followed by a
        wait for the camera-done IO line."""
        from tesseract_robotics.tesseract_common import Isometry3d, Translation3d

        program = CompositeInstruction("DEFAULT")
        for x in (0.0, 0.1, 0.2):
            wp = CartesianWaypoint(Isometry3d.Identity() * Translation3d(x, 0.0, 0.5))
            mi = MoveInstruction(
                CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
                MoveInstructionType_FREESPACE,
                "DEFAULT",
            )
            program.push_back(MoveInstructionPoly_wrap_MoveInstruction(mi))
            program.push_back(WaitInstruction(WaitInstructionType.DIGITAL_INPUT_HIGH, 1))

        assert len(program) == 6
        for i in range(len(program)):
            child = program[i]
            if i % 2 == 0:
                assert child.isMoveInstruction()
            else:
                assert child.isWaitInstruction()
                assert child.asWaitInstruction().getWaitType() == WaitInstructionType.DIGITAL_INPUT_HIGH


class TestTimerInstruction:
    """Test TimerInstruction: construction, predicates, casts, implicit conversion."""

    def test_constructor(self):
        ti = TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 4)
        assert ti.getTimerType() == TimerInstructionType.DIGITAL_OUTPUT_HIGH
        assert ti.getTimerTime() == 2.0
        assert ti.getTimerIO() == 4

    def test_setters_round_trip(self):
        ti = TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0)
        ti.setTimerType(TimerInstructionType.DIGITAL_OUTPUT_LOW)
        ti.setTimerTime(3.5)
        ti.setTimerIO(9)
        assert ti.getTimerType() == TimerInstructionType.DIGITAL_OUTPUT_LOW
        assert ti.getTimerTime() == 3.5
        assert ti.getTimerIO() == 9

    def test_predicate_and_implicit_conversion(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0))

        ip = composite[0]
        assert ip.isTimerInstruction()
        assert not ip.isWaitInstruction()
        assert not ip.isMoveInstruction()

    def test_round_trip_cast(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_LOW, 2.0, 3))
        ip = composite[0]

        from_method = ip.asTimerInstruction()
        from_helper = InstructionPoly_as_TimerInstruction(ip)
        assert from_method.getTimerType() == TimerInstructionType.DIGITAL_OUTPUT_LOW
        assert from_method.getTimerTime() == 2.0
        assert from_method.getTimerIO() == 3
        assert from_helper.getTimerTime() == from_method.getTimerTime()

    def test_wrong_type_cast_raises(self):
        import pytest

        composite = CompositeInstruction("DEFAULT")
        composite.push_back(TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0))
        ip = composite[0]

        with pytest.raises(RuntimeError):
            ip.asWaitInstruction()


class TestSetAnalogInstruction:
    """Test SetAnalogInstruction: construction, predicates, casts, implicit conversion."""

    def test_constructor(self):
        si = SetAnalogInstruction("speed", 1, 0.75)
        assert si.getKey() == "speed"
        assert si.getIndex() == 1
        assert si.getValue() == 0.75

    def test_predicate_and_implicit_conversion(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetAnalogInstruction("speed", 1, 0.75))

        ip = composite[0]
        assert ip.isSetAnalogInstruction()
        assert not ip.isSetDigitalInstruction()
        assert not ip.isMoveInstruction()

    def test_round_trip_cast(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetAnalogInstruction("speed", 1, 0.75))
        ip = composite[0]

        from_method = ip.asSetAnalogInstruction()
        from_helper = InstructionPoly_as_SetAnalogInstruction(ip)
        assert (from_method.getKey(), from_method.getIndex(), from_method.getValue()) == (
            "speed",
            1,
            0.75,
        )
        assert from_helper.getKey() == from_method.getKey()

    def test_wrong_type_cast_raises(self):
        import pytest

        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetAnalogInstruction("k", 0, 0.0))
        ip = composite[0]

        with pytest.raises(RuntimeError):
            ip.asSetDigitalInstruction()


class TestSetDigitalInstruction:
    """Test SetDigitalInstruction: construction, predicates, casts, implicit conversion."""

    def test_constructor(self):
        si = SetDigitalInstruction("gripper", 2, True)
        assert si.getKey() == "gripper"
        assert si.getIndex() == 2
        assert si.getValue() is True

    def test_predicate_and_implicit_conversion(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetDigitalInstruction("gripper", 2, True))

        ip = composite[0]
        assert ip.isSetDigitalInstruction()
        assert not ip.isSetAnalogInstruction()
        assert not ip.isMoveInstruction()

    def test_round_trip_cast(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetDigitalInstruction("gripper", 2, True))
        ip = composite[0]

        from_method = ip.asSetDigitalInstruction()
        from_helper = InstructionPoly_as_SetDigitalInstruction(ip)
        assert (from_method.getKey(), from_method.getIndex(), from_method.getValue()) == (
            "gripper",
            2,
            True,
        )
        assert from_helper.getKey() == from_method.getKey()

    def test_wrong_type_cast_raises(self):
        import pytest

        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetDigitalInstruction("k", 0, True))
        ip = composite[0]

        with pytest.raises(RuntimeError):
            ip.asSetAnalogInstruction()


class TestSetToolInstruction:
    """Test SetToolInstruction: construction, predicates, casts, implicit conversion."""

    def test_constructor(self):
        si = SetToolInstruction(5)
        assert si.getTool() == 5

    def test_predicate_and_implicit_conversion(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetToolInstruction(5))

        ip = composite[0]
        assert ip.isSetToolInstruction()
        assert not ip.isSetAnalogInstruction()
        assert not ip.isMoveInstruction()

    def test_round_trip_cast(self):
        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetToolInstruction(5))
        ip = composite[0]

        from_method = ip.asSetToolInstruction()
        from_helper = InstructionPoly_as_SetToolInstruction(ip)
        assert from_method.getTool() == 5
        assert from_helper.getTool() == from_method.getTool()

    def test_wrong_type_cast_raises(self):
        import pytest

        composite = CompositeInstruction("DEFAULT")
        composite.push_back(SetToolInstruction(5))
        ip = composite[0]

        with pytest.raises(RuntimeError):
            ip.asMoveInstruction()
