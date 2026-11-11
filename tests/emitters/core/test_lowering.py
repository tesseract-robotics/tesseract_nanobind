"""lower(): tesseract CompositeInstruction to the typed, frame-tagged event IR."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.command_language import (
    CompositeInstruction,
    MoveInstructionType,
    SetToolInstruction,
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
)
from tesseract_robotics.common import ManipulatorInfo
from tesseract_robotics.emitters.core.errors import (
    EmptyProgramError,
    InvalidEventError,
    MissingManipulatorInfoError,
    TcpOffsetError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    SetDigital,
    ToolChange,
)
from tesseract_robotics.emitters.core.external_axes import (
    ExternalAxisKind,
    ExternalAxisLayout,
    ExternalAxisSpec,
)
from tesseract_robotics.emitters.core.lowering import lower
from tesseract_robotics.planning import Pose
from tests.emitters.programs import JOINT_NAMES, cart_instruction, composite, joint_instruction


def test_linear_cartesian_lowers_to_framed_cartesianmove() -> None:
    ir = lower(composite(cart_instruction(Pose.from_xyz(0.6, -0.1, 0.8), profile="P")))
    [event] = ir.events
    assert isinstance(event, CartesianMove)
    assert event.kind is MoveKind.LINEAR and event.profile == "P"
    assert event.target.frames.working_frame == "base_link"
    assert event.target.frames.tcp_frame == "tool0"
    assert not event.target.frames.has_tcp_offset
    assert np.allclose(event.target.pose.translation, [0.6, -0.1, 0.8])
    assert event.seed_joints is None


def test_seed_is_carried() -> None:
    seed = np.deg2rad([1, 2, 3, 4, 5, 6])
    ir = lower(composite(cart_instruction(Pose.from_xyz(0.6, 0, 0.8), seed_rad=seed)))
    assert np.allclose(ir.events[0].seed_joints, seed)


def test_move_manipulator_info_overrides_composite() -> None:
    move = cart_instruction(Pose.from_xyz(0.6, 0, 0.8))
    override = ManipulatorInfo()
    override.working_frame = "fixture"
    move.setManipulatorInfo(override)
    ir = lower(composite(move))
    frames = ir.events[0].target.frames
    assert frames.working_frame == "fixture" and frames.tcp_frame == "tool0"


def test_nested_composite_inherits_frames() -> None:
    inner = CompositeInstruction("inner")
    inner.push_back(cart_instruction(Pose.from_xyz(0.5, 0, 0.5)))
    ir = lower(composite(inner))
    assert ir.events[0].target.frames.working_frame == "base_link"


def test_tcp_offset_pose_is_carried() -> None:
    c = composite(cart_instruction(Pose.from_xyz(0.5, 0, 0.5)))
    info = ManipulatorInfo("manipulator", "base_link", "tool0", Pose.from_xyz(0, 0, 0.1))
    c.setManipulatorInfo(info)
    frames = lower(c).events[0].target.frames
    assert frames.has_tcp_offset
    assert np.allclose(frames.tcp_offset.translation, [0, 0, 0.1])


def test_named_tcp_offset_raises() -> None:
    c = composite(cart_instruction(Pose.from_xyz(0.5, 0, 0.5)))
    c.setManipulatorInfo(ManipulatorInfo("manipulator", "base_link", "tool0", "flange_offset"))
    with pytest.raises(TcpOffsetError, match="flange_offset"):
        lower(c)


def test_missing_working_frame_raises() -> None:
    c = CompositeInstruction("bare")
    c.push_back(cart_instruction(Pose.from_xyz(0.5, 0, 0.5)))
    with pytest.raises(MissingManipulatorInfoError, match="working_frame"):
        lower(c)


def test_joint_move_carries_frames() -> None:
    ir = lower(composite(joint_instruction(np.zeros(6))))
    assert isinstance(ir.events[0], JointMove) and ir.events[0].frames.tcp_frame == "tool0"


def test_timer_lowers_to_two_events_dwell_then_setdigital() -> None:
    ir = lower(composite(TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 7)))
    assert isinstance(ir.events[0], Dwell) and ir.events[0].seconds == 2.0
    assert isinstance(ir.events[1], SetDigital)
    assert ir.events[1].key is None and ir.events[1].index == 7 and ir.events[1].value is True


def test_negative_wait_raises() -> None:
    with pytest.raises(InvalidEventError):
        lower(composite(WaitInstruction(-1.0)))


def test_settool_lowers_to_toolchange() -> None:
    ir = lower(composite(SetToolInstruction(7)))
    assert isinstance(ir.events[0], ToolChange) and ir.events[0].tool_id == 7


def test_circular_raises() -> None:
    move = cart_instruction(Pose.from_xyz(0.5, 0, 0.5), MoveInstructionType.CIRCULAR)
    with pytest.raises(UnsupportedInstructionError, match="CIRCULAR"):
        lower(composite(move))


def test_empty_raises() -> None:
    with pytest.raises(EmptyProgramError, match="no leaf instructions"):
        lower(CompositeInstruction("empty"))


def test_coordinated_cartesian_raises() -> None:
    layout = ExternalAxisLayout.build(
        JOINT_NAMES, [ExternalAxisSpec.build("track", ExternalAxisKind.LINEAR)]
    )
    with pytest.raises(UnsupportedInstructionError, match="coordinated Cartesian"):
        lower(composite(cart_instruction(Pose.from_xyz(0.5, 0, 0.5))), external_axes=layout)
