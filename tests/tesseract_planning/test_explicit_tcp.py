"""The planning API takes frames from the caller and never picks one (gh-30).

`tcp_frame` and `working_frame` are required wherever a `ManipulatorInfo` is built, `fk` / `ik`
require `tip_link`, and a `tcp_offset` (named SRDF TCP or `Isometry3d`) reaches the planner
unchanged, where `Environment.findTCPOffset` resolves it (gh-133).
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.planning import (
    CartesianTarget,
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    TaskComposer,
)
from tesseract_robotics.tesseract_common import Isometry3d, ManipulatorInfo

GROUP = "manipulator"
TCP_FRAME = "tool0"
WORKING_FRAME = "base_link"
JOINTS_ZERO = [0.0] * 6
# A goal the ABB IRB2400 reaches from zero without passing a singularity.
JOINTS_GOAL = [0.4, 0.2, -0.1, 0.3, -0.4, 0.5]

# Tool offset from tool0 along its z axis, in m: large next to the planned-pose tolerance below,
# so landing tool0 (not the tool) on the target fails the test.
TOOL_OFFSET_Z_M = 0.1
# TrajOpt's default Cartesian constraint converges to ~1e-4 m / rad; 1e-3 leaves margin for the
# solver and is two orders of magnitude below TOOL_OFFSET_Z_M.
PLANNED_POSE_ATOL = 1e-3


def _offset() -> Isometry3d:
    matrix = np.eye(4)
    matrix[2, 3] = TOOL_OFFSET_Z_M
    return Isometry3d(matrix)


def _offset_matrix(info: ManipulatorInfo) -> np.ndarray:
    assert isinstance(info.tcp_offset, Isometry3d), info.tcp_offset
    return info.tcp_offset.matrix


@pytest.fixture(scope="module")
def robot() -> Robot:
    return Robot.from_tesseract_support("abb_irb2400")


class TestFramesAreRequired:
    def test_motion_program_without_frames_raises(self):
        with pytest.raises(TypeError, match="tcp_frame"):
            MotionProgram(GROUP)  # ty: ignore[missing-argument]

    def test_motion_program_without_working_frame_raises(self):
        with pytest.raises(TypeError, match="working_frame"):
            MotionProgram(GROUP, TCP_FRAME)  # ty: ignore[missing-argument]

    def test_get_manipulator_info_without_frames_raises(self, robot):
        with pytest.raises(TypeError, match="tcp_frame"):
            robot.get_manipulator_info(GROUP)

    def test_get_manipulator_info_without_working_frame_raises(self, robot):
        with pytest.raises(TypeError, match="working_frame"):
            robot.get_manipulator_info(GROUP, TCP_FRAME)

    def test_fk_without_tip_link_raises(self, robot):
        with pytest.raises(TypeError, match="tip_link"):
            robot.fk(GROUP, JOINTS_ZERO)

    def test_ik_without_tip_link_raises(self, robot):
        with pytest.raises(TypeError, match="tip_link"):
            robot.ik(GROUP, Pose.from_xyz(0.8, 0.0, 1.2))

    def test_to_composite_instruction_has_no_tcp_override(self):
        program = MotionProgram(GROUP, TCP_FRAME, WORKING_FRAME).move_to(JointTarget(JOINTS_ZERO))
        joint_names = [f"joint_{i}" for i in range(1, 7)]
        with pytest.raises(TypeError):
            program.to_composite_instruction(joint_names, TCP_FRAME)  # ty: ignore[too-many-positional-arguments]


class TestFramesReachManipulatorInfo:
    def test_get_manipulator_info_fields(self, robot):
        info = robot.get_manipulator_info(GROUP, "link_6", "world")

        assert (info.manipulator, info.tcp_frame, info.working_frame) == (GROUP, "link_6", "world")
        np.testing.assert_array_equal(_offset_matrix(info), np.eye(4))

    def test_get_manipulator_info_is_not_shared(self, robot):
        """Each call builds its own ManipulatorInfo: mutating one does not leak into the next."""
        first = robot.get_manipulator_info(GROUP, TCP_FRAME, WORKING_FRAME)
        first.tcp_frame = "link_6"

        assert robot.get_manipulator_info(GROUP, TCP_FRAME, WORKING_FRAME).tcp_frame == TCP_FRAME

    def test_get_manipulator_info_named_tcp_offset(self, robot):
        info = robot.get_manipulator_info(GROUP, TCP_FRAME, WORKING_FRAME, tcp_offset="laser")
        assert info.tcp_offset == "laser"

    def test_get_manipulator_info_isometry_tcp_offset(self, robot):
        info = robot.get_manipulator_info(GROUP, TCP_FRAME, WORKING_FRAME, tcp_offset=_offset())
        np.testing.assert_array_equal(_offset_matrix(info), _offset().matrix)

    def test_tcp_offset_is_keyword_only(self, robot):
        with pytest.raises(TypeError):
            robot.get_manipulator_info(GROUP, TCP_FRAME, WORKING_FRAME, "laser")

    def test_composite_carries_program_frames(self, robot):
        program = MotionProgram(GROUP, TCP_FRAME, "world", tcp_offset="welder").move_to(
            JointTarget(JOINTS_ZERO)
        )

        info = program.to_composite_instruction(robot.get_joint_names(GROUP)).getManipulatorInfo()

        assert (info.manipulator, info.tcp_frame, info.working_frame) == (GROUP, TCP_FRAME, "world")
        assert info.tcp_offset == "welder"
        # the name is the environment's to resolve: abb_irb2400.srdf's welder TCP
        np.testing.assert_array_equal(robot.env.findTCPOffset(info).translation, [0.1, 1.0, 0.2])

    def test_composite_default_tcp_offset_is_identity(self, robot):
        program = MotionProgram(GROUP, TCP_FRAME, WORKING_FRAME).move_to(JointTarget(JOINTS_ZERO))

        info = program.to_composite_instruction(robot.get_joint_names(GROUP)).getManipulatorInfo()

        np.testing.assert_array_equal(_offset_matrix(info), np.eye(4))

    @pytest.mark.parametrize("tip_link", ["link_6", TCP_FRAME])
    def test_fk_returns_named_tip_link(self, robot, tip_link):
        state = robot.env.getState(robot.get_joint_names(GROUP), np.asarray(JOINTS_GOAL))

        pose = robot.fk(GROUP, JOINTS_GOAL, tip_link)

        np.testing.assert_array_equal(pose.matrix, state.link_transforms[tip_link].matrix)


def test_planner_applies_tcp_offset(robot):
    """TrajOpt puts the tool (tool0 * offset), not tool0, on a Cartesian target."""
    target = robot.fk(GROUP, JOINTS_GOAL, TCP_FRAME) @ Pose(_offset())
    joint_names = robot.get_joint_names(GROUP)
    robot.set_joints(JOINTS_ZERO, joint_names=joint_names)
    program = (
        MotionProgram(GROUP, TCP_FRAME, WORKING_FRAME, tcp_offset=_offset())
        .set_joint_names(joint_names)
        .move_to(JointTarget(JOINTS_ZERO))
        .move_to(CartesianTarget(target))
    )

    result = TaskComposer.from_config().plan(robot, program, pipeline="TrajOptPipeline")

    assert result.successful, result.message
    flange = robot.fk(GROUP, result.trajectory[-1].positions, TCP_FRAME)
    tool = flange @ Pose(_offset())
    np.testing.assert_allclose(tool.matrix, target.matrix, rtol=0, atol=PLANNED_POSE_ATOL)
    assert np.linalg.norm(flange.translation - target.translation) > TOOL_OFFSET_Z_M / 2
