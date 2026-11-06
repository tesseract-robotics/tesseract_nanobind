"""The viewer turns id-based tesseract results into plain data: ids cross as str (#141)."""

import numpy as np
import pytest

from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
)
from tesseract_robotics.viewer.poses import link_poses
from tesseract_robotics.viewer.tcp_path import tcp_path
from tesseract_robotics.viewer.trajectory import Trajectory


@pytest.fixture(scope="module")
def robot():
    return Robot.from_tesseract_support("abb_irb2400")


@pytest.fixture(scope="module")
def trajectory(robot):
    joints = robot.get_joint_names("manipulator")
    program = CompositeInstruction()
    for q in (np.zeros(len(joints)), np.full(len(joints), 0.1)):
        wp = JointWaypointPoly(JointWaypoint(joints, q))
        mi = MoveInstruction(wp, MoveInstructionType.FREESPACE)
        program.push_back(MoveInstructionPoly(mi))
    return Trajectory.from_composite(program)


def test_trajectory_carries_joint_names_as_str(robot, trajectory):
    assert all(type(n) is str for n in trajectory.joint_names)
    assert list(trajectory.joint_names) == robot.get_joint_names("manipulator")


def test_link_poses_are_keyed_by_str(robot, trajectory):
    poses = link_poses(robot.env, trajectory.joint_names, trajectory.positions[0])
    assert all(type(link) is str for link in poses)
    assert "base_link" in poses


def test_tcp_path_takes_the_tcp_frame_as_str(robot, trajectory):
    manip = robot.get_manipulator_info("manipulator", "tool0", "base_link")
    assert tcp_path(robot.env, trajectory, manip).segments.shape[0] > 0
