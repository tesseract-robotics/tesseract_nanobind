"""The viewer turns id-based tesseract results into JSON: ids must cross as str (#141)."""

import asyncio
import json

import numpy as np
import pytest

from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType,
)
from tesseract_robotics.viewer.tesseract_env_to_gltf import tesseract_env_to_gltf
from tesseract_robotics.viewer.tesseract_viewer_aio import TesseractViewerAIO
from tesseract_robotics.viewer.util import tesseract_trajectory_to_list, trajectory_list_to_json


@pytest.fixture(scope="module")
def robot():
    return Robot.from_tesseract_support("abb_irb2400")


@pytest.fixture(scope="module")
def trajectory(robot):
    joints = robot.get_joint_names("manipulator")
    program = CompositeInstruction()
    for q in (np.zeros(len(joints)), np.full(len(joints), 0.1)):
        wp = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joints, q))
        mi = MoveInstruction(wp, MoveInstructionType.FREESPACE)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))
    return program


def test_env_to_gltf_names_links_and_joints_by_str(robot, trajectory):
    gltf = json.loads(tesseract_env_to_gltf(robot.env, trajectory=trajectory))
    link_names = {
        n["extras"]["tesseract_link"]["name"]
        for n in gltf["nodes"]
        if "tesseract_link" in n.get("extras", {})
    }
    assert "base_link" in link_names
    assert "link_tool0" in {n["name"] for n in gltf["nodes"]}


def test_trajectory_json_carries_joint_names_as_str(robot, trajectory):
    joint_names, traj = tesseract_trajectory_to_list(trajectory)
    assert all(type(n) is str for n in joint_names)
    payload = json.loads(trajectory_list_to_json(joint_names, traj))
    assert payload["joint_names"] == robot.get_joint_names("manipulator")


def test_plot_trajectory_marker_parents_on_root_link_str(robot, trajectory):
    viewer = TesseractViewerAIO()
    viewer.t_env = robot.env
    manip = robot.get_manipulator_info("manipulator")

    async def plot():
        await viewer.plot_trajectory(trajectory, manip, update_now=False)

    asyncio.run(plot())
    markers = json.loads(viewer.markers_json)["markers"]
    assert {m["parent_link_name"] for m in markers} == {"base_link"}
