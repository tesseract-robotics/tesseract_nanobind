"""Fixtures shared by the task-composer binding tests: the shipped config, a factory, and a
TrajOpt freespace problem on the KUKA iiwa that the shipped `TrajOptPipeline` solves."""

import gc
from collections.abc import Callable, Iterator
from pathlib import Path

import numpy as np
import pytest

import tesseract_robotics
from tesseract_robotics.planning import JointTarget, MotionProgram, Robot
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerNode,
    TaskComposerPluginFactory,
)

# Joint start and goal of the freespace move (rad), from the gh-48 pipeline-run test.
J_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
J_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])


@pytest.fixture(scope="module")
def config_path() -> Path:
    """The shipped task composer plugin config."""
    return tesseract_robotics.get_task_composer_config_path()


@pytest.fixture(scope="module")
def factory(config_path: Path) -> Iterator[TaskComposerPluginFactory]:
    """One factory per module: every factory loads the plugins again (#137)."""
    factory = TaskComposerPluginFactory(config_path, GeneralResourceLocator())
    yield factory
    del factory
    gc.collect()


@pytest.fixture(scope="module")
def make_storage() -> Callable[[TaskComposerNode], TaskComposerDataStorage]:
    """Build the input data storage for a planning node: program, environment, profiles."""
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(J_START, joint_names=joint_names)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(joint_names)
        .move_to(JointTarget(J_START))
        .move_to(JointTarget(J_END))
    )
    composite = program.to_composite_instruction(joint_names)
    assignCurrentStateAsSeed(composite, robot.env)
    profiles = create_trajopt_default_profiles()

    def make(node: TaskComposerNode) -> TaskComposerDataStorage:
        keys = node.getInputKeys()
        port = "planning_input" if keys.has("planning_input") else "program"
        storage = TaskComposerDataStorage()
        storage.setData(keys.get(port), AnyPoly_wrap_CompositeInstruction(composite))
        storage.setData("environment", AnyPoly_wrap_EnvironmentConst(robot.env))
        storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))
        return storage

    return make
