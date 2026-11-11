"""Shared plumbing of the low-level examples, in bindings only.

The counterpart of the C++ examples' `example.h` / `example.cpp`: loading a
`tesseract_support` robot, adding a fixed obstacle link, and running a task
composer pipeline the way each C++ `run()` does (plugin factory, executor, data
storage, future). Nothing here uses `tesseract_robotics.planning`.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from tesseract_robotics import get_task_composer_config_path, get_tesseract_resource_path
from tesseract_robotics.command_language import CompositeInstruction, ProfileDictionary
from tesseract_robotics.common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.environment import AddLinkCommand, Environment
from tesseract_robotics.geometry import Geometry
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link, Visual
from tesseract_robotics.task_composer import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerPluginFactory,
)

#: The executor every C++ example creates from the plugin config.
EXECUTOR_NAME = "TaskflowExecutor"


class EnvironmentInitError(RuntimeError):
    """`Environment.init` rejected a URDF/SRDF pair."""


class CommandRejectedError(RuntimeError):
    """`Environment.applyCommand` returned False."""


class ResourceNotFoundError(FileNotFoundError):
    """The resource locator resolved no file for a URL."""


@dataclass(frozen=True)
class TrajectoryState:
    """One state waypoint of a planned program."""

    joint_names: list[str]
    positions: np.ndarray


@dataclass(frozen=True)
class PipelineResult:
    """Outcome of one task composer run.

    Attributes:
        successful: The task context reported success.
        message: Why it failed, or "Planning successful".
        raw_results: The output program; None on failure.
        trajectory: The output program's top-level state waypoints.
    """

    successful: bool
    message: str
    raw_results: CompositeInstruction | None = None
    trajectory: list[TrajectoryState] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.trajectory)


def locate_file(locator: GeneralResourceLocator, url: str) -> Path:
    """The file a `package://` URL resolves to.

    Raises:
        ResourceNotFoundError: the locator resolves nothing for `url`.
    """
    resource = locator.locateResource(url)
    if resource is None:
        raise ResourceNotFoundError(url)
    return Path(resource.getFilePath())


def load_support_environment(
    robot: str, locator: GeneralResourceLocator | None = None
) -> Environment:
    """Environment from `package://tesseract/support/urdf/<robot>.urdf` and `.srdf`.

    Args:
        robot: The `tesseract_support` robot name, e.g. `abb_irb2400`.
        locator: Resource locator; a `GeneralResourceLocator` if omitted.

    Returns:
        The initialized environment.

    Raises:
        EnvironmentInitError: the URDF/SRDF pair did not load.
    """
    locator = locator or GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    urdf = locate_file(locator, f"package://tesseract/support/urdf/{robot}.urdf")
    srdf = locate_file(locator, f"package://tesseract/support/urdf/{robot}.srdf")
    env = Environment()
    if not env.init(urdf, srdf, locator):
        raise EnvironmentInitError(f"failed to initialize {robot} from {urdf}, {srdf}")
    return env


def add_fixed_link(
    env: Environment,
    link_name: str,
    joint_name: str,
    geometry: Geometry,
    origin: Isometry3d,
    parent_link: str = "base_link",
    joint_origin: Isometry3d | None = None,
) -> None:
    """Add a link with one visual and one collision, fixed to `parent_link`.

    The shape of the C++ examples' `addSphere()` / `addBox()` commands.

    Args:
        env: The environment to add the link to.
        link_name: The new link.
        joint_name: Its fixed joint.
        geometry: The visual and collision geometry.
        origin: The geometry's origin in the link.
        parent_link: The joint's parent link.
        joint_origin: The joint's parent-to-joint transform; the identity if omitted.

    Raises:
        CommandRejectedError: the environment rejected the AddLinkCommand.
    """
    link = Link(link_name)
    visual = Visual()
    visual.origin = origin
    visual.geometry = geometry
    link.addVisual(visual)
    collision = Collision()
    collision.origin = origin
    collision.geometry = geometry
    link.addCollision(collision)

    joint = Joint(joint_name)
    joint.parent_link_id = parent_link
    joint.child_link_id = link_name
    joint.type = JointType.FIXED
    if joint_origin is not None:
        joint.parent_to_joint_origin_transform = joint_origin

    if not env.applyCommand(AddLinkCommand(link, joint)):
        raise CommandRejectedError(f"AddLinkCommand({link_name}) rejected")


@functools.cache
def task_composer_factory() -> TaskComposerPluginFactory:
    """The plugin factory of the package's task composer config, built once per process.

    Building it loads the plugin libraries (seconds), so the examples share one.
    """
    return TaskComposerPluginFactory(
        get_task_composer_config_path(),
        GeneralResourceLocator(paths=[get_tesseract_resource_path()]),
    )


def run_pipeline(
    env: Environment,
    program: CompositeInstruction,
    profiles: ProfileDictionary,
    pipeline: str,
) -> PipelineResult:
    """Run `pipeline` on `program`, as the C++ examples' `run()` does.

    Args:
        env: The planning environment.
        program: The input program (`planning_input`).
        profiles: The profile dictionary.
        pipeline: The task composer node name, e.g. `TrajOptPipeline`.

    Returns:
        The run's outcome.
    """
    factory = task_composer_factory()
    task = factory.createTaskComposerNode(pipeline)
    output_key = task.getOutputPortMappings().single("program")
    executor = factory.createTaskComposerExecutor(EXECUTOR_NAME)

    data = TaskComposerDataStorage()
    data.setData("planning_input", AnyPoly_wrap_CompositeInstruction(program))
    data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    future = executor.run(task, data)
    future.wait()
    if not future.context.isSuccessful():
        return PipelineResult(False, f"{pipeline} failed")

    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))
    trajectory = [
        TrajectoryState([jid.name() for jid in state.getJointIds()], np.array(state.getPosition()))
        for state in (
            instr.asMoveInstruction().getWaypoint().asStateWaypoint()
            for instr in results
            if instr.isMoveInstruction()
            and instr.asMoveInstruction().getWaypoint().isStateWaypoint()
        )
    ]
    return PipelineResult(True, "Planning successful", results, trajectory)
