"""
Task Composer helpers for high-level planning pipelines.

Wraps the TaskComposer system with a simpler interface that handles
all the AnyPoly wrapping and boilerplate.

Example:
    from tesseract_robotics.planning import Robot, MotionProgram, TaskComposer

    robot = Robot.from_tesseract_support("abb_irb2400")
    program = MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link").move_to(...)

    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    if result.successful:
        for waypoint in result.trajectory:
            print(waypoint)
"""

from __future__ import annotations

import copy
import threading
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Final

import numpy as np
from loguru import logger

if TYPE_CHECKING:
    from tesseract_robotics.planning.core import Robot
    from tesseract_robotics.planning.program import MotionProgram

from tesseract_robotics.command_language import (
    CompositeInstruction,
    ProfileDictionary,
)
from tesseract_robotics.common import (
    GeneralResourceLocator,
)
from tesseract_robotics.motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.planning.errors import (
    ConflictingExecutorError,
    InvalidThreadCountError,
    PipelineLayoutError,
    PipelineWarmupError,
    UnknownPipelineError,
)
from tesseract_robotics.planning.profile_types import check_descartes_profile_types
from tesseract_robotics.planning.profiles.pipelines import default_profiles_for
from tesseract_robotics.task_composer import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerExecutor,
    TaskComposerNodeInfo,
    TaskComposerPluginError,
    TaskComposerPluginFactory,
    TaskflowTaskComposerExecutor,
)


@dataclass
class TrajectoryPoint:
    """
    Single point in a planned trajectory.

    Attributes:
        joint_names: Names of joints
        positions: Joint positions (radians)
        velocities: Joint velocities (optional)
        accelerations: Joint accelerations (optional)
        time: Time from start in seconds; set only where the waypoint carries a time
            parameterization (a waypoint without velocities has no time, not time zero)
    """

    joint_names: list[str]
    positions: np.ndarray
    velocities: np.ndarray | None = None
    accelerations: np.ndarray | None = None
    time: float | None = None

    def as_dict(self) -> dict[str, float]:
        """Return positions as {name: position} dictionary."""
        return dict(zip(self.joint_names, self.positions))

    def __repr__(self) -> str:
        pos_str = ", ".join(f"{p:.4f}" for p in self.positions)
        time_str = f", time={self.time:.3f}" if self.time is not None else ""
        return f"TrajectoryPoint([{pos_str}]{time_str})"


def _optional_array(values: np.ndarray) -> np.ndarray | None:
    """A state waypoint's optional vector: an empty vector is how the waypoint says "not set"."""
    return np.array(values) if len(values) > 0 else None


@dataclass
class PlanningResult:
    """
    Result from a planning operation.

    Attributes:
        successful: Whether planning succeeded
        message: Status or error message
        trajectory: List of trajectory points (if successful)
        raw_results: Raw CompositeInstruction output
        task_infos: The run's node infos (name, status_message, elapsed_time, ...), one per
            node that ran, in no particular order; empty if the pipeline never ran.
        data_storage: The run's data storage, including each nested pipeline's own storage
            under its uuid; None if the pipeline never ran.
    """

    successful: bool
    message: str = ""
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    raw_results: CompositeInstruction | None = None
    task_infos: list[TaskComposerNodeInfo] = field(default_factory=list)
    data_storage: TaskComposerDataStorage | None = None

    def __bool__(self) -> bool:
        """Allow `if result:` checks."""
        return self.successful

    def __len__(self) -> int:
        """Return number of waypoints in trajectory."""
        return len(self.trajectory)

    def __iter__(self) -> Iterator[TrajectoryPoint]:
        """Iterate over trajectory points."""
        return iter(self.trajectory)

    def __getitem__(self, idx: int) -> TrajectoryPoint:
        """Get trajectory point by index."""
        return self.trajectory[idx]

    def to_numpy(self) -> np.ndarray:
        """
        Convert trajectory to numpy array.

        Returns:
            Array of shape (N, num_joints) with joint positions; shape (0, 0) for an empty
            trajectory, which does not know its joint count.
        """
        if not self.trajectory:
            return np.empty((0, 0))
        return np.array([pt.positions for pt in self.trajectory])


# What a pipeline's ErrorTask, the pipeline node, and every composite a failure propagates
# through report: "Error (Abort Triggered)" after a node took its failure branch, "Aborted" after a
# node threw. The node that actually failed carries its own, specific message.
ABORT_PROPAGATION_MESSAGES: Final = frozenset({"Error (Abort Triggered)", "Aborted"})
# TaskComposerNodeInfo.status_code of a node that failed: 0 took its failure branch, -1 threw.
FAILED_STATUS_CODES: Final = frozenset({0, -1})


def _failure_message(
    infos: Iterable[TaskComposerNodeInfo], aborting: TaskComposerNodeInfo | None = None
) -> str:
    """Name the node that failed, not the ErrorTask a pipeline routes every failure into.

    When a node fails, the pipeline triggers an abort, and its ErrorTask, the pipeline itself and each
    composite on the way up all report "Error (Abort Triggered)" (or "Aborted" when the node threw). The informative message — a planner
    that ran out of iterations, a contact check that found a collision — is on the node that failed.
    Reporting the aborting node alone returned "ErrorTask: Error (Abort Triggered)" for a TrajOpt
    solve that had ended at OPT_SCO_ITERATION_LIMIT.

    Args:
        infos: The failed run's `TaskComposerNodeInfo` objects (`name`, `status_code`,
            `status_message`).
        aborting: The aborting node's info, `getInfo(getAbortingNode())` (`None` when no node
            aborted), reported when no node carries a specific message.

    Returns:
        `"<node>: <message>"` for every failed node with a specific message, sorted by node and
        joined with `"; "`; else the aborting node's message; else `"Planning failed"`.
    """
    specific = sorted(
        f"{info.name}: {info.status_message}"
        for info in infos
        if info.status_code in FAILED_STATUS_CODES
        and info.status_message
        and info.status_message not in ABORT_PROPAGATION_MESSAGES
    )
    if specific:
        return "; ".join(specific)
    if aborting is not None and aborting.status_message:
        return f"{aborting.name}: {aborting.status_message}"
    return "Planning failed"


# Pipelines the `plan_*` conveniences run when none is named.
DEFAULT_FREESPACE_PIPELINE: Final = "TrajOptPipeline"
DEFAULT_OMPL_PIPELINE: Final = "FreespacePipeline"
DEFAULT_CARTESIAN_PIPELINE: Final = "DescartesDPipeline"

# Pipelines `warmup()` loads when none are named: the ones the `plan_*` conveniences and the
# IFOPT variants run.
DEFAULT_WARMUP_PIPELINES: Final = (
    "TrajOptPipeline",
    "TrajOptIfoptPipeline",
    "FreespacePipeline",
    "FreespaceIfoptPipeline",
    "CartesianPipeline",
    "OMPLPipeline",
)

# Name of the executor plugin the factory builds when no thread count is given.
EXECUTOR_PLUGIN_NAME: Final = "TaskflowExecutor"


class TaskComposer:
    """
    High-level interface for task composer planning pipelines.

    Wraps the TaskComposerPluginFactory and handles all the AnyPoly
    boilerplate for setting up and running planning tasks.

    Available Pipelines (the full registry: `get_available_pipelines()`):
        FreespacePipeline: OMPL + TrajOpt smoothing + time param (recommended)
        CartesianPipeline: Cartesian path with TrajOpt
        OMPLPipeline: OMPL sampling-based planning only
        TrajOptPipeline: TrajOpt optimization only
        DescartesDPipeline: Descartes graph search, double precision
            (DescartesFPipeline in float; the *NPC variants skip the
            post-plan contact check)

    Error model: a configuration or programming error (unknown pipeline, profiles of the
    wrong type, a malformed program) raises; a planner that ran and found no solution returns
    a `PlanningResult` with `successful=False` and the failing node's message.

    Example:
        ```python
        composer = TaskComposer.from_config()

        # Plan with TrajOpt (optimization-based)
        result = composer.plan_freespace(robot, program)

        # Plan with OMPL + TrajOpt smoothing (sampling-based: RRT, RRT*, PRM, etc.)
        result = composer.plan_ompl(robot, program)

        # Plan Cartesian path
        result = composer.plan_cartesian(robot, program)

        # Or specify pipeline directly
        result = composer.plan(robot, program, pipeline="TrajOptPipeline")
        ```
    """

    def __init__(
        self,
        factory: TaskComposerPluginFactory,
        locator: GeneralResourceLocator | None = None,
        num_threads: int | None = None,
        executor: TaskComposerExecutor | None = None,
    ):
        """
        Initialize TaskComposer.

        Args:
            factory: Initialized TaskComposerPluginFactory
            locator: Resource locator
            num_threads: Number of threads for executor (overrides YAML config); not capped to
                the core count
            executor: Pre-configured executor (overrides num_threads and YAML)

        Raises:
            TypeError: If factory is not a TaskComposerPluginFactory
            ConflictingExecutorError: If both num_threads and executor are provided
            InvalidThreadCountError: If num_threads is below 1

        Note:
            Use TaskComposer.from_config() for simpler initialization.
            Do not pass a Robot or Environment - use the factory pattern.
        """
        if not isinstance(factory, TaskComposerPluginFactory):
            type_name = type(factory).__name__
            raise TypeError(
                f"Expected TaskComposerPluginFactory, got {type_name}. "
                f"Use TaskComposer.from_config() for easy initialization, or "
                f"create a factory with TaskComposerPluginFactory(config_path, locator)."
            )
        if num_threads is not None and executor is not None:
            raise ConflictingExecutorError("Cannot specify both num_threads and executor")
        if num_threads is not None and num_threads < 1:
            raise InvalidThreadCountError(f"num_threads must be at least 1, got {num_threads}")
        self.factory = factory
        self.locator = locator or GeneralResourceLocator()
        self._num_threads = num_threads
        self._executor = executor
        self._executor_lock = threading.Lock()

    @classmethod
    def from_config(
        cls,
        config_path: str | Path | None = None,
        locator: GeneralResourceLocator | None = None,
        num_threads: int | None = None,
        executor: TaskComposerExecutor | None = None,
        warmup: bool | list[str] = False,
    ) -> TaskComposer:
        """
        Create TaskComposer from config file.

        If no config_path is provided, uses get_task_composer_config_path():
        TESSERACT_TASK_COMPOSER_CONFIG_FILE when set, else the bundled config.

        Args:
            config_path: Path to task composer config YAML
            locator: Resource locator
            num_threads: Number of threads for executor (overrides YAML config)
            executor: Pre-configured executor (overrides num_threads and YAML)
            warmup: Create the pipelines' task nodes at startup, so the first plan does not.
                    True = load common pipelines, list = specific pipelines, False = skip.

        Returns:
            Initialized TaskComposer

        Raises:
            PipelineWarmupError: A pipeline to warm up does not load.

        Example:
            ```python
            # Use YAML config (default)
            composer = TaskComposer.from_config()

            # Pre-load plugins at init (recommended for interactive use)
            composer = TaskComposer.from_config(warmup=True)

            # Override thread count
            composer = TaskComposer.from_config(num_threads=4)

            # Use custom executor
            from tesseract_robotics.task_composer import TaskflowTaskComposerExecutor
            executor = TaskflowTaskComposerExecutor("MyExecutor", 8)
            composer = TaskComposer.from_config(executor=executor)
            ```
        """
        from tesseract_robotics import get_task_composer_config_path

        locator = locator or GeneralResourceLocator()
        if config_path is None:
            # TaskComposerConfigNotFoundError (a FileNotFoundError) names what it checked
            config_path = get_task_composer_config_path()

        # A Path: the factory reads a `str` as YAML text (#337).
        factory = TaskComposerPluginFactory(Path(config_path), locator)

        instance = cls(factory, locator, num_threads=num_threads, executor=executor)

        # Pre-load plugins if requested
        if warmup:
            pipelines = warmup if isinstance(warmup, list) else None
            loaded = instance.warmup(pipelines)
            logger.debug(f"Warmed up {len(loaded)} pipelines: {loaded}")

        return instance

    def warmup(self, pipelines: list[str] | None = None) -> list[str]:
        """
        Create the pipelines' task nodes ahead of the first plan.

        The TaskComposer loads its tasks as plugins (boost_plugin_loader / dlopen).
        Building the composer loads the factory (about 0.3 to 1.0 s on an M1 Max);
        the first plan through a pipeline is slower than later ones through the same
        composer.

        Call warmup() once at application startup to keep that off the first plan.

        Args:
            pipelines: Specific pipelines to load, or None for commonly used ones.
                       Pass [] (empty list) to skip loading (useful for tests).

        Returns:
            The pipeline names that were loaded (every name asked for).

        Raises:
            PipelineWarmupError: A pipeline does not load; the message names it and the
                factory's reason.

        Example:
            ```python
            # At application startup
            composer = TaskComposer.from_config()
            composer.warmup()  # creates the common pipelines' nodes

            # Later planning calls are fast
            result = composer.plan(robot, program)  # does not pay for node creation
            ```
        """
        if pipelines is None:
            pipelines = list(DEFAULT_WARMUP_PIPELINES)

        for name in pipelines:
            try:
                self.factory.createTaskComposerNode(name)
            except TaskComposerPluginError as e:
                raise PipelineWarmupError(f"Pipeline '{name}' failed to load: {e}") from e

        return list(pipelines)

    @property
    def executor(self) -> TaskComposerExecutor:
        """The task executor, created on first use (thread-safe)."""
        with self._executor_lock:
            if self._executor is None:
                if self._num_threads is not None:
                    self._executor = TaskflowTaskComposerExecutor(
                        EXECUTOR_PLUGIN_NAME, self._num_threads
                    )
                else:
                    # Use factory (respects YAML config)
                    self._executor = self.factory.createTaskComposerExecutor(EXECUTOR_PLUGIN_NAME)
            return self._executor

    def plan(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        pipeline: str = "TrajOptPipeline",
        profiles: ProfileDictionary | None = None,
        auto_seed: bool = True,
    ) -> PlanningResult:
        """
        Plan a motion program using specified pipeline.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction; never modified
            pipeline: Pipeline name (e.g., "TrajOptPipeline", "OMPLPipeline")
            profiles: Custom profiles; None uses the defaults of `pipeline`
            auto_seed: Seed Cartesian waypoints with current robot state (default: True), on
                a copy of the program. Set to False if you've already provided waypoint seeds.

        Returns:
            PlanningResult: with a trajectory if the planner succeeded, with `successful=False`
            and the failing node's message if it ran and found no solution.

        Raises:
            UnknownPipelineError: `pipeline` is not a configured task node of the factory.
            NoDefaultProfilesError: `profiles` is None and `pipeline` has no default profiles.
            PipelineLayoutError: `pipeline` lacks the `program` output or an input port.
            ProfileTypeMismatchError: a Descartes node of `pipeline` reads a scalar type that
                `profiles` holds only as the other type under some profile name.
            EmptyProgramError: `program` is a `MotionProgram` with no targets.
            TaskComposerPluginError: a plugin of `pipeline` fails to load.
        """
        plugins = self.factory.getTaskComposerNodePlugins()
        if pipeline not in plugins:
            raise UnknownPipelineError(
                f"Unknown pipeline '{pipeline}'; configured: {', '.join(sorted(plugins))}"
            )

        # Convert MotionProgram to CompositeInstruction if needed
        if isinstance(program, CompositeInstruction):
            composite = program
        else:
            joint_names = robot.get_joint_names(program.group_name)
            composite = program.to_composite_instruction(joint_names)

        if auto_seed:
            # assignCurrentStateAsSeed writes into the composite: seed a copy
            composite = copy.deepcopy(composite)
            assignCurrentStateAsSeed(composite, robot.env)

        if profiles is None:
            profiles = default_profiles_for(pipeline)

        task = self.factory.createTaskComposerNode(pipeline)

        # A Descartes node never finds a profile of the other scalar type: refuse before planning.
        check_descartes_profile_types(self.factory, pipeline, task, profiles)

        output_ports = task.getOutputPortMappings()
        if not output_ports.contains("program"):
            raise PipelineLayoutError(f"Pipeline '{pipeline}' has no 'program' output key")
        output_key = output_ports.single("program")
        # Different pipelines use different input keys
        input_ports = task.getInputPortMappings()
        if input_ports.contains("planning_input"):
            input_key = input_ports.single("planning_input")
        elif input_ports.contains("program"):
            input_key = input_ports.single("program")
        else:
            raise PipelineLayoutError(
                f"Pipeline '{pipeline}' has neither a 'planning_input' nor a 'program' input key"
            )

        # Wrap data for task composer
        program_anypoly = AnyPoly_wrap_CompositeInstruction(composite)
        environment_anypoly = AnyPoly_wrap_EnvironmentConst(robot.env)
        profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

        task_data = TaskComposerDataStorage()
        task_data.setData(input_key, program_anypoly)
        task_data.setData("environment", environment_anypoly)
        task_data.setData("profiles", profiles_anypoly)

        future = None
        try:
            future = self.executor.run(task, task_data)
            future.wait()

            task_infos = future.context.task_infos
            node_infos = list(task_infos.getInfoMap().values()) if task_infos is not None else []
            if not future.context.isSuccessful():
                message = (
                    _failure_message(node_infos, task_infos.getInfo(task_infos.getAbortingNode()))
                    if task_infos is not None
                    else "Planning failed"
                )
                return PlanningResult(
                    successful=False,
                    message=message,
                    task_infos=node_infos,
                    data_storage=future.context.data_storage,
                )

            # Extract results
            output_composite = AnyPoly_as_CompositeInstruction(
                future.context.data_storage.getData(output_key)
            )

            trajectory = self._extract_trajectory(output_composite)

            return PlanningResult(
                successful=True,
                message="Planning successful",
                trajectory=trajectory,
                raw_results=output_composite,
                task_infos=node_infos,
                data_storage=future.context.data_storage,
            )

        finally:
            # Cleanup to prevent memory issues
            del task_data
            del program_anypoly
            del environment_anypoly
            del profiles_anypoly
            if future is not None:
                del future

    def plan_freespace(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        pipeline: str | None = None,
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan freespace motion.

        Uses TrajOpt for collision-free point-to-point motion.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction
            pipeline: Pipeline name (default: `DEFAULT_FREESPACE_PIPELINE`, "TrajOptPipeline")
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(
            robot, program, pipeline=pipeline or DEFAULT_FREESPACE_PIPELINE, profiles=profiles
        )

    def plan_ompl(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        pipeline: str | None = None,
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan freespace motion with OMPL sampling-based planners.

        Uses OMPL (RRT, RRT*, PRM, etc.) for collision-free motion planning. Good for complex
        environments where optimization-based planners may get stuck in local minima.

        The default pipeline is `FreespacePipeline`: OMPL finds the path, TrajOpt smooths it, and
        time parameterization follows. A raw OMPL path with no smoothing step comes from
        `pipeline="OMPLPipeline"`. The module-level `plan_ompl` shares this default.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction
            pipeline: Pipeline name (default: `DEFAULT_OMPL_PIPELINE`, "FreespacePipeline")
            profiles: Custom profiles (use to configure specific OMPL planner)

        Returns:
            PlanningResult
        """
        return self.plan(
            robot, program, pipeline=pipeline or DEFAULT_OMPL_PIPELINE, profiles=profiles
        )

    def plan_cartesian(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        pipeline: str | None = None,
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan Cartesian motion.

        Uses Descartes for precise Cartesian path following.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction
            pipeline: Pipeline name (default: `DEFAULT_CARTESIAN_PIPELINE`, "DescartesDPipeline",
                the double-precision Descartes pipeline; "DescartesFPipeline" is the float
                one, and their "NPC" variants skip the post-plan contact check)
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(
            robot, program, pipeline=pipeline or DEFAULT_CARTESIAN_PIPELINE, profiles=profiles
        )

    def _extract_trajectory(self, composite: CompositeInstruction) -> list[TrajectoryPoint]:
        """Extract trajectory points from CompositeInstruction.

        Velocity and acceleration are set only where the planner (or a time parameterization
        task) wrote them: OMPL and TrajOpt output positions only; a pipeline that ends in time
        parameterization (FreespacePipeline, the Descartes pipelines) fills all three. A
        waypoint without velocities is not time-parameterized, so its `time` is None.
        """
        trajectory = []

        for instr in composite:
            if not instr.isMoveInstruction():
                continue

            wp = instr.asMoveInstruction().getWaypoint()

            if not wp.isStateWaypoint():
                continue

            state_wp = wp.asStateWaypoint()
            velocities = _optional_array(state_wp.getVelocity())
            point = TrajectoryPoint(
                joint_names=[jid.name() for jid in state_wp.getJointIds()],
                positions=np.array(state_wp.getPosition()),
                velocities=velocities,
                accelerations=_optional_array(state_wp.getAcceleration()),
                time=float(state_wp.getTime()) if velocities is not None else None,
            )

            trajectory.append(point)

        return trajectory

    def get_available_pipelines(self) -> list[str]:
        """
        Get the pipeline names this composer can run.

        Takes the configured node names from the factory
        (`TaskComposerPluginFactory.getTaskComposerNodePlugins`), so a config
        that uses `!include` is read as the C++ factory expanded it. Each name is
        then created once to check that its C++ plugin loads; names that raise
        `TaskComposerPluginError` are logged and left out (listing is introspection,
        so one broken plugin does not hide the rest; `warmup` raises instead).

        Returns:
            Sorted list of pipeline names that can be used with plan().
            Common pipelines include:
                - FreespacePipeline: OMPL + TrajOpt smoothing + time param
                - CartesianPipeline: Cartesian path with TrajOpt
                - OMPLPipeline: OMPL sampling-based planning only
                - TrajOptPipeline: TrajOpt optimization only
                - DescartesFPipeline: Descartes graph search
        """
        available = []
        failed = []
        for name in sorted(self.factory.getTaskComposerNodePlugins()):
            try:
                self.factory.createTaskComposerNode(name)
                available.append(name)
            except TaskComposerPluginError as e:
                failed.append((name, str(e)))

        if failed:
            logger.warning(
                f"Failed to load {len(failed)} plugins: "
                + ", ".join(f"{name}" for name, _ in failed)
            )
            for name, err in failed:
                logger.debug(f"  {name}: {err}")

        return available
