"""
Task Composer helpers for high-level planning pipelines.

Wraps the TaskComposer system with a simpler interface that handles
all the AnyPoly wrapping and boilerplate.

Example:
    from tesseract_robotics.planning import Robot, MotionProgram, TaskComposer

    robot = Robot.from_tesseract_support("abb_irb2400")
    program = MotionProgram("manipulator").move_to(...)

    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    if result.successful:
        for waypoint in result.trajectory:
            print(waypoint)
"""

from __future__ import annotations

import os
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from loguru import logger

if TYPE_CHECKING:
    from tesseract_robotics.planning.core import Robot
    from tesseract_robotics.planning.program import MotionProgram

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    ProfileDictionary,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
)
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerExecutor,
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
        time: Time from start (optional)
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


def _extract_array_field(getter, field_name: str) -> np.ndarray | None:
    """Extract optional array field (velocity/acceleration) from StateWaypoint."""
    try:
        val = getter()
        if val is not None and len(val) > 0:
            return np.array(val)
    except (AttributeError, RuntimeError) as e:
        logger.debug(f"{field_name} not available: {type(e).__name__}")
    return None


def _extract_time_field(getter, field_name: str) -> float | None:
    """Extract optional time field from StateWaypoint."""
    try:
        val = getter()
        if val is not None:
            return float(val)
    except (AttributeError, RuntimeError) as e:
        logger.debug(f"{field_name} not available: {type(e).__name__}")
    return None


@dataclass
class PlanningResult:
    """
    Result from a planning operation.

    Attributes:
        successful: Whether planning succeeded
        message: Status or error message
        trajectory: List of trajectory points (if successful)
        raw_results: Raw CompositeInstruction output
    """

    successful: bool
    message: str = ""
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    raw_results: CompositeInstruction | None = None

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
            Array of shape (N, num_joints) with joint positions
        """
        if not self.trajectory:
            return np.array([])
        return np.array([pt.positions for pt in self.trajectory])


class TaskComposer:
    """
    High-level interface for task composer planning pipelines.

    Wraps the TaskComposerPluginFactory and handles all the AnyPoly
    boilerplate for setting up and running planning tasks.

    Example:
        composer = TaskComposer.from_config()

        # Plan with TrajOpt (optimization-based)
        result = composer.plan_freespace(robot, program)

        # Plan with OMPL (sampling-based: RRT, RRT*, PRM, etc.)
        result = composer.plan_ompl(robot, program)

        # Plan Cartesian path
        result = composer.plan_cartesian(robot, program)

        # Or specify pipeline directly
        result = composer.plan(robot, program, pipeline="TrajOptPipeline")
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
            num_threads: Number of threads for executor (overrides YAML config)
            executor: Pre-configured executor (overrides num_threads and YAML)

        Raises:
            TypeError: If factory is not a TaskComposerPluginFactory
            ValueError: If both num_threads and executor are provided

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
            raise ValueError("Cannot specify both num_threads and executor")
        self.factory = factory
        self.locator = locator or GeneralResourceLocator()
        self._num_threads = num_threads
        self._executor = executor

    @classmethod
    def from_config(
        cls,
        config_path: str | Path | None = None,
        locator: GeneralResourceLocator | None = None,
        num_threads: int | None = None,
        executor: TaskComposerExecutor | None = None,
    ) -> TaskComposer:
        """
        Create TaskComposer from config file.

        If no config_path is provided, uses TESSERACT_TASK_COMPOSER_CONFIG_FILE
        environment variable or falls back to the default config.

        Args:
            config_path: Path to task composer config YAML
            locator: Resource locator
            num_threads: Number of threads for executor (overrides YAML config)
            executor: Pre-configured executor (overrides num_threads and YAML)

        Returns:
            Initialized TaskComposer

        Example:
            # Use YAML config (default)
            composer = TaskComposer.from_config()

            # Override thread count
            composer = TaskComposer.from_config(num_threads=4)

            # Use custom executor
            from tesseract_robotics.tesseract_task_composer import TaskflowTaskComposerExecutor
            executor = TaskflowTaskComposerExecutor("MyExecutor", 8)
            composer = TaskComposer.from_config(executor=executor)
        """
        locator = locator or GeneralResourceLocator()
        tried_paths: list[str] = []

        if config_path is None:
            # Try environment variable first
            env_path = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
            if env_path:
                tried_paths.append(f"TESSERACT_TASK_COMPOSER_CONFIG_FILE={env_path}")
                if Path(env_path).is_file():
                    config_path = env_path

            if config_path is None:
                # Fall back to TESSERACT_TASK_COMPOSER_DIR
                composer_dir = os.environ.get("TESSERACT_TASK_COMPOSER_DIR")
                if composer_dir:
                    dir_path = Path(composer_dir) / "config/task_composer_plugins.yaml"
                    tried_paths.append(f"TESSERACT_TASK_COMPOSER_DIR: {dir_path}")
                    if dir_path.is_file():
                        config_path = str(dir_path)

        if config_path is None:
            # Last resort: use bundled config
            from tesseract_robotics import get_task_composer_config_path

            bundled = get_task_composer_config_path()
            if bundled:
                tried_paths.append(f"bundled config: {bundled}")
                if bundled.is_file():
                    config_path = str(bundled)

        if config_path is None:
            paths_tried = "\n  - ".join(tried_paths) if tried_paths else "(none)"
            raise ValueError(
                f"No valid task composer config found.\n"
                f"Tried:\n  - {paths_tried}\n"
                f"Set TESSERACT_TASK_COMPOSER_CONFIG_FILE or install with bundled data."
            )

        factory = TaskComposerPluginFactory(
            FilesystemPath(str(config_path)),
            locator,
        )

        return cls(factory, locator, num_threads=num_threads, executor=executor)

    @property
    def executor(self) -> TaskflowTaskComposerExecutor:
        """Get or create task executor (lazy initialization)."""
        if self._executor is None:
            if self._num_threads is not None:
                # Create executor with explicit thread count (capped to available cores)
                max_threads = os.cpu_count() or 1
                threads = min(self._num_threads, max_threads)
                self._executor = TaskflowTaskComposerExecutor("TaskflowExecutor", threads)
            else:
                # Use factory (respects YAML config)
                self._executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")
        return self._executor

    def plan(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        pipeline: str = "TrajOptPipeline",
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan a motion program using specified pipeline.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction
            pipeline: Pipeline name (e.g., "TrajOptPipeline", "OMPLPipeline")
            profiles: Custom profiles (uses defaults if None)

        Returns:
            PlanningResult with trajectory if successful
        """
        # Convert MotionProgram to CompositeInstruction if needed
        if hasattr(program, "to_composite_instruction"):
            joint_names = robot.get_joint_names(program.group_name)
            tcp_frame = program.tcp_frame
            if tcp_frame is None:
                manip_info = robot.get_manipulator_info(program.group_name)
                tcp_frame = manip_info.tcp_frame
            composite = program.to_composite_instruction(joint_names, tcp_frame)
        else:
            composite = program

        # Auto-seed Cartesian waypoints with current robot state
        assignCurrentStateAsSeed(composite, robot.env)

        # Setup profiles - create pipeline-specific defaults if not provided
        # Each pipeline may use multiple planners (e.g., OMPL + TrajOpt)
        if profiles is None:
            if pipeline == "FreespacePipeline" or "Freespace" in pipeline:
                # FreespacePipeline: OMPL (global planning) + TrajOpt (smoothing)
                from .profiles import create_freespace_pipeline_profiles

                profiles = create_freespace_pipeline_profiles()
            elif pipeline == "CartesianPipeline" or "Cartesian" in pipeline:
                # CartesianPipeline: Descartes (sampling) + TrajOpt (optimization)
                from .profiles import create_cartesian_pipeline_profiles

                profiles = create_cartesian_pipeline_profiles()
            elif "TrajOpt" in pipeline:
                # TrajOptPipeline: TrajOpt only
                from .profiles import create_trajopt_default_profiles

                profiles = create_trajopt_default_profiles()
            elif "OMPL" in pipeline:
                from .profiles import create_ompl_default_profiles

                profiles = create_ompl_default_profiles()
            elif "Descartes" in pipeline:
                from .profiles import create_descartes_default_profiles

                profiles = create_descartes_default_profiles()
            else:
                profiles = ProfileDictionary()

        # Create task
        task = self.factory.createTaskComposerNode(pipeline)
        if task is None:
            return PlanningResult(
                successful=False,
                message=f"Pipeline '{pipeline}' not found",
            )

        output_key = task.getOutputKeys().get("program")
        # Different pipelines use different input keys
        input_keys = task.getInputKeys()
        if input_keys.has("planning_input"):
            input_key = input_keys.get("planning_input")
        elif input_keys.has("program"):
            input_key = input_keys.get("program")
        else:
            return PlanningResult(
                successful=False,
                message=f"Pipeline '{pipeline}' has unknown input key configuration",
            )

        # Wrap data for task composer
        program_anypoly = AnyPoly_wrap_CompositeInstruction(composite)
        environment_anypoly = AnyPoly_wrap_EnvironmentConst(robot.env)
        profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

        task_data = TaskComposerDataStorage()
        task_data.setData(input_key, program_anypoly)
        task_data.setData("environment", environment_anypoly)
        task_data.setData("profiles", profiles_anypoly)

        # Execute
        future = None
        try:
            future = self.executor.run(task, task_data)
            future.wait()

            if not future.context.isSuccessful():
                return PlanningResult(
                    successful=False,
                    message="Planning failed",
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
            )

        except (TypeError, RuntimeError, ValueError, AttributeError, KeyError) as e:
            logger.debug(f"Planning failed with {type(e).__name__}: {e}")
            return PlanningResult(
                successful=False,
                message=f"{type(e).__name__}: {e}",
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
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan freespace motion using TrajOpt pipeline.

        Convenience method that uses TrajOptPipeline for optimization-based
        collision-free motion planning. For sampling-based planning, use plan_ompl().

        Args:
            robot: Robot instance
            program: Motion program
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(robot, program, pipeline="TrajOptPipeline", profiles=profiles)

    def plan_ompl(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan freespace motion using OMPL pipeline.

        Uses sampling-based planners (RRT, RRT*, PRM, etc.) for collision-free
        motion planning. Good for complex environments where optimization-based
        planners may get stuck in local minima.

        Args:
            robot: Robot instance
            program: Motion program
            profiles: Custom profiles (use to configure specific OMPL planner)

        Returns:
            PlanningResult
        """
        return self.plan(robot, program, pipeline="OMPLPipeline", profiles=profiles)

    def plan_cartesian(
        self,
        robot: Robot,
        program: MotionProgram | CompositeInstruction,
        profiles: ProfileDictionary | None = None,
    ) -> PlanningResult:
        """
        Plan Cartesian motion using Descartes pipeline.

        For motions requiring precise Cartesian paths.

        Args:
            robot: Robot instance
            program: Motion program
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(robot, program, pipeline="DescartesPipeline", profiles=profiles)

    def _extract_trajectory(self, composite: CompositeInstruction) -> list[TrajectoryPoint]:
        """Extract trajectory points from CompositeInstruction."""
        trajectory = []

        for instr in composite:
            if not instr.isMoveInstruction():
                continue

            move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            wp = move_instr.getWaypoint()

            if not wp.isStateWaypoint():
                continue

            state_wp = WaypointPoly_as_StateWaypointPoly(wp)

            # Extract optional velocity/acceleration/time if available
            #
            # WHY THESE MAY BE MISSING:
            # - OMPL planners output only positions (no velocity/acceleration)
            # - Time parameterization adds velocity/acceleration/time
            # - TrajOpt may or may not include dynamics depending on profile
            # - Simple interpolation typically lacks dynamics data
            #
            # PIPELINE IMPLICATIONS:
            # - If you need velocities: ensure pipeline includes time parameterization
            # - FreespacePipeline: OMPL → interpolation → TOTG (adds vel/acc/time)
            # - TrajOptPipeline: may include velocity costs but not always populated
            #
            # To enable velocity/acceleration output, use a pipeline with
            # TimeOptimalTrajectoryGeneration (TOTG) or IterativeSplineParameterization
            point = TrajectoryPoint(
                joint_names=list(state_wp.getNames()),
                positions=np.array(state_wp.getPosition()),
                velocities=_extract_array_field(state_wp.getVelocity, "Velocity"),
                accelerations=_extract_array_field(state_wp.getAcceleration, "Acceleration"),
                time=_extract_time_field(state_wp.getTime, "Time"),
            )

            trajectory.append(point)

        return trajectory

    def get_available_pipelines(self) -> list[str]:
        """
        Get list of available pipeline names.

        Returns:
            List of pipeline names that can be used with plan()
        """
        # Common pipelines - actual availability depends on config
        return [
            "TrajOptPipeline",
            "OMPLPipeline",
            "DescartesPipeline",
            "FreespaceMotionPipeline",
            "CartesianMotionPipeline",
        ]
