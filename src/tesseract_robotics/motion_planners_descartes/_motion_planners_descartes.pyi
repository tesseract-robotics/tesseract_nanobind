"""tesseract_motion_planners_descartes Python bindings"""

from collections.abc import Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.collision
import tesseract_robotics.command_language
import tesseract_robotics.common
import tesseract_robotics.environment
import tesseract_robotics.kinematics
import tesseract_robotics.motion_planners


class ProfileReturnTypeError(TypeError):
    """
    A Python DescartesMoveProfile's create* returned an object that is not the EdgeEvaluator / WaypointSampler / StateEvaluator the planner calls.
    """

class DescartesStateD:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None: ...

    @property
    def values(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @values.setter
    def values(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

class DescartesStateSampleD:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, state: DescartesStateD, cost: float) -> None: ...

    @overload
    def __init__(self, state: DescartesStateD) -> None:
        """A sample with cost 0"""

    @property
    def state(self) -> DescartesStateD: ...

    @state.setter
    def state(self, arg: DescartesStateD, /) -> None: ...

    @property
    def cost(self) -> float: ...

    @cost.setter
    def cost(self, arg: float, /) -> None: ...

class DescartesEdgeEvaluatorD:
    def __init__(self) -> None: ...

    def evaluate(self, start: DescartesStateD, end: DescartesStateD) -> tuple[bool, float]:
        """Returns (is_valid, cost) for the edge between two states"""

class DescartesWaypointSamplerD:
    def __init__(self) -> None: ...

    def sample(self) -> list[DescartesStateSampleD]:
        """Return the list of valid state samples for this waypoint"""

    def __str__(self) -> str: ...

class DescartesStateEvaluatorD:
    def __init__(self) -> None: ...

    def evaluate(self, solution: DescartesStateD) -> tuple[bool, float]:
        """Returns (is_valid, cost) for a state"""

class DescartesSolverProfileD(tesseract_robotics.command_language.Profile):
    """
    Base of the Descartes solver profiles on `double` states, the type a DescartesDMotionPlannerTaskFactory node reads (DescartesDPipeline, DescartesDNPCPipeline).
    """

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every Descartes solver profile carries"""

class DescartesLadderGraphSolverProfileD(DescartesSolverProfileD):
    def __init__(self) -> None: ...

    @property
    def num_threads(self) -> int:
        """Number of threads to use during planning (default: 1)"""

    @num_threads.setter
    def num_threads(self, arg: int, /) -> None: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    __hash__: None = None

    @classmethod
    def __init_subclass__(cls: type[object], **kwargs: object) -> None: ...

class DescartesMoveProfileD(tesseract_robotics.command_language.Profile):
    """
    Base of the Descartes move profiles on `double` states, the type a DescartesDMotionPlannerTaskFactory node reads (DescartesDPipeline, DescartesDNPCPipeline).
    """

    def __init__(self) -> None: ...

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every Descartes move profile carries"""

    def createKinematicGroup(self, manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> tesseract_robotics.kinematics.KinematicGroup:
        """The kinematic group for `manip_info`'s manipulator and IK solver"""

    def createWaypointSampler(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesWaypointSamplerD:
        """The waypoint sampler the planner calls for `move_instruction`"""

    def createEdgeEvaluator(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesEdgeEvaluatorD:
        """The edge evaluator the planner calls into `move_instruction`"""

    def createStateEvaluator(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesStateEvaluatorD:
        """The state evaluator the planner calls at `move_instruction`"""

class DescartesDefaultMoveProfileD(DescartesMoveProfileD):
    def __init__(self) -> None: ...

    @property
    def target_pose_fixed(self) -> bool: ...

    @target_pose_fixed.setter
    def target_pose_fixed(self, arg: bool, /) -> None: ...

    @property
    def target_pose_sample_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @target_pose_sample_axis.setter
    def target_pose_sample_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def target_pose_sample_resolution(self) -> float: ...

    @target_pose_sample_resolution.setter
    def target_pose_sample_resolution(self, arg: float, /) -> None: ...

    @property
    def target_pose_sample_min(self) -> float: ...

    @target_pose_sample_min.setter
    def target_pose_sample_min(self, arg: float, /) -> None: ...

    @property
    def target_pose_sample_max(self) -> float: ...

    @target_pose_sample_max.setter
    def target_pose_sample_max(self, arg: float, /) -> None: ...

    @property
    def manipulator_ik_solver(self) -> str: ...

    @manipulator_ik_solver.setter
    def manipulator_ik_solver(self, arg: str, /) -> None: ...

    @property
    def allow_collision(self) -> bool: ...

    @allow_collision.setter
    def allow_collision(self, arg: bool, /) -> None: ...

    @property
    def enable_collision(self) -> bool: ...

    @enable_collision.setter
    def enable_collision(self, arg: bool, /) -> None: ...

    @property
    def vertex_contact_manager_config(self) -> tesseract_robotics.collision.ContactManagerConfig: ...

    @vertex_contact_manager_config.setter
    def vertex_contact_manager_config(self, arg: tesseract_robotics.collision.ContactManagerConfig, /) -> None: ...

    @property
    def vertex_collision_check_config(self) -> tesseract_robotics.collision.CollisionCheckConfig: ...

    @vertex_collision_check_config.setter
    def vertex_collision_check_config(self, arg: tesseract_robotics.collision.CollisionCheckConfig, /) -> None: ...

    @property
    def enable_edge_collision(self) -> bool: ...

    @enable_edge_collision.setter
    def enable_edge_collision(self, arg: bool, /) -> None: ...

    @property
    def edge_contact_manager_config(self) -> tesseract_robotics.collision.ContactManagerConfig: ...

    @edge_contact_manager_config.setter
    def edge_contact_manager_config(self, arg: tesseract_robotics.collision.ContactManagerConfig, /) -> None: ...

    @property
    def edge_collision_check_config(self) -> tesseract_robotics.collision.CollisionCheckConfig: ...

    @edge_collision_check_config.setter
    def edge_collision_check_config(self, arg: tesseract_robotics.collision.CollisionCheckConfig, /) -> None: ...

    @property
    def use_redundant_joint_solutions(self) -> bool: ...

    @use_redundant_joint_solutions.setter
    def use_redundant_joint_solutions(self, arg: bool, /) -> None: ...

    @property
    def debug(self) -> bool: ...

    @debug.setter
    def debug(self, arg: bool, /) -> None: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    __hash__: None = None

    @classmethod
    def __init_subclass__(cls: type[object], **kwargs: object) -> None: ...

class DescartesMotionPlannerD(tesseract_robotics.motion_planners.MotionPlanner):
    def __init__(self, name: str) -> None: ...

class DescartesStateF:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, values: Annotated[NDArray[numpy.float32], dict(shape=(None,), writable=False)] | Sequence[float]) -> None: ...

    @property
    def values(self) -> Annotated[NDArray[numpy.float32], dict(shape=(None,), order='C')]: ...

    @values.setter
    def values(self, arg: Annotated[NDArray[numpy.float32], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

class DescartesStateSampleF:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, state: DescartesStateF, cost: float) -> None: ...

    @overload
    def __init__(self, state: DescartesStateF) -> None:
        """A sample with cost 0"""

    @property
    def state(self) -> DescartesStateF: ...

    @state.setter
    def state(self, arg: DescartesStateF, /) -> None: ...

    @property
    def cost(self) -> float: ...

    @cost.setter
    def cost(self, arg: float, /) -> None: ...

class DescartesEdgeEvaluatorF:
    def __init__(self) -> None: ...

    def evaluate(self, start: DescartesStateF, end: DescartesStateF) -> tuple[bool, float]:
        """Returns (is_valid, cost) for the edge between two states"""

class DescartesWaypointSamplerF:
    def __init__(self) -> None: ...

    def sample(self) -> list[DescartesStateSampleF]:
        """Return the list of valid state samples for this waypoint"""

    def __str__(self) -> str: ...

class DescartesStateEvaluatorF:
    def __init__(self) -> None: ...

    def evaluate(self, solution: DescartesStateF) -> tuple[bool, float]:
        """Returns (is_valid, cost) for a state"""

class DescartesSolverProfileF(tesseract_robotics.command_language.Profile):
    """
    Base of the Descartes solver profiles on `float` states, the type a DescartesFMotionPlannerTaskFactory node reads (every shipped Cartesian and raster task).
    """

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every Descartes solver profile carries"""

class DescartesLadderGraphSolverProfileF(DescartesSolverProfileF):
    def __init__(self) -> None: ...

    @property
    def num_threads(self) -> int:
        """Number of threads to use during planning (default: 1)"""

    @num_threads.setter
    def num_threads(self, arg: int, /) -> None: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    __hash__: None = None

    @classmethod
    def __init_subclass__(cls: type[object], **kwargs: object) -> None: ...

class DescartesMoveProfileF(tesseract_robotics.command_language.Profile):
    """
    Base of the Descartes move profiles on `float` states, the type a DescartesFMotionPlannerTaskFactory node reads (every shipped Cartesian and raster task).
    """

    def __init__(self) -> None: ...

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every Descartes move profile carries"""

    def createKinematicGroup(self, manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> tesseract_robotics.kinematics.KinematicGroup:
        """The kinematic group for `manip_info`'s manipulator and IK solver"""

    def createWaypointSampler(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesWaypointSamplerF:
        """The waypoint sampler the planner calls for `move_instruction`"""

    def createEdgeEvaluator(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesEdgeEvaluatorF:
        """The edge evaluator the planner calls into `move_instruction`"""

    def createStateEvaluator(self, move_instruction: tesseract_robotics.command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.common.ManipulatorInfo, env: tesseract_robotics.environment.Environment) -> DescartesStateEvaluatorF:
        """The state evaluator the planner calls at `move_instruction`"""

class DescartesDefaultMoveProfileF(DescartesMoveProfileF):
    def __init__(self) -> None: ...

    @property
    def target_pose_fixed(self) -> bool: ...

    @target_pose_fixed.setter
    def target_pose_fixed(self, arg: bool, /) -> None: ...

    @property
    def target_pose_sample_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @target_pose_sample_axis.setter
    def target_pose_sample_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def target_pose_sample_resolution(self) -> float: ...

    @target_pose_sample_resolution.setter
    def target_pose_sample_resolution(self, arg: float, /) -> None: ...

    @property
    def target_pose_sample_min(self) -> float: ...

    @target_pose_sample_min.setter
    def target_pose_sample_min(self, arg: float, /) -> None: ...

    @property
    def target_pose_sample_max(self) -> float: ...

    @target_pose_sample_max.setter
    def target_pose_sample_max(self, arg: float, /) -> None: ...

    @property
    def manipulator_ik_solver(self) -> str: ...

    @manipulator_ik_solver.setter
    def manipulator_ik_solver(self, arg: str, /) -> None: ...

    @property
    def allow_collision(self) -> bool: ...

    @allow_collision.setter
    def allow_collision(self, arg: bool, /) -> None: ...

    @property
    def enable_collision(self) -> bool: ...

    @enable_collision.setter
    def enable_collision(self, arg: bool, /) -> None: ...

    @property
    def vertex_contact_manager_config(self) -> tesseract_robotics.collision.ContactManagerConfig: ...

    @vertex_contact_manager_config.setter
    def vertex_contact_manager_config(self, arg: tesseract_robotics.collision.ContactManagerConfig, /) -> None: ...

    @property
    def vertex_collision_check_config(self) -> tesseract_robotics.collision.CollisionCheckConfig: ...

    @vertex_collision_check_config.setter
    def vertex_collision_check_config(self, arg: tesseract_robotics.collision.CollisionCheckConfig, /) -> None: ...

    @property
    def enable_edge_collision(self) -> bool: ...

    @enable_edge_collision.setter
    def enable_edge_collision(self, arg: bool, /) -> None: ...

    @property
    def edge_contact_manager_config(self) -> tesseract_robotics.collision.ContactManagerConfig: ...

    @edge_contact_manager_config.setter
    def edge_contact_manager_config(self, arg: tesseract_robotics.collision.ContactManagerConfig, /) -> None: ...

    @property
    def edge_collision_check_config(self) -> tesseract_robotics.collision.CollisionCheckConfig: ...

    @edge_collision_check_config.setter
    def edge_collision_check_config(self, arg: tesseract_robotics.collision.CollisionCheckConfig, /) -> None: ...

    @property
    def use_redundant_joint_solutions(self) -> bool: ...

    @use_redundant_joint_solutions.setter
    def use_redundant_joint_solutions(self, arg: bool, /) -> None: ...

    @property
    def debug(self) -> bool: ...

    @debug.setter
    def debug(self, arg: bool, /) -> None: ...

    def __eq__(self, other: object, /) -> bool: ...

    def __ne__(self, other: object, /) -> bool: ...

    __hash__: None = None

    @classmethod
    def __init_subclass__(cls: type[object], **kwargs: object) -> None: ...

class DescartesMotionPlannerF(tesseract_robotics.motion_planners.MotionPlanner):
    def __init__(self, name: str) -> None: ...

def sampleToolAxis(tool_pose: tesseract_robotics.common.Isometry3d, axis: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], resolution: float, minimum: float, maximum: float) -> list[tesseract_robotics.common.Isometry3d]:
    """
    Given a tool pose create samples from [minimum, maximum] around the provided axis.
    """

def sampleToolXAxis(tool_pose: tesseract_robotics.common.Isometry3d, resolution: float, minimum: float, maximum: float) -> list[tesseract_robotics.common.Isometry3d]:
    """
    Given a tool pose create samples from [minimum, maximum] around the x axis.
    """

def sampleToolYAxis(tool_pose: tesseract_robotics.common.Isometry3d, resolution: float, minimum: float, maximum: float) -> list[tesseract_robotics.common.Isometry3d]:
    """
    Given a tool pose create samples from [minimum, maximum] around the y axis.
    """

def sampleToolZAxis(tool_pose: tesseract_robotics.common.Isometry3d, resolution: float, minimum: float, maximum: float) -> list[tesseract_robotics.common.Isometry3d]:
    """
    Given a tool pose create samples from [minimum, maximum] around the z axis.
    """

def sampleFixed(tool_pose: tesseract_robotics.common.Isometry3d) -> list[tesseract_robotics.common.Isometry3d]:
    """This will return the tool pose as the only sample."""
