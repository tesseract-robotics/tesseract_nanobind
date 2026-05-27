"""tesseract_motion_planners_descartes Python bindings"""

from typing import Annotated, TypeAlias, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners


class DescartesStateD:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> None: ...

    @property
    def values(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @values.setter
    def values(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class DescartesStateSampleD:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, state: DescartesStateD, cost: float) -> None: ...

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

class DescartesStateEvaluatorD:
    def __init__(self) -> None: ...

    def evaluate(self, solution: DescartesStateD) -> tuple[bool, float]:
        """Returns (is_valid, cost) for a state"""

class DescartesSolverProfileD(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

class DescartesLadderGraphSolverProfileD(DescartesSolverProfileD):
    def __init__(self) -> None: ...

    @property
    def num_threads(self) -> int:
        """Number of threads to use during planning (default: 1)"""

    @num_threads.setter
    def num_threads(self, arg: int, /) -> None: ...

def cast_DescartesSolverProfileD(profile: DescartesLadderGraphSolverProfileD) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Cast DescartesLadderGraphSolverProfileD to Profile for use with ProfileDictionary
    """

class DescartesMoveProfileD(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self) -> None: ...

    def getKey(self) -> int: ...

    def createWaypointSampler(self, move_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, env: "tesseract_environment::Environment") -> DescartesWaypointSamplerD: ...

    def createEdgeEvaluator(self, move_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, env: "tesseract_environment::Environment") -> DescartesEdgeEvaluatorD: ...

    def createStateEvaluator(self, move_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, env: "tesseract_environment::Environment") -> DescartesStateEvaluatorD: ...

def cast_DescartesMoveProfileD(profile: DescartesMoveProfileD) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Cast a DescartesMoveProfileD (including Python subclasses) to Profile for use with ProfileDictionary
    """

DescartesPlanProfileD: TypeAlias = DescartesMoveProfileD

class DescartesDefaultMoveProfileD(DescartesMoveProfileD):
    def __init__(self) -> None: ...

    @property
    def target_pose_fixed(self) -> bool: ...

    @target_pose_fixed.setter
    def target_pose_fixed(self, arg: bool, /) -> None: ...

    @property
    def target_pose_sample_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @target_pose_sample_axis.setter
    def target_pose_sample_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

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
    def vertex_collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig: ...

    @vertex_collision_check_config.setter
    def vertex_collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

    @property
    def enable_edge_collision(self) -> bool: ...

    @enable_edge_collision.setter
    def enable_edge_collision(self, arg: bool, /) -> None: ...

    @property
    def edge_collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig: ...

    @edge_collision_check_config.setter
    def edge_collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

    @property
    def use_redundant_joint_solutions(self) -> bool: ...

    @use_redundant_joint_solutions.setter
    def use_redundant_joint_solutions(self, arg: bool, /) -> None: ...

    @property
    def debug(self) -> bool: ...

    @debug.setter
    def debug(self, arg: bool, /) -> None: ...

DescartesDefaultPlanProfileD: TypeAlias = DescartesDefaultMoveProfileD

def cast_DescartesPlanProfileD(profile: DescartesMoveProfileD) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Cast a DescartesMoveProfileD to Profile (legacy alias of cast_DescartesMoveProfileD)
    """

class DescartesMotionPlannerD:
    def __init__(self, name: str) -> None: ...

    def getName(self) -> str: ...

    def solve(self, request: tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerRequest) -> tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerResponse: ...

    def terminate(self) -> bool: ...

    def clear(self) -> None: ...

    def clone(self) -> tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner: ...
