from typing import Annotated

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language


class DescartesSolverProfileD(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def getStaticKey() -> int: ...

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

class DescartesPlanProfileD(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def getStaticKey() -> int: ...

class DescartesDefaultPlanProfileD(DescartesPlanProfileD):
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

def cast_DescartesPlanProfileD(profile: DescartesDefaultPlanProfileD) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Cast DescartesDefaultPlanProfileD to Profile for use with ProfileDictionary
    """

class DescartesMotionPlannerD:
    def __init__(self, name: str) -> None: ...

    def getName(self) -> str: ...

    def solve(self, request: "tesseract_planning::PlannerRequest") -> "tesseract_planning::PlannerResponse": ...

    def terminate(self) -> bool: ...

    def clear(self) -> None: ...
