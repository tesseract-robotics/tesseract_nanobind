"""tesseract_motion_planners Python bindings"""

from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_kinematics._tesseract_kinematics
import tesseract_robotics.tesseract_state_solver._tesseract_state_solver


class PlannerRequest:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def env(self) -> tesseract_robotics.tesseract_environment._tesseract_environment.Environment: ...

    @env.setter
    def env(self, arg: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, /) -> None: ...

    @property
    def profiles(self) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary: ...

    @profiles.setter
    def profiles(self, arg: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, /) -> None: ...

    @property
    def instructions(self) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction: ...

    @instructions.setter
    def instructions(self, arg: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, /) -> None: ...

    @property
    def verbose(self) -> bool: ...

    @verbose.setter
    def verbose(self, arg: bool, /) -> None: ...

    @property
    def format_result_as_input(self) -> bool: ...

    @format_result_as_input.setter
    def format_result_as_input(self, arg: bool, /) -> None: ...

class PlannerResponse:
    def __init__(self) -> None: ...

    @property
    def results(self) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction: ...

    @results.setter
    def results(self, arg: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, /) -> None: ...

    @property
    def successful(self) -> bool: ...

    @successful.setter
    def successful(self, arg: bool, /) -> None: ...

    @property
    def message(self) -> str: ...

    @message.setter
    def message(self, arg: str, /) -> None: ...

    def __bool__(self) -> bool: ...

class MotionPlanner:
    def getName(self) -> str: ...

    def solve(self, request: PlannerRequest) -> PlannerResponse: ...

    def terminate(self) -> bool: ...

    def clear(self) -> None: ...

    @staticmethod
    def checkRequest(request: PlannerRequest) -> bool: ...

    @staticmethod
    def checkRequestReason(request: PlannerRequest) -> str:
        """
        Why `checkRequest(request)` fails: the C++ reason, or an empty string for a valid request.
        """

    @staticmethod
    def assignSolution(mi: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, joint_names: Sequence[str], joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], format_result_as_input: bool) -> None:
        """
        Write a joint solution into `mi` in place: a state waypoint, or with `format_result_as_input` the seed of a Cartesian / the position of a joint waypoint.
        """

    def clone(self) -> MotionPlanner: ...

def assignCurrentStateAsSeed(composite_instructions: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> None:
    """
    Assign the current environment state as seed to all CartesianWaypoints in the program
    """

def formatProgram(composite_instructions: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> bool:
    """
    Reorder joint and state waypoints in place to the kinematic group's joint order. Returns True if the program needed formatting.
    """

@overload
def toToolpath(ci: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> list[tesseract_robotics.tesseract_common._tesseract_common.VectorIsometry3d]:
    """
    World-frame TCP poses of a program, one VectorIsometry3d per sub-composite.
    """

@overload
def toToolpath(mi: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> list[tesseract_robotics.tesseract_common._tesseract_common.VectorIsometry3d]:
    """World-frame TCP pose of a move instruction."""

@overload
def toToolpath(instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.InstructionPoly, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> list[tesseract_robotics.tesseract_common._tesseract_common.VectorIsometry3d]:
    """World-frame TCP poses of a composite or move instruction."""

@overload
def contactCheckProgram(manager: tesseract_robotics.tesseract_collision._tesseract_collision.ContinuousContactManager, state_solver: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.StateSolver, program: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, config: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig) -> tuple[tesseract_robotics.tesseract_collision._tesseract_collision.ContactTrajectoryResults, list[tesseract_robotics.tesseract_collision._tesseract_collision.ContactResultMap]]:
    """
    Continuous collision check over a program. Returns (ContactTrajectoryResults, list[ContactResultMap]), one map per step.
    """

@overload
def contactCheckProgram(manager: tesseract_robotics.tesseract_collision._tesseract_collision.DiscreteContactManager, state_solver: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.StateSolver, program: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, config: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig) -> tuple[tesseract_robotics.tesseract_collision._tesseract_collision.ContactTrajectoryResults, list[tesseract_robotics.tesseract_collision._tesseract_collision.ContactResultMap]]:
    """
    Discrete collision check over a program. Returns (ContactTrajectoryResults, list[ContactResultMap]), one map per step.
    """

class RobotConfig(enum.Enum):
    """
    Six-axis arm configuration: wrist Flip/No-flip, elbow Up/Down, arm fronT/Back.
    """

    NUT = 0

    FUT = 1

    NDT = 2

    FDT = 3

    NDB = 4

    FDB = 5

    NUB = 6

    FUB = 7

def getRobotConfig(joint_group: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, base_link: str, tcp_frame: str, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], sign_correction: Annotated[NDArray[numpy.int32], dict(shape=(2), order='C')] | Sequence[int] = [1, 1]) -> RobotConfig:
    """
    Configuration of a six-axis arm. The last six joint values are the arm's; `sign_correction` flips the sign of joint 3 and joint 5 for the manufacturer's convention.
    """

def getJointTurns(joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
    """Whole half-turns per joint, `int(q / pi)`: non-zero beyond +-pi."""

def isWithinJointLimits(joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], coupling_limits: Annotated[NDArray[numpy.float64], dict(shape=(2), order='C')] | Sequence[float]) -> bool:
    """
    True if `joint_values` are within `limits` and joint 4 + joint 6 (third and first from the end) are within `coupling_limits`.
    """
