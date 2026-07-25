"""tesseract_motion_planners Python bindings"""

import enum
from typing import Annotated

import numpy
from numpy.typing import NDArray


class PlannerRequest:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def env(self) -> "tesseract::environment::Environment": ...

    @env.setter
    def env(self, arg: "tesseract::environment::Environment", /) -> None: ...

    @property
    def profiles(self) -> "tesseract::common::ProfileDictionary": ...

    @profiles.setter
    def profiles(self, arg: "tesseract::common::ProfileDictionary", /) -> None: ...

    @property
    def instructions(self) -> "tesseract::command_language::CompositeInstruction": ...

    @instructions.setter
    def instructions(self, arg: "tesseract::command_language::CompositeInstruction", /) -> None: ...

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
    def results(self) -> "tesseract::command_language::CompositeInstruction": ...

    @results.setter
    def results(self, arg: "tesseract::command_language::CompositeInstruction", /) -> None: ...

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

    def clone(self) -> MotionPlanner: ...

def assignCurrentStateAsSeed(composite_instructions: "tesseract::command_language::CompositeInstruction", env: "tesseract::environment::Environment") -> None:
    """
    Assign the current environment state as seed to all CartesianWaypoints in the program
    """

class RobotConfig(enum.Enum):
    NUT = 0

    FUT = 1

    NDT = 2

    FDT = 3

    NDB = 4

    FDB = 5

    NUB = 6

    FUB = 7

def getRobotConfig(joint_group: "tesseract::kinematics::JointGroup", base_link: str, tcp_frame: str, joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], sign_correction: Annotated[NDArray[numpy.int32], dict(shape=(2), order='C')] = ...) -> RobotConfig:
    """
    Classify the robot kinematic configuration (NUT/FUT/NDT/...) at a joint state. sign_correction fixes the sign of joints 3 and 5 per manufacturer (ABB IRB2400: [-1, 1]).
    """

def getJointTurns(joint_values: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
    """
    Per-joint turn count (joint_value / pi, truncated); non-zero flags a redundant solution.
    """
