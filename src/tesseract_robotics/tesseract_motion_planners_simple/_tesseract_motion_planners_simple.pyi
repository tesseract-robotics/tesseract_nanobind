"""tesseract_motion_planners_simple Python bindings"""

from collections.abc import Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_kinematics._tesseract_kinematics
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners
import tesseract_robotics.tesseract_state_solver._tesseract_state_solver


class SimpleMotionPlanner(tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner):
    def __init__(self, name: str = 'SIMPLE') -> None: ...

    def solve(self, request: tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerRequest) -> tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerResponse:
        """
        Plan `request`. An exception a Python profile's generate raises propagates from here.
        """

def generateInterpolatedProgram(instructions: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, state_longest_valid_segment_length: float = 0.08726646259971647, translation_longest_valid_segment_length: float = 0.15, rotation_longest_valid_segment_length: float = 0.08726646259971647, min_steps: int = 1) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction:
    """Generate an interpolated program from a composite instruction"""

@overload
def interpolate(start: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, stop: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, steps: int) -> tesseract_robotics.tesseract_common._tesseract_common.VectorIsometry3d:
    """
    Poses from `start` to `stop`: `steps + 1` of them, translation lerped, rotation slerped.
    """

@overload
def interpolate(start: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], stop: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], steps: int) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
    """
    Joint states from `start` to `stop`, one per column: `steps + 1` columns.
    """

def interpolate_waypoint(start: tesseract_robotics.tesseract_command_language._tesseract_command_language.WaypointPoly, stop: tesseract_robotics.tesseract_command_language._tesseract_command_language.WaypointPoly, steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.WaypointPoly]:
    """
    Waypoints from `start` to `stop`: `steps + 1` of them, of the start's type.
    """

@overload
def getInterpolatedInstructions(joint_names: Sequence[str], states: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')] | Sequence[Sequence[float]], base_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]:
    """
    Move instructions for the columns of `states` after the first, each a child of `base_instruction`; the last is `base_instruction` itself.
    """

@overload
def getInterpolatedInstructions(poses: tesseract_robotics.tesseract_common._tesseract_common.VectorIsometry3d, joint_names: Sequence[str], states: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')] | Sequence[Sequence[float]], base_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]:
    """
    Cartesian move instructions for the columns of `states` after the first, seeded with those states; the last is `base_instruction` itself.
    """

class JointGroupInstructionInfo:
    """
    A move instruction resolved against a joint group (no inverse kinematics)
    """

    def __init__(self, plan_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo) -> None: ...

    @property
    def instruction(self) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly:
        """The move instruction this info describes (not a copy)"""

    @property
    def manip(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup: ...

    @property
    def working_frame(self) -> str: ...

    @working_frame.setter
    def working_frame(self, arg: str, /) -> None: ...

    @property
    def working_frame_transform(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d: ...

    @working_frame_transform.setter
    def working_frame_transform(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def tcp_frame(self) -> str: ...

    @tcp_frame.setter
    def tcp_frame(self, arg: str, /) -> None: ...

    @property
    def tcp_offset(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d: ...

    @tcp_offset.setter
    def tcp_offset(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def has_cartesian_waypoint(self) -> bool: ...

    @has_cartesian_waypoint.setter
    def has_cartesian_waypoint(self, arg: bool, /) -> None: ...

    def calcCartesianPose(self, jp: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], in_world: bool = True) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """The tool pose at joint state `jp`, in world or in the working frame."""

    def extractCartesianPose(self, in_world: bool = True) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """The Cartesian waypoint's pose, in world or in the working frame."""

    def extractJointPosition(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The joint or state waypoint's position."""

class KinematicGroupInstructionInfo:
    """
    A move instruction resolved against a kinematic group (with inverse kinematics)
    """

    def __init__(self, plan_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo) -> None: ...

    @property
    def instruction(self) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly:
        """The move instruction this info describes (not a copy)"""

    @property
    def manip(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.KinematicGroup: ...

    @property
    def working_frame(self) -> str: ...

    @working_frame.setter
    def working_frame(self, arg: str, /) -> None: ...

    @property
    def working_frame_transform(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d: ...

    @working_frame_transform.setter
    def working_frame_transform(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def tcp_frame(self) -> str: ...

    @tcp_frame.setter
    def tcp_frame(self, arg: str, /) -> None: ...

    @property
    def tcp_offset(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d: ...

    @tcp_offset.setter
    def tcp_offset(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def has_cartesian_waypoint(self) -> bool: ...

    @has_cartesian_waypoint.setter
    def has_cartesian_waypoint(self, arg: bool, /) -> None: ...

    def calcCartesianPose(self, jp: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], in_world: bool = True) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """The tool pose at joint state `jp`, in world or in the working frame."""

    def extractCartesianPose(self, in_world: bool = True) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """The Cartesian waypoint's pose, in world or in the working frame."""

    def extractJointPosition(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The joint or state waypoint's position."""

@overload
def interpolateJointJointWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, linear_steps: int, freespace_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateJointJointWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateJointJointWaypoint(prev: JointGroupInstructionInfo, base: JointGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateJointCartWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, linear_steps: int, freespace_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateJointCartWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateJointCartWaypoint(prev: JointGroupInstructionInfo, base: JointGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartJointWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, linear_steps: int, freespace_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartJointWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartJointWaypoint(prev: JointGroupInstructionInfo, base: JointGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartCartWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, linear_steps: int, freespace_steps: int, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartCartWaypoint(prev: KinematicGroupInstructionInfo, base: KinematicGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def interpolateCartCartWaypoint(prev: JointGroupInstructionInfo, base: JointGroupInstructionInfo, state_lvs_length: float, translation_lvs_length: float, rotation_lvs_length: float, min_steps: int, max_steps: int, scene_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]: ...

@overload
def getClosestJointSolution(info: KinematicGroupInstructionInfo, seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    The inverse-kinematics solution for `info`'s Cartesian waypoint closest to `seed`.
    """

@overload
def getClosestJointSolution(info1: KinematicGroupInstructionInfo, info2: KinematicGroupInstructionInfo, seed: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
    """
    The pair of inverse-kinematics solutions, one per Cartesian waypoint, closest to each other, as `(solution1, solution2)`.
    """

class SimplePlannerMoveProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every simple-planner move profile registers under."""

    def generate(self, prev_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, prev_seed: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, base_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, next_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.InstructionPoly, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, global_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo) -> list[tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly]:
        """
        The move instructions the simple planner emits for `base_instruction`, ending with it. `next_instruction` is a null InstructionPoly for the last move.
        """

class SimplePlannerCompositeProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key this profile type registers under in a ProfileDictionary."""

class SimplePlannerFixedSizeMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, freespace_steps: int = 10, linear_steps: int = 10) -> None: ...

    @property
    def freespace_steps(self) -> int: ...

    @freespace_steps.setter
    def freespace_steps(self, arg: int, /) -> None: ...

    @property
    def linear_steps(self) -> int: ...

    @linear_steps.setter
    def linear_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerFixedSizeMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerFixedSizeMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerFixedSizeMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerFixedSizeAssignMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, freespace_steps: int = 10, linear_steps: int = 10) -> None: ...

    @property
    def freespace_steps(self) -> int: ...

    @freespace_steps.setter
    def freespace_steps(self, arg: int, /) -> None: ...

    @property
    def linear_steps(self) -> int: ...

    @linear_steps.setter
    def linear_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerFixedSizeAssignMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerFixedSizeAssignMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerFixedSizeAssignMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerFixedSizeAssignNoIKMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, freespace_steps: int = 10, linear_steps: int = 10) -> None: ...

    @property
    def freespace_steps(self) -> int: ...

    @freespace_steps.setter
    def freespace_steps(self, arg: int, /) -> None: ...

    @property
    def linear_steps(self) -> int: ...

    @linear_steps.setter
    def linear_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerFixedSizeAssignNoIKMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerFixedSizeAssignNoIKMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerFixedSizeAssignNoIKMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerLVSMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, state_longest_valid_segment_length: float = 0.08726646259971647, translation_longest_valid_segment_length: float = 0.1, rotation_longest_valid_segment_length: float = 0.08726646259971647, min_steps: int = 1, max_steps: int = 2147483647) -> None: ...

    @property
    def state_longest_valid_segment_length(self) -> float: ...

    @state_longest_valid_segment_length.setter
    def state_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def translation_longest_valid_segment_length(self) -> float: ...

    @translation_longest_valid_segment_length.setter
    def translation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def rotation_longest_valid_segment_length(self) -> float: ...

    @rotation_longest_valid_segment_length.setter
    def rotation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def min_steps(self) -> int: ...

    @min_steps.setter
    def min_steps(self, arg: int, /) -> None: ...

    @property
    def max_steps(self) -> int: ...

    @max_steps.setter
    def max_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerLVSMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerLVSMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerLVSMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerLVSNoIKMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, state_longest_valid_segment_length: float = 0.08726646259971647, translation_longest_valid_segment_length: float = 0.1, rotation_longest_valid_segment_length: float = 0.08726646259971647, min_steps: int = 1, max_steps: int = 2147483647) -> None: ...

    @property
    def state_longest_valid_segment_length(self) -> float: ...

    @state_longest_valid_segment_length.setter
    def state_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def translation_longest_valid_segment_length(self) -> float: ...

    @translation_longest_valid_segment_length.setter
    def translation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def rotation_longest_valid_segment_length(self) -> float: ...

    @rotation_longest_valid_segment_length.setter
    def rotation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def min_steps(self) -> int: ...

    @min_steps.setter
    def min_steps(self, arg: int, /) -> None: ...

    @property
    def max_steps(self) -> int: ...

    @max_steps.setter
    def max_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerLVSNoIKMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerLVSNoIKMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerLVSNoIKMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerLVSAssignMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, state_longest_valid_segment_length: float = 0.08726646259971647, translation_longest_valid_segment_length: float = 0.1, rotation_longest_valid_segment_length: float = 0.08726646259971647, min_steps: int = 1, max_steps: int = 2147483647) -> None: ...

    @property
    def state_longest_valid_segment_length(self) -> float: ...

    @state_longest_valid_segment_length.setter
    def state_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def translation_longest_valid_segment_length(self) -> float: ...

    @translation_longest_valid_segment_length.setter
    def translation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def rotation_longest_valid_segment_length(self) -> float: ...

    @rotation_longest_valid_segment_length.setter
    def rotation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def min_steps(self) -> int: ...

    @min_steps.setter
    def min_steps(self, arg: int, /) -> None: ...

    @property
    def max_steps(self) -> int: ...

    @max_steps.setter
    def max_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerLVSAssignMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerLVSAssignMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerLVSAssignMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class SimplePlannerLVSAssignNoIKMoveProfile(SimplePlannerMoveProfile):
    def __init__(self, state_longest_valid_segment_length: float = 0.08726646259971647, translation_longest_valid_segment_length: float = 0.1, rotation_longest_valid_segment_length: float = 0.08726646259971647, min_steps: int = 1, max_steps: int = 2147483647) -> None: ...

    @property
    def state_longest_valid_segment_length(self) -> float: ...

    @state_longest_valid_segment_length.setter
    def state_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def translation_longest_valid_segment_length(self) -> float: ...

    @translation_longest_valid_segment_length.setter
    def translation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def rotation_longest_valid_segment_length(self) -> float: ...

    @rotation_longest_valid_segment_length.setter
    def rotation_longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def min_steps(self) -> int: ...

    @min_steps.setter
    def min_steps(self, arg: int, /) -> None: ...

    @property
    def max_steps(self) -> int: ...

    @max_steps.setter
    def max_steps(self, arg: int, /) -> None: ...

    def __eq__(self, arg: SimplePlannerLVSAssignNoIKMoveProfile, /) -> bool: ...

    def __ne__(self, arg: SimplePlannerLVSAssignNoIKMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> SimplePlannerLVSAssignNoIKMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """
