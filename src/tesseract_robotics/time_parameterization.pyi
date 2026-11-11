"""tesseract_time_parameterization Python bindings"""

from collections.abc import Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.command_language
import tesseract_robotics.environment


class StaleTrajectoryError(RuntimeError):
    """
    The program changed after the InstructionsTrajectory was built; build a new one.
    """

class InstructionsTrajectory:
    def __init__(self, program: tesseract_robotics.command_language.CompositeInstruction) -> None:
        """
        Build a trajectory view over the move instructions of `program`.

        The trajectory keeps `program` alive and reads and writes its waypoints in place
        (`setData` changes `program`). Once instructions are appended, inserted or erased
        in `program`, every method raises `StaleTrajectoryError`: build a new
        trajectory. Each call re-flattens `program`, O(number of instructions).

        Raises:
            StaleTrajectoryError: on any method call after `program` changed (a `RuntimeError`).
            IndexError: an index method got `i` outside `[0, size())`; negative `i` is not wrapped.
        """

    def getPosition(self, i: int) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Joint positions of waypoint `i`."""

    def getVelocity(self, i: int) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Joint velocities of waypoint `i`."""

    def getAcceleration(self, i: int) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Joint accelerations of waypoint `i`."""

    def getTimeFromStart(self, i: int) -> float:
        """Time of waypoint `i` from the start [s]."""

    def setData(self, i: int, velocity: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], acceleration: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], time: float) -> None:
        """
        Write velocity, acceleration and time from start [s] into waypoint `i`.
        """

    def size(self) -> int:
        """The number of waypoints."""

    def dof(self) -> int:
        """The number of joints per waypoint."""

    def empty(self) -> bool:
        """True if there is no waypoint."""

    def isTimeStrictlyIncreasing(self) -> bool:
        """True if every waypoint's time is above the previous one's."""

class TimeOptimalTrajectoryGenerationCompositeProfile(tesseract_robotics.command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, max_velocity_scaling_factor: float, max_acceleration_scaling_factor: float, path_tolerance: float, min_angle_change: float) -> None: ...

    @property
    def override_limits(self) -> bool:
        """
        Use `velocity_limits` / `acceleration_limits` (/ `jerk_limits`) instead of the joint group's.
        """

    @override_limits.setter
    def override_limits(self, arg: bool, /) -> None: ...

    @property
    def velocity_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) velocity limits [m/s or rad/s], used when `override_limits`.
        """

    @velocity_limits.setter
    def velocity_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def acceleration_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) acceleration limits [m/s² or rad/s²], used when `override_limits`.
        """

    @acceleration_limits.setter
    def acceleration_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def max_velocity_scaling_factor(self) -> float:
        """Fraction of the velocity limits to plan with, in (0, 1]."""

    @max_velocity_scaling_factor.setter
    def max_velocity_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def max_acceleration_scaling_factor(self) -> float:
        """Fraction of the acceleration limits to plan with, in (0, 1]."""

    @max_acceleration_scaling_factor.setter
    def max_acceleration_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def path_tolerance(self) -> float:
        """
        How far the path may deviate from the waypoints where TOTG blends them [rad or m, joint space].
        """

    @path_tolerance.setter
    def path_tolerance(self, arg: float, /) -> None: ...

    @property
    def min_angle_change(self) -> float:
        """Smallest direction change [rad] TOTG treats as a corner to blend."""

    @min_angle_change.setter
    def min_angle_change(self, arg: float, /) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every profile of this class carries in a ProfileDictionary."""

    def __eq__(self, arg: TimeOptimalTrajectoryGenerationCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: TimeOptimalTrajectoryGenerationCompositeProfile, /) -> bool: ...

    __hash__: None = None

class IterativeSplineParameterizationCompositeProfile(tesseract_robotics.command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, max_velocity_scaling_factor: float, max_acceleration_scaling_factor: float) -> None: ...

    @property
    def add_points(self) -> bool:
        """
        Add points between the first two and the last two waypoints for smoother start and end.
        """

    @add_points.setter
    def add_points(self, arg: bool, /) -> None: ...

    @property
    def override_limits(self) -> bool:
        """
        Use `velocity_limits` / `acceleration_limits` (/ `jerk_limits`) instead of the joint group's.
        """

    @override_limits.setter
    def override_limits(self, arg: bool, /) -> None: ...

    @property
    def velocity_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) velocity limits [m/s or rad/s], used when `override_limits`.
        """

    @velocity_limits.setter
    def velocity_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def acceleration_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) acceleration limits [m/s² or rad/s²], used when `override_limits`.
        """

    @acceleration_limits.setter
    def acceleration_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def max_velocity_scaling_factor(self) -> float:
        """Fraction of the velocity limits to plan with, in (0, 1]."""

    @max_velocity_scaling_factor.setter
    def max_velocity_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def max_acceleration_scaling_factor(self) -> float:
        """Fraction of the acceleration limits to plan with, in (0, 1]."""

    @max_acceleration_scaling_factor.setter
    def max_acceleration_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def minimum_time_delta(self) -> float:
        """Smallest time step between waypoints [s]."""

    @minimum_time_delta.setter
    def minimum_time_delta(self, arg: float, /) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every profile of this class carries in a ProfileDictionary."""

    def __eq__(self, arg: IterativeSplineParameterizationCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: IterativeSplineParameterizationCompositeProfile, /) -> bool: ...

    __hash__: None = None

class IterativeSplineParameterizationMoveProfile(tesseract_robotics.command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, max_velocity_scaling_factor: float, max_acceleration_scaling_factor: float) -> None: ...

    @property
    def max_velocity_scaling_factor(self) -> float:
        """Fraction of the velocity limits to plan with, in (0, 1]."""

    @max_velocity_scaling_factor.setter
    def max_velocity_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def max_acceleration_scaling_factor(self) -> float:
        """Fraction of the acceleration limits to plan with, in (0, 1]."""

    @max_acceleration_scaling_factor.setter
    def max_acceleration_scaling_factor(self, arg: float, /) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every profile of this class carries in a ProfileDictionary."""

    def __eq__(self, arg: IterativeSplineParameterizationMoveProfile, /) -> bool: ...

    def __ne__(self, arg: IterativeSplineParameterizationMoveProfile, /) -> bool: ...

    __hash__: None = None

class TimeParameterization:
    def getName(self) -> str:
        """The name this parameterization registers its profiles under."""

    def compute(self, composite_instruction: tesseract_robotics.command_language.CompositeInstruction, env: tesseract_robotics.environment.Environment, profiles: tesseract_robotics.command_language.ProfileDictionary) -> bool: ...

class TimeOptimalTrajectoryGeneration(TimeParameterization):
    def __init__(self, name: str = 'TOTG') -> None: ...

class IterativeSplineParameterization(TimeParameterization):
    def __init__(self, name: str = 'ISP') -> None: ...

class ConstantTCPSpeedParameterizationCompositeProfile(tesseract_robotics.command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, max_translational_velocity: float, max_rotational_velocity: float, max_translational_acceleration: float, max_rotational_acceleration: float, max_velocity_scaling_factor: float = 1.0, max_acceleration_scaling_factor: float = 1.0) -> None: ...

    @property
    def max_translational_velocity(self) -> float:
        """TCP speed limit [m/s]."""

    @max_translational_velocity.setter
    def max_translational_velocity(self, arg: float, /) -> None: ...

    @property
    def max_rotational_velocity(self) -> float:
        """TCP angular speed limit [rad/s]."""

    @max_rotational_velocity.setter
    def max_rotational_velocity(self, arg: float, /) -> None: ...

    @property
    def max_translational_acceleration(self) -> float:
        """TCP acceleration limit [m/s²]."""

    @max_translational_acceleration.setter
    def max_translational_acceleration(self, arg: float, /) -> None: ...

    @property
    def max_rotational_acceleration(self) -> float:
        """TCP angular acceleration limit [rad/s²]."""

    @max_rotational_acceleration.setter
    def max_rotational_acceleration(self, arg: float, /) -> None: ...

    @property
    def max_velocity_scaling_factor(self) -> float:
        """Fraction of the velocity limits to plan with, in (0, 1]."""

    @max_velocity_scaling_factor.setter
    def max_velocity_scaling_factor(self, arg: float, /) -> None: ...

    @property
    def max_acceleration_scaling_factor(self) -> float:
        """Fraction of the acceleration limits to plan with, in (0, 1]."""

    @max_acceleration_scaling_factor.setter
    def max_acceleration_scaling_factor(self, arg: float, /) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every profile of this class carries in a ProfileDictionary."""

    def __eq__(self, arg: ConstantTCPSpeedParameterizationCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: ConstantTCPSpeedParameterizationCompositeProfile, /) -> bool: ...

    __hash__: None = None

class ConstantTCPSpeedParameterization(TimeParameterization):
    def __init__(self, name: str = 'ConstantTCPSpeed') -> None: ...

class RuckigTrajectorySmoothingCompositeProfile(tesseract_robotics.command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, duration_extension_fraction: float, max_duration_extension_factor: float) -> None: ...

    @property
    def duration_extension_fraction(self) -> float:
        """Fraction by which each retry stretches the segment durations."""

    @duration_extension_fraction.setter
    def duration_extension_fraction(self, arg: float, /) -> None: ...

    @property
    def max_duration_extension_factor(self) -> float:
        """Largest total stretch before smoothing gives up."""

    @max_duration_extension_factor.setter
    def max_duration_extension_factor(self, arg: float, /) -> None: ...

    @property
    def override_limits(self) -> bool:
        """
        Use `velocity_limits` / `acceleration_limits` (/ `jerk_limits`) instead of the joint group's.
        """

    @override_limits.setter
    def override_limits(self, arg: bool, /) -> None: ...

    @property
    def velocity_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) velocity limits [m/s or rad/s], used when `override_limits`.
        """

    @velocity_limits.setter
    def velocity_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def acceleration_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) acceleration limits [m/s² or rad/s²], used when `override_limits`.
        """

    @acceleration_limits.setter
    def acceleration_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def jerk_limits(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')]:
        """
        Per-joint (lower, upper) jerk limits [m/s³ or rad/s³], used when `override_limits`.
        """

    @jerk_limits.setter
    def jerk_limits(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @staticmethod
    def createKey() -> int:
        """The key every profile of this class carries in a ProfileDictionary."""

    def __eq__(self, arg: RuckigTrajectorySmoothingCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: RuckigTrajectorySmoothingCompositeProfile, /) -> bool: ...

    __hash__: None = None

class RuckigTrajectorySmoothing(TimeParameterization):
    def __init__(self, name: str = 'RuckigTrajectorySmoothing') -> None: ...

class ScalingsLengthError(ValueError):
    """rescaleTimings got a scalings list that is not len(program) long."""

def rescaleTimings(program: tesseract_robotics.command_language.CompositeInstruction, scalings: Sequence[float]) -> None:
    """
    Rescale the timing of each sub-composite of `program` in place.

    Every element of `program` must be a CompositeInstruction; `scalings[i]` scales
    element i linearly (0.5 is half speed). A scaling below 1e-6 is replaced by 1.0
    with only a logged warning.

    Raises:
        ScalingsLengthError: `scalings` is not `len(program)` long (a `ValueError`).
    """
