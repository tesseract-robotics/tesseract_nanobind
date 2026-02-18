from typing import Annotated, TypeAlias

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners


class TrajOptIfoptCartesianWaypointConfig:
    def __init__(self) -> None: ...

    @property
    def enabled(self) -> bool: ...

    @enabled.setter
    def enabled(self, arg: bool, /) -> None: ...

    @property
    def use_tolerance_override(self) -> bool: ...

    @use_tolerance_override.setter
    def use_tolerance_override(self, arg: bool, /) -> None: ...

    @property
    def lower_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]: ...

    @lower_tolerance.setter
    def lower_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')], /) -> None: ...

    @property
    def upper_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]: ...

    @upper_tolerance.setter
    def upper_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')], /) -> None: ...

    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]: ...

    @coeff.setter
    def coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')], /) -> None: ...

class TrajOptIfoptJointWaypointConfig:
    def __init__(self) -> None: ...

    @property
    def enabled(self) -> bool: ...

    @enabled.setter
    def enabled(self, arg: bool, /) -> None: ...

    @property
    def use_tolerance_override(self) -> bool: ...

    @use_tolerance_override.setter
    def use_tolerance_override(self, arg: bool, /) -> None: ...

    @property
    def lower_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @lower_tolerance.setter
    def lower_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def upper_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @upper_tolerance.setter
    def upper_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @coeff.setter
    def coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class TrajOptIfoptMoveProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def getStaticKey() -> int: ...

TrajOptIfoptPlanProfile: TypeAlias = TrajOptIfoptMoveProfile

class TrajOptIfoptCompositeProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def getStaticKey() -> int: ...

class TrajOptIfoptSolverProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def getStaticKey() -> int: ...

class TrajOptIfoptDefaultMoveProfile(TrajOptIfoptMoveProfile):
    def __init__(self) -> None: ...

    @property
    def cartesian_cost_config(self) -> TrajOptIfoptCartesianWaypointConfig: ...

    @cartesian_cost_config.setter
    def cartesian_cost_config(self, arg: TrajOptIfoptCartesianWaypointConfig, /) -> None: ...

    @property
    def cartesian_constraint_config(self) -> TrajOptIfoptCartesianWaypointConfig: ...

    @cartesian_constraint_config.setter
    def cartesian_constraint_config(self, arg: TrajOptIfoptCartesianWaypointConfig, /) -> None: ...

    @property
    def joint_cost_config(self) -> TrajOptIfoptJointWaypointConfig: ...

    @joint_cost_config.setter
    def joint_cost_config(self, arg: TrajOptIfoptJointWaypointConfig, /) -> None: ...

    @property
    def joint_constraint_config(self) -> TrajOptIfoptJointWaypointConfig: ...

    @joint_constraint_config.setter
    def joint_constraint_config(self, arg: TrajOptIfoptJointWaypointConfig, /) -> None: ...

TrajOptIfoptDefaultPlanProfile: TypeAlias = TrajOptIfoptDefaultMoveProfile

class TrajOptIfoptDefaultCompositeProfile(TrajOptIfoptCompositeProfile):
    def __init__(self) -> None: ...

    @property
    def smooth_velocities(self) -> bool: ...

    @smooth_velocities.setter
    def smooth_velocities(self, arg: bool, /) -> None: ...

    @property
    def velocity_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @velocity_coeff.setter
    def velocity_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def smooth_accelerations(self) -> bool: ...

    @smooth_accelerations.setter
    def smooth_accelerations(self, arg: bool, /) -> None: ...

    @property
    def acceleration_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @acceleration_coeff.setter
    def acceleration_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def smooth_jerks(self) -> bool: ...

    @smooth_jerks.setter
    def smooth_jerks(self, arg: bool, /) -> None: ...

    @property
    def jerk_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @jerk_coeff.setter
    def jerk_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class TrajOptIfoptOSQPSolverProfile(TrajOptIfoptSolverProfile):
    def __init__(self) -> None: ...

def ProfileDictionary_addTrajOptIfoptMoveProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptMoveProfile) -> None:
    """Add TrajOptIfopt move profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptIfoptPlanProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptMoveProfile) -> None:
    """Add TrajOptIfopt plan profile to ProfileDictionary (legacy alias)"""

def ProfileDictionary_addTrajOptIfoptCompositeProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptCompositeProfile) -> None:
    """Add TrajOptIfopt composite profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptIfoptSolverProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptSolverProfile) -> None:
    """Add TrajOptIfopt solver profile to ProfileDictionary"""

class TrajOptIfoptMotionPlanner:
    def __init__(self, name: str) -> None: ...

    def getName(self) -> str: ...

    def solve(self, request: tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerRequest) -> tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.PlannerResponse: ...

    def terminate(self) -> bool: ...

    def clear(self) -> None: ...

    def clone(self) -> tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner: ...
