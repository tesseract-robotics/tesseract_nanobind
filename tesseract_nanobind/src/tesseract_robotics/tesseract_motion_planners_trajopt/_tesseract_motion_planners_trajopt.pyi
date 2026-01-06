import enum
from typing import Annotated

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.trajopt_ifopt._trajopt_ifopt

# NOTE: 0.33 API renamed CollisionEvaluatorType enum values:
# SINGLE_TIMESTEP -> DISCRETE
# DISCRETE_CONTINUOUS -> LVS_DISCRETE
# CAST_CONTINUOUS -> CONTINUOUS/LVS_CONTINUOUS
# This stub is outdated - prefer importing from tesseract_collision:
# from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
class CollisionEvaluatorType(enum.Enum):
    # 0.33 actual values: NONE, DISCRETE, LVS_DISCRETE, CONTINUOUS, LVS_CONTINUOUS
    NONE = 0
    DISCRETE = 1
    LVS_DISCRETE = 2
    CONTINUOUS = 3
    LVS_CONTINUOUS = 4

class TrajOptCartesianWaypointConfig:
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
    def lower_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")]: ...
    @lower_tolerance.setter
    def lower_tolerance(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")], /
    ) -> None: ...
    @property
    def upper_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")]: ...
    @upper_tolerance.setter
    def upper_tolerance(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")], /
    ) -> None: ...
    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")]: ...
    @coeff.setter
    def coeff(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order="C")], /
    ) -> None: ...

class TrajOptJointWaypointConfig:
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
    def lower_tolerance(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @lower_tolerance.setter
    def lower_tolerance(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...
    @property
    def upper_tolerance(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @upper_tolerance.setter
    def upper_tolerance(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...
    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @coeff.setter
    def coeff(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...

# NOTE: 0.33 API removed CollisionCostConfig and CollisionConstraintConfig
# Both are now replaced by TrajOptCollisionConfig from trajopt_ifopt module
# Import: from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

class TrajOptPlanProfile(
    tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile
):
    def getKey(self) -> int: ...
    @staticmethod
    def getStaticKey() -> int: ...

class TrajOptCompositeProfile(
    tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile
):
    def getKey(self) -> int: ...
    @staticmethod
    def getStaticKey() -> int: ...

class TrajOptDefaultPlanProfile(TrajOptPlanProfile):
    def __init__(self) -> None: ...
    @property
    def cartesian_cost_config(self) -> TrajOptCartesianWaypointConfig: ...
    @cartesian_cost_config.setter
    def cartesian_cost_config(self, arg: TrajOptCartesianWaypointConfig, /) -> None: ...
    @property
    def cartesian_constraint_config(self) -> TrajOptCartesianWaypointConfig: ...
    @cartesian_constraint_config.setter
    def cartesian_constraint_config(self, arg: TrajOptCartesianWaypointConfig, /) -> None: ...
    @property
    def joint_cost_config(self) -> TrajOptJointWaypointConfig: ...
    @joint_cost_config.setter
    def joint_cost_config(self, arg: TrajOptJointWaypointConfig, /) -> None: ...
    @property
    def joint_constraint_config(self) -> TrajOptJointWaypointConfig: ...
    @joint_constraint_config.setter
    def joint_constraint_config(self, arg: TrajOptJointWaypointConfig, /) -> None: ...

class TrajOptDefaultCompositeProfile(TrajOptCompositeProfile):
    def __init__(self) -> None: ...
    # NOTE: 0.33 API removed contact_test_type, longest_valid_segment_fraction/length from this class
    # They are now in collision_cost_config.collision_check_config
    @property
    def collision_cost_config(
        self,
    ) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig: ...
    @collision_cost_config.setter
    def collision_cost_config(
        self, arg: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig, /
    ) -> None: ...
    @property
    def collision_constraint_config(
        self,
    ) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig: ...
    @collision_constraint_config.setter
    def collision_constraint_config(
        self, arg: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig, /
    ) -> None: ...
    @property
    def smooth_velocities(self) -> bool: ...
    @smooth_velocities.setter
    def smooth_velocities(self, arg: bool, /) -> None: ...
    @property
    def velocity_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @velocity_coeff.setter
    def velocity_coeff(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...
    @property
    def smooth_accelerations(self) -> bool: ...
    @smooth_accelerations.setter
    def smooth_accelerations(self, arg: bool, /) -> None: ...
    @property
    def acceleration_coeff(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @acceleration_coeff.setter
    def acceleration_coeff(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...
    @property
    def smooth_jerks(self) -> bool: ...
    @smooth_jerks.setter
    def smooth_jerks(self, arg: bool, /) -> None: ...
    @property
    def jerk_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    @jerk_coeff.setter
    def jerk_coeff(
        self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")], /
    ) -> None: ...
    @property
    def avoid_singularity(self) -> bool: ...
    @avoid_singularity.setter
    def avoid_singularity(self, arg: bool, /) -> None: ...
    @property
    def avoid_singularity_coeff(self) -> float: ...
    @avoid_singularity_coeff.setter
    def avoid_singularity_coeff(self, arg: float, /) -> None: ...

def ProfileDictionary_addTrajOptPlanProfile(
    dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary,
    ns: str,
    profile_name: str,
    profile: TrajOptPlanProfile,
) -> None:
    """Add TrajOpt plan profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptCompositeProfile(
    dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary,
    ns: str,
    profile_name: str,
    profile: TrajOptCompositeProfile,
) -> None:
    """Add TrajOpt composite profile to ProfileDictionary"""

class TrajOptMotionPlanner:
    def __init__(self, name: str) -> None: ...
    def getName(self) -> str: ...
    def solve(
        self, request: "tesseract_planning::PlannerRequest"
    ) -> "tesseract_planning::PlannerResponse": ...
    def terminate(self) -> bool: ...
    def clear(self) -> None: ...
