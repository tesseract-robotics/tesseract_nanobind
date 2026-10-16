"""tesseract_motion_planners_ompl Python bindings"""

from collections.abc import Sequence
import enum
from typing import TypeAlias

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners


class OMPLSolverConfig:
    def __init__(self) -> None: ...

    @property
    def planning_time(self) -> float:
        """Max planning time allowed in seconds (default: 5.0)"""

    @planning_time.setter
    def planning_time(self, arg: float, /) -> None: ...

    @property
    def max_solutions(self) -> int:
        """Max number of solutions to find before exiting (default: 10)"""

    @max_solutions.setter
    def max_solutions(self, arg: int, /) -> None: ...

    @property
    def simplify(self) -> bool:
        """Simplify trajectory (default: false). Ignores n_output_states if true."""

    @simplify.setter
    def simplify(self, arg: bool, /) -> None: ...

    @property
    def optimize(self) -> bool:
        """Use all planning time to optimize trajectory (default: true)"""

    @optimize.setter
    def optimize(self, arg: bool, /) -> None: ...

    def addPlanner(self, planner: OMPLPlannerConfigurator) -> None:
        """Add a planner configurator (each runs in parallel)"""

    def clearPlanners(self) -> None:
        """Clear all planners"""

    def __eq__(self, arg: OMPLSolverConfig, /) -> bool: ...

    def __ne__(self, arg: OMPLSolverConfig, /) -> bool: ...

    __hash__: None = None

    @property
    def planners(self) -> list[OMPLPlannerConfigurator]:
        """
        Planner configurators, one parallel planner each. The list is copied on access: assign a new list (or use addPlanner/clearPlanners) to change it.
        """

    @planners.setter
    def planners(self, arg: Sequence[OMPLPlannerConfigurator], /) -> None: ...

class OMPLPlannerType(enum.Enum):
    SBL = 0

    EST = 1

    LBKPIECE1 = 2

    BKPIECE1 = 3

    KPIECE1 = 4

    BiTRRT = 5

    RRT = 6

    RRTConnect = 7

    RRTstar = 8

    TRRT = 9

    PRM = 10

    PRMstar = 11

    LazyPRMstar = 12

    SPARS = 13

class OMPLPlannerConfigurator:
    def getType(self) -> OMPLPlannerType: ...

class SBLConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

class ESTConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def goal_bias(self) -> float: ...

    @goal_bias.setter
    def goal_bias(self, arg: float, /) -> None: ...

class LBKPIECE1Configurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def border_fraction(self) -> float: ...

    @border_fraction.setter
    def border_fraction(self, arg: float, /) -> None: ...

    @property
    def min_valid_path_fraction(self) -> float: ...

    @min_valid_path_fraction.setter
    def min_valid_path_fraction(self, arg: float, /) -> None: ...

class BKPIECE1Configurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def border_fraction(self) -> float: ...

    @border_fraction.setter
    def border_fraction(self, arg: float, /) -> None: ...

    @property
    def failed_expansion_score_factor(self) -> float: ...

    @failed_expansion_score_factor.setter
    def failed_expansion_score_factor(self, arg: float, /) -> None: ...

    @property
    def min_valid_path_fraction(self) -> float: ...

    @min_valid_path_fraction.setter
    def min_valid_path_fraction(self, arg: float, /) -> None: ...

class KPIECE1Configurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def goal_bias(self) -> float: ...

    @goal_bias.setter
    def goal_bias(self, arg: float, /) -> None: ...

    @property
    def border_fraction(self) -> float: ...

    @border_fraction.setter
    def border_fraction(self, arg: float, /) -> None: ...

    @property
    def failed_expansion_score_factor(self) -> float: ...

    @failed_expansion_score_factor.setter
    def failed_expansion_score_factor(self, arg: float, /) -> None: ...

    @property
    def min_valid_path_fraction(self) -> float: ...

    @min_valid_path_fraction.setter
    def min_valid_path_fraction(self, arg: float, /) -> None: ...

class BiTRRTConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def temp_change_factor(self) -> float: ...

    @temp_change_factor.setter
    def temp_change_factor(self, arg: float, /) -> None: ...

    @property
    def cost_threshold(self) -> float: ...

    @cost_threshold.setter
    def cost_threshold(self, arg: float, /) -> None: ...

    @property
    def init_temperature(self) -> float: ...

    @init_temperature.setter
    def init_temperature(self, arg: float, /) -> None: ...

    @property
    def frontier_threshold(self) -> float: ...

    @frontier_threshold.setter
    def frontier_threshold(self, arg: float, /) -> None: ...

    @property
    def frontier_node_ratio(self) -> float: ...

    @frontier_node_ratio.setter
    def frontier_node_ratio(self, arg: float, /) -> None: ...

class RRTConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def goal_bias(self) -> float: ...

    @goal_bias.setter
    def goal_bias(self, arg: float, /) -> None: ...

class RRTConnectConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

class RRTstarConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def goal_bias(self) -> float: ...

    @goal_bias.setter
    def goal_bias(self, arg: float, /) -> None: ...

    @property
    def delay_collision_checking(self) -> bool: ...

    @delay_collision_checking.setter
    def delay_collision_checking(self, arg: bool, /) -> None: ...

class TRRTConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def range(self) -> float: ...

    @range.setter
    def range(self, arg: float, /) -> None: ...

    @property
    def goal_bias(self) -> float: ...

    @goal_bias.setter
    def goal_bias(self, arg: float, /) -> None: ...

    @property
    def temp_change_factor(self) -> float: ...

    @temp_change_factor.setter
    def temp_change_factor(self, arg: float, /) -> None: ...

    @property
    def init_temperature(self) -> float: ...

    @init_temperature.setter
    def init_temperature(self, arg: float, /) -> None: ...

    @property
    def frontier_threshold(self) -> float: ...

    @frontier_threshold.setter
    def frontier_threshold(self, arg: float, /) -> None: ...

    @property
    def frontier_node_ratio(self) -> float: ...

    @frontier_node_ratio.setter
    def frontier_node_ratio(self, arg: float, /) -> None: ...

class PRMConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def max_nearest_neighbors(self) -> int: ...

    @max_nearest_neighbors.setter
    def max_nearest_neighbors(self, arg: int, /) -> None: ...

class PRMstarConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

class LazyPRMstarConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

class SPARSConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...

    @property
    def max_failures(self) -> int: ...

    @max_failures.setter
    def max_failures(self, arg: int, /) -> None: ...

    @property
    def dense_delta_fraction(self) -> float: ...

    @dense_delta_fraction.setter
    def dense_delta_fraction(self, arg: float, /) -> None: ...

    @property
    def sparse_delta_fraction(self) -> float: ...

    @sparse_delta_fraction.setter
    def sparse_delta_fraction(self, arg: float, /) -> None: ...

    @property
    def stretch_factor(self) -> float: ...

    @stretch_factor.setter
    def stretch_factor(self, arg: float, /) -> None: ...

class OMPLMoveProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int: ...

    def createSolverConfig(self) -> OMPLSolverConfig:
        """The solver config the planner uses for this profile"""

OMPLPlanProfile: TypeAlias = OMPLMoveProfile

class OMPLRealVectorMoveProfile(OMPLMoveProfile):
    def __init__(self) -> None: ...

    @property
    def solver_config(self) -> OMPLSolverConfig:
        """The OMPL parallel planner solver config"""

    @solver_config.setter
    def solver_config(self, arg: OMPLSolverConfig, /) -> None: ...

    @property
    def contact_manager_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig:
        """The contact manager configuration (collision margins)"""

    @contact_manager_config.setter
    def contact_manager_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig, /) -> None: ...

    @property
    def collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig:
        """The collision check configuration"""

    @collision_check_config.setter
    def collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

    def __eq__(self, arg: OMPLRealVectorMoveProfile, /) -> bool: ...

    def __ne__(self, arg: OMPLRealVectorMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> OMPLRealVectorMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

OMPLRealVectorPlanProfile: TypeAlias = OMPLRealVectorMoveProfile

def OMPLMoveProfile_as_ProfileConstPtr(profile: OMPLMoveProfile) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Convert OMPLMoveProfile to Profile::ConstPtr for use with ProfileDictionary.addProfile
    """

def OMPLPlanProfile_as_ProfileConstPtr(profile: OMPLMoveProfile) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """Convert OMPLPlanProfile to Profile::ConstPtr (legacy alias)"""

def ProfileDictionary_addOMPLMoveProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: OMPLMoveProfile) -> None:
    """Add OMPL move profile to ProfileDictionary"""

def ProfileDictionary_addOMPLProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: OMPLMoveProfile) -> None:
    """Add OMPL plan profile to ProfileDictionary (legacy alias)"""

class OMPLMotionPlanner(tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner):
    def __init__(self, name: str) -> None: ...

def RNG_setSeed(seed: int) -> None:
    """
    Seed OMPL's global RNG seed generator for reproducible planning.

    Samplers created after this call draw deterministic seeds, so repeated solve()
    calls produce identical paths. Re-seeding after planning has started logs an
    OMPL error (harmless: only pre-existing RNG instances stay nondeterministic).
    A seed of 0 is rejected once generation has started.
    """

def RNG_getSeed() -> int:
    """
    Get the first seed used by OMPL's RNG seed generator.

    Log this value to make a stochastic planning failure reproducible by passing
    it to RNG_setSeed in a subsequent run.
    """
