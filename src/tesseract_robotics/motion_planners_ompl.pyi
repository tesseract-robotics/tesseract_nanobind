"""tesseract_motion_planners_ompl Python bindings"""

from collections.abc import Sequence
import enum

import tesseract_robotics.collision
import tesseract_robotics.command_language
import tesseract_robotics.motion_planners


class OMPLObjectiveReturnTypeError(TypeError):
    pass

class OMPLStateCostError(TypeError):
    pass

class OMPLStateSpaceTypeError(TypeError):
    pass

class SpaceInformation:
    """
    The planning problem's OMPL space information (opaque). The objectives are built on it.
    """

class Cost:
    """An OMPL cost value"""

    def __init__(self, value: float = 0.0) -> None: ...

    def value(self) -> float: ...

    def __float__(self) -> float: ...

    def __repr__(self) -> str: ...

class OptimizationObjective:
    """
    Base of the OMPL optimization objectives. The planner's optimize loop ends when isSatisfied(cost of the best path) is true, or at planning_time.
    """

    def isSatisfied(self, cost: Cost) -> bool:
        """True if cost is better than the cost threshold"""

    def getCostThreshold(self) -> Cost: ...

    def setCostThreshold(self, cost: Cost) -> None: ...

    def getDescription(self) -> str: ...

class PathLengthOptimizationObjective(OptimizationObjective):
    """Joint-space path length (the planner's default objective)"""

    def __init__(self, si: SpaceInformation) -> None: ...

class MaximizeMinClearanceObjective(OptimizationObjective):
    """
    Maximise the smallest clearance the state validity checker reports along the path. tesseract's OMPL state validators do not report clearance (OMPL's default, 0, for every state), so on this stack the term is constant.
    """

    def __init__(self, si: SpaceInformation) -> None: ...

class MechanicalWorkOptimizationObjective(OptimizationObjective):
    """Mechanical work along the path, plus a small path-length term"""

    def __init__(self, si: SpaceInformation, path_length_weight: float = 1e-05) -> None: ...

class StateCostIntegralObjective(OptimizationObjective):
    """
    Integral of a per-state cost along the path. Subclass it and override stateCost(self, values) -> float, where values holds the state's joint positions (rad or m per joint); the planner's thread takes the GIL for every call. Without an override the cost of every state is 1, so the integral is path length.
    """

    def __init__(self, si: SpaceInformation, enable_motion_cost_interpolation: bool = False) -> None: ...

    def isMotionCostInterpolationEnabled(self) -> bool: ...

class MultiOptimizationObjective(OptimizationObjective):
    """Weighted sum of objectives"""

    def __init__(self, si: SpaceInformation) -> None: ...

    def addObjective(self, objective: OptimizationObjective, weight: float) -> None: ...

    def getObjectiveCount(self) -> int: ...

    def getObjectiveWeight(self, index: int) -> float: ...

    def setObjectiveWeight(self, index: int, weight: float) -> None: ...

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
    def simplify_time(self) -> float:
        """
        Max time in seconds to spend simplifying when simplify is true (default: 0.0). <= 0 runs simplification to completion (unbounded); > 0 time-bounds it.
        """

    @simplify_time.setter
    def simplify_time(self, arg: float, /) -> None: ...

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
        Planner configurators, one parallel planner each. The list is copied on access; its configurators are shared with the config, so editing one edits the config. Assign a new list (or use addPlanner/clearPlanners) to change which planners it holds.
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

class OMPLMoveProfile(tesseract_robotics.command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int: ...

    def createSolverConfig(self) -> OMPLSolverConfig:
        """The solver config the planner uses for this profile"""

class OMPLRealVectorMoveProfile(OMPLMoveProfile):
    def __init__(self) -> None: ...

    @property
    def solver_config(self) -> OMPLSolverConfig:
        """The OMPL parallel planner solver config"""

    @solver_config.setter
    def solver_config(self, arg: OMPLSolverConfig, /) -> None: ...

    @property
    def contact_manager_config(self) -> tesseract_robotics.collision.ContactManagerConfig:
        """The contact manager configuration (collision margins)"""

    @contact_manager_config.setter
    def contact_manager_config(self, arg: tesseract_robotics.collision.ContactManagerConfig, /) -> None: ...

    @property
    def collision_check_config(self) -> tesseract_robotics.collision.CollisionCheckConfig:
        """The collision check configuration"""

    @collision_check_config.setter
    def collision_check_config(self, arg: tesseract_robotics.collision.CollisionCheckConfig, /) -> None: ...

    def __eq__(self, arg: OMPLRealVectorMoveProfile, /) -> bool: ...

    def __ne__(self, arg: OMPLRealVectorMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> OMPLRealVectorMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.

        Nested sub-configs are replaced, not merged: a key naming one rebuilds it from that struct's own defaults, so fields left out lose the profile's values. This is per struct; here `solver_config:` starts with no planners (OMPLSolverConfig() holds none).
        """

class OMPLMotionPlanner(tesseract_robotics.motion_planners.MotionPlanner):
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
