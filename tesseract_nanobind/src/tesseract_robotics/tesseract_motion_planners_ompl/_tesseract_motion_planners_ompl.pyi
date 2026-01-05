import enum

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language

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

    def getNumPlanners(self) -> int:
        """Get number of planners (threads)"""

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

class SBLConfigurator(OMPLPlannerConfigurator):
    def __init__(self) -> None: ...
    @property
    def range(self) -> float: ...
    @range.setter
    def range(self, arg: float, /) -> None: ...

class OMPLPlanProfile(
    tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile
):
    def getKey(self) -> int: ...
    @staticmethod
    def getStaticKey() -> int: ...

class OMPLRealVectorPlanProfile(OMPLPlanProfile):
    def __init__(self) -> None: ...
    @property
    def solver_config(self) -> OMPLSolverConfig:
        """The OMPL parallel planner solver config"""

    @solver_config.setter
    def solver_config(self, arg: OMPLSolverConfig, /) -> None: ...
    @property
    def collision_check_config(
        self,
    ) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig:
        """The collision check configuration"""

    @collision_check_config.setter
    def collision_check_config(
        self,
        arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig,
        /,
    ) -> None: ...

def OMPLPlanProfile_as_ProfileConstPtr(
    profile: OMPLPlanProfile,
) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile:
    """
    Convert OMPLPlanProfile to Profile::ConstPtr for use with ProfileDictionary.addProfile
    """

def ProfileDictionary_addOMPLProfile(
    dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary,
    ns: str,
    profile_name: str,
    profile: OMPLPlanProfile,
) -> None:
    """Add OMPL plan profile to ProfileDictionary (cross-module workaround)"""

class OMPLMotionPlanner:
    def __init__(self, name: str) -> None: ...
    def getName(self) -> str: ...
    def solve(
        self, request: "tesseract_planning::PlannerRequest"
    ) -> "tesseract_planning::PlannerResponse": ...
    def terminate(self) -> bool: ...
    def clear(self) -> None: ...
