"""tesseract_motion_planners_trajopt_ifopt Python bindings"""

from collections.abc import Sequence
from typing import Annotated, TypeAlias

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners
import tesseract_robotics.trajopt_ifopt._trajopt_ifopt
import tesseract_robotics.trajopt_sqp._trajopt_sqp


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
    def lower_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @lower_tolerance.setter
    def lower_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def upper_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @upper_tolerance.setter
    def upper_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @coeff.setter
    def coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    def __eq__(self, arg: TrajOptIfoptCartesianWaypointConfig, /) -> bool: ...

    def __ne__(self, arg: TrajOptIfoptCartesianWaypointConfig, /) -> bool: ...

    __hash__: None = None

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
    def lower_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def upper_tolerance(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @upper_tolerance.setter
    def upper_tolerance(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @coeff.setter
    def coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    def __eq__(self, arg: TrajOptIfoptJointWaypointConfig, /) -> bool: ...

    def __ne__(self, arg: TrajOptIfoptJointWaypointConfig, /) -> bool: ...

    __hash__: None = None

class TrajOptIfoptTermInfos:
    """
    Constraint and cost sets a TrajOpt-Ifopt profile adds to the planner's problem
    """

    def __init__(self) -> None: ...

    @property
    def constraints(self) -> list[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet]: ...

    @constraints.setter
    def constraints(self, arg: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet], /) -> None: ...

    @property
    def squared_costs(self) -> list[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet]: ...

    @squared_costs.setter
    def squared_costs(self, arg: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet], /) -> None: ...

    @property
    def absolute_costs(self) -> list[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet]: ...

    @absolute_costs.setter
    def absolute_costs(self, arg: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet], /) -> None: ...

    @property
    def hinge_costs(self) -> list[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet]: ...

    @hinge_costs.setter
    def hinge_costs(self, arg: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet], /) -> None: ...

class TrajOptIfoptWaypointInfo:
    """
    What a TrajOpt-Ifopt move profile builds for one waypoint: its variables and terms
    """

    def __init__(self) -> None: ...

    @property
    def term_infos(self) -> TrajOptIfoptTermInfos: ...

    @term_infos.setter
    def term_infos(self, arg: TrajOptIfoptTermInfos, /) -> None: ...

    @property
    def node(self, /) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Node | None:
        """The waypoint's variable node (owned by this object), or None"""

    @property
    def fixed(self) -> bool:
        """Whether the waypoint is fixed (no collision terms at this step)"""

    @fixed.setter
    def fixed(self, arg: bool, /) -> None: ...

class TrajOptIfoptMoveProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every TrajOpt-Ifopt move profile carries"""

    def create(self, move_instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.MoveInstructionPoly, composite_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, index: int) -> TrajOptIfoptWaypointInfo:
        """Build the waypoint info the planner adds for this move instruction"""

TrajOptIfoptPlanProfile: TypeAlias = TrajOptIfoptMoveProfile

class TrajOptIfoptCompositeProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every TrajOpt-Ifopt composite profile carries"""

    def create(self, composite_manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, nodes: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Node], fixed_indices: Sequence[int]) -> TrajOptIfoptTermInfos:
        """
        Build the trajectory-wide terms the planner adds over all waypoint nodes
        """

class TrajOptIfoptSolverProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    @property
    def opt_params(self) -> tesseract_robotics.trajopt_sqp._trajopt_sqp.SQPParameters: ...

    @opt_params.setter
    def opt_params(self, arg: tesseract_robotics.trajopt_sqp._trajopt_sqp.SQPParameters, /) -> None: ...

    @property
    def callbacks(self) -> list[tesseract_robotics.trajopt_sqp._trajopt_sqp.SQPCallback]:
        """SQP callbacks registered on the solver, called once per SQP iteration"""

    @callbacks.setter
    def callbacks(self, arg: Sequence[tesseract_robotics.trajopt_sqp._trajopt_sqp.SQPCallback], /) -> None: ...

    def createOptimizationCallbacks(self) -> list[tesseract_robotics.trajopt_sqp._trajopt_sqp.SQPCallback]: ...

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int:
        """The key every TrajOpt-Ifopt solver profile carries"""

    def create(self, verbose: bool = False) -> tesseract_robotics.trajopt_sqp._trajopt_sqp.TrustRegionSQPSolver:
        """Build the TrustRegionSQPSolver the planner solves with"""

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

    def __eq__(self, arg: TrajOptIfoptDefaultMoveProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptIfoptDefaultMoveProfile, /) -> bool: ...

    __hash__: None = None

TrajOptIfoptDefaultPlanProfile: TypeAlias = TrajOptIfoptDefaultMoveProfile

class TrajOptIfoptDefaultCompositeProfile(TrajOptIfoptCompositeProfile):
    def __init__(self) -> None: ...

    @property
    def collision_cost_config(self) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig: ...

    @collision_cost_config.setter
    def collision_cost_config(self, arg: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig, /) -> None: ...

    @property
    def collision_constraint_config(self) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig: ...

    @collision_constraint_config.setter
    def collision_constraint_config(self, arg: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig, /) -> None: ...

    @property
    def smooth_velocities(self) -> bool: ...

    @smooth_velocities.setter
    def smooth_velocities(self, arg: bool, /) -> None: ...

    @property
    def velocity_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @velocity_coeff.setter
    def velocity_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def smooth_accelerations(self) -> bool: ...

    @smooth_accelerations.setter
    def smooth_accelerations(self, arg: bool, /) -> None: ...

    @property
    def acceleration_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @acceleration_coeff.setter
    def acceleration_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def smooth_jerks(self) -> bool: ...

    @smooth_jerks.setter
    def smooth_jerks(self, arg: bool, /) -> None: ...

    @property
    def jerk_coeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @jerk_coeff.setter
    def jerk_coeff(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    def __eq__(self, arg: TrajOptIfoptDefaultCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptIfoptDefaultCompositeProfile, /) -> bool: ...

    __hash__: None = None

class TrajOptIfoptOSQPSolverProfile(TrajOptIfoptSolverProfile):
    def __init__(self) -> None: ...

    def setPolish(self, polish: bool) -> None:
        """Enable solution polishing (default: true)"""

    def setWarmStart(self, warm_start: bool) -> None:
        """Enable warm-starting (default: true)"""

    def setAdaptiveRho(self, adaptive_rho: bool) -> None:
        """Enable adaptive step size (default: true)"""

    def setAdaptiveRhoInterval(self, interval: int) -> None:
        """
        Adapt rho every N iterations. OSQP's default 0 adapts on wall-clock timing,
        making solutions nondeterministic run-to-run; set a fixed interval (e.g. 25)
        for reproducible optimization.
        """

    def setMaxIteration(self, max_iter: int) -> None:
        """Max OSQP iterations per QP solve (default: 8192)"""

    def setAbsoluteTolerance(self, abs_tol: float) -> None:
        """Absolute convergence tolerance (default: 1e-4)"""

    def setRelativeTolerance(self, rel_tol: float) -> None:
        """Relative convergence tolerance (default: 1e-6)"""

    def setVerbosity(self, verbose: bool) -> None:
        """Enable OSQP console output (default: false)"""

    def __eq__(self, arg: TrajOptIfoptOSQPSolverProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptIfoptOSQPSolverProfile, /) -> bool: ...

    __hash__: None = None

def ProfileDictionary_addTrajOptIfoptMoveProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptMoveProfile) -> None:
    """Add TrajOptIfopt move profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptIfoptPlanProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptMoveProfile) -> None:
    """Add TrajOptIfopt plan profile to ProfileDictionary (legacy alias)"""

def ProfileDictionary_addTrajOptIfoptCompositeProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptCompositeProfile) -> None:
    """Add TrajOptIfopt composite profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptIfoptSolverProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptIfoptSolverProfile) -> None:
    """Add TrajOptIfopt solver profile to ProfileDictionary"""

def createJointPositionConstraint(joint_waypoint: tesseract_robotics.tesseract_command_language._tesseract_command_language.JointWaypointPoly, var: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet:
    """
    Joint position constraint at a waypoint: equality to its position, or its tolerance band
    """

def createCollisionConstraints(vars: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Var], env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, manip_info: tesseract_robotics.tesseract_common._tesseract_common.ManipulatorInfo, config: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.TrajOptCollisionConfig, fixed_indices: Sequence[int], fixed_sparsity: bool) -> list[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet]:
    """
    Collision constraint sets of config's collision check type: one per unfixed step (DISCRETE)
    or one per segment (LVS_DISCRETE, LVS_CONTINUOUS, CONTINUOUS); none for NONE
    """

def createJointVelocityConstraint(target: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], vars: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet:
    """Joint velocity constraint over vars toward target"""

def createJointAccelerationConstraint(target: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], vars: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet:
    """Joint acceleration constraint over vars toward target"""

def createJointJerkConstraint(target: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], vars: Sequence[tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet:
    """Joint jerk constraint over vars toward target"""

class TrajOptIfoptMotionPlanner(tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner):
    def __init__(self, name: str) -> None: ...
