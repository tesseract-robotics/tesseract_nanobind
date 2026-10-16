"""tesseract_motion_planners_trajopt Python bindings"""

from collections.abc import Sequence
from typing import Annotated, TypeAlias

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners
import tesseract_robotics.trajopt_ifopt._trajopt_ifopt


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

    def __eq__(self, arg: TrajOptCartesianWaypointConfig, /) -> bool: ...

    def __ne__(self, arg: TrajOptCartesianWaypointConfig, /) -> bool: ...

    __hash__: None = None

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

    def __eq__(self, arg: TrajOptJointWaypointConfig, /) -> bool: ...

    def __ne__(self, arg: TrajOptJointWaypointConfig, /) -> bool: ...

    __hash__: None = None

class TrajOptMoveProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int: ...

TrajOptPlanProfile: TypeAlias = TrajOptMoveProfile

class TrajOptCompositeProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int: ...

class TrajOptDefaultMoveProfile(TrajOptMoveProfile):
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

    def __eq__(self, arg: TrajOptDefaultMoveProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptDefaultMoveProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> TrajOptDefaultMoveProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

TrajOptDefaultPlanProfile: TypeAlias = TrajOptDefaultMoveProfile

class TrajOptDefaultCompositeProfile(TrajOptCompositeProfile):
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

    @property
    def avoid_singularity(self) -> bool: ...

    @avoid_singularity.setter
    def avoid_singularity(self, arg: bool, /) -> None: ...

    @property
    def avoid_singularity_coeff(self) -> float: ...

    @avoid_singularity_coeff.setter
    def avoid_singularity_coeff(self, arg: float, /) -> None: ...

    @staticmethod
    def computeLongestValidSegmentLength(joint_limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), order='F')] | Sequence[Sequence[float]], longest_valid_segment_fraction: float, longest_valid_segment_length: float) -> float:
        """
        Collision-check step length from the joint limits (n x 2, lower/upper) and a
        longest-valid-segment fraction and length: the smaller of fraction * extent and
        length where both are positive, else the positive one, else 0.01 * extent,
        with extent = ||upper - lower||.
        """

    def __eq__(self, arg: TrajOptDefaultCompositeProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptDefaultCompositeProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> TrajOptDefaultCompositeProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

class BasicTrustRegionSQPParameters:
    def __init__(self) -> None: ...

    @property
    def improve_ratio_threshold(self) -> float:
        """Minimum ratio exact_improve/approx_improve to accept step"""

    @improve_ratio_threshold.setter
    def improve_ratio_threshold(self, arg: float, /) -> None: ...

    @property
    def min_trust_box_size(self) -> float:
        """If trust region gets any smaller, exit and report convergence"""

    @min_trust_box_size.setter
    def min_trust_box_size(self, arg: float, /) -> None: ...

    @property
    def min_approx_improve(self) -> float:
        """If model improves less than this, exit and report convergence"""

    @min_approx_improve.setter
    def min_approx_improve(self, arg: float, /) -> None: ...

    @property
    def min_approx_improve_frac(self) -> float:
        """If model improves less than this fraction, exit and report convergence"""

    @min_approx_improve_frac.setter
    def min_approx_improve_frac(self, arg: float, /) -> None: ...

    @property
    def max_iter(self) -> int:
        """The max number of iterations"""

    @max_iter.setter
    def max_iter(self, arg: int, /) -> None: ...

    @property
    def trust_shrink_ratio(self) -> float:
        """
        If improvement is less than improve_ratio_threshold, shrink trust region by this ratio
        """

    @trust_shrink_ratio.setter
    def trust_shrink_ratio(self, arg: float, /) -> None: ...

    @property
    def trust_expand_ratio(self) -> float:
        """
        If improvement is greater than improve_ratio_threshold, expand trust region by this ratio
        """

    @trust_expand_ratio.setter
    def trust_expand_ratio(self, arg: float, /) -> None: ...

    @property
    def cnt_tolerance(self) -> float:
        """
        After convergence of penalty subproblem, if constraint violation is less than this, we're done
        """

    @cnt_tolerance.setter
    def cnt_tolerance(self, arg: float, /) -> None: ...

    @property
    def max_merit_coeff_increases(self) -> float:
        """Max number of times that the constraints' cost will be increased"""

    @max_merit_coeff_increases.setter
    def max_merit_coeff_increases(self, arg: float, /) -> None: ...

    @property
    def max_qp_solver_failures(self) -> int:
        """
        Max number of times the QP solver can fail before optimization is aborted
        """

    @max_qp_solver_failures.setter
    def max_qp_solver_failures(self, arg: int, /) -> None: ...

    @property
    def merit_coeff_increase_ratio(self) -> float:
        """Ratio that we increase coeff each time"""

    @merit_coeff_increase_ratio.setter
    def merit_coeff_increase_ratio(self, arg: float, /) -> None: ...

    @property
    def max_time(self) -> float:
        """Max time in seconds that the optimizer will run"""

    @max_time.setter
    def max_time(self, arg: float, /) -> None: ...

    @property
    def initial_merit_error_coeff(self) -> float:
        """Initial coefficient that is used to scale the constraints"""

    @initial_merit_error_coeff.setter
    def initial_merit_error_coeff(self, arg: float, /) -> None: ...

    @property
    def inflate_constraints_individually(self) -> bool:
        """
        If true, merit coeffs will only be inflated for the constraints that failed
        """

    @inflate_constraints_individually.setter
    def inflate_constraints_individually(self, arg: bool, /) -> None: ...

    @property
    def trust_box_size(self) -> float:
        """Current size of trust region (component-wise)"""

    @trust_box_size.setter
    def trust_box_size(self, arg: float, /) -> None: ...

    @property
    def log_results(self) -> bool:
        """Log results to file"""

    @log_results.setter
    def log_results(self, arg: bool, /) -> None: ...

    @property
    def log_dir(self) -> str:
        """Directory to store log results"""

    @log_dir.setter
    def log_dir(self, arg: str, /) -> None: ...

    @property
    def num_threads(self) -> int:
        """If greater than one, multi threaded functions are called"""

    @num_threads.setter
    def num_threads(self, arg: int, /) -> None: ...

    def __eq__(self, arg: BasicTrustRegionSQPParameters, /) -> bool: ...

    def __ne__(self, arg: BasicTrustRegionSQPParameters, /) -> bool: ...

    __hash__: None = None

class TrajOptSolverProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    @property
    def opt_params(self) -> BasicTrustRegionSQPParameters:
        """Optimization parameters"""

    @opt_params.setter
    def opt_params(self, arg: BasicTrustRegionSQPParameters, /) -> None: ...

    def getKey(self) -> int: ...

    @staticmethod
    def createKey() -> int: ...

    def createOptimizationParameters(self) -> BasicTrustRegionSQPParameters:
        """The optimization parameters the planner uses for this profile"""

class OSQPSettings:
    def __init__(self) -> None:
        """
        OSQP's own defaults (osqp_set_default_settings). TrajOptOSQPSolverProfile().settings holds trajopt's defaults instead.
        """

    @property
    def device(self) -> int: ...

    @device.setter
    def device(self, arg: int, /) -> None: ...

    @property
    def allocate_solution(self) -> int: ...

    @allocate_solution.setter
    def allocate_solution(self, arg: int, /) -> None: ...

    @property
    def verbose(self) -> int: ...

    @verbose.setter
    def verbose(self, arg: int, /) -> None: ...

    @property
    def profiler_level(self) -> int: ...

    @profiler_level.setter
    def profiler_level(self, arg: int, /) -> None: ...

    @property
    def warm_starting(self) -> int: ...

    @warm_starting.setter
    def warm_starting(self, arg: int, /) -> None: ...

    @property
    def scaling(self) -> int: ...

    @scaling.setter
    def scaling(self, arg: int, /) -> None: ...

    @property
    def polishing(self) -> int: ...

    @polishing.setter
    def polishing(self, arg: int, /) -> None: ...

    @property
    def rho(self) -> float: ...

    @rho.setter
    def rho(self, arg: float, /) -> None: ...

    @property
    def rho_is_vec(self) -> int: ...

    @rho_is_vec.setter
    def rho_is_vec(self, arg: int, /) -> None: ...

    @property
    def sigma(self) -> float: ...

    @sigma.setter
    def sigma(self, arg: float, /) -> None: ...

    @property
    def alpha(self) -> float: ...

    @alpha.setter
    def alpha(self, arg: float, /) -> None: ...

    @property
    def cg_max_iter(self) -> int: ...

    @cg_max_iter.setter
    def cg_max_iter(self, arg: int, /) -> None: ...

    @property
    def cg_tol_reduction(self) -> int: ...

    @cg_tol_reduction.setter
    def cg_tol_reduction(self, arg: int, /) -> None: ...

    @property
    def cg_tol_fraction(self) -> float: ...

    @cg_tol_fraction.setter
    def cg_tol_fraction(self, arg: float, /) -> None: ...

    @property
    def adaptive_rho(self) -> int: ...

    @adaptive_rho.setter
    def adaptive_rho(self, arg: int, /) -> None: ...

    @property
    def adaptive_rho_interval(self) -> int: ...

    @adaptive_rho_interval.setter
    def adaptive_rho_interval(self, arg: int, /) -> None: ...

    @property
    def adaptive_rho_fraction(self) -> float: ...

    @adaptive_rho_fraction.setter
    def adaptive_rho_fraction(self, arg: float, /) -> None: ...

    @property
    def adaptive_rho_tolerance(self) -> float: ...

    @adaptive_rho_tolerance.setter
    def adaptive_rho_tolerance(self, arg: float, /) -> None: ...

    @property
    def max_iter(self) -> int: ...

    @max_iter.setter
    def max_iter(self, arg: int, /) -> None: ...

    @property
    def eps_abs(self) -> float: ...

    @eps_abs.setter
    def eps_abs(self, arg: float, /) -> None: ...

    @property
    def eps_rel(self) -> float: ...

    @eps_rel.setter
    def eps_rel(self, arg: float, /) -> None: ...

    @property
    def eps_prim_inf(self) -> float: ...

    @eps_prim_inf.setter
    def eps_prim_inf(self, arg: float, /) -> None: ...

    @property
    def eps_dual_inf(self) -> float: ...

    @eps_dual_inf.setter
    def eps_dual_inf(self, arg: float, /) -> None: ...

    @property
    def scaled_termination(self) -> int: ...

    @scaled_termination.setter
    def scaled_termination(self, arg: int, /) -> None: ...

    @property
    def check_termination(self) -> int: ...

    @check_termination.setter
    def check_termination(self, arg: int, /) -> None: ...

    @property
    def check_dualgap(self) -> int: ...

    @check_dualgap.setter
    def check_dualgap(self, arg: int, /) -> None: ...

    @property
    def time_limit(self) -> float: ...

    @time_limit.setter
    def time_limit(self, arg: float, /) -> None: ...

    @property
    def delta(self) -> float: ...

    @delta.setter
    def delta(self, arg: float, /) -> None: ...

    @property
    def polish_refine_iter(self) -> int: ...

    @polish_refine_iter.setter
    def polish_refine_iter(self, arg: int, /) -> None: ...

    def __eq__(self, arg: OSQPSettings, /) -> bool: ...

    def __ne__(self, arg: OSQPSettings, /) -> bool: ...

    __hash__: None = None

class TrajOptOSQPSolverProfile(TrajOptSolverProfile):
    def __init__(self) -> None: ...

    @property
    def settings(self) -> OSQPSettings:
        """
        OSQP settings; trajopt's defaults (sco::OSQPModelConfig::setDefaultOSQPSettings)
        """

    @settings.setter
    def settings(self, arg: OSQPSettings, /) -> None: ...

    @property
    def update_workspace(self) -> bool:
        """
        Update the OSQP workspace for subsequent optimizations, instead of recreating it each time
        """

    @update_workspace.setter
    def update_workspace(self, arg: bool, /) -> None: ...

    def __eq__(self, arg: TrajOptOSQPSolverProfile, /) -> bool: ...

    def __ne__(self, arg: TrajOptOSQPSolverProfile, /) -> bool: ...

    __hash__: None = None

    @staticmethod
    def from_yaml(config: str) -> TrajOptOSQPSolverProfile:
        """
        Construct the profile from its YAML config text; malformed YAML raises ValueError.
        """

def ProfileDictionary_addTrajOptMoveProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptMoveProfile) -> None:
    """Add TrajOpt move profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptPlanProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptMoveProfile) -> None:
    """Add TrajOpt plan profile to ProfileDictionary (legacy alias)"""

def ProfileDictionary_addTrajOptCompositeProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptCompositeProfile) -> None:
    """Add TrajOpt composite profile to ProfileDictionary"""

def ProfileDictionary_addTrajOptSolverProfile(dict: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary, ns: str, profile_name: str, profile: TrajOptSolverProfile) -> None:
    """Add TrajOpt solver profile to ProfileDictionary"""

class TrajOptMotionPlanner(tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners.MotionPlanner):
    def __init__(self, name: str) -> None: ...
