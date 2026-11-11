"""trajopt_sqp Python bindings - SQP solver for trajectory optimization"""

from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray
import scipy.sparse

import tesseract_robotics.trajopt_ifopt._trajopt_ifopt


class QPProblemExpiredError(RuntimeError):
    """
    A QPProblemView used after the SQPCallback call it was passed to returned.
    """

class VariablesSizeError(ValueError):
    """QPProblem.setVariables with a vector that is not getNumNLPVars() long."""

class QPSizeError(ValueError):
    """
    QPSolver input whose size is not the (num_vars, num_cnts) of its init().
    """

class QPSolutionSizeError(ValueError):
    """A convex evaluator's var_vals that is not getNumQPVars() long."""

class NotConvexifiedError(ValueError):
    """A convex evaluator called before the problem's first convexify()."""

class CostPenaltyType(enum.Enum):
    """Penalty type for cost terms"""

    SQUARED = 0
    """Squared penalty (L2)"""

    ABSOLUTE = 1
    """Absolute penalty (L1)"""

    HINGE = 2
    """Hinge penalty"""

class SQPStatus(enum.Enum):
    """Status codes for SQP optimization"""

    RUNNING = 0
    """Optimization is currently running"""

    CONVERGED = 1
    """Optimization converged"""

    ITERATION_LIMIT = 2
    """Reached iteration limit"""

    PENALTY_ITERATION_LIMIT = 3
    """Reached penalty iteration limit"""

    TIME_LIMIT = 4
    """Reached time limit"""

    QP_SOLVE_FAILED = 5
    """QP solver failed"""

    STOPPED_BY_CALLBACK = 6
    """Stopped by callback"""

def toString(status: SQPStatus) -> str:
    """trajopt's text for a solver status (e.g. 'SQP_CONVERGED')"""

class QPSolverStatus(enum.Enum):
    """Status of QP solver"""

    UNINITIALIZED = 0
    """Solver not initialized"""

    INITIALIZED = 1
    """Solver initialized"""

    FAILED = 2
    """QP solver failed"""

class SQPParameters:
    """Parameters controlling SQP optimization"""

    def __init__(self) -> None: ...

    def __eq__(self, arg: SQPParameters, /) -> bool: ...

    def __ne__(self, arg: SQPParameters, /) -> bool: ...

    __hash__: None = None

    @property
    def improve_ratio_threshold(self) -> float:
        """
        Minimum ratio exact_improve/approx_improve to accept step (default: 0.25)
        """

    @improve_ratio_threshold.setter
    def improve_ratio_threshold(self, arg: float, /) -> None: ...

    @property
    def min_trust_box_size(self) -> float:
        """NLP converges if trust region smaller than this (default: 1e-4)"""

    @min_trust_box_size.setter
    def min_trust_box_size(self, arg: float, /) -> None: ...

    @property
    def min_approx_improve(self) -> float:
        """
        NLP converges if approx_merit_improve smaller than this (default: 1e-4)
        """

    @min_approx_improve.setter
    def min_approx_improve(self, arg: float, /) -> None: ...

    @property
    def min_approx_improve_frac(self) -> float:
        """NLP converges if approx_merit_improve/best_exact_merit < this"""

    @min_approx_improve_frac.setter
    def min_approx_improve_frac(self, arg: float, /) -> None: ...

    @property
    def max_iter(self) -> int:
        """
        Max convexifications per penalty iteration; rejected trust-region steps do not count (default: 50)
        """

    @max_iter.setter
    def max_iter(self, arg: int, /) -> None: ...

    @property
    def trust_shrink_ratio(self) -> float:
        """Trust region scale factor when shrinking (default: 0.1)"""

    @trust_shrink_ratio.setter
    def trust_shrink_ratio(self, arg: float, /) -> None: ...

    @property
    def trust_expand_ratio(self) -> float:
        """Trust region scale factor when expanding (default: 1.5)"""

    @trust_expand_ratio.setter
    def trust_expand_ratio(self, arg: float, /) -> None: ...

    @property
    def cnt_tolerance(self) -> float:
        """Constraint violation tolerance (default: 1e-4)"""

    @cnt_tolerance.setter
    def cnt_tolerance(self, arg: float, /) -> None: ...

    @property
    def max_merit_coeff_increases(self) -> float:
        """Max times constraints will be inflated (default: 5)"""

    @max_merit_coeff_increases.setter
    def max_merit_coeff_increases(self, arg: float, /) -> None: ...

    @property
    def max_qp_solver_failures(self) -> int:
        """Max QP solver failures before abort (default: 3)"""

    @max_qp_solver_failures.setter
    def max_qp_solver_failures(self, arg: int, /) -> None: ...

    @property
    def merit_coeff_increase_ratio(self) -> float:
        """Scale factor for constraint inflation (default: 10)"""

    @merit_coeff_increase_ratio.setter
    def merit_coeff_increase_ratio(self, arg: float, /) -> None: ...

    @property
    def max_time(self) -> float:
        """Max optimization time in seconds"""

    @max_time.setter
    def max_time(self, arg: float, /) -> None: ...

    @property
    def initial_merit_error_coeff(self) -> float:
        """Initial constraint scaling coefficient (default: 10)"""

    @initial_merit_error_coeff.setter
    def initial_merit_error_coeff(self, arg: float, /) -> None: ...

    @property
    def inflate_constraints_individually(self) -> bool:
        """If true, only violated constraints are inflated (default: true)"""

    @inflate_constraints_individually.setter
    def inflate_constraints_individually(self, arg: bool, /) -> None: ...

    @property
    def sum_constraint_set_violations(self) -> bool:
        """
        If true, cnt_tolerance is tested against a set's summed row violations; if false, against its largest row. Applied by TrustRegionSQPSolver.init() (default: true)
        """

    @sum_constraint_set_violations.setter
    def sum_constraint_set_violations(self, arg: bool, /) -> None: ...

    @property
    def initial_trust_box_size(self) -> float:
        """Initial trust region size (default: 0.1)"""

    @initial_trust_box_size.setter
    def initial_trust_box_size(self, arg: float, /) -> None: ...

    @property
    def log_results(self) -> bool:
        """Enable logging (unused)"""

    @log_results.setter
    def log_results(self, arg: bool, /) -> None: ...

    @property
    def log_dir(self) -> str:
        """Log directory (unused)"""

    @log_dir.setter
    def log_dir(self, arg: str, /) -> None: ...

class ConstraintViolations:
    """Per merit unit constraint violations (non-negative; 0 = satisfied)"""

    def __init__(self) -> None: ...

    @property
    def raw(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Unweighted violation per merit unit; its sum is compared against cnt_tolerance
        """

    @raw.setter
    def raw(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def weighted(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Violation times row weight; the merit charges weighted.dot(merit_error_coeffs)
        """

    @weighted.setter
    def weighted(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

class SQPResults:
    """Results and state from SQP optimization"""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, num_vars: int, num_cnts: int, num_costs: int) -> None: ...

    @property
    def best_exact_merit(self) -> float:
        """Lowest cost ever achieved"""

    @best_exact_merit.setter
    def best_exact_merit(self, arg: float, /) -> None: ...

    @property
    def new_exact_merit(self) -> float:
        """Cost achieved this iteration"""

    @new_exact_merit.setter
    def new_exact_merit(self, arg: float, /) -> None: ...

    @property
    def best_approx_merit(self) -> float:
        """Lowest convexified cost ever achieved"""

    @best_approx_merit.setter
    def best_approx_merit(self, arg: float, /) -> None: ...

    @property
    def new_approx_merit(self) -> float:
        """Convexified cost this iteration"""

    @new_approx_merit.setter
    def new_approx_merit(self, arg: float, /) -> None: ...

    @property
    def best_var_vals(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Variable values for best_exact_merit"""

    @best_var_vals.setter
    def best_var_vals(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def new_var_vals(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Variable values this iteration"""

    @new_var_vals.setter
    def new_var_vals(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def approx_merit_improve(self) -> float:
        """Convexified cost improvement this iteration"""

    @approx_merit_improve.setter
    def approx_merit_improve(self, arg: float, /) -> None: ...

    @property
    def exact_merit_improve(self) -> float:
        """Exact cost improvement this iteration"""

    @exact_merit_improve.setter
    def exact_merit_improve(self, arg: float, /) -> None: ...

    @property
    def merit_improve_ratio(self) -> float:
        """Cost improvement as ratio of total cost"""

    @merit_improve_ratio.setter
    def merit_improve_ratio(self, arg: float, /) -> None: ...

    @property
    def box_size(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Trust region box size (var_vals +/- box_size)"""

    @box_size.setter
    def box_size(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def merit_error_coeffs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Coefficients weighting constraint violations"""

    @merit_error_coeffs.setter
    def merit_error_coeffs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def best_constraint_violations(self) -> ConstraintViolations:
        """Constraint violations for best solution (positive = violation)"""

    @best_constraint_violations.setter
    def best_constraint_violations(self, arg: ConstraintViolations, /) -> None: ...

    @property
    def new_constraint_violations(self) -> ConstraintViolations:
        """Constraint violations this iteration"""

    @new_constraint_violations.setter
    def new_constraint_violations(self, arg: ConstraintViolations, /) -> None: ...

    @property
    def best_approx_constraint_violations(self) -> ConstraintViolations:
        """Convexified constraint violations for best solution"""

    @best_approx_constraint_violations.setter
    def best_approx_constraint_violations(self, arg: ConstraintViolations, /) -> None: ...

    @property
    def new_approx_constraint_violations(self) -> ConstraintViolations:
        """Convexified constraint violations this iteration"""

    @new_approx_constraint_violations.setter
    def new_approx_constraint_violations(self, arg: ConstraintViolations, /) -> None: ...

    @property
    def best_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Cost values for best solution"""

    @best_costs.setter
    def best_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def new_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Cost values this iteration"""

    @new_costs.setter
    def new_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def best_approx_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified costs for best solution"""

    @best_approx_costs.setter
    def best_approx_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def new_approx_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified costs this iteration"""

    @new_approx_costs.setter
    def new_approx_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def constraint_names(self) -> list[str]:
        """Names of constraint sets"""

    @constraint_names.setter
    def constraint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def cost_names(self) -> list[str]:
        """Names of cost terms"""

    @cost_names.setter
    def cost_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def penalty_iteration(self) -> int: ...

    @penalty_iteration.setter
    def penalty_iteration(self, arg: int, /) -> None: ...

    @property
    def convexify_iteration(self) -> int: ...

    @convexify_iteration.setter
    def convexify_iteration(self, arg: int, /) -> None: ...

    @property
    def trust_region_iteration(self) -> int: ...

    @trust_region_iteration.setter
    def trust_region_iteration(self, arg: int, /) -> None: ...

    @property
    def overall_iteration(self) -> int: ...

    @overall_iteration.setter
    def overall_iteration(self, arg: int, /) -> None: ...

    def print(self) -> None:
        """Print results to console"""

class QPSolver:
    """Abstract base class for QP solvers"""

    def init(self, num_vars: int, num_cnts: int) -> bool:
        """Initialize the QP solver"""

    def clear(self) -> bool:
        """Clear the QP solver"""

    def solve(self) -> bool:
        """Solve the QP"""

    def getSolution(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the solution vector"""

    def getSolverStatus(self) -> QPSolverStatus:
        """Get solver status"""

    def updateHessianMatrix(self, hessian: scipy.sparse.csr_matrix[float]) -> bool:
        """
        Set the Hessian H (sparse, num_vars square) of the objective x'Hx + g'x
        """

    def updateGradient(self, gradient: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> bool:
        """Set the gradient g (num_vars)"""

    def updateLowerBound(self, lower_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> bool:
        """Set the constraint lower bounds (num_cnts)"""

    def updateUpperBound(self, upper_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> bool:
        """Set the constraint upper bounds (num_cnts)"""

    def updateBounds(self, lower_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], upper_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> bool:
        """Set both constraint bounds (num_cnts each)"""

    def updateLinearConstraintsMatrix(self, linear_constraints_matrix: scipy.sparse.csr_matrix[float]) -> bool:
        """Set the constraint matrix A (sparse, num_cnts x num_vars)"""

    def setWarmStart(self, qp_problem: QPProblem) -> bool:
        """
        Seed the next solve from the problem's variables (and its slacks); a no-op while the solver's warm-start option is off
        """

    @property
    def verbosity(self) -> int:
        """Verbosity level (0 = silent)"""

    @verbosity.setter
    def verbosity(self, arg: int, /) -> None: ...

class OSQPEigenSolver(QPSolver):
    """OSQP-based QP solver"""

    def __init__(self) -> None: ...

    def init(self, num_vars: int, num_cnts: int) -> bool: ...

    def clear(self) -> bool: ...

    def solve(self) -> bool: ...

    def getSolution(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def getSolverStatus(self) -> QPSolverStatus: ...

    @overload
    def setWarmStart(self, qp_problem: QPProblem) -> bool:
        """
        Seed the next solve from the problem's variables (and its slacks); a no-op while warm-starting is off
        """

    @overload
    def setWarmStart(self, warm_start: bool) -> None:
        """
        Warm-start each QP solve from the previous solution (OSQP `warm_starting`).
        """

    def setPolish(self, polish: bool) -> None:
        """Polish the ADMM solution (OSQP `polishing`)."""

    def getPolish(self) -> bool: ...

    def getWarmStart(self) -> bool: ...

    def setAdaptiveRho(self, adaptive_rho: bool) -> None:
        """Adapt the ADMM step size rho (OSQP `adaptive_rho`)."""

    def getAdaptiveRho(self) -> bool: ...

    def setAdaptiveRhoInterval(self, interval: int) -> None:
        """
        Adapt rho every `interval` iterations. OSQP's 0 adapts on wall-clock timing, which makes solutions differ run to run; a fixed interval (e.g. 25) makes them reproducible.
        """

    def getAdaptiveRhoInterval(self) -> int: ...

    def setMaxIteration(self, max_iter: int) -> None:
        """Maximum ADMM iterations per QP solve."""

    def getMaxIteration(self) -> int: ...

    def setAbsoluteTolerance(self, abs_tol: float) -> None:
        """Absolute convergence tolerance (OSQP `eps_abs`)."""

    def getAbsoluteTolerance(self) -> float: ...

    def setRelativeTolerance(self, rel_tol: float) -> None:
        """Relative convergence tolerance (OSQP `eps_rel`)."""

    def getRelativeTolerance(self) -> float: ...

    def setVerbosity(self, verbose: bool) -> None:
        """Write OSQP's progress to the console."""

    def getVerbosity(self) -> bool: ...

class QPProblem:
    """Abstract base class for QP problems (convexified NLP)"""

    def addConstraintSet(self, constraint_set: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet) -> None:
        """Add a set of constraints"""

    def addCostSet(self, constraint_set: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet, penalty_type: CostPenaltyType) -> None:
        """Add a cost term with specified penalty type"""

    def setup(self) -> None:
        """Setup the QP problem (call after adding all sets)"""

    def getVariableValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current optimization variable values"""

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None:
        """
        Set the NLP variables and update every component (a TrajOptQPProblem also recounts its dynamic rows). Raises VariablesSizeError unless len(x) == getNumNLPVars().
        """

    def convexify(self) -> None:
        """Run the full convexification routine"""

    def evaluateTotalConvexCost(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> float:
        """
        Evaluate the convexified total cost at var_vals.

        Args:
            var_vals: A point of the QP built by the last convexify(): getNumQPVars()
                entries, the NLP variables followed by the slack variables, as in a QP
                solution. To evaluate at an NLP point, append zeros for the slacks.

        Returns:
            The sum of evaluateConvexCosts(var_vals).

        Raises:
            QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.
            NotConvexifiedError: before the first convexify().
        """

    def evaluateConvexCosts(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Evaluate each cost term of the convexified problem at var_vals.

        Args:
            var_vals: A point of the QP built by the last convexify(): getNumQPVars()
                entries, the NLP variables followed by the slack variables, as in a QP
                solution. To evaluate at an NLP point, append zeros for the slacks.

        Returns:
            One convexified cost per cost term, in getNLPCostNames() order.

        Raises:
            QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.
            NotConvexifiedError: before the first convexify().
        """

    def getTotalExactCost(self) -> float:
        """Get exact (non-convexified) total cost"""

    def getExactCosts(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current exact costs"""

    def evaluateConvexConstraintViolations(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> ConstraintViolations:
        """
        Evaluate the convexified constraint violations at var_vals.

        Args:
            var_vals: A point of the QP built by the last convexify(): getNumQPVars()
                entries, the NLP variables followed by the slack variables, as in a QP
                solution. To evaluate at an NLP point, append zeros for the slacks.

        Returns:
            One violation per constraint set, in getNLPConstraintNames() order; 0 where
            satisfied.

        Raises:
            QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.
            NotConvexifiedError: before the first convexify().
        """

    def getExactConstraintViolations(self) -> ConstraintViolations:
        """Get current exact constraint violations"""

    def setSumConstraintSetViolations(self, sum: bool) -> None:
        """
        Reduce a merit unit's row violations to their sum (True) or to the largest (False) in ConstraintViolations.raw. TrustRegionSQPSolver.init() overwrites this with SQPParameters.sum_constraint_set_violations.
        """

    def scaleBoxSize(self, scale: float) -> None:
        """Uniformly scale the trust region box size"""

    def setBoxSize(self, box_size: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None:
        """Set trust region box size"""

    def setConstraintMeritCoeff(self, merit_coeff: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None:
        """Set constraint merit coefficients"""

    def getBoxSize(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get trust region box size"""

    def getConstraintMeritCoeff(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the constraint merit coefficients, one per constraint set"""

    def getHessian(self) -> scipy.sparse.csr_matrix[float]:
        """
        The QP Hessian (sparse, getNumQPVars() square), from the last convexify()
        """

    def getGradient(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The QP gradient (getNumQPVars()), from the last convexify()"""

    def getConstraintMatrix(self) -> scipy.sparse.csr_matrix[float]:
        """
        The QP constraint matrix (sparse, getNumQPConstraints() x getNumQPVars()), from the last convexify()
        """

    def getBoundsLower(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        The QP constraint lower bounds (getNumQPConstraints()), from the last convexify()
        """

    def getBoundsUpper(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        The QP constraint upper bounds (getNumQPConstraints()), from the last convexify()
        """

    def print(self) -> None:
        """Print problem to console"""

    def getNumNLPVars(self) -> int:
        """Number of NLP variables"""

    def getNumNLPConstraints(self) -> int:
        """Number of NLP constraints"""

    def getNumNLPCosts(self) -> int:
        """Number of NLP cost terms"""

    def getNumQPVars(self) -> int:
        """Number of QP variables (includes slack)"""

    def getNumQPConstraints(self) -> int:
        """Number of QP constraints"""

    def getNLPConstraintNames(self) -> list[str]:
        """Get constraint names"""

    def getNLPCostNames(self) -> list[str]:
        """Get cost names"""

class TrajOptQPProblem(QPProblem):
    """
    QP problem over trajopt_ifopt variables: the problem tesseract_planning's
    TrajOpt-Ifopt planner builds. Costs and constraints are added directly.
    """

    def __init__(self, variables: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Variables) -> None:
        """
        Construct over the optimization variables (e.g. createNodesVariables(...))
        """

    def addConstraintSet(self, constraint_set: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet) -> None: ...

    def addCostSet(self, constraint_set: tesseract_robotics.trajopt_ifopt._trajopt_ifopt.ConstraintSet, penalty_type: CostPenaltyType) -> None: ...

    def setup(self) -> None: ...

    def convexify(self) -> None: ...

    def print(self) -> None: ...

class QPProblemView(QPProblem):
    """
    The problem an SQPCallback receives from a solve whose problem has no Python object (one a
    planner built in C++). Valid only during that call; afterwards every method raises
    QPProblemExpiredError.
    """

class SQPCallback:
    """Base class for SQP optimization callbacks"""

    def __init__(self) -> None: ...

    def execute(self, problem: QPProblem, sqp_results: SQPResults) -> bool:
        """
        Called during SQP. Return false to stop optimization. problem is valid only during the call: a problem passed to solve() from Python arrives as that object; one a planner built in C++ arrives as a QPProblemView, which raises QPProblemExpiredError once the call has returned. sqp_results is a copy the callback may keep.
        """

class PIQPKKTSolver(enum.Enum):
    """PIQP's KKT system solver"""

    dense_cholesky = 0

    sparse_ldlt = 1

    sparse_ldlt_eq_cond = 2

    sparse_ldlt_ineq_cond = 3

    sparse_ldlt_cond = 4

    sparse_multistage = 5

class PIQPSettings:
    """PIQP solver settings (piqp::Settings<double>)"""

    def __init__(self) -> None:
        """PIQP's own defaults"""

    @property
    def rho_init(self) -> float:
        """Initial primal proximal penalty"""

    @rho_init.setter
    def rho_init(self, arg: float, /) -> None: ...

    @property
    def delta_init(self) -> float:
        """Initial augmented Lagrangian penalty"""

    @delta_init.setter
    def delta_init(self, arg: float, /) -> None: ...

    @property
    def eps_abs(self) -> float:
        """Absolute tolerance"""

    @eps_abs.setter
    def eps_abs(self, arg: float, /) -> None: ...

    @property
    def eps_rel(self) -> float:
        """Relative tolerance"""

    @eps_rel.setter
    def eps_rel(self, arg: float, /) -> None: ...

    @property
    def check_duality_gap(self) -> bool:
        """Also stop on the duality gap"""

    @check_duality_gap.setter
    def check_duality_gap(self, arg: bool, /) -> None: ...

    @property
    def eps_duality_gap_abs(self) -> float:
        """Absolute duality gap tolerance"""

    @eps_duality_gap_abs.setter
    def eps_duality_gap_abs(self, arg: float, /) -> None: ...

    @property
    def eps_duality_gap_rel(self) -> float:
        """Relative duality gap tolerance"""

    @eps_duality_gap_rel.setter
    def eps_duality_gap_rel(self, arg: float, /) -> None: ...

    @property
    def infeasibility_threshold(self) -> float:
        """Threshold for the infeasibility detection"""

    @infeasibility_threshold.setter
    def infeasibility_threshold(self, arg: float, /) -> None: ...

    @property
    def reg_lower_limit(self) -> float:
        """Lower limit of the regularization"""

    @reg_lower_limit.setter
    def reg_lower_limit(self, arg: float, /) -> None: ...

    @property
    def reg_finetune_lower_limit(self) -> float:
        """Fine-tune lower limit of the regularization"""

    @reg_finetune_lower_limit.setter
    def reg_finetune_lower_limit(self, arg: float, /) -> None: ...

    @property
    def reg_finetune_primal_update_threshold(self) -> int:
        """Primal updates before fine-tuning the regularization"""

    @reg_finetune_primal_update_threshold.setter
    def reg_finetune_primal_update_threshold(self, arg: int, /) -> None: ...

    @property
    def reg_finetune_dual_update_threshold(self) -> int:
        """Dual updates before fine-tuning the regularization"""

    @reg_finetune_dual_update_threshold.setter
    def reg_finetune_dual_update_threshold(self, arg: int, /) -> None: ...

    @property
    def max_iter(self) -> int:
        """Maximum number of iterations"""

    @max_iter.setter
    def max_iter(self, arg: int, /) -> None: ...

    @property
    def max_factor_retires(self) -> int:
        """Maximum number of factorization retries"""

    @max_factor_retires.setter
    def max_factor_retires(self, arg: int, /) -> None: ...

    @property
    def preconditioner_scale_cost(self) -> bool:
        """Scale the cost in the preconditioner"""

    @preconditioner_scale_cost.setter
    def preconditioner_scale_cost(self, arg: bool, /) -> None: ...

    @property
    def preconditioner_reuse_on_update(self) -> bool:
        """Reuse the preconditioner when the problem is updated"""

    @preconditioner_reuse_on_update.setter
    def preconditioner_reuse_on_update(self, arg: bool, /) -> None: ...

    @property
    def preconditioner_iter(self) -> int:
        """Preconditioner iterations"""

    @preconditioner_iter.setter
    def preconditioner_iter(self, arg: int, /) -> None: ...

    @property
    def tau(self) -> float:
        """Maximum step size fraction to the boundary"""

    @tau.setter
    def tau(self, arg: float, /) -> None: ...

    @property
    def kkt_solver(self) -> PIQPKKTSolver:
        """KKT system solver"""

    @kkt_solver.setter
    def kkt_solver(self, arg: PIQPKKTSolver, /) -> None: ...

    @property
    def iterative_refinement_always_enabled(self) -> bool:
        """Always run iterative refinement"""

    @iterative_refinement_always_enabled.setter
    def iterative_refinement_always_enabled(self, arg: bool, /) -> None: ...

    @property
    def iterative_refinement_eps_abs(self) -> float:
        """Iterative refinement absolute tolerance"""

    @iterative_refinement_eps_abs.setter
    def iterative_refinement_eps_abs(self, arg: float, /) -> None: ...

    @property
    def iterative_refinement_eps_rel(self) -> float:
        """Iterative refinement relative tolerance"""

    @iterative_refinement_eps_rel.setter
    def iterative_refinement_eps_rel(self, arg: float, /) -> None: ...

    @property
    def iterative_refinement_max_iter(self) -> int:
        """Iterative refinement maximum iterations"""

    @iterative_refinement_max_iter.setter
    def iterative_refinement_max_iter(self, arg: int, /) -> None: ...

    @property
    def iterative_refinement_min_improvement_rate(self) -> float:
        """Iterative refinement minimum improvement rate"""

    @iterative_refinement_min_improvement_rate.setter
    def iterative_refinement_min_improvement_rate(self, arg: float, /) -> None: ...

    @property
    def iterative_refinement_static_regularization_eps(self) -> float:
        """Iterative refinement static regularization"""

    @iterative_refinement_static_regularization_eps.setter
    def iterative_refinement_static_regularization_eps(self, arg: float, /) -> None: ...

    @property
    def iterative_refinement_static_regularization_rel(self) -> float:
        """Iterative refinement relative static regularization"""

    @iterative_refinement_static_regularization_rel.setter
    def iterative_refinement_static_regularization_rel(self, arg: float, /) -> None: ...

    @property
    def verbose(self) -> bool:
        """Print solver progress"""

    @verbose.setter
    def verbose(self, arg: bool, /) -> None: ...

    @property
    def compute_timings(self) -> bool:
        """Measure the solver timings"""

    @compute_timings.setter
    def compute_timings(self, arg: bool, /) -> None: ...

    def __eq__(self, arg: PIQPSettings, /) -> bool: ...

    def __ne__(self, arg: PIQPSettings, /) -> bool: ...

    __hash__: None = None

class PIQPSolver(QPSolver):
    """PIQP-based (interior point) QP solver"""

    def __init__(self) -> None:
        """With trajopt's default PIQP settings"""

    @property
    def settings(self) -> PIQPSettings:
        """PIQP settings, applied when the solver is next set up"""

    @settings.setter
    def settings(self, arg: PIQPSettings, /) -> None: ...

    @staticmethod
    def defaultSettings() -> PIQPSettings:
        """The PIQP settings trajopt uses by default"""

    def reusedSetup(self) -> bool:
        """
        Whether the last solve reused the previous setup (only the vectors changed)
        """

class TrustRegionSQPSolver:
    """
    Trust region SQP solver for trajectory optimization.

    Key methods for incremental optimization:
      - init(qp_prob): Initialize solver with problem
      - stepSQPSolver(): Run ONE SQP iteration (for real-time control)
      - getResults(): Get current optimization results
      - setBoxSize(): Control trust region size
    """

    def __init__(self, qp_solver: QPSolver) -> None:
        """Construct with a QP solver (e.g., OSQPEigenSolver)"""

    def init(self, qp_prob: QPProblem) -> bool:
        """Initialize the solver with a QP problem for incremental solving"""

    def solve(self, qp_prob: QPProblem) -> None:
        """Run complete SQP optimization"""

    def stepSQPSolver(self) -> bool:
        """
        Run a SINGLE SQP convexification step.

        Used for online planning, where the caller updates the problem between steps.

        Returns True when no further step should follow at the current penalty: the step
        converged, the QP failure budget was spent, or a callback stopped the solve;
        getStatus() tells which. It does not mean the SQP solver has converged.

        Example:
        ```python
        solver.init(problem)
        while solver.getStatus() == SQPStatus.RUNNING:
            solver.stepSQPSolver()
            # Check constraints, update environment, etc.
        ```
        """

    def verifySQPSolverConvergence(self) -> bool:
        """Check if SQP constraints are satisfied"""

    def adjustPenalty(self) -> None:
        """
        Increase penalty on constraints when SQP reports convergence but constraints violated
        """

    def runTrustRegionLoop(self) -> None:
        """Run trust region loop (adjusts box size)"""

    def solveQPProblem(self) -> SQPStatus:
        """Solve current QP problem, store results, call callbacks"""

    def setBoxSize(self, box_size: float) -> None:
        """Set trust region box size (for online planning control)"""

    def callCallbacks(self) -> bool:
        """
        Call all registered callbacks. Returns false if any callback returned false.
        """

    def printStepInfo(self) -> None:
        """Print info about current optimization state"""

    def registerCallback(self, callback: SQPCallback) -> None:
        """Register an optimization callback"""

    def getStatus(self) -> SQPStatus:
        """Get current SQP status"""

    def getResults(self) -> SQPResults:
        """Get current SQP results"""

    @property
    def verbose(self) -> bool:
        """If true, print debug info to console"""

    @verbose.setter
    def verbose(self, arg: bool, /) -> None: ...

    @property
    def params(self) -> SQPParameters:
        """SQP parameters (modify before calling init/solve)"""

    @params.setter
    def params(self, arg: SQPParameters, /) -> None: ...

    @property
    def qp_solver(self) -> QPSolver:
        """The QP solver used internally"""

    @qp_solver.setter
    def qp_solver(self, arg: QPSolver, /) -> None: ...

    @property
    def qp_problem(self) -> QPProblem:
        """The current QP problem"""

    @qp_problem.setter
    def qp_problem(self, arg: QPProblem, /) -> None: ...
