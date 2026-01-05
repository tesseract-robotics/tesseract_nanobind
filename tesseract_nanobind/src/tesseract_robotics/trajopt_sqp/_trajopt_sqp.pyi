from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.ifopt._ifopt


class ConstraintType(enum.Enum):
    """Type of constraint"""

    EQ = 0
    """Equality constraint"""

    INEQ = 1
    """Inequality constraint"""

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

    NLP_CONVERGED = 1
    """NLP successfully converged"""

    ITERATION_LIMIT = 2
    """Reached iteration limit"""

    PENALTY_ITERATION_LIMIT = 3
    """Reached penalty iteration limit"""

    OPT_TIME_LIMIT = 4
    """Reached time limit"""

    QP_SOLVER_ERROR = 5
    """QP solver failed"""

    CALLBACK_STOPPED = 6
    """Stopped by callback"""

class QPSolverStatus(enum.Enum):
    """Status of QP solver"""

    UNITIALIZED = 0
    """Solver not initialized"""

    INITIALIZED = 1
    """Solver initialized"""

    QP_ERROR = 2
    """QP solver error"""

class SQPParameters:
    """Parameters controlling SQP optimization"""

    def __init__(self) -> None: ...

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
    def max_iterations(self) -> int:
        """Max number of QP calls allowed (default: 50)"""

    @max_iterations.setter
    def max_iterations(self, arg: int, /) -> None: ...

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
    def best_var_vals(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def new_var_vals(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Variable values this iteration"""

    @new_var_vals.setter
    def new_var_vals(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

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
    def box_size(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def merit_error_coeffs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Coefficients weighting constraint violations"""

    @merit_error_coeffs.setter
    def merit_error_coeffs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def best_constraint_violations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Constraint violations for best solution (positive = violation)"""

    @best_constraint_violations.setter
    def best_constraint_violations(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def new_constraint_violations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Constraint violations this iteration"""

    @new_constraint_violations.setter
    def new_constraint_violations(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def best_approx_constraint_violations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified constraint violations for best solution"""

    @best_approx_constraint_violations.setter
    def best_approx_constraint_violations(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def new_approx_constraint_violations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified constraint violations this iteration"""

    @new_approx_constraint_violations.setter
    def new_approx_constraint_violations(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def best_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Cost values for best solution"""

    @best_costs.setter
    def best_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def new_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Cost values this iteration"""

    @new_costs.setter
    def new_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def best_approx_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified costs for best solution"""

    @best_approx_costs.setter
    def best_approx_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def new_approx_costs(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Convexified costs this iteration"""

    @new_approx_costs.setter
    def new_approx_costs(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

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

    def updateGradient(self, gradient: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> bool: ...

    def updateLowerBound(self, lower_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> bool: ...

    def updateUpperBound(self, upper_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> bool: ...

    def updateBounds(self, lower_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], upper_bound: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> bool: ...

class QPProblem:
    """Abstract base class for QP problems (convexified NLP)"""

    def addVariableSet(self, variable_set: tesseract_robotics.ifopt._ifopt.VariableSet) -> None:
        """Add a set of optimization variables"""

    def addConstraintSet(self, constraint_set: tesseract_robotics.ifopt._ifopt.ConstraintSet) -> None:
        """Add a set of constraints"""

    def addCostSet(self, constraint_set: tesseract_robotics.ifopt._ifopt.ConstraintSet, penalty_type: CostPenaltyType) -> None:
        """Add a cost term with specified penalty type"""

    def setup(self) -> None:
        """Setup the QP problem (call after adding all sets)"""

    def getVariableValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current optimization variable values"""

    def convexify(self) -> None:
        """Run the full convexification routine"""

    def evaluateTotalConvexCost(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> float:
        """Evaluate convexified cost at given point"""

    def evaluateConvexCosts(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Evaluate individual convexified costs"""

    def evaluateTotalExactCost(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> float:
        """Evaluate exact (non-convexified) cost"""

    def evaluateExactCosts(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Evaluate individual exact costs"""

    def getExactCosts(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current exact costs"""

    def evaluateConvexConstraintViolations(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Evaluate convexified constraint violations"""

    def evaluateExactConstraintViolations(self, var_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Evaluate exact constraint violations"""

    def getExactConstraintViolations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current exact constraint violations"""

    def scaleBoxSize(self, scale: float) -> None:
        """Uniformly scale the trust region box size"""

    def setBoxSize(self, box_size: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> None:
        """Set trust region box size"""

    def setConstraintMeritCoeff(self, merit_coeff: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> None:
        """Set constraint merit coefficients"""

    def getBoxSize(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get trust region box size"""

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

class IfoptQPProblem(QPProblem):
    """QP problem wrapper for ifopt::Problem (general NLP)"""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, nlp: IfoptProblem) -> None:
        """Construct from ifopt Problem"""

    def addVariableSet(self, variable_set: tesseract_robotics.ifopt._ifopt.VariableSet) -> None: ...

    def addConstraintSet(self, constraint_set: tesseract_robotics.ifopt._ifopt.ConstraintSet) -> None: ...

    def addCostSet(self, constraint_set: tesseract_robotics.ifopt._ifopt.ConstraintSet, penalty_type: CostPenaltyType) -> None: ...

    def setup(self) -> None: ...

    def convexify(self) -> None: ...

    def print(self) -> None: ...

class SQPCallback:
    """Base class for SQP optimization callbacks"""

    def __init__(self) -> None: ...

    def execute(self, problem: QPProblem, sqp_results: SQPResults) -> bool:
        """Called during SQP. Return false to stop optimization."""

class TrustRegionSQPSolver:
    """
    Trust region SQP solver for trajectory optimization.

    Key methods for real-time/incremental optimization:
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

        This is the key method for real-time/online planning.
        Returns True if QP solve converged (does not mean SQP converged).

        Typical usage:
          solver.init(problem)
          while solver.getStatus() == SQPStatus.RUNNING:
              solver.stepSQPSolver()
              # Check constraints, update environment, etc.
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

class IfoptProblem:
    """ifopt::Problem - generic NLP with variables, costs, constraints"""

    def __init__(self) -> None: ...

    def AddVariableSet(self, variable_set: tesseract_robotics.ifopt._ifopt.Component) -> None: ...

    def AddConstraintSet(self, constraint_set: tesseract_robotics.ifopt._ifopt.ConstraintSet) -> None: ...

    def AddCostSet(self, cost_set: tesseract_robotics.ifopt._ifopt.ConstraintSet) -> None: ...

    def GetNumberOfOptimizationVariables(self) -> int: ...

    def GetNumberOfConstraints(self) -> int: ...

    def GetVariableValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def PrintCurrent(self) -> None: ...
