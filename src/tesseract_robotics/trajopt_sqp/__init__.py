# trajopt_sqp Python bindings - SQP solver for trajectory optimization

# IMPORTANT: Import ifopt first to register base classes before derived classes
from tesseract_robotics import ifopt as _ifopt  # noqa: F401

from ._trajopt_sqp import *  # noqa: F403

__all__ = [  # noqa: F405
    # Enums
    "ConstraintType",
    "CostPenaltyType",
    "SQPStatus",
    "QPSolverStatus",
    # Structs
    "SQPParameters",
    "SQPResults",
    # Classes - solvers
    "QPSolver",
    "OSQPEigenSolver",
    # Classes - problems
    "QPProblem",
    "IfoptQPProblem",
    # Classes - main solver
    "TrustRegionSQPSolver",
    # Classes - callbacks
    "SQPCallback",
    # ifopt types (Problem only - VariableSet/ConstraintSet are in ifopt module)
    "IfoptProblem",
]
