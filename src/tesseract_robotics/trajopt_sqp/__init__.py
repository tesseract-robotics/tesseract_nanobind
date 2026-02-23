# trajopt_sqp Python bindings - SQP solver for trajectory optimization

# IMPORTANT: Import trajopt_ifopt first to register base classes before derived classes
from tesseract_robotics import trajopt_ifopt as _trajopt_ifopt  # noqa: F401

from ._trajopt_sqp import *  # noqa: F403

__all__ = [  # noqa: F405
    # Enums
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
    # trajopt_ifopt types
    "IfoptProblem",
]
