"""A Python constraint set's exception reaches the caller with its own type (#156).

`ConstraintSet`'s trampoline lives in `_trajopt_ifopt`; `TrajOptQPProblem` and
`TrustRegionSQPSolver` are bound in `_trajopt_sqp`. An exception raised by a Python override
crosses that module boundary as `nanobind::python_error`, and a caller must be able to tell a
defect in its own set (`IndexError` here) from trajopt's `std::runtime_error` (`RuntimeError`).
"""

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

N_NODES = 3
PINNED_INDEX = 1
SEED = 1.0  # rad, every node's initial joint value
JOINT_LIMIT = 10.0  # rad, symmetric joint bound


class _RaisingSet(ti.ConstraintSet):
    """A squared pin cost whose getValues raises IndexError."""

    def __init__(self):
        super().__init__("raising", 1)

    def getValues(self) -> np.ndarray:
        raise IndexError("raised by a Python getValues")

    def getBounds(self) -> list:
        return [ti.Bounds(0.0, 0.0)]

    def getJacobian(self):
        n = len(self.getVariables().getValues())
        return scipy.sparse.csr_matrix(([1.0], ([0], [PINNED_INDEX])), shape=(1, n))

    def update(self) -> int:
        return self.getRows()

    def getCoefficients(self) -> np.ndarray:
        return np.ones(1)


def _problem():
    nodes = ti.createNodesVariables(
        "trajectory",
        ["j0"],
        [np.array([SEED]) for _ in range(N_NODES)],
        ti.toBounds(np.array([[-JOINT_LIMIT, JOINT_LIMIT]])),
    )
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addCostSet(_RaisingSet(), tsqp.CostPenaltyType.SQUARED)
    problem.setup()
    return problem


def _solve(problem):
    tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver()).solve(problem)


def _init(problem):
    tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver()).init(problem)


@pytest.mark.parametrize(
    "call",
    [_solve, _init, tsqp.TrajOptQPProblem.convexify, tsqp.TrajOptQPProblem.getExactCosts],
    ids=["solve", "init", "convexify", "getExactCosts"],
)
def test_python_exception_keeps_its_type(call):
    with pytest.raises(IndexError, match="raised by a Python getValues"):
        call(_problem())
