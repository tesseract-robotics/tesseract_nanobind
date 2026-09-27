"""What a Python SQPCallback receives: the solver's own problem, and a snapshot of the results.

The trampoline used to cast both arguments of execute() with nanobind's policy for const&
arguments, which is a copy. For a problem without a copy constructor (TrajOptQPProblem) the copy
aborted the process; for IfoptQPProblem every callback got a detached deep copy of the whole QP
problem on every trial. The problem now arrives by reference: it is the Python object that was
passed to solve(). The results are still copied per call, so a callback may keep them.

The runs that aborted before the fix happen in a child process, so an abort fails one test
instead of the whole session.
"""

from __future__ import annotations

import re
import subprocess
import sys

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

N_NODES = 4
PINNED_NODE = 2
SEED = 1.0  # rad, every node's initial joint value
TARGET = 2.0  # rad, where the squared pin cost pulls the pinned node
JOINT_LIMIT = 10.0  # rad, symmetric joint bound
# The pin sits 1 rad from the seed and the initial trust box is 0.1 rad, so the solve needs
# several trials: at least two results to compare.
MIN_TRIALS = 2

_CHILD_PROBLEM = f"""\
import numpy as np
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

nodes = ti.createNodesVariables(
    "trajectory",
    ["j0"],
    [np.array([{SEED}])] * {N_NODES},
    ti.toBounds(np.array([[-{JOINT_LIMIT}, {JOINT_LIMIT}]])),
)
pin = ti.JointPosConstraint(
    np.array([{TARGET}]), nodes.getNodes()[{PINNED_NODE}].getVar("joints"), np.ones(1), "pin"
)
qp = tsqp.TrajOptQPProblem(nodes)
qp.addCostSet(pin, tsqp.CostPenaltyType.SQUARED)
qp.setup()
solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
"""

_CHILD_TRIVIAL_CALLBACK = (
    _CHILD_PROBLEM
    + """\

class Continue(tsqp.SQPCallback):
    def execute(self, problem, sqp_results):
        return True

solver.registerCallback(Continue())
solver.solve(qp)
print("SOLVE COMPLETED:", solver.getStatus().name)
"""
)

_CHILD_IDENTITY = (
    _CHILD_PROBLEM
    + """\

class Identity(tsqp.SQPCallback):
    def __init__(self):
        super().__init__()
        self.same = []

    def execute(self, problem, sqp_results):
        self.same.append(problem is qp)
        return True

identity = Identity()
solver.registerCallback(identity)
solver.solve(qp)
print(f"IDENTITY calls={len(identity.same)} same={all(identity.same)}")
"""
)


def _run_child(script: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )


def _child_failure(proc: subprocess.CompletedProcess) -> str:
    return f"child died (rc={proc.returncode}, SIGABRT is -6): {proc.stderr[-500:]}"


class _Recorder(tsqp.SQPCallback):
    """Records what each execute() call receives; always lets the solve continue."""

    def __init__(self, solved_problem):
        super().__init__()
        self._solved_problem = solved_problem
        self.same_problem: list[bool] = []
        self.results: list = []
        self.new_var_vals_at_call: list[np.ndarray] = []

    def execute(self, problem, sqp_results) -> bool:
        self.same_problem.append(problem is self._solved_problem)
        self.results.append(sqp_results)
        self.new_var_vals_at_call.append(np.array(sqp_results.new_var_vals, copy=True))
        return True


def _solve_ifopt_problem_with_recorder() -> _Recorder:
    nodes = ti.createNodesVariables(
        "trajectory",
        ["j0"],
        [np.array([SEED])] * N_NODES,
        ti.toBounds(np.array([[-JOINT_LIMIT, JOINT_LIMIT]])),
    )
    pin = ti.JointPosConstraint(
        np.array([TARGET]), nodes.getNodes()[PINNED_NODE].getVar("joints"), np.ones(1), "pin"
    )
    qp = tsqp.IfoptQPProblem(tsqp.IfoptProblem(nodes))
    qp.addCostSet(pin, tsqp.CostPenaltyType.SQUARED)
    qp.setup()
    recorder = _Recorder(qp)
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.registerCallback(recorder)
    solver.solve(qp)
    return recorder


class TestSQPCallbackArguments:
    def test_a_callback_on_trajopt_qp_problem_lets_the_solve_complete(self):
        proc = _run_child(_CHILD_TRIVIAL_CALLBACK)
        assert proc.returncode == 0, _child_failure(proc)
        assert "SOLVE COMPLETED:" in proc.stdout

    def test_the_callback_receives_the_solved_ifopt_qp_problem(self):
        recorder = _solve_ifopt_problem_with_recorder()
        assert recorder.same_problem, "the callback was never called"
        assert all(recorder.same_problem), f"copies received: {recorder.same_problem}"

    def test_the_callback_receives_the_solved_trajopt_qp_problem(self):
        proc = _run_child(_CHILD_IDENTITY)
        assert proc.returncode == 0, _child_failure(proc)
        match = re.search(r"IDENTITY calls=(\d+) same=(True|False)", proc.stdout)
        assert match, proc.stdout
        assert int(match.group(1)) > 0, "the callback was never called"
        assert match.group(2) == "True", proc.stdout

    def test_kept_results_are_snapshots_of_their_trial(self):
        """Passes before and after the fix: results were and are copied per call."""
        recorder = _solve_ifopt_problem_with_recorder()
        assert len(recorder.results) >= MIN_TRIALS
        assert len(recorder.results) == len(recorder.new_var_vals_at_call)
        assert not np.array_equal(
            recorder.new_var_vals_at_call[0], recorder.new_var_vals_at_call[-1]
        ), "the trials did not move: the snapshot check would be vacuous"
        for kept, at_call in zip(recorder.results, recorder.new_var_vals_at_call):
            np.testing.assert_array_equal(np.array(kept.new_var_vals), at_call)
