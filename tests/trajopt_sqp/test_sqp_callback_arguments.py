"""What a Python SQPCallback receives: the solver's own problem, and a snapshot of the results.

The trampoline used to cast both arguments of execute() with nanobind's default
automatic_reference policy, which copies lvalue-reference arguments. TrajOptQPProblem has no
copy constructor, so the copy aborted the process. The problem now arrives by reference: it is
the Python object that was passed to solve(). The results are still copied per call, so a
callback may keep them.

Every check runs in a child process that keeps its solver alive through the check and prints
one sentinel line; the parent asserts exit 0, then the sentinel. An abort fails one test instead
of the session, and a regression to results by reference (a non-owning view of the solver's
results) fails the snapshot check cleanly instead of reading freed memory in the test process.
"""

from __future__ import annotations

import re
import subprocess
import sys

N_NODES = 4
PINNED_NODE = 2
SEED = 1.0  # rad, every node's initial joint value
TARGET = 2.0  # rad, where the squared pin cost pulls the pinned node
JOINT_LIMIT = 10.0  # rad, symmetric joint bound
# The pin sits 1 rad from the seed and the initial trust box is 0.1 rad, so the solve needs
# several trials: at least two results to compare.
MIN_TRIALS = 2
# s; a child imports the bindings and solves in about 1-2 s. A deadlock must fail the test, not
# hang the session, and 60 s leaves room for a loaded runner.
CHILD_TIMEOUT_S = 60.0

_PROBLEM = f"""\
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

_TRIVIAL_CALLBACK = (
    _PROBLEM
    + """
class Continue(tsqp.SQPCallback):
    def execute(self, problem, sqp_results):
        return True


solver.registerCallback(Continue())
solver.solve(qp)
print("SOLVE COMPLETED:", solver.getStatus().name)
"""
)

_IDENTITY = (
    _PROBLEM
    + """
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

_SNAPSHOT = (
    _PROBLEM
    + """
class Snapshots(tsqp.SQPCallback):
    def __init__(self):
        super().__init__()
        self.kept = []
        self.at_call = []

    def execute(self, problem, sqp_results):
        self.kept.append(sqp_results)
        self.at_call.append(np.array(sqp_results.new_var_vals, copy=True))
        return True


snapshots = Snapshots()
solver.registerCallback(snapshots)
solver.solve(qp)
# solver is still alive here, so even a live reference to its results would read valid memory.
moved = len(snapshots.at_call) > 1 and not np.array_equal(
    snapshots.at_call[0], snapshots.at_call[-1]
)
kept_match = all(
    np.array_equal(np.array(kept.new_var_vals), at_call)
    for kept, at_call in zip(snapshots.kept, snapshots.at_call)
)
print(f"SNAPSHOT trials={len(snapshots.kept)} moved={moved} kept_match={kept_match}")
"""
)


def _run_child(script: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
    )


def _sentinel(proc: subprocess.CompletedProcess, pattern: str) -> re.Match:
    """The child's sentinel match, after asserting that the child exited cleanly."""
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGABRT is -6): {proc.stderr[-500:]}"
    )
    match = re.search(pattern, proc.stdout)
    assert match, f"no sentinel in the child's stdout: {proc.stdout[-500:]}"
    return match


class TestSQPCallbackArguments:
    def test_a_callback_on_trajopt_qp_problem_lets_the_solve_converge(self):
        match = _sentinel(_run_child(_TRIVIAL_CALLBACK), r"SOLVE COMPLETED: (\w+)")
        assert match.group(1) == "NLP_CONVERGED"

    def test_the_callback_receives_the_solved_problem(self):
        match = _sentinel(_run_child(_IDENTITY), r"IDENTITY calls=(\d+) same=(True|False)")
        assert int(match.group(1)) > 0, "the callback was never called"
        assert match.group(2) == "True", "the callback received a copy of the problem"

    def test_kept_results_are_snapshots_of_their_trial(self):
        """Results were copied per call before the fix too; the fix must keep that."""
        match = _sentinel(
            _run_child(_SNAPSHOT),
            r"SNAPSHOT trials=(\d+) moved=(True|False) kept_match=(True|False)",
        )
        assert int(match.group(1)) >= MIN_TRIALS
        assert match.group(2) == "True", "the trials did not move: the check would be vacuous"
        assert match.group(3) == "True", "kept results changed after their call: not snapshots"
