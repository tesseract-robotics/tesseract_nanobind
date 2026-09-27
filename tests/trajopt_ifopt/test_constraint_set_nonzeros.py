"""A Python ConstraintSet reports a valid Jacobian non-zero hint.

getNonZeros() is a reservation hint: TrajOptQPProblem reserves storage for the sum of its cost
sets' hints. trajopt's default, -1, means "unset", and a Python set cannot change it (the member
is protected and the getter is not virtual), so a Python cost set on its own made the solver
reserve SIZE_MAX entries and raise "ValueError: vector". The trampoline gives every Python set
the hint 0, which the contract allows: a hint that is too small only costs reallocations.
"""

from __future__ import annotations

from tesseract_robotics import trajopt_ifopt as ti


class _PythonSet(ti.ConstraintSet):
    """A Python set that only constructs its base: the hint is the trampoline's."""

    def __init__(self):
        super().__init__("python_set", 1)


def test_a_python_constraint_set_reports_a_zero_non_zero_hint():
    assert _PythonSet().getNonZeros() == 0
