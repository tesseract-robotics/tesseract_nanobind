"""#330: every trajopt_sqp enumerator is the UPPER_SNAKE form of its C++ kCamelCase name."""

import re

import pytest

from tesseract_robotics import trajopt_sqp as tsqp

# C++ enumerators, trajopt_sqp/types.h (SQPStatus) and qp_solver.h (QPSolverStatus), trajopt 0.35.0
CPP_ENUMERATORS = {
    "SQPStatus": [
        "kRunning",
        "kConverged",
        "kIterationLimit",
        "kPenaltyIterationLimit",
        "kTimeLimit",
        "kQPSolveFailed",
        "kStoppedByCallback",
    ],
    "QPSolverStatus": ["kUninitialized", "kInitialized", "kFailed"],
}

OLD_NAMES = {
    "SQPStatus": ["NLP_CONVERGED", "OPT_TIME_LIMIT", "QP_SOLVER_ERROR", "CALLBACK_STOPPED"],
    "QPSolverStatus": ["UNITIALIZED", "QP_ERROR"],
}


def upper_snake(enumerator: str) -> str:
    """`kQPSolveFailed` -> `QP_SOLVE_FAILED`."""
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", "_", enumerator[1:]).upper()


def test_upper_snake():
    assert upper_snake("kQPSolveFailed") == "QP_SOLVE_FAILED"
    assert upper_snake("kStoppedByCallback") == "STOPPED_BY_CALLBACK"
    assert upper_snake("kPenaltyIterationLimit") == "PENALTY_ITERATION_LIMIT"


@pytest.mark.parametrize("enum_name", sorted(CPP_ENUMERATORS))
def test_member_names_are_the_cpp_names(enum_name):
    members = list(getattr(tsqp, enum_name).__members__)
    assert sorted(members) == sorted(upper_snake(e) for e in CPP_ENUMERATORS[enum_name])


@pytest.mark.parametrize(
    ("enum_name", "old"), [(e, n) for e, names in OLD_NAMES.items() for n in names]
)
def test_old_names_removed(enum_name, old):
    assert not hasattr(getattr(tsqp, enum_name), old)
