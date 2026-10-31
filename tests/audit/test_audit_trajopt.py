"""Contract tests of the audit's group 2 mechanisms (`trajopt_ifopt`, `trajopt_sqp`; #256).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")
parse = load("audit_parse")

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
SOLVER_PREFIX = ("tesseract/solver/",)
# The fixture's third-party root, as `header_rel` spells it (relative to the repo).
FIXTURE_THIRD_PARTY = "tests/audit/fixtures/include/thirdparty_fixture/"


@pytest.fixture(scope="module")
def solver_report():
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(parse, "UNBOUND_DEPENDENCY_ROOTS", (FIXTURE_THIRD_PARTY,))
        return audit.audit_tu(
            "solver",
            FIXTURES / "solver_bindings.cpp",
            FIXTURES / "_solver.pyi",
            FIXTURES / "solver_init.py",
            SOLVER_PREFIX,
            (FIXTURE_INCLUDE,),
            stub_roots=(FIXTURES,),
        )


# T11: compound-assignment operators get dunders.


def test_inplace_operators_are_dunders():
    assert audit.OPERATOR_DUNDERS["operator+="] == "__iadd__"
    assert audit.OPERATOR_DUNDERS["operator-="] == "__isub__"


def test_bound_inplace_operators_cover_the_cpp_operators(solver_report):
    rows = {g.symbol for g in solver_report.gaps} | {d.name for d in solver_report.deviations}
    assert not rows & {"Shift.__iadd__", "Shift.__isub__", "Shift.operator+=", "Shift.operator-="}


# T10 + R15: `k-prefix-enumerator`.


@pytest.mark.parametrize(
    ("leaf", "python"),
    [
        ("kSquared", "SQUARED"),
        ("kLowerBound", "LOWER_BOUND"),
        ("kQPSolveFailed", "QP_SOLVE_FAILED"),
        ("kSplitToTwoInequalities", "SPLIT_TO_TWO_INEQUALITIES"),
        ("kStackRows", "STACK_ROWS"),
        ("kSE3", "SE3"),
        ("kConverged", "CONVERGED"),
    ],
)
def test_k_enumerator_python_name(leaf, python):
    assert audit.k_enumerator_python_name(leaf) == python


@pytest.mark.parametrize("leaf", ["RUNNING", "key", "k", "kx", "k_lower", "ALL"])
def test_k_enumerator_python_name_rejects_other_spellings(leaf):
    assert audit.k_enumerator_python_name(leaf) is None


def test_k_prefix_accepts_both_sides_of_a_mechanical_pair(solver_report):
    accepted = {(a.symbol, a.rule) for a in solver_report.accepted}
    for pair in (
        ("Status.kRunning", "Status.RUNNING"),
        ("Status.kQPSolveFailed", "Status.QP_SOLVE_FAILED"),
    ):
        for symbol in pair:
            assert (symbol, "k-prefix-enumerator") in accepted


def test_k_prefix_rejects_non_mechanical_names(solver_report):
    assert "Status.kConverged" in {g.symbol for g in solver_report.gaps}
    assert "Status.NLP_CONVERGED" in {d.name for d in solver_report.deviations}


# R13: `unbound-dependency-type`, third-party half.


# The 10 core modules and the group 1 modules: no binding of theirs includes OsqpEigen.
CORE_AND_G1 = (
    "tesseract_collision",
    "tesseract_common",
    "tesseract_environment",
    "tesseract_geometry",
    "tesseract_kinematics",
    "tesseract_scene_graph",
    "tesseract_serialization",
    "tesseract_srdf",
    "tesseract_state_solver",
    "tesseract_urdf",
    "tesseract_command_language",
    "tesseract_time_parameterization",
)


@pytest.fixture(scope="module")
def core_and_g1_reports():
    """The core and group 1 reports: R13 must not change any of their rows."""
    return {m: audit.audit_module(m) for m in CORE_AND_G1}


def test_third_party_typed_members_are_accepted(solver_report):
    accepted = {(a.symbol, a.rule) for a in solver_report.accepted}
    assert ("Engine.setDefaults", "unbound-dependency-type") in accepted
    assert ("Engine.settings_", "unbound-dependency-type") in accepted


def test_third_party_overload_of_a_bound_method_is_accepted(solver_report):
    # Engine.tune(n) is bound; its C++ overload takes vendor::Settings first.
    assert ("Engine.tune", "unbound-dependency-type") in {
        (a.symbol, a.rule) for a in solver_report.accepted
    }
    assert "Engine.tune" not in {g.symbol for g in solver_report.gaps}


def test_solver_fixture_rows_exact(solver_report):
    assert {(g.symbol, g.kind) for g in solver_report.gaps} == {
        ("Status.kConverged", audit.Kind.ENUMERATOR),
    }
    assert {(d.name, d.kind) for d in solver_report.deviations} == {
        ("Status.NLP_CONVERGED", audit.Kind.ENUMERATOR),
        ("Engine.tune", audit.Kind.OVERLOAD),  # tune(n: int) matches no C++ overload's types
    }
    assert {(a.symbol, a.rule) for a in solver_report.accepted} == {
        ("Status.kRunning", "k-prefix-enumerator"),
        ("Status.RUNNING", "k-prefix-enumerator"),
        ("Status.kQPSolveFailed", "k-prefix-enumerator"),
        ("Status.QP_SOLVE_FAILED", "k-prefix-enumerator"),
        ("Engine.setDefaults", "unbound-dependency-type"),
        ("Engine.settings_", "unbound-dependency-type"),
        ("Engine.tune", "unbound-dependency-type"),
    }
    # Shift, Engine, their __init__s, Engine.warmStart, Status, Shift's two in-place operators,
    # and Engine.tune: a bound name whose only C++ overload is accepted counts as covered.
    assert solver_report.covered == 9


def test_unbound_dependency_changes_no_core_row(core_and_g1_reports):
    for report in core_and_g1_reports.values():
        assert "unbound-dependency-type" not in {a.rule for a in report.accepted}
