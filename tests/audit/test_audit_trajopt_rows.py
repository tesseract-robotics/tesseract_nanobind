"""Real-row contract tests of the group 2 audit (`trajopt_ifopt`, `trajopt_sqp`; #256).

The mechanisms' fixture tests are in `test_audit_trajopt.py`; these pin the rows the two real
modules produce against the triage ledgers (`phaseF-triage-sqp.md`,
`phaseF-triage-trajopt-ifopt.md`). Run in the `audit` env only (libclang):
`pixi run -e audit audit-test`.
"""

import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")

G2 = ("trajopt_ifopt", "trajopt_sqp")


@pytest.fixture(scope="module")
def g2_reports():
    return {m: audit.audit_module(m) for m in G2}


def open_rows(report):
    return {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


def accepted_pairs(report):
    return {(a.symbol, a.rule) for a in report.accepted}


# Task 1: the two modules are audited.


@pytest.mark.parametrize("module", G2)
def test_g2_module_is_audited(module):
    assert module in own.AUDITED_HEADER_PREFIX
    assert module not in own.UNAUDITED_MODULES
    assert audit.resolve_module(module) == module


def test_trajopt_common_is_owned_by_trajopt_ifopt():
    prefixes = own.owner_prefixes()
    assert own.header_owner("trajopt_common/collision_types.h", prefixes) == "trajopt_ifopt"
    assert own.header_owner("trajopt_sqp/qp_problem.h", prefixes) == "trajopt_sqp"


def test_trajopt_sqp_includes_of_trajopt_ifopt_headers_are_not_delegated(g2_reports):
    # trajopt_sqp's binding includes trajopt_ifopt/core/{component,constraint_set}.h, and
    # trajopt_ifopt's own binding includes both: neither module audits them for the other.
    assert g2_reports["trajopt_sqp"].delegated == {}
    assert g2_reports["trajopt_ifopt"].delegated == {}


# Task 2 (T11): compound-assignment operators are keyed by their dunders.


def test_unbound_inplace_operator_is_a_gap_under_its_dunder(g2_reports):
    # trajopt_ifopt Bounds: unbound until #286 lands.
    gaps = {g.symbol for g in g2_reports["trajopt_ifopt"].gaps}
    assert {"Bounds.__iadd__", "Bounds.__isub__"} <= gaps
    assert not {"Bounds.operator+=", "Bounds.operator-="} & gaps


# Task 3 (T10 + R15): `k-prefix-enumerator`.


@pytest.mark.parametrize(
    ("module", "symbol"),
    [
        ("trajopt_sqp", "CostPenaltyType.kSquared"),
        ("trajopt_sqp", "CostPenaltyType.SQUARED"),
        ("trajopt_sqp", "SQPStatus.kPenaltyIterationLimit"),
        ("trajopt_sqp", "SQPStatus.PENALTY_ITERATION_LIMIT"),
        ("trajopt_sqp", "QPSolverStatus.kInitialized"),
        ("trajopt_ifopt", "BoundsType.kRangeBound"),
        ("trajopt_ifopt", "BoundsType.RANGE_BOUND"),
        ("trajopt_ifopt", "RangeBoundHandling.kSplitToTwoInequalities"),
        ("trajopt_ifopt", "RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES"),
    ],
)
def test_real_k_prefix_rows_accepted(g2_reports, module, symbol):
    report = g2_reports[module]
    assert (symbol, "k-prefix-enumerator") in accepted_pairs(report)
    assert symbol not in open_rows(report)


K_PREFIX_ACCEPTED = {
    # sqp ledger rows 1-3, 24, 29, 30, 32, 50-52, 54, 58, 61, 63.
    "trajopt_sqp": {
        *(f"CostPenaltyType.{e}" for e in ("kAbsolute", "kHinge", "kSquared")),
        *(f"CostPenaltyType.{e}" for e in ("ABSOLUTE", "HINGE", "SQUARED")),
        *(f"SQPStatus.{e}" for e in ("kIterationLimit", "kPenaltyIterationLimit", "kRunning")),
        *(f"SQPStatus.{e}" for e in ("ITERATION_LIMIT", "PENALTY_ITERATION_LIMIT", "RUNNING")),
        "QPSolverStatus.kInitialized",
        "QPSolverStatus.INITIALIZED",
    },
    # trajopt_ifopt ledger rows 3-7, 60, 61, 94-98, 102, 103.
    "trajopt_ifopt": {
        *(
            f"BoundsType.{e}"
            for e in ("kEquality", "kLowerBound", "kRangeBound", "kUnbounded", "kUpperBound")
        ),
        *(
            f"BoundsType.{e}"
            for e in ("EQUALITY", "LOWER_BOUND", "RANGE_BOUND", "UNBOUNDED", "UPPER_BOUND")
        ),
        *(f"RangeBoundHandling.{e}" for e in ("kKeepAsIs", "kSplitToTwoInequalities")),
        *(f"RangeBoundHandling.{e}" for e in ("KEEP_AS_IS", "SPLIT_TO_TWO_INEQUALITIES")),
    },
}


@pytest.mark.parametrize("module", G2)
def test_real_k_prefix_rows_exact(g2_reports, module):
    accepted = {a.symbol for a in g2_reports[module].accepted if a.rule == "k-prefix-enumerator"}
    assert accepted == K_PREFIX_ACCEPTED[module]


@pytest.mark.parametrize(
    ("cpp", "python"),
    [
        ("SQPStatus.kConverged", "SQPStatus.NLP_CONVERGED"),
        ("SQPStatus.kTimeLimit", "SQPStatus.OPT_TIME_LIMIT"),
        ("SQPStatus.kQPSolveFailed", "SQPStatus.QP_SOLVER_ERROR"),
        ("SQPStatus.kStoppedByCallback", "SQPStatus.CALLBACK_STOPPED"),
        ("QPSolverStatus.kUninitialized", "QPSolverStatus.UNITIALIZED"),
        ("QPSolverStatus.kFailed", "QPSolverStatus.QP_ERROR"),
    ],
)
def test_real_non_mechanical_enumerators_stay_reported(g2_reports, cpp, python):
    """Binding-fix #330 (rename to the native names) closes these; the rule must not."""
    report = g2_reports["trajopt_sqp"]
    assert cpp in {g.symbol for g in report.gaps}
    assert python in {d.name for d in report.deviations}


# Task 4 (R13): `unbound-dependency-type`, third-party half.


@pytest.mark.parametrize(
    "symbol",
    [
        "OSQPEigenSolver.setDefaultOSQPSettings",
        "OSQPEigenSolver.solver_",
        "OSQPEigenSolver.setWarmStart",
    ],
)
def test_real_osqp_rows_accepted(g2_reports, symbol):
    report = g2_reports["trajopt_sqp"]
    assert (symbol, "unbound-dependency-type") in accepted_pairs(report)
    assert symbol not in {d.name for d in report.deviations}


@pytest.mark.parametrize(
    "symbol", ["OSQPEigenSolver.setDefaultOSQPSettings", "OSQPEigenSolver.solver_"]
)
def test_real_osqp_whole_symbols_leave_no_gap(g2_reports, symbol):
    # setWarmStart is not here: its C++ QPProblem overload stays a gap (#283).
    assert symbol not in {g.symbol for g in g2_reports["trajopt_sqp"].gaps}


def test_real_osqp_warm_start_from_problem_stays_a_gap(g2_reports):
    """The C++ setWarmStart(const QPProblem&) is unbound: binding-fix #283."""
    assert ("OSQPEigenSolver.setWarmStart", "1") in {
        (g.symbol, g.arity) for g in g2_reports["trajopt_sqp"].gaps
    }


# Task 5: symbol- and overload-level rules R22-R27.


@pytest.mark.parametrize(
    ("symbol", "rule"),
    [
        ("Var.getParent", "parent-back-pointer"),
        ("Node.getParent", "parent-back-pointer"),
        ("Component.print", "stdout-printer"),
        ("NodesVariables.addObserver", "unimplemented-observer"),
        ("calcNumericalCostGradient", "ifopt-problem-only"),
        ("createNodesVariables", "nodes-variables-factory"),
    ],
)
def test_real_trajopt_ifopt_symbol_rules(g2_reports, symbol, rule):
    report = g2_reports["trajopt_ifopt"]
    assert (symbol, rule) in accepted_pairs(report)
    assert symbol not in open_rows(report)


def test_real_split_var_pair_accepts_both_overloads(g2_reports):
    report = g2_reports["trajopt_ifopt"]
    name = "ContinuousCollisionConstraint.__init__"
    assert [a.rule for a in report.accepted if a.symbol == name] == ["split-var-pair"] * 2
    assert name not in open_rows(report)


def test_every_g2_table_entry_fires(g2_reports):
    """A stale entry (renamed upstream, or the binding fix landed) produces no Accepted row."""
    for (module, name), rule in audit.ACCEPTED_SYMBOLS.items():
        if module in g2_reports:
            assert (name, rule) in accepted_pairs(g2_reports[module]), name
    for (module, name, _), rule in audit.ACCEPTED_CPP_OVERLOADS.items():
        if module in g2_reports:
            assert (name, rule) in accepted_pairs(g2_reports[module]), name
    for (module, name, _), rule in audit.ACCEPTED_PY_OVERLOADS.items():
        if module in g2_reports:
            assert (name, rule) in accepted_pairs(g2_reports[module]), name


@pytest.mark.parametrize(
    "rule",
    [
        "k-prefix-enumerator",
        "unbound-dependency-type",
        "parent-back-pointer",
        "stdout-printer",
        "unimplemented-observer",
        "split-var-pair",
        "ifopt-problem-only",
        "nodes-variables-factory",
    ],
)
def test_g2_rule_is_justified_in_the_report(rule):
    """Reads the committed report: passes once it is regenerated with the G2 rules."""
    assert audit.ACCEPTED[rule]
    assert f"`{rule}`" in audit.REPORT_PATH.read_text(encoding="utf-8")


# Task 6: not-API headers, scoped by their owning prefix (TF-1).

G2_NOT_API = [
    ("trajopt_sqp", "trajopt_sqp/callbacks/cartesian_error_plotter.h"),
    ("trajopt_sqp", "trajopt_sqp/callbacks/clear_plotter.h"),
    ("trajopt_sqp", "trajopt_sqp/callbacks/collision_plotter.h"),
    ("trajopt_sqp", "trajopt_sqp/callbacks/joint_state_plotter.h"),
    ("trajopt_sqp", "trajopt_sqp/callbacks/wait_for_input.h"),
    ("trajopt_sqp", "trajopt_sqp/expressions.h"),
    ("trajopt_sqp", "trajopt_sqp/ifopt_qp_problem.h"),
    ("trajopt_ifopt", "trajopt_ifopt/constraints/collision/weighted_average_methods.h"),
    ("trajopt_ifopt", "trajopt_ifopt/core/cost_term.h"),
    ("trajopt_ifopt", "trajopt_ifopt/core/problem.h"),
    ("trajopt_ifopt", "trajopt_ifopt/costs/absolute_cost.h"),
    ("trajopt_ifopt", "trajopt_ifopt/costs/squared_cost.h"),
    ("trajopt_ifopt", "trajopt_ifopt/variable_sets/nodes_observer.h"),
    *[
        ("trajopt_ifopt", f"trajopt_common/{h}")
        for h in (
            "basic_array.hpp",
            "clock.hpp",
            "collision_utils.h",
            "config.hpp",
            "eigen_conversions.hpp",
            "eigen_slicing.hpp",
            "interpolation.hpp",
            "logging.hpp",
            "stl_to_string.hpp",
            "utils.hpp",
            "vector_ops.hpp",
        )
    ],
]


@pytest.mark.parametrize(("module", "header"), G2_NOT_API)
def test_g2_not_api_header_has_no_row(g2_reports, module, header):
    assert header not in {g.symbol for g in g2_reports[module].gaps}


@pytest.mark.parametrize("symbol", ["Cache", "CompositeDifferentiable"])
def test_g2_not_api_transitive_class_has_no_row(g2_reports, symbol):
    """cache.h and core/composite.h reach the TU through other headers, not as direct includes."""
    assert symbol not in {g.symbol for g in g2_reports["trajopt_ifopt"].gaps}


@pytest.mark.parametrize(
    "symbol",
    [
        # trajopt_ifopt ledger rows 85, 91, 92, 15: binding-fix, next to not-API headers.
        "trajopt_ifopt/constraints/collision/continuous_collision_numerical_constraint.h",
        "trajopt_ifopt/utils/numeric_differentiation.h",
        "trajopt_ifopt/utils/trajopt_utils.h",
        "CollisionCacheData",
    ],
)
def test_g2_binding_fix_neighbour_stays_a_gap(g2_reports, symbol):
    assert symbol in {g.symbol for g in g2_reports["trajopt_ifopt"].gaps}


G2_PATTERNS = {
    "trajopt_sqp/": {"callbacks/*", "expressions.h", "ifopt_qp_problem.h"},
    "trajopt_ifopt/": {
        "constraints/collision/weighted_average_methods.h",
        "core/problem.h",
        "core/cost_term.h",
        "costs/*",
        "core/composite.h",
        "variable_sets/nodes_observer.h",
    },
    "trajopt_common/": {
        "basic_array.hpp",
        "eigen_conversions.hpp",
        "eigen_slicing.hpp",
        "interpolation.hpp",
        "utils.hpp",
        "vector_ops.hpp",
        "stl_to_string.hpp",
        "clock.hpp",
        "config.hpp",
        "logging.hpp",
        "collision_utils.h",
        "cache.h",
    },
}


@pytest.mark.parametrize("prefix", sorted(G2_PATTERNS))
def test_g2_patterns_exact(prefix):
    assert set(own.UNAUDITED_HEADERS[prefix]) == G2_PATTERNS[prefix]


def test_no_g2_pattern_under_any_prefix():
    g2 = set().union(*G2_PATTERNS.values())
    assert not g2 & set(own.UNAUDITED_HEADERS[own.ANY_PREFIX])


@pytest.mark.parametrize(
    "spelling",
    [
        "tesseract/common/utils.hpp",
        "tesseract/common/config.hpp",
        "tesseract/common/logging.hpp",
        "tesseract/common/cache.h",
    ],
)
def test_g2_patterns_are_scoped_to_their_prefix(spelling):
    # The spellings need not exist: the test pins the pattern table, not the include tree.
    assert not own.is_unaudited(spelling, ("tesseract/common/",))


# The rows of the merged binding fixes (#345): what the audit could not see.


@pytest.mark.parametrize("symbol", ["QPProblem.setVariables", "TrajOptQPProblem.setVariables"])
def test_real_sized_double_array(g2_reports, symbol):
    """`setVariables(const double* x)` (qp_problem.h:49, trajopt_qp_problem.h:30) reads
    `getNumNLPVars()` doubles; Python passes a size-checked array (#282)."""
    report = g2_reports["trajopt_sqp"]
    assert (symbol, "sized-double-array") in accepted_pairs(report)
    assert symbol not in {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


def test_real_problem_overload_of_the_constraint_gradient(g2_reports):
    """ifopt_utils.h:108 `(const double*, Problem&, double)` is the `IfoptQPProblem` path; the
    `(Variables&, ConstraintSet&, double)` overload is bound (#295)."""
    report = g2_reports["trajopt_ifopt"]
    pair = ("calcNumericalConstraintGradient", "ifopt-problem-only")
    assert pair in accepted_pairs(report)
    assert "calcNumericalConstraintGradient" not in {g.symbol for g in report.gaps}
