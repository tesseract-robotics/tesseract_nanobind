"""Real-row contract tests of the audited motion-planner modules (#256 group 3).

Each test pins one triage outcome of `docs/superpowers/state/phaseF-triage-{mp-base,mp-plan,
sqp,tp}.md` against the module's report. Variants applied (Jelle 2026-10-16): C6 = 6a
(`from_yaml`), C8 = 7a (bind the hooks), C3 = 8a (trampolines), C25 = not API.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import fnmatch

import clang.cindex as ci
import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")

MP = "tesseract_motion_planners"
SIMPLE = "tesseract_motion_planners_simple"
OMPL = "tesseract_motion_planners_ompl"
TRAJOPT = "tesseract_motion_planners_trajopt"
IFOPT = "tesseract_motion_planners_trajopt_ifopt"
DESCARTES = "tesseract_motion_planners_descartes"
G3 = (MP, SIMPLE, OMPL, TRAJOPT, IFOPT)


@pytest.fixture(scope="module")
def g3_reports():
    return {m: audit.audit_module(m) for m in G3}


@pytest.fixture(scope="module")
def descartes_report():
    return audit.audit_module(DESCARTES)


def gap_symbols(report):
    return {g.symbol for g in report.gaps}


def open_names(report):
    return gap_symbols(report) | {d.name for d in report.deviations}


def accepted_pairs(report):
    return {(a.symbol, a.rule) for a in report.accepted}


# Task 1: the five non-Descartes modules are audited.


@pytest.mark.parametrize("module", (*G3, DESCARTES))
def test_g3_module_is_audited(module):
    assert module in own.AUDITED_HEADER_PREFIX
    assert module not in own.UNAUDITED_MODULES
    assert audit.resolve_module(module) == module


@pytest.mark.parametrize(
    ("spelling", "owner"),
    [
        ("tesseract/motion_planners/planner.h", MP),
        ("tesseract/motion_planners/simple/interpolation.h", SIMPLE),
        ("tesseract/motion_planners/ompl/utils.h", OMPL),
        ("tesseract/motion_planners/trajopt/trajopt_utils.h", TRAJOPT),
        ("trajopt_sco/optimizers.hpp", TRAJOPT),
        ("trajopt/problem_description.hpp", TRAJOPT),
        ("trajopt_sqp/qp_problem.h", "trajopt_sqp"),  # `trajopt/` is not a prefix of `trajopt_sqp/`
        ("tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h", IFOPT),
        ("tesseract/motion_planners/descartes/descartes_utils.h", DESCARTES),
        ("descartes_light/types.h", DESCARTES),
    ],
)
def test_g3_header_ownership(spelling, owner):
    assert own.header_owner(spelling, own.owner_prefixes()) == owner


def test_parent_module_does_not_report_planner_headers(g3_reports):
    rows = gap_symbols(g3_reports[MP])
    assert not any(r.startswith("tesseract/motion_planners/ompl/") for r in rows)
    assert g3_reports[MP].delegated == {}


@pytest.mark.parametrize(
    ("module", "cls"),
    [
        (SIMPLE, "SimpleMotionPlanner"),
        (OMPL, "OMPLMotionPlanner"),
        (TRAJOPT, "TrajOptMotionPlanner"),
        (IFOPT, "TrajOptIfoptMotionPlanner"),
    ],
)
def test_planner_base_gaps_from_t5(g3_reports, module, cls):
    """PR 0b's T5 rows: four planners bound without their `MotionPlanner` base (g3/01)."""
    assert (f"{cls}(MotionPlanner)", audit.Kind.BASE) in {
        (g.symbol, g.kind) for g in g3_reports[module].gaps
    }


def test_sco_var_is_not_trajopt_ifopt_var(g3_reports):
    """T3: `sco::Var` is no longer compared against `trajopt_ifopt.Var` (trajopt rows 104-108)."""
    rows = gap_symbols(g3_reports[TRAJOPT])
    assert not {"Var.__init__", "Var.__str__", "Var.value", "Var.var_rep"} & rows
    assert ("Var", "bound-in-other-module") not in accepted_pairs(g3_reports[TRAJOPT])


# Task 2 (T1): profile YAML ctors have no YAML-text sibling.


@pytest.mark.parametrize(
    ("module", "symbol"),
    [
        (SIMPLE, "SimplePlannerLVSMoveProfile.__init__"),
        (SIMPLE, "SimplePlannerFixedSizeMoveProfile.__init__"),
        (OMPL, "OMPLRealVectorMoveProfile.__init__"),
        (TRAJOPT, "TrajOptDefaultCompositeProfile.__init__"),
        (TRAJOPT, "TrajOptOSQPSolverProfile.__init__"),
        (IFOPT, "TrajOptIfoptDefaultMoveProfile.__init__"),
    ],
)
def test_real_profile_yaml_ctor_is_a_gap(g3_reports, module, symbol):
    """No profile binds a YAML-text sibling yet: binding-fix g3/08 (C6 = 6a)."""
    report = g3_reports[module]
    assert (symbol, "2") in {(g.symbol, g.arity) for g in report.gaps}
    assert (symbol, "yaml-node") not in accepted_pairs(report)


# Task 3: not-API headers, scoped by owning prefix (TF-1).

OMPL_DIR = "tesseract/motion_planners/ompl/"
G3_NOT_API = [
    *[
        (OMPL, OMPL_DIR + h)
        for h in (
            "collision_cost_objective.h",
            "compound_state_validator.h",
            "continuous_motion_validator.h",
            "discrete_motion_validator.h",
            "state_collision_validator.h",
            "utils.h",
            "weighted_real_vector_state_sampler.h",
        )
    ],
    (TRAJOPT, "tesseract/motion_planners/trajopt/trajopt_utils.h"),
    *[
        (TRAJOPT, f"trajopt/{h}")
        for h in (
            "collision_terms.hpp",
            "file_write_callback.hpp",
            "json_marshal.hpp",
            "kinematic_terms.hpp",
            "plot_callback.hpp",
            "problem_description.hpp",
            "trajectory_costs.hpp",
            "typedefs.hpp",
            "utils.hpp",
        )
    ],
    *[
        (TRAJOPT, f"trajopt_sco/{h}")
        for h in (
            "bpmpd_interface.hpp",
            "bpmpd_io.hpp",
            "expr_op_overloads.hpp",
            "expr_ops.hpp",
            "expr_vec_ops.hpp",
            "gurobi_interface.hpp",
            "qpoases_interface.hpp",
            "solver_utils.hpp",
        )
    ],
]


@pytest.mark.parametrize(("module", "header"), G3_NOT_API)
def test_g3_not_api_header_has_no_row(g3_reports, module, header):
    assert header not in gap_symbols(g3_reports[module])


@pytest.mark.parametrize(
    "symbol",
    [
        "AffExpr",
        "OptProb",
        "Model",
        "ModelType",
        "OSQPModelConfig",
        "VarRep",
        "Var",
        "vecSum",
        "createModel",
    ],
)
def test_trajopt_sco_transitive_symbols_have_no_row(g3_reports, symbol):
    """Declared in transitively included trajopt_sco headers, which `trajopt_sco/*` makes not API."""
    assert symbol not in gap_symbols(g3_reports[TRAJOPT])


def test_optimizers_hpp_stays_audited(g3_reports):
    report = g3_reports[TRAJOPT]
    assert "BasicTrustRegionSQPParameters.__eq__" in gap_symbols(report)  # binding-fix I-mp-plan-2
    # A row, never no row: accepted under solver-behind-profile (Task 5).
    assert ("Optimizer", "solver-behind-profile") in accepted_pairs(report)


@pytest.mark.parametrize(
    "spelling",
    [
        "tesseract/motion_planners/utils.h",  # the parent module's own header (I-mp-base-3)
        "tesseract/common/utils.h",
        "tesseract/kinematics/utils.h",
        "tesseract/collision/utils.h",
        "trajopt_sqp/qp_problem.h",  # `trajopt/` does not prefix `trajopt_sqp/`
        "trajopt_ifopt/variable_sets/joint_position_variable.h",
        "tesseract/motion_planners/simple/interpolation.h",
        "tesseract/motion_planners/descartes/descartes_utils.h",
    ],
)
def test_g3_patterns_are_scoped(spelling):
    prefixes = own.owner_prefixes()
    module, _ = own.owning_prefix(spelling, prefixes)
    assert not own.is_unaudited(spelling, prefixes[module])


# Task 4 (R13): OMPL root, return types, `std::function`, not-API half.


@pytest.mark.parametrize(
    ("module", "symbol"),
    [
        (OMPL, "OMPLMoveProfile.createSimpleSetup"),
        (OMPL, "OMPLMoveProfile.createStateExtractor"),
        (OMPL, "OMPLPlannerConfigurator.create"),
        (OMPL, "OMPLRealVectorMoveProfile.createSimpleSetup"),
        (OMPL, "OMPLRealVectorMoveProfile.createStateExtractor"),
        (OMPL, "RRTConnectConfigurator.create"),
        (OMPL, "RRTstarConfigurator.create"),
        (OMPL, "SBLConfigurator.create"),
        (TRAJOPT, "TrajOptMotionPlanner.createProblem"),
        (TRAJOPT, "TrajOptOSQPSolverProfile.createSolverConfig"),
        (TRAJOPT, "TrajOptOSQPSolverProfile.getSolverType"),
        (TRAJOPT, "TrajOptSolverProfile.callbacks"),
        (TRAJOPT, "TrajOptSolverProfile.createOptimizationCallbacks"),
        (TRAJOPT, "TrajOptSolverProfile.createSolverConfig"),
        (TRAJOPT, "TrajOptSolverProfile.getSolverType"),
        # G2's third-party half (`OsqpEigen/`), sqp rows 79 and 85.
        (IFOPT, "TrajOptIfoptOSQPSolverProfile.qp_settings"),
        (IFOPT, "operator=="),
    ],
)
def test_real_unbound_dependency_rows(g3_reports, module, symbol):
    report = g3_reports[module]
    assert (symbol, "unbound-dependency-type") in accepted_pairs(report)
    assert symbol not in gap_symbols(report)


@pytest.mark.parametrize(
    ("module", "symbol"),
    [
        # Bindable or owned types: these stay binding-fix rows.
        (TRAJOPT, "TrajOptOSQPSolverProfile.settings"),  # osqp/ is not a root (I-mp-plan-7)
        (TRAJOPT, "operator=="),  # OSQPSettings, same
        (IFOPT, "TrajOptIfoptMoveProfile.create"),  # I-sqp-6 (g3/06)
        (IFOPT, "TrajOptIfoptSolverProfile.callbacks"),  # I-sqp-5: trajopt_sqp is audited
        (IFOPT, "TrajOptIfoptSolverProfile.createOptimizationCallbacks"),  # I-sqp-5
        (OMPL, "OMPLSolverConfig.planners"),  # I-mp-plan-3 (g3/04)
    ],
)
def test_real_bindable_rows_stay_gaps(g3_reports, module, symbol):
    assert symbol in gap_symbols(g3_reports[module])


# Task 5: symbol-level rules R2, R14, R21 and the explicit R13 entries.


@pytest.mark.parametrize(
    ("module", "symbol", "rule"),
    [
        (MP, "PlannerRequest.data", "type-erased-data"),
        (MP, "PlannerResponse.data", "type-erased-data"),
        (OMPL, "RNG_setSeed", "third-party-global"),
        (OMPL, "RNG_getSeed", "third-party-global"),
        *[
            (TRAJOPT, s, "solver-behind-profile")
            for s in (
                "Optimizer",
                "BasicTrustRegionSQP",
                "BasicTrustRegionSQPMultiThreaded",
                "OptResults",
                "BasicTrustRegionSQPResults",
                "OptStatus",
                "toString",
            )
        ],
        *[
            (TRAJOPT, s, "unbound-dependency-type")
            for s in (
                "TrajOptMoveProfile.create",
                "TrajOptDefaultMoveProfile.create",
                "TrajOptCompositeProfile.create",
                "TrajOptDefaultCompositeProfile.create",
            )
        ],
    ],
)
def test_real_g3_symbol_rules(g3_reports, module, symbol, rule):
    report = g3_reports[module]
    assert (symbol, rule) in accepted_pairs(report)
    assert symbol not in open_names(report)


def test_type_erased_data_premise():
    """The rule's premise: both fields are `std::shared_ptr<void>` in C++."""
    tu = audit.parse_tu(own.binding_source(MP))
    fields = {
        c.semantic_parent.spelling: c.type.spelling  # declared spelling; libc++'s canonical one is std::__1::…
        for c in tu.cursor.walk_preorder()
        if c.kind == ci.CursorKind.FIELD_DECL
        and c.spelling == "data"
        and c.semantic_parent.spelling in ("PlannerRequest", "PlannerResponse")
    }
    assert set(fields) == {"PlannerRequest", "PlannerResponse"}
    assert set(fields.values()) == {"std::shared_ptr<void>"}


def test_solver_behind_profile_leaves_the_parameters_audited(g3_reports):
    assert "BasicTrustRegionSQPParameters.__eq__" in gap_symbols(g3_reports[TRAJOPT])


def test_copied_vector_field_mutator_waits_for_its_premise(g3_reports):
    """ompl rows 38-39: no rule before `g3/04` binds `planners` (spec §3)."""
    names = {d.name for d in g3_reports[OMPL].deviations}
    assert {"OMPLSolverConfig.addPlanner", "OMPLSolverConfig.clearPlanners"} <= names


# Task 7 (C8 = 7a, bind): the profile hooks stay binding-fix gaps (g3/06).

PROFILE_HOOKS = [
    (OMPL, "OMPLMoveProfile.createSolverConfig"),
    (OMPL, "OMPLRealVectorMoveProfile.createSolverConfig"),
    (TRAJOPT, "TrajOptSolverProfile.createOptimizationParameters"),
    *[
        (SIMPLE, f"{p}.generate")
        for p in (
            "SimplePlannerMoveProfile",
            "SimplePlannerFixedSizeMoveProfile",
            "SimplePlannerFixedSizeAssignMoveProfile",
            "SimplePlannerFixedSizeAssignNoIKMoveProfile",
            "SimplePlannerLVSMoveProfile",
            "SimplePlannerLVSNoIKMoveProfile",
            "SimplePlannerLVSAssignMoveProfile",
            "SimplePlannerLVSAssignNoIKMoveProfile",
        )
    ],
    *[
        (IFOPT, f"{p}.create")
        for p in (
            "TrajOptIfoptMoveProfile",
            "TrajOptIfoptDefaultMoveProfile",
            "TrajOptIfoptCompositeProfile",
            "TrajOptIfoptDefaultCompositeProfile",
            "TrajOptIfoptSolverProfile",
            "TrajOptIfoptOSQPSolverProfile",
        )
    ],
    (IFOPT, "TrajOptIfoptTermInfos"),
    (IFOPT, "TrajOptIfoptWaypointInfo"),
]


def test_profile_hook_list_is_the_c8_table():
    assert len(PROFILE_HOOKS) == 19


@pytest.mark.parametrize(("module", "symbol"), PROFILE_HOOKS)
def test_real_profile_hooks_stay_gaps(g3_reports, module, symbol):
    """C8 = bind: no rule may accept a hook; each stays a gap until g3/06 binds it."""
    report = g3_reports[module]
    assert symbol in gap_symbols(report)
    assert symbol not in {a.symbol for a in report.accepted}


# Task 8 (C3 = 8a, trampolines).


@pytest.mark.parametrize("symbol", ["TrajOptTermInfos", "TrajOptWaypointInfo"])
def test_trajopt_term_infos_are_unbound_dependency(g3_reports, symbol):
    """Their payload is classic-TrajOpt `TermInfo`, which `trajopt/*` makes not API."""
    report = g3_reports[TRAJOPT]
    assert (symbol, "unbound-dependency-type") in accepted_pairs(report)
    assert symbol not in gap_symbols(report)


@pytest.mark.parametrize(
    "symbol",
    [
        "JointGroupInstructionInfo",
        "KinematicGroupInstructionInfo",
        "getClosestJointSolution",
        "interpolateCartCartWaypoint",
        "interpolateCartJointWaypoint",
        "interpolateJointCartWaypoint",
        "interpolateJointJointWaypoint",
    ],
)
def test_simple_author_helpers_stay_gaps(g3_reports, symbol):
    """C3 = trampolines: the helpers a Python `generate` override calls get bound (g3/07)."""
    assert symbol in gap_symbols(g3_reports[SIMPLE])


# Task 10.1: Descartes; `bound_classes` maps the `…D` instances to their templates.

DESCARTES_RENAMES = [
    ("DescartesMoveProfile", "DescartesMoveProfileD"),
    ("DescartesSolverProfile", "DescartesSolverProfileD"),
    ("DescartesDefaultMoveProfile", "DescartesDefaultMoveProfileD"),
    ("DescartesLadderGraphSolverProfile", "DescartesLadderGraphSolverProfileD"),
    ("DescartesMotionPlanner", "DescartesMotionPlannerD"),
    ("State", "DescartesStateD"),
    ("StateSample", "DescartesStateSampleD"),
    ("EdgeEvaluator", "DescartesEdgeEvaluatorD"),
    ("WaypointSampler", "DescartesWaypointSamplerD"),
    ("StateEvaluator", "DescartesStateEvaluatorD"),
]
DESCARTES_PREFIXED = [
    "DescartesStateD",
    "DescartesStateSampleD",
    "DescartesEdgeEvaluatorD",
    "DescartesWaypointSamplerD",
    "DescartesStateEvaluatorD",
]


@pytest.mark.parametrize(("cpp", "python"), DESCARTES_RENAMES)
def test_descartes_instances_map_to_their_templates(descartes_report, cpp, python):
    rows = open_names(descartes_report)
    assert cpp not in rows and python not in rows


@pytest.mark.parametrize("python", DESCARTES_PREFIXED)
def test_prefixed_descartes_names_accepted(descartes_report, python):
    assert (python, "template-instance-name") in accepted_pairs(descartes_report)


def test_bound_classes_resolve_descartes_instances_to_cpp_api_names():
    """0b's index must name the same qualified class `cpp_api` records for the template."""
    tu = audit.parse_tu(own.binding_source(DESCARTES))
    bound = audit.bound_classes(tu)
    prefixes = own.owner_prefixes()
    cpp = audit.cpp_api(tu, audit.audited_headers(tu, DESCARTES, prefixes, audit.INCLUDE_DIRS))
    for cpp_name, python in DESCARTES_RENAMES:
        assert bound[python] == cpp[cpp_name].qualified, python


def test_descartes_state_is_not_ompl_state(descartes_report):
    """T3(b): the probe matched descartes_light::State to `ompl_base.State` by leaf name."""
    assert ("State", "bound-in-other-module") not in accepted_pairs(descartes_report)
    # Compared against DescartesStateD now: its bound ctor and `values` cover the C++ ones.
    assert not {"State.__init__", "State.values"} & gap_symbols(descartes_report)


def test_descartes_planner_base_gap(descartes_report):
    assert ("DescartesMotionPlannerD(MotionPlanner)", audit.Kind.BASE) in {
        (g.symbol, g.kind) for g in descartes_report.gaps
    }


# Task 10.2 and 10.3: T13 and T12 real rows (mechanisms from g3-mech).


def test_real_descartes_state_ctor_template_is_no_method(descartes_report):
    assert "State.State<FloatType>" not in gap_symbols(descartes_report)


def test_real_descartes_state_assign_has_no_row(descartes_report):
    assert "State.operator=" not in gap_symbols(descartes_report)


# Task 10.4: R32 `eigen-value-wrapper`.

STATE_OPERATOR_LINES = {
    65,
    82,
    90,
    98,
    120,
    125,
    127,
    135,
    138,
    141,
    147,
}  # descartes_light/types.h


def test_state_operators_are_eigen_value_wrapper_rows(descartes_report):
    accepted = [a for a in descartes_report.accepted if a.rule == "eigen-value-wrapper"]
    assert {int(a.location.rsplit(":", 1)[1]) for a in accepted} == STATE_OPERATOR_LINES
    assert all(a.location.startswith("descartes_light/types.h:") for a in accepted)
    assert not [
        g
        for g in descartes_report.gaps
        if g.location.startswith("descartes_light/types.h:") and g.kind is audit.Kind.OPERATOR
    ]


# Task 10.5: Descartes not-API headers (C25 = not API).

DESCARTES_NOT_API = [
    *[
        f"descartes_light/bgl/{h}"
        for h in (
            "bgl_dfs_solver.h",
            "bgl_dijkstra_solver.h",
            "bgl_solver.h",
            "boost_graph_types.h",
            "event_visitors.h",
            "utils.h",
            "impl/bgl_dfs_solver.hpp",
            "impl/bgl_dijkstra_solver.hpp",
            "impl/bgl_solver.hpp",
            "impl/event_visitors.hpp",
            "impl/utils.hpp",
        )
    ],
    *[
        f"descartes_light/edge_evaluators/{h}_edge_evaluator.h"
        for h in ("compound", "euclidean_distance", "normalized", "timing")
    ],
    "descartes_light/samplers/fixed_joint_waypoint_sampler.h",
    *[
        f"descartes_light/solvers/ladder_graph/{h}"
        for h in (
            "impl/ladder_graph_solver.hpp",
            "ladder_graph.h",
            "ladder_graph_dag_search.h",
            "ladder_graph_solver.h",
        )
    ],
    *[
        f"descartes_light/state_evaluators/{h}_state_evaluator.h"
        for h in ("euclidean_distance", "normalized")
    ],
    *[
        f"tesseract/motion_planners/descartes/{h}"
        for h in (
            "descartes_collision.h",
            "descartes_collision_edge_evaluator.h",
            "descartes_robot_sampler.h",
            "descartes_vertex_evaluator.h",
            "yaml_extensions.h",
        )
    ],
]


@pytest.mark.parametrize("header", DESCARTES_NOT_API)
def test_descartes_not_api_header_has_no_row(descartes_report, header):
    assert header not in gap_symbols(descartes_report)


@pytest.mark.parametrize("symbol", ["BuildStatus", "SearchResult", "Solver"])
def test_descartes_solver_interface_has_no_row(descartes_report, symbol):
    assert symbol not in gap_symbols(descartes_report)


@pytest.mark.parametrize(
    "symbol", ["DescartesSolverProfile.create", "DescartesLadderGraphSolverProfile.create"]
)
def test_descartes_solver_profile_create_is_unbound_dependency(descartes_report, symbol):
    """Returns unique_ptr<descartes_light::Solver<double>>; core/solver.h is not API (Task 4)."""
    assert (symbol, "unbound-dependency-type") in accepted_pairs(descartes_report)


@pytest.mark.parametrize(
    "symbol",
    [
        *[
            f"sample{s}" for s in ("Fixed", "ToolAxis", "ToolXAxis", "ToolYAxis", "ToolZAxis")
        ],  # g3/20
        "DescartesMoveProfile.createKinematicGroup",  # C3 = 8a: g3/07
    ],
)
def test_descartes_binding_fix_rows_stay_gaps(descartes_report, symbol):
    assert symbol in gap_symbols(descartes_report)


# Every G3 table entry fires: a stale entry fails here, never silently.


def test_every_g3_table_entry_fires(g3_reports, descartes_report):
    reports = {**g3_reports, DESCARTES: descartes_report}
    stale = [
        (module, symbol, rule)
        for (module, symbol), rule in audit.ACCEPTED_SYMBOLS.items()
        if module in reports and (symbol, rule) not in accepted_pairs(reports[module])
    ]
    stale += [
        (module, cls, "eigen-value-wrapper")
        for module, cls in audit.EIGEN_VALUE_WRAPPERS
        if not [
            a
            for a in reports[module].accepted
            if a.rule == "eigen-value-wrapper" and a.symbol.startswith(f"{cls}.")
        ]
    ]
    assert stale == []


def test_every_g3_not_api_pattern_hides_a_header():
    prefixes = own.owner_prefixes()
    modules = {*G3, DESCARTES}
    unused = []
    for prefix, patterns in own.UNAUDITED_HEADERS.items():
        hit = own.owning_prefix(prefix, prefixes)
        if hit is None or hit[0] not in modules:
            continue
        module = hit[0]
        below = [
            spelling.removeprefix(prefix)
            for spelling in audit.prefix_headers(module, prefixes, audit.INCLUDE_DIRS).values()
            if spelling.startswith(prefix)
        ]
        unused += [
            f"{prefix}{p}" for p in patterns if not any(fnmatch.fnmatchcase(b, p) for b in below)
        ]
    assert unused == []
