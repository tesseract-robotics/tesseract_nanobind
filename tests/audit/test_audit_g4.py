"""Real-row tests of the task-composer audit (#256 group 4; ledger phaseF-triage-tc.md).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import pytest

from tests.audit._load import audit

TC = "tesseract_task_composer"
TCP = "tesseract_task_composer_planning"
KIN = "tesseract_kinematics"
COLLISION = "tesseract_collision"
PLUGIN_AUTHOR = "plugin-author-factory-interface"
# C20 (Jelle 2026-10-16: apply to core): the core won't-fix factory interfaces.
CORE_FACTORY_INTERFACES = (
    (KIN, "FwdKinFactory"),
    (KIN, "InvKinFactory"),
    (COLLISION, "DiscreteContactManagerFactory"),
    (COLLISION, "ContinuousContactManagerFactory"),
)
TC_NOT_API = (
    "tesseract/task_composer/task_composer_plugin_factory_utils.h",
    "tesseract/task_composer/task_composer_task_plugin_factory.h",
    "tesseract/task_composer/taskflow/taskflow_task_composer_plugin_factories.h",
    "tesseract/task_composer/taskflow/taskflow_task_composer_future.h",
    "tesseract/task_composer/yaml_utils.h",
)
TC_PREFIX = "tesseract/task_composer/"
TCP_PREFIX = "tesseract/task_composer/planning/"
ANY_POLY_H = "tesseract/common/any_poly.h"
# Location of each tcp profile's `(YAML::Node, ProfilePluginFactory)` ctor: accepted by
# `yaml-node` since each profile binds `from_yaml` (#331).
TCP_PROFILE_YAML_CTORS = {
    "ContactCheckProfile": "contact_check_profile.h:51",
    "FixStateBoundsProfile": "fix_state_bounds_profile.h:58",
    "FixStateCollisionProfile": "fix_state_collision_profile.h:75",
    "KinematicLimitsCheckProfile": "kinematic_limits_check_profile.h:42",
    "MinLengthProfile": "min_length_profile.h:52",
    "ProfileSwitchProfile": "profile_switch_profile.h:47",
    "UpsampleTrajectoryProfile": "upsample_trajectory_profile.h:49",
}
# Header rows left after the not-API patterns: none, since graph, log, pipeline and task are
# bound (#324, #325).
TC_HEADER_GAPS: set[str] = set()


@pytest.fixture(scope="module")
def g4_reports():
    return {m: audit.audit_module(m) for m in (TC, TCP)}


@pytest.fixture(scope="module")
def core_reports():
    return {m: audit.audit_module(m) for m in (KIN, COLLISION)}


def test_g4_modules_are_audited():
    for m in (TC, TCP):
        assert m in audit.AUDITED_HEADER_PREFIX and m not in audit.UNAUDITED_MODULES


@pytest.mark.parametrize(
    ("symbol", "rule"),
    [
        ("AnyPoly.as", "template-instance-name"),
        *[
            (f"AnyPoly_as_{t}", "template-instance-name")
            for t in ("CompositeInstruction", "TaskComposerDataStorage", "ContactResultMapVector")
        ],
        *[
            (f"AnyPoly_wrap_{t}", "template-instance-name")
            for t in (
                "CompositeInstruction",
                "ProfileDictionary",
                "EnvironmentConst",
                "TaskComposerDataStorage",
            )
        ],
        ("AnyPoly.__init__", "template-instance-name"),  # any_poly.h:121, :138 (Task 1)
        ("AnyPoly.__init__", "type-erasure-interface"),  # any_poly.h:115
        ("AnyInterface", "type-erasure-interface"),
        ("AnyWrapper", "type-erasure-interface"),
        ("AnyPoly.get", "type-erasure-interface"),
        ("AnyPoly.getType", "type-index"),  # structural (G1 R3)
        ("AnyPoly.getTypeName", "type-index"),  # the demangled name (#322)
        # task_composer_node_info.h:52; the free `serialize` is in cereal_serialization.h (#319)
        ("TaskComposerNodeInfo.__init__", "serialization-default-ctor"),
        ("FutureStatus", "std-library-enum"),
        ("TaskComposerFuture.waitUntil", "clock-deadline"),
        ("TaskComposerNode.dump", "dot-dump-hook"),
        ("TaskComposerNodeFactory", PLUGIN_AUTHOR),
        ("TaskComposerExecutorFactory", PLUGIN_AUTHOR),
        ("TaskComposerExecutor.run", "context-from-data-storage"),  # C23: keep the rule
        ("TaskflowTaskComposerExecutor.__init__", "plugin-config-ctor"),
        # (name, config, plugin_factory): what TaskComposerTaskFactory<T>::create calls
        ("TaskComposerGraph.__init__", "plugin-config-ctor"),
        ("TaskComposerPipeline.__init__", "plugin-config-ctor"),
        ("TaskComposerGraph.dump", "dot-dump-hook"),  # the override of Node.dump
        ("TaskComposerGraph.addNodePython", "python-hook-method"),  # bound as addNode
        ("TaskComposerServer.addTaskPython", "python-hook-method"),  # bound as addTask
        ("TaskComposerNodePorts.toString", "to-string-as-str"),
    ],
)
def test_tc_rows_accepted(g4_reports, symbol, rule):
    assert (symbol, rule) in {(a.symbol, a.rule) for a in g4_reports[TC].accepted}


@pytest.mark.parametrize(("module", "symbol"), CORE_FACTORY_INTERFACES)
def test_core_factory_interfaces_accepted(core_reports, module, symbol):
    """C20: the rule also accepts the four core factory interfaces (core report diff)."""
    report = core_reports[module]
    assert (symbol, PLUGIN_AUTHOR) in {(a.symbol, a.rule) for a in report.accepted}
    assert symbol not in {g.symbol for g in report.gaps}


def test_class_extension_trait_has_no_row(g4_reports):
    """task_composer_log.h:46 `TESSERACT_CLASS_EXTENSION` specialises `extension<T>`, a
    template of serialization_extensions.h (not API): the specialisation is not API either."""
    report = g4_reports[TC]
    assert "extension" not in {g.symbol for g in report.gaps}
    assert "extension" not in {a.symbol for a in report.accepted}


def test_any_poly_ctor_rules_by_location(g4_reports):
    """Each AnyPoly ctor gets its own rule; the overload-table key "1" must not hide :121/:138."""
    accepted = g4_reports[TC].accepted
    assert sorted((a.rule, a.location) for a in accepted if a.symbol == "AnyPoly.__init__") == [
        ("template-instance-name", f"{ANY_POLY_H}:121"),
        ("template-instance-name", f"{ANY_POLY_H}:138"),
        ("type-erasure-interface", f"{ANY_POLY_H}:115"),
    ]


def test_any_poly_has_no_rows_left(g4_reports):
    """`__eq__`/`__ne__` are bound (#348); `getTypeName` is demangled and accepted (#322)."""
    report = g4_reports[TC]
    prefixes = ("AnyPoly", "AnyInterface", "AnyWrapper")
    assert not {g.symbol for g in report.gaps if g.symbol.startswith(prefixes)}
    assert not {d.name for d in report.deviations if d.name.startswith(prefixes)}
    assert "AnyPoly.operator=" not in {g.symbol for g in report.gaps}  # T12 (G3)


def test_node_info_default_ctor_is_not_a_gap(g4_reports):
    """Only the deserialization ctor is accepted; `TaskComposerNodeInfo(node)` is bound (#319)."""
    gaps = {(g.symbol, g.arity) for g in g4_reports[TC].gaps}
    assert not {a for s, a in gaps if s == "TaskComposerNodeInfo.__init__"}


def test_executor_run_context_overload_still_matched(g4_reports):
    report = g4_reports[TC]
    assert "TaskComposerExecutor.run" not in {g.symbol for g in report.gaps}
    assert "TaskComposerExecutor.run" not in {d.name for d in report.deviations}


def test_plugin_factory_yaml_ctor_keeps_yaml_node(g4_reports):
    """`TaskComposerPluginFactory(YAML::Node, locator)` has a bound `(str, locator)` sibling."""
    assert ("TaskComposerPluginFactory.__init__", "yaml-node") in {
        (a.symbol, a.rule) for a in g4_reports[TC].accepted
    }


@pytest.mark.parametrize(("profile", "where"), TCP_PROFILE_YAML_CTORS.items())
def test_tcp_profile_yaml_ctor_is_accepted(g4_reports, profile, where):
    """The YAML ctor's Python form is the bound `from_yaml` (#331): `yaml-node`, not a gap."""
    report = g4_reports[TCP]
    symbol = f"{profile}.__init__"
    location = f"{TCP_PREFIX}profiles/{where}"
    assert (symbol, location) not in {(g.symbol, g.location) for g in report.gaps}
    assert (symbol, "yaml-node") in {(a.symbol, a.rule) for a in report.accepted}


# Task 3: prefix-scoped not-API headers.


def test_tc_not_api_headers_have_no_rows(g4_reports):
    assert not set(TC_NOT_API) & {g.symbol for g in g4_reports[TC].gaps}
    assert f"{TCP_PREFIX}planning_task_composer_plugin_factories.h" not in {
        g.symbol for g in g4_reports[TCP].gaps
    }


def test_header_rows_left_exact(g4_reports):
    """Absence alone would also pass if every header row vanished."""
    header = audit.Kind.HEADER
    assert {g.symbol for g in g4_reports[TC].gaps if g.kind == header} == TC_HEADER_GAPS
    assert {g.symbol for g in g4_reports[TCP].gaps if g.kind == header} == set()


def test_tc_patterns_do_not_reach_planning(monkeypatch, g4_reports):
    """A planning header sees only its own prefix's patterns (G1 TF-1)."""
    monkeypatch.setitem(audit.UNAUDITED_HEADERS[TC_PREFIX], "planning/*", "probe")
    tcp = audit.audit_module(TCP)
    assert {g.symbol for g in tcp.gaps} == {g.symbol for g in g4_reports[TCP].gaps}


def test_tc_yaml_utils_pattern_keeps_the_common_header():
    common = audit.audit_module("tesseract_common")
    assert "tesseract/common/yaml_utils.h" in {g.symbol for g in common.gaps}


# Task 4B (C19 = not-api): `nodes/*` under both prefixes.


def test_node_headers_not_api(g4_reports):
    tc = {g.symbol for g in g4_reports[TC].gaps}
    tcp = {g.symbol for g in g4_reports[TCP].gaps}
    assert not {s for s in tc if s.startswith(f"{TC_PREFIX}nodes/")}
    assert not {s for s in tcp if s.startswith(f"{TCP_PREFIX}nodes/")}


def test_nodes_pattern_only_under_task_composer_prefixes():
    keys = {k for k, pats in audit.UNAUDITED_HEADERS.items() if "nodes/*" in pats}
    assert keys == {TC_PREFIX, TCP_PREFIX}


# Task 5: every G4 table entry fires.


def test_every_g4_table_entry_fires(g4_reports, core_reports):
    reports = {**g4_reports, **core_reports}
    accepted = {(m, a.symbol, a.rule) for m, r in reports.items() for a in r.accepted}
    core = set(CORE_FACTORY_INTERFACES)
    entries = [
        *[
            (m, n, rule)
            for (m, n), rule in audit.ACCEPTED_SYMBOLS.items()
            if m in (TC, TCP) or (m, n) in core
        ],
        *[
            (m, n, rule)
            for (m, n, _), rule in audit.ACCEPTED_CPP_OVERLOADS.items()
            if m in (TC, TCP)
        ],
        *[
            (m, n, rule)
            for (m, n, _), rule in audit.ACCEPTED_PY_OVERLOADS.items()
            if m in (TC, TCP)
        ],
        *[
            (m, n, "template-instance-name")
            for (m, n) in audit.TEMPLATE_INSTANCES
            if m in (TC, TCP)
        ],
    ]
    assert entries, "no G4 table entries"
    stale = [e for e in entries if e not in accepted]
    assert stale == []
