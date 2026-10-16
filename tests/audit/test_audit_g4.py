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
TCP_PROFILES = (
    "ContactCheckProfile",
    "FixStateBoundsProfile",
    "FixStateCollisionProfile",
    "KinematicLimitsCheckProfile",
    "MinLengthProfile",
    "ProfileSwitchProfile",
    "UpsampleTrajectoryProfile",
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
        ("FutureStatus", "std-library-enum"),
        ("TaskComposerFuture.waitUntil", "clock-deadline"),
        ("TaskComposerNode.dump", "dot-dump-hook"),
        ("TaskComposerNodeFactory", PLUGIN_AUTHOR),
        ("TaskComposerExecutorFactory", PLUGIN_AUTHOR),
        ("TaskComposerExecutor.run", "context-from-data-storage"),  # C23: keep the rule
        ("TaskflowTaskComposerExecutor.__init__", "plugin-config-ctor"),
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


def test_any_poly_rows_left_are_value_equality_only(g4_reports):
    """What remains of AnyPoly: `__eq__`/`__ne__` (g1/00-value-equality.md), `getTypeName` (I-tc-15)."""
    report = g4_reports[TC]
    prefixes = ("AnyPoly", "AnyInterface", "AnyWrapper")
    assert {g.symbol for g in report.gaps if g.symbol.startswith(prefixes)} == {
        "AnyPoly.__eq__",
        "AnyPoly.__ne__",
    }
    # getTypeName returns a mangled name: binding-fix I-tc-15, not type-index (premise fails).
    assert {d.name for d in report.deviations if d.name.startswith(prefixes)} == {
        "AnyPoly.getTypeName"
    }
    assert "AnyPoly.operator=" not in {g.symbol for g in report.gaps}  # T12 (G3)


def test_executor_run_context_overload_still_matched(g4_reports):
    report = g4_reports[TC]
    assert "TaskComposerExecutor.run" not in {g.symbol for g in report.gaps}
    assert "TaskComposerExecutor.run" not in {d.name for d in report.deviations}


def test_plugin_factory_yaml_ctor_keeps_yaml_node(g4_reports):
    """`TaskComposerPluginFactory(YAML::Node, locator)` has a bound `(str, locator)` sibling (T1)."""
    assert ("TaskComposerPluginFactory.__init__", "yaml-node") in {
        (a.symbol, a.rule) for a in g4_reports[TC].accepted
    }


@pytest.mark.parametrize("profile", TCP_PROFILES)
def test_tcp_profile_yaml_ctor_is_a_gap(g4_reports, profile):
    """C6 = 6a: no `from_yaml` sibling yet, so the YAML ctor is binding-fix g3/08-profile-yaml.md."""
    report = g4_reports[TCP]
    symbol = f"{profile}.__init__"
    assert symbol in {g.symbol for g in report.gaps}
    assert (symbol, "yaml-node") not in {(a.symbol, a.rule) for a in report.accepted}


# Task 3: prefix-scoped not-API headers.


def test_tc_not_api_headers_have_no_rows(g4_reports):
    assert not set(TC_NOT_API) & {g.symbol for g in g4_reports[TC].gaps}
    assert f"{TCP_PREFIX}planning_task_composer_plugin_factories.h" not in {
        g.symbol for g in g4_reports[TCP].gaps
    }


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
