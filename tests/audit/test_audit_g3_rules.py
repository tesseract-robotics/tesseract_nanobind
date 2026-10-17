"""Fixture contract tests of the group 3 rules: R13's extension (Task 4) and C6 = 6a (Task 6).

The planner fixture TU is g3-mech's (`tests/audit/fixtures/planner/planner_bindings.cpp`); this file
audits it with the fixture's third-party root and not-API pattern in place, as the real
modules see `ompl-<v>/ompl/` and `trajopt_sco/*`.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")
parse = load("audit_parse")

FIXTURES = Path(__file__).resolve().parent / "fixtures"
# g3-mech's planner fixture: its own stub root; headers stay under `include/tesseract/planner/`.
PLANNER = FIXTURES / "planner"
FIXTURE_INCLUDE = FIXTURES / "include"
PLANNER_PREFIX = ("tesseract/planner/",)
# The fixture's third-party root, as `header_rel` spells it (relative to the repo).
FIXTURE_THIRD_PARTY = "tests/audit/fixtures/include/thirdparty_fixture/"
FIXTURE_NOT_API = {"internal/*": "fixture internals"}
# The fixture's forward-declared third-party namespace (planner.h `vendorfwd::Planner`).
FIXTURE_NAMESPACES = ("vendorfwd::",)


def _planner_report():
    return audit.audit_tu(
        "planner",
        PLANNER / "planner_bindings.cpp",
        PLANNER / "_planner.pyi",
        PLANNER / "planner_init.py",
        PLANNER_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(PLANNER,),
    )


@pytest.fixture(scope="module")
def planner_report():
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(parse, "UNBOUND_DEPENDENCY_ROOTS", (FIXTURE_THIRD_PARTY,))
        mp.setattr(parse, "UNBOUND_DEPENDENCY_NAMESPACES", FIXTURE_NAMESPACES)
        # An `UNAUDITED_HEADERS` key must be a module's prefix (`UnknownPrefixError`).
        mp.setitem(own.AUDITED_HEADER_PREFIX, "planner", PLANNER_PREFIX)
        mp.setitem(own.UNAUDITED_HEADERS, PLANNER_PREFIX[0], FIXTURE_NOT_API)
        return _planner_report()


def accepted_pairs(report):
    return {(a.symbol, a.rule) for a in report.accepted}


# Task 4: R13 `unbound-dependency-type` over returns, `std::function` and not-API headers.


def test_third_party_param_and_return_accepted(planner_report):
    assert ("Configurator.create", "unbound-dependency-type") in accepted_pairs(planner_report)
    assert "Configurator.getType" not in {g.symbol for g in planner_report.gaps}


def test_std_function_over_a_third_party_type_accepted(planner_report):
    assert ("Hooks.createStateExtractor", "unbound-dependency-type") in accepted_pairs(
        planner_report
    )


def test_forward_declared_third_party_type_accepted_by_namespace(planner_report):
    """No third-party header declares `vendorfwd::Planner`, only the owned header: its namespace."""
    assert ("ForwardHooks.create", "unbound-dependency-type") in accepted_pairs(planner_report)


def test_forward_declared_type_needs_its_namespace():
    """Without the namespace entry the forward declaration is an owned type: a gap."""
    assert "ForwardHooks.create" in {g.symbol for g in _planner_report().gaps}


def test_ompl_types_are_identified_by_namespace():
    """tesseract's OMPL headers only forward-declare OMPL types (`ompl::base::Planner`)."""
    assert parse.UNBOUND_DEPENDENCY_NAMESPACES == ("ompl::",)


def test_not_api_return_and_field_accepted(planner_report):
    accepted = accepted_pairs(planner_report)
    assert ("Hooks.createTerm", "unbound-dependency-type") in accepted
    assert ("Hooks.callbacks", "unbound-dependency-type") in accepted


def test_owned_or_plain_types_stay_gaps(planner_report):
    assert {"Hooks.on_step", "Hooks.makeConfigurator"} <= {g.symbol for g in planner_report.gaps}


def test_not_api_half_needs_an_unaudited_header(monkeypatch):
    """Without the `internal/*` pattern, `internal::Term` is an audited type: no acceptance."""
    monkeypatch.setattr(parse, "UNBOUND_DEPENDENCY_ROOTS", (FIXTURE_THIRD_PARTY,))
    report = _planner_report()
    assert {"Hooks.createTerm", "Hooks.callbacks"} <= {g.symbol for g in report.gaps}


def test_header_is_unaudited_reads_the_owning_prefix(monkeypatch):
    monkeypatch.setitem(own.AUDITED_HEADER_PREFIX, "planner", PLANNER_PREFIX)
    monkeypatch.setitem(own.UNAUDITED_HEADERS, PLANNER_PREFIX[0], FIXTURE_NOT_API)
    prefixes = {"planner": PLANNER_PREFIX, "other": ("tesseract/other/",)}
    assert own.header_is_unaudited("tesseract/planner/internal/term.h", prefixes)
    assert not own.header_is_unaudited("tesseract/planner/planner.h", prefixes)
    assert not own.header_is_unaudited("tesseract/other/internal/term.h", prefixes)
    assert not own.header_is_unaudited("vendor/internal/term.h", prefixes)  # no owner


def test_ompl_root_is_the_versioned_include():
    """`header_rel` spells OMPL headers `ompl-<v>/ompl/…`; a bare `ompl/` root never matches."""
    roots = parse.UNBOUND_DEPENDENCY_ROOTS
    assert "OsqpEigen/" in roots
    ompl = [r for r in roots if r.endswith("/ompl/")]
    assert len(ompl) == 1 and ompl[0].startswith("ompl-")
    assert (audit.CONDA_INCLUDE / ompl[0]).is_dir()


def test_planner_fixture_rows_exact(planner_report):
    assert {(g.symbol, g.kind, g.arity) for g in planner_report.gaps} == {
        ("Profile.__init__", audit.Kind.CONSTRUCTOR, "2"),
        ("UnboundSibling.__init__", audit.Kind.CONSTRUCTOR, "1"),
        ("Mismatched.__init__", audit.Kind.CONSTRUCTOR, "2"),
        ("DerivedProfile.__init__", audit.Kind.CONSTRUCTOR, "1"),
        ("IntYamlProfile.__init__", audit.Kind.CONSTRUCTOR, "1"),
        ("Hooks.on_step", audit.Kind.FIELD, audit.NOT_AN_OVERLOAD),
        ("Hooks.makeConfigurator", audit.Kind.METHOD, audit.NOT_AN_OVERLOAD),
        ("AssignHolder.__eq__", audit.Kind.OPERATOR, audit.NOT_AN_OVERLOAD),
        ("AssignHolder.operator+", audit.Kind.OPERATOR, audit.NOT_AN_OVERLOAD),
    }
    # A Python-only `from_yaml` is accepted only beside a YAML ctor it can stand for (C6 6a).
    assert {d.name for d in planner_report.deviations} == {
        "BaseProfile.from_yaml",
        "IntYamlProfile.from_yaml",
    }


# Task 6, C6 = 6a: a static `from_yaml(text)` is the YAML-text sibling.


def test_from_yaml_is_the_yaml_sibling(planner_report):
    accepted = accepted_pairs(planner_report)
    assert ("TextProfile.__init__", "yaml-node") in accepted
    assert ("TextProfile.from_yaml", "yaml-node") in accepted
    assert "TextProfile.from_yaml" not in {d.name for d in planner_report.deviations}
    assert "TextProfile.__init__" not in {g.symbol for g in planner_report.gaps}
    assert ("Profile.__init__", "2") in {(g.symbol, g.arity) for g in planner_report.gaps}


def test_from_yaml_needs_a_str_parameter(planner_report):
    """`from_yaml(config: int)` is no YAML-text sibling: the YAML ctor stays a gap."""
    assert ("IntYamlProfile.__init__", "1") in {(g.symbol, g.arity) for g in planner_report.gaps}
    assert "IntYamlProfile.from_yaml" in {d.name for d in planner_report.deviations}


def test_yaml_node_text_states_the_from_yaml_premise():
    assert "from_yaml" in audit.ACCEPTED["yaml-node"]


# Task 10: a parameter typed by a renamed class matches the Python class's name.

RENAMED_PARAMS = FIXTURES / "renamed_params"


def _renamed_params_report():
    here = "tests/audit/fixtures/renamed_params/evaluator.h"
    state = frozenset({("class", "State")})

    def method(params):
        return [audit.CppOverload(audit.Arity(1, 1), (), f"{here}:9", params=params)]

    def cls(name):
        return audit.CppSymbol(name, audit.Kind.CLASS, f"{here}:1", qualified=f"dl::{name}")

    def ctor(name):
        return audit.CppSymbol(
            f"{name}.__init__",
            audit.Kind.CONSTRUCTOR,
            f"{here}:2",
            [audit.CppOverload(audit.Arity(0, 0), (), f"{here}:2")],
        )

    cpp = {
        "State": cls("State"),
        "State.__init__": ctor("State"),
        "Evaluator": cls("Evaluator"),
        "Evaluator.__init__": ctor("Evaluator"),
        "Evaluator.evaluate": audit.CppSymbol(
            "Evaluator.evaluate", audit.Kind.METHOD, f"{here}:9", method((state,))
        ),
        "Evaluator.evaluateAll": audit.CppSymbol(
            "Evaluator.evaluateAll",
            audit.Kind.METHOD,
            f"{here}:10",
            method((frozenset({("seq", ("class", "State"))}),)),
        ),
    }
    stub = RENAMED_PARAMS / "_renamed_params.pyi"
    py = audit.py_api(audit.load_stub(stub), audit.rel(stub))
    bound = {"renamed_params": {"StateD": "dl::State", "EvaluatorD": "dl::Evaluator"}}
    return audit.match(
        "renamed_params",
        cpp,
        frozenset({"State", "Evaluator", "evaluate", "evaluateAll"}),
        py,
        audit.rel(stub),
        stub_roots=(RENAMED_PARAMS,),
        bound=lambda m: bound.get(m, {}),
    )


def test_renamed_class_parameter_matches_its_python_name():
    """`evaluate(const State&)` is bound as `EvaluatorD.evaluate(state: StateD)`: covered, also
    inside a container."""
    report = _renamed_params_report()
    assert not [g for g in report.gaps if g.symbol.startswith("Evaluator.")]
    assert not [d for d in report.deviations if d.kind is audit.Kind.OVERLOAD]
