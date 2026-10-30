"""Contract tests of the #256 group 1 audit mechanisms: `UNAUDITED_HEADERS` keyed by owning
prefix (TF-1), member-template instances (T2), overload-level acceptance (T4), free functions
bound as methods (T7), Poly alternatives (T9), and the group 1 rules on the `poly` fixture.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import dataclasses
import fnmatch
from pathlib import Path

import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")
match_ = load("audit_match")  # monkeypatch the module that calls a function, not `audit`
rules_ = load("audit_rules")  # `_check_rule_keys` reads the tables here

FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"

# The 10 core modules audited before #256: their rows must not move under TF-1.
CORE_MODULES = (
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
)


# TF-1: UNAUDITED_HEADERS keyed by owning prefix.


def test_unaudited_patterns_are_keyed_by_a_known_prefix():
    known = {p for ps in own.owner_prefixes().values() for p in ps} | {own.ANY_PREFIX}
    assert set(own.UNAUDITED_HEADERS) <= known


def test_nested_prefix_header_sees_only_its_own_patterns(monkeypatch):
    """A header under a nested prefix is matched against that prefix's patterns and
    `ANY_PREFIX`, never its parent's: a `nodes/*` entry for `tesseract/task_composer/` must not
    hide `tesseract/task_composer/planning/nodes/*` (G4, C19)."""
    monkeypatch.setattr(
        own,
        "UNAUDITED_HEADERS",
        {own.ANY_PREFIX: {}, "tesseract/task_composer/": {"nodes/*": "tc nodes"}},
    )
    spelling = "tesseract/task_composer/planning/nodes/x.h"
    both = ("tesseract/task_composer/", "tesseract/task_composer/planning/")
    assert not own.is_unaudited(spelling, both)
    assert own.is_unaudited("tesseract/task_composer/nodes/x.h", both)


def test_common_yaml_utils_stays_audited():
    """Regression guard shared with G2/G4: their `yaml_utils.h`-style entries are scoped, so
    `tesseract/common/yaml_utils.h` keeps its core rows."""
    assert not own.is_unaudited("tesseract/common/yaml_utils.h", ("tesseract/common/",))


def test_unknown_prefix_key_raises(monkeypatch):
    """The key check is not cached: every `is_unaudited` call compares the table's keys with
    `owner_prefixes()`, so a key added after the first call still fails loud."""
    assert not own.is_unaudited("tesseract/collision/x.h", ("tesseract/collision/",))
    monkeypatch.setitem(own.UNAUDITED_HEADERS, "tesseract/colision/", {"x.h": "typo"})
    with pytest.raises(own.UnknownPrefixError, match="tesseract/colision/"):
        own.is_unaudited("tesseract/collision/x.h", ("tesseract/collision/",))


def test_every_unaudited_header_pattern_hides_a_header():
    """The not-API table's "every entry fires" test (plan review 2026-10-16): a pattern that
    matches no installed header is stale (upstream renamed or removed it) and must go."""
    prefixes = own.owner_prefixes()
    for key, patterns in own.UNAUDITED_HEADERS.items():
        owners = (
            [(m, p) for m, ps in prefixes.items() for p in ps]
            if key == own.ANY_PREFIX
            else [(m, key) for m, ps in prefixes.items() if key in ps]
        )
        below = [
            s.removeprefix(p)
            for m, p in owners
            for s in audit.prefix_headers(m, prefixes, audit.INCLUDE_DIRS).values()
            if s.startswith(p)
        ]
        for pattern in patterns:
            assert any(fnmatch.fnmatchcase(h, pattern) for h in below), (key, pattern)


@pytest.mark.parametrize(
    ("spelling", "prefixes", "hidden"),
    [
        ("tesseract/collision/bullet/bullet_utils.h", ("tesseract/collision/",), True),
        ("tesseract/common/unit_test_utils.h", ("tesseract/common/",), True),
        ("tesseract/command_language/test_suite/x.h", ("tesseract/command_language/",), True),
        (
            "tesseract/command_language/cereal_serialization_impl.hpp",
            ("tesseract/command_language/",),
            True,
        ),
        # Scoped: the kinematics plugin patterns no longer reach other prefixes.
        ("tesseract/state_solver/kdl/kdl_state_solver.h", ("tesseract/state_solver/",), False),
        (
            "tesseract/time_parameterization/kdl/cereal_serialization.h",
            ("tesseract/time_parameterization/",),
            False,
        ),
        (
            "tesseract/time_parameterization/unit_test_utils.h",
            ("tesseract/time_parameterization/",),
            False,
        ),
    ],
)
def test_unaudited_patterns_apply_below_their_prefix_only(spelling, prefixes, hidden):
    assert own.is_unaudited(spelling, prefixes) is hidden


@pytest.mark.parametrize("module", CORE_MODULES)
def test_rekeyed_core_patterns_hide_the_same_headers(module):
    """Under the old unscoped table a core pattern matched below every prefix. For each core
    header, no pattern of *another* core prefix matches it, unless the binding includes the
    header directly (then it was audited anyway): re-keying changes no core row. Planning
    prefixes' patterns (G2-G4) were never in the old table and are not compared."""
    prefixes = own.AUDITED_HEADER_PREFIX[module]
    core_prefixes = {p for m in CORE_MODULES for p in own.AUDITED_HEADER_PREFIX[m]}
    direct = audit.direct_include_spellings([own.binding_source(module)])
    for spelling in audit.prefix_headers(module, own.owner_prefixes(), audit.INCLUDE_DIRS).values():
        if spelling in direct:
            continue
        prefix = max((p for p in prefixes if spelling.startswith(p)), key=len)
        below = spelling.removeprefix(prefix)
        for key, patterns in own.UNAUDITED_HEADERS.items():
            if key == prefix or key not in core_prefixes:
                continue
            assert not any(fnmatch.fnmatchcase(below, p) for p in patterns), (spelling, key)


# The poly fixture: one TU exercising every group 1 mechanism.

POLY_PREFIX = ("tesseract/poly/",)
POLY_SYMBOLS = {
    ("poly", "ThingInterface"): "type-erasure-interface",
    ("poly", "ThingPoly.getThing"): "type-erasure-interface",
    ("poly", "Box.setThing"): "reference-setter",
}
FIXTURE_OVERLOAD_RULE = "fixture-overload"
POLY_STUBS = FIXTURES / "poly"


def audit_poly():
    return audit.audit_tu(
        "poly",
        FIXTURES / "poly_bindings.cpp",
        POLY_STUBS / "_poly.pyi",
        POLY_STUBS / "poly_init.py",
        POLY_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(POLY_STUBS,),
    )


@pytest.fixture(scope="module")
def poly_report():
    with pytest.MonkeyPatch.context() as mp:
        for key, rule in POLY_SYMBOLS.items():
            mp.setitem(audit.ACCEPTED_SYMBOLS, key, rule)
        mp.setitem(
            audit.ACCEPTED, FIXTURE_OVERLOAD_RULE, "Fixture rule for overload-level acceptance."
        )
        mp.setitem(audit.TEMPLATE_INSTANCES, ("poly", "ThingPoly.as"), ("ThingPoly.asSquare",))
        mp.setitem(audit.POLY_ALTERNATIVES, "ThingPoly", ("Square", "Circle"))
        mp.setitem(
            audit.ACCEPTED_CPP_OVERLOADS, ("poly", "Shelf.place", "3"), FIXTURE_OVERLOAD_RULE
        )
        mp.setitem(
            audit.ACCEPTED_PY_OVERLOADS, ("poly", "Shelf.place", ("str",)), FIXTURE_OVERLOAD_RULE
        )
        return audit_poly()


def accepted_pairs(report):
    return {(a.symbol, a.rule) for a in report.accepted}


def open_names(report):
    return {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


# T2: member-template instances.


def test_member_template_accepted_when_its_instances_are_bound(poly_report):
    assert ("ThingPoly.as", "template-instance-name") in accepted_pairs(poly_report)
    assert ("ThingPoly.asSquare", "template-instance-name") in accepted_pairs(poly_report)
    assert not {"ThingPoly.as", "ThingPoly.asSquare"} & open_names(poly_report)


def test_member_template_without_listed_instances_stays_a_gap(poly_report):
    assert "ThingPoly.is" in {g.symbol for g in poly_report.gaps}


def test_member_template_with_a_missing_instance_stays_a_gap(monkeypatch):
    monkeypatch.setitem(
        audit.TEMPLATE_INSTANCES,
        ("poly", "ThingPoly.as"),
        ("ThingPoly.asSquare", "ThingPoly.asCircle"),
    )
    assert "ThingPoly.as" in {g.symbol for g in audit_poly().gaps}


# T4: overload-level acceptance; R4 reference-wrapper-view.


def test_cpp_overload_accepted_by_table(poly_report):
    accepted = [(a.symbol, a.rule) for a in poly_report.accepted if a.symbol == "Shelf.place"]
    assert accepted.count(("Shelf.place", FIXTURE_OVERLOAD_RULE)) == 2  # 3-ary C++, str Python
    assert "Shelf.place" not in open_names(poly_report)


# A structural `absent_ok` with no premise: G3's `yaml-node` premise (a covered str/path
# sibling) would reject a fake YAML overload inside `_absent_ok_accepts`.
STRUCTURAL_RULE = "serialization-default-ctor"


def _structural_place(monkeypatch):
    """`audit_tu` with the 3-ary `Shelf.place` carrying a structural `absent_ok`, and the
    overload table naming the same overload."""
    real = match_.cpp_api

    def cpp_api(*args, **kwargs):
        cpp = real(*args, **kwargs)
        if "Shelf.place" in cpp:
            cpp["Shelf.place"].overloads = [
                dataclasses.replace(ov, absent_ok=STRUCTURAL_RULE) if str(ov.arity) == "3" else ov
                for ov in cpp["Shelf.place"].overloads
            ]
        return cpp

    monkeypatch.setattr(match_, "cpp_api", cpp_api)
    monkeypatch.setitem(audit.ACCEPTED, FIXTURE_OVERLOAD_RULE, "x")
    monkeypatch.setitem(
        audit.ACCEPTED_CPP_OVERLOADS, ("poly", "Shelf.place", "3"), FIXTURE_OVERLOAD_RULE
    )


def test_absent_ok_wins_over_the_table(monkeypatch):
    """Control: a structural `absent_ok` accepts first, so the table entry does not fire."""
    _structural_place(monkeypatch)
    pairs = accepted_pairs(audit_poly())
    assert ("Shelf.place", STRUCTURAL_RULE) in pairs
    assert ("Shelf.place", FIXTURE_OVERLOAD_RULE) not in pairs


def test_failed_absent_ok_falls_through_to_the_table(monkeypatch):
    """G3/G4 contract: when `_absent_ok_accepts` says no, the overload tables still apply."""
    _structural_place(monkeypatch)
    monkeypatch.setattr(match_, "_absent_ok_accepts", lambda ov, sym, ps, from_yaml=None: False)
    pairs = accepted_pairs(audit_poly())
    assert ("Shelf.place", FIXTURE_OVERLOAD_RULE) in pairs
    assert ("Shelf.place", STRUCTURAL_RULE) not in pairs


def test_overload_table_entry_with_an_unknown_rule_raises(monkeypatch):
    monkeypatch.setitem(
        audit.ACCEPTED_PY_OVERLOADS, ("poly", "Shelf.place", ("str",)), "no-such-rule"
    )
    with pytest.raises(audit.UnknownAcceptedRuleError, match="no-such-rule"):
        audit._check_rule_keys()


@pytest.mark.parametrize(
    ("table", "key"),
    [
        ("ACCEPTED_SYMBOLS", ("poly", "Box.setThing")),
        ("ACCEPTED_CPP_OVERLOADS", ("poly", "Shelf.place", "3")),
    ],
)
def test_every_rule_table_is_checked(monkeypatch, table, key):
    monkeypatch.setitem(getattr(audit, table), key, "no-such-rule")
    with pytest.raises(audit.UnknownAcceptedRuleError, match="no-such-rule"):
        audit._check_rule_keys()


def test_template_instances_need_their_rule(monkeypatch):
    """`TEMPLATE_INSTANCES` names `template-instance-name` implicitly: the rule must exist.
    The other tables are emptied, so only the instance entry can raise."""
    monkeypatch.setattr(rules_, "ACCEPTED_SYMBOLS", {})
    monkeypatch.setattr(rules_, "ACCEPTED_CPP_OVERLOADS", {})
    monkeypatch.setattr(rules_, "ACCEPTED_PY_OVERLOADS", {})
    monkeypatch.setattr(
        rules_, "TEMPLATE_INSTANCES", {("poly", "ThingPoly.as"): ("ThingPoly.asSquare",)}
    )
    monkeypatch.delitem(audit.ACCEPTED, "template-instance-name")
    with pytest.raises(audit.UnknownAcceptedRuleError, match="ThingPoly.as"):
        audit._check_rule_keys()


def test_py_overload_table_selects_by_annotations_not_arity(poly_report):
    """`Shelf.place(name: str)` is accepted; `Shelf.place(thing: ThingPoly)`, also arity 1,
    is covered on its own: the key is the annotation tuple."""
    places = sorted(
        a.location.rsplit(".", 1)[1].split(":")[0]
        for a in poly_report.accepted
        if a.symbol == "Shelf.place"
    )
    assert places == ["h", "pyi"]  # the 3-ary C++ overload and the `str` Python overload
    assert ("Shelf.place", audit.Kind.OVERLOAD) not in {
        (d.name, d.kind) for d in poly_report.deviations
    }


def test_py_overload_table_key_with_other_annotations_accepts_nothing(monkeypatch):
    """`("int",)` has the arity of `place(name: str)` but not its annotations: no match."""
    monkeypatch.setattr(
        match_,
        "ACCEPTED_PY_OVERLOADS",
        {("poly", "Shelf.place", ("int",)): FIXTURE_OVERLOAD_RULE},
    )
    monkeypatch.setitem(audit.ACCEPTED, FIXTURE_OVERLOAD_RULE, "x")
    report = audit_poly()
    assert ("Shelf.place", audit.Kind.OVERLOAD) in {(d.name, d.kind) for d in report.deviations}


def test_one_error_names_every_offending_entry(monkeypatch):
    monkeypatch.setitem(audit.ACCEPTED_SYMBOLS, ("poly", "Box.setThing"), "no-such-rule")
    monkeypatch.setitem(audit.ACCEPTED_CPP_OVERLOADS, ("poly", "Shelf.place", "3"), "other-rule")
    with pytest.raises(audit.UnknownAcceptedRuleError) as err:
        audit._check_rule_keys()
    assert "no-such-rule" in str(err.value) and "other-rule" in str(err.value)


def test_overload_table_keys_select_one_overload(monkeypatch):
    """A table entry for an arity no overload has accepts nothing."""
    monkeypatch.setattr(
        match_, "ACCEPTED_CPP_OVERLOADS", {("poly", "Shelf.place", "2"): FIXTURE_OVERLOAD_RULE}
    )
    monkeypatch.setattr(match_, "ACCEPTED_PY_OVERLOADS", {})
    monkeypatch.setitem(audit.ACCEPTED, FIXTURE_OVERLOAD_RULE, "x")
    report = audit_poly()
    assert ("Shelf.place", "3") in {(g.symbol, g.arity) for g in report.gaps}
    assert ("Shelf.place", audit.Kind.OVERLOAD) in {(d.name, d.kind) for d in report.deviations}


def test_reference_wrapper_vector_parameter_recorded():
    tu = audit.parse_tu(FIXTURES / "poly_bindings.cpp", (FIXTURE_INCLUDE,))
    headers = audit.audited_headers(
        tu, "poly", {"poly": POLY_PREFIX}, (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS)
    )
    cpp = audit.cpp_api(tu, headers)
    views = [ov.reference_wrapper_view for ov in cpp["View.__init__"].overloads]
    assert sorted(views) == [False, True]


def test_reference_wrapper_view_accepted_beside_a_covered_sibling(poly_report):
    assert ("View.__init__", "reference-wrapper-view") in accepted_pairs(poly_report)
    assert "View.__init__" not in open_names(poly_report)


def test_reference_wrapper_view_needs_a_covered_sibling(poly_report):
    gaps = [(g.symbol, g.arity) for g in poly_report.gaps if g.symbol == "Lonely.__init__"]
    assert gaps == [("Lonely.__init__", "1"), ("Lonely.__init__", "1")]
    assert ("Lonely.__init__", "reference-wrapper-view") not in accepted_pairs(poly_report)


# T7 free-function-as-method; R3 type-index; R1, R10 by ACCEPTED_SYMBOLS.


def test_free_predicate_bound_as_a_method_is_accepted(poly_report):
    assert ("isSquare", "free-function-as-method") in accepted_pairs(poly_report)
    assert "isRound" in {g.symbol for g in poly_report.gaps}


def test_free_binary_operator_bound_as_the_dunder_is_accepted(poly_report):
    """`operator==(const Square&, const Square&)` is `Square.__eq__`; `operator!=` on two
    `Circle`s stays a gap, `Circle` binds no `__ne__`."""
    assert ("operator==", "free-function-as-method") in accepted_pairs(poly_report)
    assert "operator==" not in open_names(poly_report)
    assert "operator!=" in {g.symbol for g in poly_report.gaps}


def test_type_index_getter_accepted(poly_report):
    assert ("ThingPoly.getType", "type-index") in accepted_pairs(poly_report)


def test_whole_symbol_rule_reads_the_symbol_level_absent_ok():
    """Fields have no overloads: G2 sets `CppSymbol.absent_ok` on them."""
    field = audit.CppSymbol("X.f", audit.Kind.FIELD, "x.h:1", absent_ok="type-index")
    assert audit._whole_symbol_rule(field) == "type-index"
    bare = audit.CppSymbol("X.g", audit.Kind.METHOD, "x.h:2")
    assert audit._whole_symbol_rule(bare) is None


def _overload(absent_ok):
    return audit.CppOverload(audit.Arity(0, 0), (), "x.h:3", absent_ok=absent_ok)


def test_whole_symbol_rule_needs_one_rule_on_every_overload(monkeypatch):
    """One shared whole-symbol rule accepts; a missing or a second rule leaves a gap."""
    monkeypatch.setattr(match_, "WHOLE_SYMBOL_ABSENT_OK", frozenset({"type-index", "other"}))
    sym = audit.CppSymbol("X.h", audit.Kind.METHOD, "x.h:3")
    sym.overloads = [_overload("type-index"), _overload("type-index")]
    assert match_._whole_symbol_rule(sym) == "type-index"
    sym.overloads = [_overload("type-index"), _overload(None)]
    assert match_._whole_symbol_rule(sym) is None
    sym.overloads = [_overload("type-index"), _overload("other")]
    assert match_._whole_symbol_rule(sym) is None
    sym.overloads = [_overload("yaml-node")]
    assert match_._whole_symbol_rule(sym) is None


def test_type_erasure_and_reference_setter_symbols_accepted(poly_report):
    for (_, name), rule in POLY_SYMBOLS.items():
        assert (name, rule) in accepted_pairs(poly_report)
        assert name not in open_names(poly_report)


# R11 vector-forwarding-no-python-form; R9 clone-as-copy.


def test_vector_forwarding_members_accepted(poly_report):
    for leaf in ("data", "rbegin", "rend", "swap", "emplace_back"):
        assert (f"Things.{leaf}", "vector-forwarding-no-python-form") in accepted_pairs(poly_report)


def test_vector_forwarding_needs_a_forwarder(monkeypatch):
    """`Things` without `reserve` in the C++ API is not a forwarder: its `data` stays a gap."""
    monkeypatch.setattr(
        match_, "VECTOR_FORWARDER_MEMBERS", ("begin", "end", "size", "push_back", "reserve")
    )
    assert "Things.data" in {g.symbol for g in audit_poly().gaps}


def test_clone_covered_by_the_copy_protocol(poly_report):
    assert "Square.clone" not in open_names(poly_report)
    assert ("Square.__copy__", "clone-as-copy") in accepted_pairs(poly_report)
    assert ("Square.__deepcopy__", "clone-as-copy") in accepted_pairs(poly_report)
    assert "Circle.clone" in {g.symbol for g in poly_report.gaps}


@pytest.fixture(scope="module")
def common_report():
    return audit.audit_module("tesseract_common")


def test_vector_forwarding_core_diff_is_the_8_joint_trajectory_rows(common_report):
    accepted = {
        a.symbol for a in common_report.accepted if a.rule == "vector-forwarding-no-python-form"
    }
    assert accepted == {
        f"JointTrajectory.{m}"
        for m in ("crbegin", "crend", "data", "emplace", "emplace_back", "rbegin", "rend", "swap")
    }
    gaps = {g.symbol for g in common_report.gaps}
    assert {"JointTrajectory.insert", "JointTrajectory.erase"} <= gaps  # binding-fix, not R11


# T9 + R8 poly-alternative-overloads.


def test_poly_alternative_overload_accepted(poly_report):
    rows = [a for a in poly_report.accepted if a.symbol == "Box.put"]
    assert [a.rule for a in rows] == ["poly-alternative-overloads"]  # put(Square) only
    assert "Box.put" not in open_names(poly_report)


def test_poly_alternatives_change_no_other_fixture_row(poly_report):
    """Exact fixture rows after Tasks 3-7."""
    assert {(g.symbol, g.arity) for g in poly_report.gaps} == {
        ("Circle.clone", "—"),
        ("ThingPoly.is", "—"),
        ("isRound", "—"),
        ("operator!=", "—"),
        ("Lonely.__init__", "1"),
    }
    assert len([g for g in poly_report.gaps if g.symbol == "Lonely.__init__"]) == 2
    assert {(d.name, d.kind) for d in poly_report.deviations} == {
        ("Lonely.__init__", audit.Kind.OVERLOAD),
    }
    assert accepted_pairs(poly_report) == {
        ("ThingInterface", "type-erasure-interface"),
        ("ThingPoly.getThing", "type-erasure-interface"),
        ("ThingPoly.getType", "type-index"),
        ("ThingPoly.as", "template-instance-name"),
        ("ThingPoly.asSquare", "template-instance-name"),
        ("isSquare", "free-function-as-method"),
        ("operator==", "free-function-as-method"),
        ("Box.put", "poly-alternative-overloads"),
        ("Box.setThing", "reference-setter"),
        *{
            (f"Things.{m}", "vector-forwarding-no-python-form")
            for m in ("data", "rbegin", "rend", "swap", "emplace_back")
        },
        ("Things.__len__", "container-protocol"),
        ("Things.__iter__", "iterator-pair"),
        ("View.__init__", "reference-wrapper-view"),
        ("Shelf.place", FIXTURE_OVERLOAD_RULE),
        ("Square.__copy__", "clone-as-copy"),
        ("Square.__deepcopy__", "clone-as-copy"),
    }


def test_poly_alternatives_need_a_listed_poly(monkeypatch):
    """Without the `POLY_ALTERNATIVES` entry, `put(Square)` stays a deviation."""
    monkeypatch.setattr(match_, "POLY_ALTERNATIVES", {})
    deviations = {(d.name, d.kind) for d in audit_poly().deviations}
    assert ("Box.put", audit.Kind.OVERLOAD) in deviations


# The ACCEPTED texts of the group 1 rules, verbatim from the plan's rule tables.
G1_RULE_TEXTS = {
    "reference-wrapper-view": "An overload taking `std::vector<std::reference_wrapper<T>>` "
    "views objects owned elsewhere; Python passes the owner to a bound sibling overload, which "
    "builds the same view (`InstructionsTrajectory(program)`). Accepted only when a sibling "
    "overload is covered.",
    "profile-key-template": "`ProfileDictionary::getProfile<ProfileType>(ns, name, default)` "
    "derives the profile key from its template argument; Python passes the key to the bound "
    "`getProfile(key, ns, name)` (`profile.getKey()`, or `ProfileType.createKey()` once bound), "
    "which returns the registered profile class, and checks `hasProfile` for the default.",
    "free-function-as-method": "A free predicate whose only parameter is `const T&` is bound as "
    "a method of `T` (`isWaitInstruction(const InstructionPoly&)` → "
    "`InstructionPoly.isWaitInstruction()`), beside `T`'s member predicates of the same shape "
    "(`isMoveInstruction`); a free binary operator on two `const T&` is bound as `T`'s dunder "
    "(`operator==(const OSQPSettings&, const OSQPSettings&)` → `OSQPSettings.__eq__`).",
    "type-index": "`getType()` returns a `std::type_index`, which has no Python form; the bound "
    "`is…()` predicates and `as…()` casts answer the same question by name, and "
    "`AnyPoly.getTypeName()` returns the demangled type name.",
    "type-erasure-interface": "The abstract `…Interface` of a type-erased `…Poly` is its C++ "
    "extension point (pure virtuals, a protected `equals`, cereal registration); Python holds "
    "the concrete classes and their Polys, and reaches the stored object through the bound casts "
    "(`InstructionPoly.as…`, and the waypoint casts), not through the accessor that returns the "
    "interface by reference.",
    "algorithm-internal-namespace": "`time_parameterization::totg::{Path, PathSegment, PathData, "
    "Trajectory}` are the implementation classes of the bound `TimeOptimalTrajectoryGeneration`; "
    "Python reaches them through `compute`.",
    "reference-setter": "C++ assigns through a getter that returns a non-const reference "
    "(`getWaypoint() = wp`); Python cannot assign to a returned reference, so a `set…` method "
    "assigns through it.",
    "vector-forwarding-no-python-form": "A class that forwards the `std::vector` interface "
    "leaves `data()` (raw pointer), `emplace`/`emplace_back` (perfect-forwarding templates), "
    "the iterator-range `insert(pos, first, last)` template, `swap(std::vector&)` and the reverse "
    "iterators unbound: Python builds the element and passes it to `push_back`/`insert`, and "
    "`reversed()` walks any sequence with `__len__` and `__getitem__`.",
    "clone-as-copy": "`__copy__`/`__deepcopy__` bind the C++ copy constructor and return the "
    "concrete type; `clone()`, which returns the `unique_ptr<…Interface>` of the type erasure, "
    "is then covered by `__deepcopy__`.",
    "poly-alternative-overloads": "A parameter typed as a type-erased Poly (`InstructionPoly`, "
    "`WaypointPoly`) is also bound once per alternative the Poly wraps "
    "(`push_back(MoveInstructionPoly)`, `MoveInstruction(CartesianWaypointPoly, …)`), so the stub "
    "names the types a caller passes; C++ converts each alternative through the Poly's "
    "converting constructor.",
}


@pytest.mark.parametrize("rule", sorted(G1_RULE_TEXTS))
def test_g1_rule_text_is_pinned(rule):
    assert audit.ACCEPTED[rule] == G1_RULE_TEXTS[rule]
