"""Real-row tests of #256 group 1: `tesseract_command_language` and
`tesseract_time_parameterization` audited, each group 1 rule firing on the rows its triage
ledger assigns it, and the premises those rules state checked in the real headers.

The mechanisms' fixture tests are in `test_audit_command_language.py`.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from collections import Counter

import pytest

from tests.audit._load import audit, load

own = load("audit_ownership")

G1 = ("tesseract_command_language", "tesseract_time_parameterization")
CL, TP = G1
CL_PREFIX = "tesseract/command_language/"

# Every rule group 1 adds (C1 decided as the rename: no `abbreviated-profile-name`).
G1_RULES = (
    "reference-wrapper-view",
    "profile-key-template",
    "free-function-as-method",
    "type-index",
    "type-erasure-interface",
    "algorithm-internal-namespace",
    "reference-setter",
    "vector-forwarding-no-python-form",
    "clone-as-copy",
    "poly-alternative-overloads",
)
# The six type-erased Polys of the command language and the interface each one stores.
POLY_INTERFACES = {
    "WaypointPoly": "WaypointInterface",
    "CartesianWaypointPoly": "CartesianWaypointInterface",
    "JointWaypointPoly": "JointWaypointInterface",
    "StateWaypointPoly": "StateWaypointInterface",
    "InstructionPoly": "InstructionInterface",
    "MoveInstructionPoly": "MoveInstructionInterface",
}
# The ten concrete command-language types, each deriving from a `…Interface`.
CONCRETE = (
    "CartesianWaypoint",
    "JointWaypoint",
    "StateWaypoint",
    "MoveInstruction",
    "CompositeInstruction",
    "WaitInstruction",
    "TimerInstruction",
    "SetAnalogInstruction",
    "SetDigitalInstruction",
    "SetToolInstruction",
)
# The time-parameterization profiles bound under abbreviations; binding-fix #329 renames them.
TP_PROFILES = {
    "TimeOptimalTrajectoryGenerationCompositeProfile": "TOTGCompositeProfile",
    "IterativeSplineParameterizationCompositeProfile": "ISPCompositeProfile",
    "IterativeSplineParameterizationMoveProfile": "ISPMoveProfile",
    "ConstantTCPSpeedParameterizationCompositeProfile": "ConstantTCPSpeedCompositeProfile",
}
# The `InstructionPoly::as<T>()` instances the binding names (cl-core rows 106-112).
INSTRUCTION_POLY_AS = tuple(
    f"InstructionPoly.as{t}Instruction"
    for t in ("Move", "Composite", "Wait", "Timer", "SetAnalog", "SetDigital", "SetTool")
)
# R11: the `std::vector` members with no Python form (cl-core rows 6-10, 27, 29, 35).
VECTOR_FORWARDING_LEAVES = (
    "crbegin", "crend", "data", "emplace", "emplace_back", "rbegin", "rend", "swap",
)  # fmt: skip


@pytest.fixture(scope="module")
def g1_reports():
    return {m: audit.audit_module(m) for m in G1}


@pytest.fixture(scope="module")
def cl_tu():
    return audit.parse_tu(own.binding_source(CL))


def accepted_pairs(report):
    return {(a.symbol, a.rule) for a in report.accepted}


def open_names(report):
    return {g.symbol for g in report.gaps} | {d.name for d in report.deviations}


def _record(tu, name):
    return next(
        c
        for c in tu.cursor.walk_preorder()
        if c.kind in audit.RECORD_KINDS and c.is_definition() and c.spelling == name
    )


# Task 1: the two modules are audited.


@pytest.mark.parametrize("module", G1)
def test_g1_module_is_audited(module):
    assert module in own.AUDITED_HEADER_PREFIX
    assert module not in own.UNAUDITED_MODULES
    assert audit.resolve_module(module) == module


def test_profile_headers_are_audited_with_command_language(g1_reports):
    """`tesseract/common/profile*.h` are under tesseract_common's prefix, but its binding does
    not include them; `header_auditor` gives them to the first includer by name, the command
    language. #344 bound `ProfileDictionary.getAllProfileEntries`; `Profile.createKey` stays a
    gap (#273)."""
    gaps = {g.symbol for g in g1_reports[CL].gaps}
    assert "Profile.createKey" in gaps
    assert "ProfileDictionary.getAllProfileEntries" not in open_names(g1_reports[CL])


def test_cereal_serialization_header_is_delegated_to_serialization(g1_reports):
    assert g1_reports[CL].delegated == {
        f"{CL_PREFIX}cereal_serialization.h": "tesseract_serialization"
    }


def test_value_equality_rows_are_covered(g1_reports):
    """#344 binds `__eq__`/`__ne__` on the six Polys and the ten concrete types (binding-fix
    `00-value-equality.md`; cl-poly rows 2, 3, 15, 17, 27, 28, 39, 41, 64, 65, 73, 78 and
    cl-core rows 144-163 were its gaps), so no value-equality row is open."""
    expected = {
        f"{c}.{dunder}" for c in (*POLY_INTERFACES, *CONCRETE) for dunder in ("__eq__", "__ne__")
    }
    assert len(expected) == 32
    assert not expected & open_names(g1_reports[CL])
    assert not [g for g in g1_reports[CL].gaps if g.symbol.endswith((".__eq__", ".__ne__"))]


# Task 3: T2 member-template instances.


def test_real_instruction_poly_as_accepted(g1_reports):
    report = g1_reports[CL]
    assert set(audit.TEMPLATE_INSTANCES[(CL, "InstructionPoly.as")]) == set(INSTRUCTION_POLY_AS)
    names = ["InstructionPoly.as", *INSTRUCTION_POLY_AS]
    for name in names:
        assert (name, "template-instance-name") in accepted_pairs(report)
    assert not set(names) & open_names(report)


# Task 4: T4 overload tables, R4 reference-wrapper-view, R18 profile-key-template.


def test_real_instructions_trajectory_view_ctor_accepted(g1_reports):
    report = g1_reports[TP]
    assert ("InstructionsTrajectory.__init__", "reference-wrapper-view") in accepted_pairs(report)
    assert "InstructionsTrajectory.__init__" not in open_names(report)


def test_real_profile_template_overload_accepted(g1_reports):
    report = g1_reports[CL]
    assert ("ProfileDictionary.getProfile", "profile-key-template") in accepted_pairs(report)
    assert "ProfileDictionary.getProfile" not in {g.symbol for g in report.gaps}
    # The vector-names overload of addProfile (binding-fix g1/11) is bound by #344, not
    # accepted by this rule.
    assert "ProfileDictionary.addProfile" not in {g.symbol for g in report.gaps}
    assert ("ProfileDictionary.addProfile", "profile-key-template") not in accepted_pairs(report)


# Task 5: T7 + R17, R3, R1, R5, R10.


@pytest.mark.parametrize(
    ("module", "symbol", "rule"),
    [
        *[
            (CL, f"is{t}Instruction", "free-function-as-method")
            for t in ("SetAnalog", "SetDigital", "SetTool", "Timer", "Wait")
        ],
        *[(CL, f"{poly}.getType", "type-index") for poly in POLY_INTERFACES],
        *[(CL, interface, "type-erasure-interface") for interface in POLY_INTERFACES.values()],
        *[
            (CL, f"{poly}.get{poly.removesuffix('Poly')}", "type-erasure-interface")
            for poly in POLY_INTERFACES
        ],
        (CL, "MoveInstruction.setWaypoint", "reference-setter"),
        (CL, "MoveInstructionPoly.setWaypoint", "reference-setter"),
        *[
            (TP, c, "algorithm-internal-namespace")
            for c in ("Path", "PathSegment", "PathData", "Trajectory")
        ],
    ],
)
def test_real_rows_accepted_by_g1_symbol_rules(g1_reports, module, symbol, rule):
    report = g1_reports[module]
    assert (symbol, rule) in accepted_pairs(report)
    assert symbol not in open_names(report)


@pytest.mark.parametrize(("poly", "interface"), POLY_INTERFACES.items())
def test_type_erasure_interface_is_the_poly_impl_pointee(cl_tu, poly, interface):
    """R1's premise: the accessor returns what the Poly stores, an abstract interface."""
    [impl] = [c for c in _record(cl_tu, poly).get_children() if c.spelling == "impl_"]
    pointee = impl.type.get_canonical().get_template_argument_type(0).get_declaration()
    assert pointee.spelling == interface
    assert _record(cl_tu, interface).is_abstract_record()


@pytest.mark.parametrize("owner", ["MoveInstruction", "MoveInstructionPoly"])
def test_reference_setter_premise(cl_tu, owner):
    """R10's premise: `getWaypoint` returns a non-const lvalue reference and C++ has no
    `setWaypoint`."""
    members = list(_record(cl_tu, owner).get_children())
    getters = [m for m in members if m.spelling == "getWaypoint"]
    assert any(
        g.result_type.kind == audit.ci.TypeKind.LVALUEREFERENCE
        and not g.result_type.get_pointee().is_const_qualified()
        for g in getters
    )
    assert not any(m.spelling == "setWaypoint" for m in members)


def test_algorithm_internal_classes_are_in_the_totg_namespace():
    """R5's premise: the four classes sit in TOTG's implementation namespace."""
    tu = audit.parse_tu(own.binding_source(TP))
    for name in ("Path", "PathSegment", "PathData", "Trajectory"):
        assert _record(tu, name).semantic_parent.spelling == "totg"


# Task 6: R11 vector-forwarding-no-python-form, R9 clone-as-copy.


@pytest.mark.parametrize("leaf", VECTOR_FORWARDING_LEAVES)
def test_real_composite_instruction_vector_members_accepted(g1_reports, leaf):
    name = f"CompositeInstruction.{leaf}"
    assert (name, "vector-forwarding-no-python-form") in accepted_pairs(g1_reports[CL])
    assert name not in open_names(g1_reports[CL])


@pytest.mark.parametrize(
    "symbol",
    [
        f"{c}.clone"
        for c in (
            *CONCRETE,
            "CartesianWaypointPoly",
            "JointWaypointPoly",
            "StateWaypointPoly",
            "MoveInstructionPoly",
        )
    ],
)
def test_real_clone_rows_are_covered_by_the_copy_protocol(g1_reports, symbol):
    """#344 (binding-fix g1/02) binds `__copy__`/`__deepcopy__`: R9 maps `clone` to
    `__deepcopy__`, so the row is covered and the two dunders are accepted as clone-as-copy."""
    cls = symbol.removesuffix(".clone")
    report = g1_reports[CL]
    assert symbol not in open_names(report)
    assert {(f"{cls}.__copy__", "clone-as-copy"), (f"{cls}.__deepcopy__", "clone-as-copy")} <= (
        accepted_pairs(report)
    )


# Task 7: T9 + R8 poly-alternative-overloads.


@pytest.mark.parametrize("poly", ["WaypointPoly", "InstructionPoly"])
def test_poly_alternatives_convert_through_a_poly_constructor(cl_tu, poly):
    """R8's premise: each alternative derives from an interface a Poly constructor takes."""
    headers = audit.audited_headers(cl_tu, CL, own.owner_prefixes(), audit.INCLUDE_DIRS)
    cpp = audit.cpp_api(cl_tu, headers)
    ctor_classes = {
        key[1]
        for ov in cpp[f"{poly}.__init__"].overloads
        for keys in ov.params
        for key in keys or ()
        if key and key[0] == "class"
    }
    ancestors = audit.class_ancestors(cl_tu)
    for alt in audit.POLY_ALTERNATIVES[poly]:
        assert ancestors[alt] & ctor_classes, alt


def test_real_poly_alternative_overloads_accepted(g1_reports):
    """`MoveInstruction.__init__`: the 6 Python overloads per waypoint alternative (cl-core rows
    121-126). The C++ ctor `move_instruction.h:71` (cl-core row 37) is covered directly since
    #344 binds its `manipulator_info`, so the rule no longer accepts it.
    `CompositeInstruction.push_back`: the 8 Python overloads per instruction alternative
    (#344 adds `MoveInstructionPoly`)."""
    report = g1_reports[CL]
    rows = [(a.symbol, a.rule) for a in report.accepted if a.rule == "poly-alternative-overloads"]
    assert rows.count(("CompositeInstruction.push_back", "poly-alternative-overloads")) == 8
    assert rows.count(("MoveInstruction.__init__", "poly-alternative-overloads")) == 6
    assert "CompositeInstruction.push_back" not in {d.name for d in report.deviations}


def test_real_move_instruction_ctors_are_covered(g1_reports):
    """g1/03 (#264): #344 binds the `path_profile` overload (cl-core row 38, was the
    `move_instruction.h:84` gap) and the trailing `manipulator_info` of both ctors, so no
    constructor gap remains. The Python overloads per waypoint alternative are accepted, not
    deviations (cl-core rows 121-126)."""
    report = g1_reports[CL]
    ctor_gaps = [
        g
        for g in report.gaps
        if g.symbol == "MoveInstruction.__init__" and g.kind is audit.Kind.CONSTRUCTOR
    ]
    assert ctor_gaps == []
    assert "MoveInstruction.__init__" not in {d.name for d in report.deviations}


# Every row the group 1 rules accept, exactly: the NEW command-language rows of the cl-core
# and cl-poly ledgers as #344 left them (cl-core row 37 is covered directly; `insert`,
# `setUserData` and the `MoveInstructionPoly` push_back are new; the 32 `__copy__`/
# `__deepcopy__` rows are R9's), and the 5 NEW time-parameterization rows.

G1_ACCEPTED = {
    CL: Counter(
        {
            **{
                (f"is{t}Instruction", "free-function-as-method"): 1
                for t in ("SetAnalog", "SetDigital", "SetTool", "Timer", "Wait")
            },
            **{(f"{poly}.getType", "type-index"): 1 for poly in POLY_INTERFACES},
            **{(i, "type-erasure-interface"): 1 for i in POLY_INTERFACES.values()},
            **{
                (f"{poly}.get{poly.removesuffix('Poly')}", "type-erasure-interface"): 1
                for poly in POLY_INTERFACES
            },
            ("MoveInstruction.setWaypoint", "reference-setter"): 1,
            ("MoveInstructionPoly.setWaypoint", "reference-setter"): 1,
            ("CompositeInstruction.setUserData", "reference-setter"): 1,
            **{
                (f"CompositeInstruction.{leaf}", "vector-forwarding-no-python-form"): 1
                for leaf in (*VECTOR_FORWARDING_LEAVES, "insert")
            },
            ("ProfileDictionary.getProfile", "profile-key-template"): 1,
            ("CompositeInstruction.push_back", "poly-alternative-overloads"): 8,
            ("MoveInstruction.__init__", "poly-alternative-overloads"): 6,
            **{
                (f"{c}.{dunder}", "clone-as-copy"): 1
                for c in (*POLY_INTERFACES, *CONCRETE)
                for dunder in ("__copy__", "__deepcopy__")
            },
        }
    ),
    TP: Counter(
        {
            ("InstructionsTrajectory.__init__", "reference-wrapper-view"): 1,
            **{
                (c, "algorithm-internal-namespace"): 1
                for c in ("Path", "PathSegment", "PathData", "Trajectory")
            },
        }
    ),
}


@pytest.mark.parametrize("module", G1)
def test_g1_rules_accept_exactly_the_ledger_rows(g1_reports, module):
    rows = Counter((a.symbol, a.rule) for a in g1_reports[module].accepted if a.rule in G1_RULES)
    assert G1_ACCEPTED[CL].total() == 82
    assert G1_ACCEPTED[TP].total() == 5
    assert rows == G1_ACCEPTED[module]


# Task 8, C1 (rename; the binding side is #329): the binding's `nb::class_` maps each
# abbreviation to its C++ class (0b T3), so the classes are covered and their members audited;
# the abbreviations stay deviations until #329 renames them.


@pytest.mark.parametrize(("cpp", "python"), TP_PROFILES.items())
def test_tp_profile_is_covered_and_its_abbreviation_stays_open(g1_reports, cpp, python):
    report = g1_reports[TP]
    assert cpp not in {g.symbol for g in report.gaps}
    assert (python, audit.Kind.CLASS) in {(d.name, d.kind) for d in report.deviations}


@pytest.mark.parametrize("cpp", TP_PROFILES)
def test_tp_profile_members_are_covered(g1_reports, cpp):
    """The members the abbreviation hid are audited; #344 binds their `__eq__`/`__ne__`
    (00-value-equality), so neither is open."""
    assert not {f"{cpp}.__eq__", f"{cpp}.__ne__"} & open_names(g1_reports[TP])


def test_isp_minimum_time_delta_is_covered(g1_reports):
    """Binding-fix g1/21, hidden behind the abbreviation before 0b T3; #344 binds it."""
    symbol = "IterativeSplineParameterizationCompositeProfile.minimum_time_delta"
    assert symbol not in open_names(g1_reports[TP])


# Task 9: every group 1 table entry fires; every group 1 rule is in the report.


def test_every_g1_table_entry_fires(g1_reports):
    """A stale entry (renamed upstream, or the binding fix landed) produces no Accepted row."""
    for (module, name), rule in audit.ACCEPTED_SYMBOLS.items():
        if module in g1_reports:
            assert (name, rule) in accepted_pairs(g1_reports[module]), name
    for (module, name), instances in audit.TEMPLATE_INSTANCES.items():
        if module in g1_reports:
            for symbol in (name, *instances):
                pair = (symbol, "template-instance-name")
                assert pair in accepted_pairs(g1_reports[module]), symbol
    for (module, name, _), rule in audit.ACCEPTED_CPP_OVERLOADS.items():
        if module in g1_reports:
            assert (name, rule) in accepted_pairs(g1_reports[module]), name
    for (module, name, _), rule in audit.ACCEPTED_PY_OVERLOADS.items():
        if module in g1_reports:
            assert (name, rule) in accepted_pairs(g1_reports[module]), name


@pytest.mark.parametrize("rule", G1_RULES)
def test_g1_rule_is_justified_in_the_report(rule):
    """Reads the committed report: passes once it is regenerated with the group 1 rules."""
    assert audit.ACCEPTED[rule]
    assert f"`{rule}`" in audit.REPORT_PATH.read_text(encoding="utf-8")


# The rows of the merged binding fixes (#344): what the audit could not see.

# `as<T>()` of the waypoint and move-instruction Polys, bound per instance (#268).
POLY_AS = {
    "WaypointPoly.as": tuple(
        f"WaypointPoly.as{t}Waypoint" for t in ("Cartesian", "Joint", "State")
    ),
    "CartesianWaypointPoly.as": ("CartesianWaypointPoly.asCartesianWaypoint",),
    "JointWaypointPoly.as": ("JointWaypointPoly.asJointWaypoint",),
    "StateWaypointPoly.as": ("StateWaypointPoly.asStateWaypoint",),
    "MoveInstructionPoly.as": ("MoveInstructionPoly.asMoveInstruction",),
}


@pytest.mark.parametrize(("template", "instances"), POLY_AS.items())
def test_real_poly_as_instances_accepted(g1_reports, template, instances):
    report = g1_reports[CL]
    assert audit.TEMPLATE_INSTANCES[(CL, template)] == instances
    for name in (template, *instances):
        assert (name, "template-instance-name") in accepted_pairs(report), name
    assert not {template, *instances} & open_names(report)


def test_real_composite_erase_is_the_delitem_protocol(g1_reports):
    """`erase(p)`/`erase(first, last)` are bound as `__delitem__` of an int and a slice (#269)."""
    report = g1_reports[CL]
    assert ("CompositeInstruction.__delitem__", "container-protocol") in accepted_pairs(report)
    assert not {"CompositeInstruction.erase", "CompositeInstruction.__delitem__"} & open_names(
        report
    )


def test_real_composite_range_insert_has_no_python_form(g1_reports):
    """composite_instruction.h:386 `template <class InputIt> insert(pos, first, last)` (#269)."""
    report = g1_reports[CL]
    pair = ("CompositeInstruction.insert", "vector-forwarding-no-python-form")
    assert pair in accepted_pairs(report)
    assert "CompositeInstruction.insert" not in open_names(report)


def test_real_push_back_of_a_move_instruction_is_a_poly_alternative(g1_reports):
    """`push_back(MoveInstruction)`: `InstructionPoly(const MoveInstructionInterface&)` converts
    it (instruction_poly.h:139)."""
    report = g1_reports[CL]
    assert "MoveInstruction" in audit.POLY_ALTERNATIVES["InstructionPoly"]
    assert "CompositeInstruction.push_back" not in open_names(report)


def test_real_set_user_data_is_a_reference_setter(g1_reports, cl_tu):
    """R10's premise: `getUserData()` returns a non-const reference, C++ has no setter."""
    members = list(_record(cl_tu, "CompositeInstruction").get_children())
    assert any(
        m.spelling == "getUserData"
        and m.result_type.kind == audit.ci.TypeKind.LVALUEREFERENCE
        and not m.result_type.get_pointee().is_const_qualified()
        for m in members
    )
    assert not any(m.spelling == "setUserData" for m in members)
    report = g1_reports[CL]
    assert ("CompositeInstruction.setUserData", "reference-setter") in accepted_pairs(report)
    assert "CompositeInstruction.setUserData" not in open_names(report)


def test_real_path_parameter_covers_a_string_path(g1_reports):
    """`toDelimitedFile(composite, const std::string& file_path, …)` bound with a
    `std::filesystem::path` (`str | os.PathLike`): it takes every `str` the C++ takes."""
    assert "toDelimitedFile" not in open_names(g1_reports[CL])


def test_undefined_upstream_function_is_accepted(g1_reports):
    """utils.h:242 declares `validateSeedStructure`; the 0.35.0 library defines no such symbol,
    so a binding cannot link it (the binding says so)."""
    report = g1_reports[CL]
    assert ("validateSeedStructure", "declared-never-defined") in accepted_pairs(report)
    assert "validateSeedStructure" not in open_names(report)
