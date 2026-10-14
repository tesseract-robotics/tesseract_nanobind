"""Contract tests of overload matching by parameter type (#247): scripts/audit_types.py and
its use in scripts/audit_bindings.py.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import ast
import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
TYPED_PREFIX = "tesseract/typed/"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


types = _load("audit_types")
audit = _load("audit_bindings")


@pytest.fixture(scope="module")
def typed_report():
    return audit.audit_tu(
        "typed",
        FIXTURES / "typed_bindings.cpp",
        FIXTURES / "_typed.pyi",
        FIXTURES / "typed_init.py",
        TYPED_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(FIXTURES,),
    )


@pytest.fixture(scope="module")
def typed_cpp():
    tu = audit.parse_tu(FIXTURES / "typed_bindings.cpp", (FIXTURE_INCLUDE,))
    headers = audit.audited_headers(tu, TYPED_PREFIX, (FIXTURE_INCLUDE, *audit.INCLUDE_DIRS))
    return audit.cpp_api(tu, headers)


@pytest.fixture(scope="module")
def typed_py():
    stub = FIXTURES / "_typed.pyi"
    return audit.py_api(ast.parse(stub.read_text(encoding="utf-8")), "_typed.pyi")


def test_every_mapping_rule_gives_the_stub_key(typed_cpp, typed_py):
    """Each C++ parameter of `kinds` maps to the key of the annotation nanobind writes for it."""
    [cpp] = typed_cpp["kinds"].overloads
    [py] = typed_py.symbols["kinds"].overloads
    names = [a.arg for a in ast.parse((FIXTURES / "_typed.pyi").read_text()).body[-1].args.args]
    assert len(cpp.params) == len(py.keys) == len(names)
    mismatched = [
        (name, sorted(c, key=repr) if c else c, p)
        for name, c, p in zip(names, cpp.params, py.keys, strict=True)
        if types.verdict(c, p) is not True
    ]
    assert mismatched == []


@pytest.mark.parametrize(
    ("annotation", "key"),
    [
        ("str", ("str",)),
        ("bool", ("bool",)),
        ("bytes", ("bytes",)),
        ("str | os.PathLike", ("path",)),
        ("list[str]", ("seq", ("str",))),
        ("dict[str, float]", ("map", ("str",), ("float",))),
        ("Item | None", ("class", "Item")),
        ("AddTrajectoryLinkCommand.Method", ("class", "AddTrajectoryLinkCommand.Method")),
        ("fixture_remote._fixture_remote.Runner", ("class", "Runner")),
        ("NDArray", ("ndarray", None)),
        ("object", None),
        ('"tesseract::fixture::Owner"', None),  # an unbound C++ type: no key
    ],
)
def test_py_key(annotation, key):
    assert types.py_key(ast.parse(annotation, mode="eval").body) == key


def test_unknown_nested_in_a_key_is_neither_a_match_nor_a_mismatch():
    assert types.verdict(frozenset({("seq", None)}), ("seq", ("str",))) is None
    assert types.verdict(frozenset({("seq", ("int",))}), ("seq", ("str",))) is False
    assert types.verdict(frozenset({("seq", ("str",))}), ("seq", ("str",))) is True
    assert types.verdict(None, ("str",)) is None


def test_typedef_name_is_an_accepted_key(typed_cpp):
    """`Items` is bound as an opaque class: the typedef's name matches besides `seq[Item]`."""
    [ov] = typed_cpp["take"].overloads
    assert ov.params == (frozenset({("seq", ("class", "Item")), ("class", "Items")}),)


def test_const_and_rvalue_pairs_collapse_to_one_overload(typed_cpp):
    assert len(typed_cpp["Seq.at"].overloads) == 1
    assert len(typed_cpp["Seq.push"].overloads) == 1


def test_same_arity_unbound_overload_is_a_gap(typed_report):
    """`pick(vector<string>)` is not covered by the bound `pick(str)` of the same arity."""
    [row] = [g for g in typed_report.gaps if g.symbol == "pick"]
    assert (row.kind, row.arity) == (audit.Kind.FUNCTION, "1")
    assert row.location.endswith("tesseract/typed/typed.h:42")


def test_typed_gaps_exact(typed_report):
    assert {(g.symbol, g.kind, g.arity) for g in typed_report.gaps} == {
        ("pick", audit.Kind.FUNCTION, "1"),
        ("roll", audit.Kind.FUNCTION, "1"),  # the stub's Sequence[Shape] is no Sequence[Ball]
    }


def test_python_overload_with_no_typed_cpp_match_is_a_deviation(typed_report):
    """`pick(flag: bool)` has the arity of both C++ overloads and the types of neither."""
    assert {(d.name, d.kind, d.arity) for d in typed_report.deviations} == {
        ("pick", audit.Kind.OVERLOAD, "1"),
        ("roll", audit.Kind.OVERLOAD, "1"),
    }


def test_derived_class_argument_binds_the_base_parameter(typed_report):
    """`apply(Command)` bound once per (transitive) subclass: covered, and no deviation."""
    rows = {g.symbol for g in typed_report.gaps} | {d.name for d in typed_report.deviations}
    assert "apply" not in rows


def test_type_fallbacks_counted(typed_report):
    """`put<T>` (template parameter) and `guess` (`object`) are covered by arity alone."""
    assert typed_report.type_fallbacks == 2


def test_report_renders_type_fallbacks(typed_report):
    prov = {"tesseract-robotics": "==0.0.0", "libclang": "clang version 0", "stubs": "abc1234"}
    text = audit.render_markdown([typed_report], prov)
    assert (
        "| module | covered | gaps | deviations | accepted | quoted types | type fallbacks |"
        in text
    )
    assert "| typed | " in text and " | 2 |\n" in text
    assert "Overloads are matched by arity only" not in text


def test_json_has_type_fallbacks(typed_report):
    prov = {"tesseract-robotics": "==0.0.0", "libclang": "clang version 0", "stubs": "abc1234"}
    assert '"type_fallbacks": 2' in audit.to_json([typed_report], prov)
