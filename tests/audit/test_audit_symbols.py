"""Contract tests of the stored C++ API (`scripts/audit_symbols.py`): the JSON form keeps
order and every key type, and `resolve_bound` decides R13 exactly as the walk did. Needs no
libclang.
"""

import json

import pytest

from ._free import load_free

sym = load_free("audit_symbols")

NESTED = ("map", ("str",), ("seq", ("union", ("class", "a::B"), ("ndarray", None))))


def _overload(**changes):
    fields = {
        "arity": sym.Arity(1, None),
        "out_params": ("std::string",),
        "location": "tesseract/x.h:7",
        "params": (frozenset({NESTED, ("class", "Items")}), None, frozenset({("bytes",)})),
        "out_positions": (1,),
        "byte_buffers": (2,),
        "yaml_position": 0,
        "names": ("a", "b", "c"),
    }
    return sym.CppOverload(**{**fields, **changes})


def _api():
    return {
        "Zeta": sym.CppSymbol("Zeta", sym.Kind.CLASS, "tesseract/x.h:3", qualified="ns::Zeta"),
        "Zeta.run": sym.CppSymbol(
            "Zeta.run",
            sym.Kind.METHOD,
            "tesseract/x.h:7",
            overloads=[
                _overload(),
                _overload(
                    absent_ok="type-index",
                    dependency=sym.Dependency(unless_bound=frozenset({"v::A", "v::B"})),
                ),
            ],
            inherited_from="ns::Base",
        ),
        "Alpha.field": sym.CppSymbol(
            "Alpha.field", sym.Kind.FIELD, "tesseract/x.h:9", dependency=sym.Dependency(always=True)
        ),
    }


def _stored(api):
    """As the cache stores and reads it back: sorted keys, tuples as lists."""
    return json.loads(json.dumps(sym.encode_symbols(api), sort_keys=True))


def test_the_stored_api_reads_back_equal_and_in_order():
    api = _api()
    back = sym.decode_symbols(_stored(api))
    assert back == api
    assert list(back) == ["Zeta", "Zeta.run", "Alpha.field"]


def test_keys_read_back_as_tuples_in_frozensets():
    back = sym.decode_symbols(_stored(_api()))
    keys = back["Zeta.run"].overloads[0].params[0]
    assert isinstance(keys, frozenset)
    assert NESTED in keys
    assert back["Zeta.run"].overloads[0].params[1] is None


@pytest.mark.parametrize("garbage", [None, {"a": 1}, [{"name": "x"}], [{"name": "x", "kind": "?"}]])
def test_a_value_that_is_no_stored_api_is_loud(garbage):
    with pytest.raises(sym.SymbolShapeError):
        sym.decode_symbols(garbage)


def test_a_dependency_holds_unless_every_class_is_bound():
    dep = sym.Dependency(unless_bound=frozenset({"v::A", "v::B"}))
    assert dep.holds(lambda q: q == "v::A")
    assert not dep.holds(lambda q: True)
    assert sym.Dependency(always=True, unless_bound=frozenset({"v::A"})).holds(lambda q: True)
    assert not sym.NO_DEPENDENCY.holds(lambda q: False)


def test_dependencies_of_several_types_unite():
    a = sym.Dependency(unless_bound=frozenset({"v::A"}))
    b = sym.Dependency(always=True, unless_bound=frozenset({"v::B"}))
    assert a | b == sym.Dependency(always=True, unless_bound=frozenset({"v::A", "v::B"}))
    assert a | sym.NO_DEPENDENCY == a


def test_resolve_sets_the_rule_where_the_dependency_holds():
    resolved = sym.resolve_bound(_api(), lambda q: q == "v::A")
    plain, dependent = resolved["Zeta.run"].overloads
    assert plain.absent_ok is None
    assert dependent.absent_ok == sym.UNBOUND_DEPENDENCY
    assert resolved["Alpha.field"].absent_ok == sym.UNBOUND_DEPENDENCY
    assert dependent.dependency == sym.NO_DEPENDENCY
    assert list(resolved) == list(_api())


def test_resolve_keeps_the_later_rule_where_every_class_is_bound():
    """`type-index` comes after R13 in the rule order: it applies when R13 does not."""
    resolved = sym.resolve_bound(_api(), lambda q: True)
    assert resolved["Zeta.run"].overloads[1].absent_ok == "type-index"


def test_resolve_does_not_ask_what_is_bound_without_a_dependency():
    """The first `is_bound` call reads every binding's bound classes."""
    api = {"f": sym.CppSymbol("f", sym.Kind.FUNCTION, "x.h:1", overloads=[_overload()])}
    api["g"] = sym.CppSymbol("g", sym.Kind.FIELD, "x.h:2", dependency=sym.Dependency(always=True))
    resolved = sym.resolve_bound(api, lambda q: pytest.fail("asked"))
    assert resolved["g"].absent_ok == sym.UNBOUND_DEPENDENCY


def test_resolve_leaves_its_input_alone():
    api = _api()
    sym.resolve_bound(api, lambda q: False)
    assert api == _api()
