"""Diff a binding's C++ API against its stub; audit one module."""

from __future__ import annotations

import functools
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import MappingProxyType

import audit_types
from audit_ownership import (
    AUDITED_HEADER_PREFIX,
    SRC,
    STUB_ROOT,
    UNAUDITED_MODULES,
    UnauditedModuleError,
    UnknownModuleError,
    binding_modules,
    binding_source,
    committed_stub,
    header_auditor,
    is_unaudited,
    owner_prefixes,
)
from audit_parse import (
    INCLUDE_DIRS,
    STRINGSTREAM,
    Arity,
    CppOverload,
    CppSymbol,
    HeaderParseError,
    Kind,
    audited_headers,
    binding_module,
    bound_classes,
    cpp_api,
    decl_names,
    direct_include_spellings,
    foreign_headers,
    header_rel,
    own_header_lines,
    parse_tu,
    prefix_headers,
)
from audit_rules import (
    ACCEPTED_CPP_OVERLOADS,
    ACCEPTED_PY_OVERLOADS,
    ACCEPTED_SYMBOLS,
    EIGEN_VALUE_WRAPPERS,
    POLY_ALTERNATIVES,
    PROTOCOL_MEMBERS,
    PROTOCOL_RULES,
    TEMPLATE_INSTANCE_RULE,
    TEMPLATE_INSTANCES,
    VECTOR_FORWARDER_MEMBERS,
    VECTOR_FORWARDING_MEMBERS,
    WHOLE_SYMBOL_ABSENT_OK,
    _k_alias,
)
from audit_stub import (
    DUNDER_OPERATORS,
    ENUM_BASES,
    NOT_AN_OVERLOAD,
    Accepted,
    Deviation,
    PyApi,
    PyOverload,
    PySymbol,
    QuotedType,
    init_accepted,
    init_findings,
    load_stub,
    py_api,
    rel,
)
from audit_types import NO_ANCESTORS, NO_BASES, Ancestors, Bases, TypeKey


class PrefixMismatchError(ValueError):
    """`audit_tu` got a module prefix that disagrees with that module's `prefixes` entry."""


def resolve_module(module: str) -> str:
    """Validate a short module name for auditing.

    Raises:
        UnknownModuleError: not a binding module.
        UnauditedModuleError: listed in `UNAUDITED_MODULES`.
        StubMissingError: no committed stub.
    """
    known = binding_modules()
    if module not in known:
        raise UnknownModuleError(f"{module!r} is not a binding module; known: {', '.join(known)}")
    if module in UNAUDITED_MODULES:
        raise UnauditedModuleError(f"{module!r} is not audited: {UNAUDITED_MODULES[module][0]}")
    load_stub(committed_stub(module))
    return module


@dataclass(frozen=True, order=True)
class Gap:
    symbol: str
    kind: Kind
    location: str
    arity: str = NOT_AN_OVERLOAD


@dataclass
class ModuleReport:
    module: str
    covered: int
    gaps: list[Gap]
    deviations: list[Deviation]
    accepted: list[Accepted]
    quoted: list[QuotedType]
    # Headers under this module's prefix that its binding does not #include but another does:
    # header spelling -> the module that audits it (`header_auditor`).
    delegated: dict[str, str] = field(default_factory=dict)
    # C++ overloads covered by arity alone: a parameter type `audit_types` has no key for.
    type_fallbacks: int = 0


@dataclass(frozen=True)
class Cover:
    """How Python overloads cover a C++ overload."""

    rule: str  # "exact" or an ACCEPTED rule
    typed: bool  # every compared parameter type is known; False: covered by arity alone


def _fits(
    params: tuple[frozenset[TypeKey] | None, ...],
    arity: Arity,
    pys: list[PyOverload],
    ancestors: Ancestors,
) -> dict[PyOverload, bool]:
    """Python overloads of an overlapping arity whose parameter types do not contradict
    `params`, each mapped to whether every compared type was known."""
    out: dict[PyOverload, bool] = {}
    for p in pys:
        if p.arity.overlaps(arity):
            verdict = audit_types.signature_verdict(params, p.keys, ancestors)
            if verdict is not False:
                out[p] = verdict is True
    return out


def _upcasts_to(
    derived: frozenset[TypeKey] | None, base: frozenset[TypeKey] | None, ancestors: Ancestors
) -> bool:
    """`derived` is a container of a class derived from `base`'s container element."""
    for d in derived or ():
        for b in base or ():
            if (
                d[0] == b[0] == "seq"
                and d[1] is not None
                and b[1] is not None
                and d[1][0] == b[1][0] == "class"
                and b[1][1] in ancestors.get(d[1][1], frozenset())
            ):
                return True
    return False


def _derived_element_sibling(
    ov: CppOverload, sym: CppSymbol, pys: list[PyOverload], ancestors: Ancestors
) -> bool:
    """A covered sibling overload equals `ov` but for container elements that `ov` takes as a
    derived class: a C++ convenience that upcasts (`CompoundMesh(vector<ConvexMesh>)`), which
    Python reaches through the base-element overload with any subclass."""
    for other in sym.overloads:
        if other is ov or other.arity != ov.arity or len(other.params) != len(ov.params):
            continue
        if _cover(other, pys, ancestors) is None:
            continue
        pairs = list(zip(ov.params, other.params))
        if any(_upcasts_to(a, b, ancestors) for a, b in pairs) and all(
            a == b or _upcasts_to(a, b, ancestors) for a, b in pairs
        ):
            return True
    return False


# Keys of a parameter that takes a config file path or YAML text: a `YAML::Node` sibling (T1).
YAML_TEXT_KEYS = frozenset({("str",), ("path",)})


def _yaml_sibling(
    ov: CppOverload, sym: CppSymbol, pys: list[PyOverload], ancestors: Ancestors = NO_ANCESTORS
) -> bool:
    """A covered sibling overload of `sym` equals `ov` but takes a config file path or YAML
    text where `ov` takes its `YAML::Node` (T1, the `yaml-node` premise)."""
    pos = ov.yaml_position
    if pos is None:
        return False
    for other in sym.overloads:
        if other is ov or other.arity != ov.arity or len(other.params) != len(ov.params):
            continue
        if not YAML_TEXT_KEYS & (other.params[pos] or frozenset()):
            continue
        if any(a != b for i, (a, b) in enumerate(zip(ov.params, other.params)) if i != pos):
            continue
        if _cover(other, pys, ancestors) is not None:
            return True
    return False


def _covered(rule: str, fits: Mapping[PyOverload, bool]) -> Cover | None:
    return Cover(rule, any(fits.values())) if fits else None


def _cover(
    ov: CppOverload, pys: list[PyOverload], ancestors: Ancestors = NO_ANCESTORS
) -> Cover | None:
    """How a C++ overload is covered by Python overloads, or None.

    Out-param rules are tried first: `checkTrajectory`'s 5-parameter C++ overload would
    otherwise pair by raw arity with the wrong 5-argument Python overload. A Python overload
    covers only if its parameter types do not contradict the C++ ones (#247).
    """
    if ov.out_params:
        reduced = ov.arity.reduced(len(ov.out_params))
        fits = _fits(ov.returned_out_params(), reduced, pys, ancestors)
        if set(ov.out_params) == {STRINGSTREAM}:
            if cover := _covered(
                "stringstream", {p: t for p, t in fits.items() if p.returns == "str"}
            ):
                return cover
        tuples = {p: t for p, t in fits.items() if p.returns.startswith("tuple[")}
        if cover := _covered("out-param", tuples):
            return cover
        if ov.returns_void and len(ov.out_params) == 1 and fits:
            return _covered(
                "out-param", fits
            )  # nothing else to return: the out-param is the result
    if ov.raw_buffer:
        return None  # no Python overload binds a raw pointer; matching arity proves nothing
    fits = _fits(ov.params, ov.arity, pys, ancestors)
    if ov.byte_buffers:
        return _covered(
            "byte-buffer",
            {
                p: t
                for p, t in fits.items()
                if all(i < len(p.params) and p.params[i] == "bytes" for i in ov.byte_buffers)
            },
        )
    return _covered(ov.mapped or "exact", fits)


def _protocol_rule(name: str, cpp: dict[str, CppSymbol], tu_names: frozenset[str]) -> str | None:
    """The ACCEPTED rule covering a Python protocol dunder that has no C++ symbol, or None."""
    owner, _, leaf = name.rpartition(".")
    if leaf not in PROTOCOL_RULES:
        return None
    rule, members = PROTOCOL_RULES[leaf]
    if owner in cpp:
        ok = all(f"{owner}.{m}" in cpp for m in members)
    else:
        ok = all(DUNDER_OPERATORS.get(m, m) in tu_names for m in members)
    return rule if ok else None


@functools.cache
def _stub_py_api(stub: Path) -> PyApi:
    """A committed stub's API, parsed once per audit run."""
    return py_api(load_stub(stub), rel(stub))


def _base_stub(base: str, py: PyApi, stub_roots: Sequence[Path]) -> tuple[PyApi, str] | None:
    """The stub declaring `base` and the class name in it, or None if no stub declares it.

    A plain name is a class of the same stub. nanobind spells a base bound in another
    extension by its module path (`tesseract_robotics.tesseract_common._tesseract_common.X`),
    which maps to `<root>/tesseract_robotics/tesseract_common/_tesseract_common.pyi`.
    """
    module, _, cls = base.rpartition(".")
    if not module:
        return py, cls
    stub = Path(*module.split(".")).with_suffix(".pyi")
    for root in stub_roots:
        if (root / stub).is_file():
            return _stub_py_api(root / stub), cls
    return None


def _py_member(name: str, py: PyApi, stub_roots: Sequence[Path] = (SRC,)) -> PySymbol | None:
    """The stub symbol for `Class.member`, looked up through the stub's base classes.

    nanobind subclasses inherit bound members, also from a base bound in another
    extension, so a C++ override needs no own binding. A base whose stub is not under
    `stub_roots` cannot be checked, and its members stay gaps.
    """
    if name in py.symbols:
        return py.symbols[name]
    owner, _, member = name.rpartition(".")
    cls = py.symbols.get(owner)
    if cls is None:
        return None
    for base in cls.bases:
        resolved = _base_stub(base, py, stub_roots)
        if resolved is None:
            continue
        base_py, base_cls = resolved
        found = _py_member(f"{base_cls}.{member}", base_py, stub_roots)
        if found is not None:
            return found
    return None


def _binding_stub(name: str, stub_rel: str, stub_roots: Sequence[Path]) -> tuple[str, PyApi] | None:
    """The other extension's module (the stub's stem without `_`) and stub under `stub_roots`
    declaring top-level class `name`, or None."""
    if "." in name:
        return None
    return next(
        (
            (stub.stem.removeprefix("_"), api)
            for root in stub_roots
            for stub in _stubs_under(root)
            if rel(stub) != stub_rel and name in (api := _stub_py_api(stub)).symbols
        ),
        None,
    )


# Module -> its `bound_classes` (top-level Python class name -> qualified C++ class).
BoundLookup = Callable[[str], Mapping[str, str]]
NO_ALIAS: Mapping[str, str] = MappingProxyType({})


def no_bound(module: str) -> Mapping[str, str]:
    """A lookup that knows no binding: classes keep their by-name identity."""
    return NO_ALIAS


@functools.cache
def _bound_classes_of(cpp: Path, extra_include_dirs: tuple[Path, ...]) -> Mapping[str, str]:
    """`bound_classes` of one binding source, parsed with bodies once per audit run."""
    return MappingProxyType(bound_classes(parse_tu(cpp, extra_include_dirs, bodies=True)))


@functools.cache
def bound_index(module: str) -> Mapping[str, str]:
    """`bound_classes` of a binding module's TU; empty for any other name (a fixture stub)."""
    if module not in binding_modules():
        return NO_ALIAS
    return _bound_classes_of(binding_source(module), ())


def _same_class(known: str | None, qualified: str) -> bool:
    """Positive evidence only: two classes differ when both qualified names are known."""
    return known is None or not qualified or known == qualified


def _unaliased_key(key: TypeKey, unalias: Mapping[str, str]) -> TypeKey:
    """`key` with each Python class this module renames spelled by its C++ name, at any depth
    (`DescartesStateD` -> `State`), so it compares with the C++ parameter keys."""
    if key is None:
        return None
    if key[0] == "class":
        return ("class", unalias.get(key[1], key[1]))
    return (key[0], *(k if isinstance(k, str) else _unaliased_key(k, unalias) for k in key[1:]))


def _unaliased_api(py: PyApi, unalias: Mapping[str, str]) -> PyApi:
    """`py` with every overload's parameter keys through `_unaliased_key`."""
    if not unalias:
        return py
    return replace(
        py,
        symbols={
            name: replace(
                sym,
                overloads=[
                    replace(o, keys=tuple(_unaliased_key(k, unalias) for k in o.keys))
                    for o in sym.overloads
                ],
            )
            for name, sym in py.symbols.items()
        },
    )


def _renamed(name: str, alias: Mapping[str, str]) -> str:
    """`name` with its top-level class replaced through `alias` (C++ <-> Python name)."""
    top, dot, rest = name.partition(".")
    return alias.get(top, top) + dot + rest


@functools.cache
def _stubs_under(root: Path) -> tuple[Path, ...]:
    """Every extension stub (`_<module>.pyi`) below `root`, listed once per audit run."""
    return tuple(sorted(root.rglob("_*.pyi")))


def _ancestor_missing(
    name: str, cpp: dict[str, CppSymbol], py: PyApi, alias: Mapping[str, str] = NO_ALIAS
) -> bool:
    parts = name.split(".")
    owners = (".".join(parts[:i]) for i in range(1, len(parts)))
    return any(o in cpp and _renamed(o, alias) not in py.symbols for o in owners)


def _inherited_from_bound(
    sym: CppSymbol, api: PyApi, stub_roots: Sequence[Path], bound: BoundLookup
) -> bool:
    """`sym` is inherited from a class that `api` or a stub under `stub_roots` binds (A).

    That class's own rows audit the member. A stub's module is its stem without the `_`;
    its `bound` names the class's Python name. A name `bound` does not know counts as the
    class (positive evidence only, as for classes bound elsewhere).
    """
    base = sym.inherited_from
    if base is None:
        return False
    if base in api.symbols:
        return True
    for root in stub_roots:
        for stub in _stubs_under(root):
            renamed = bound(stub.stem.removeprefix("_"))
            names = {p for p, q in renamed.items() if q == base or q.endswith(f"::{base}")}
            if base not in renamed:
                names.add(base)
            symbols = _stub_py_api(stub).symbols
            if any(n in symbols and symbols[n].kind is Kind.CLASS for n in names):
                return True
    return False


def poly_ancestors(ancestors: Ancestors) -> Ancestors:
    """`ancestors` with each Poly added to the ancestors of its alternatives (T9)."""
    out = {k: set(v) for k, v in ancestors.items()}
    for poly, alternatives in POLY_ALTERNATIVES.items():
        for alt in alternatives:
            out.setdefault(alt, set()).add(poly)
    return MappingProxyType({k: frozenset(v) for k, v in out.items()})


def _is_yaml_text_factory(ps: PySymbol | None) -> bool:
    """`ps` is a static `from_yaml` taking one `str`, YAML text: the Python form of its class's
    `YAML::Node` constructor (C6 6a)."""
    return ps is not None and any(p.static and p.params == ("str",) for p in ps.overloads)


def _stands_for_yaml_ctor(leaf: str, ps: PySymbol, ctor: CppSymbol | None) -> bool:
    """A Python-only `from_yaml` is the YAML-text form of a `YAML::Node` constructor of its
    class (C6 6a): a static `from_yaml(text: str)` beside a `yaml-node` constructor overload."""
    return (
        leaf == "from_yaml"
        and _is_yaml_text_factory(ps)
        and ctor is not None
        and any(ov.absent_ok == "yaml-node" for ov in ctor.overloads)
    )


def _absent_ok_accepts(
    ov: CppOverload, sym: CppSymbol, ps: PySymbol, from_yaml: PySymbol | None = None
) -> bool:
    """Whether the overload's structural `absent_ok` accepts leaving it unbound; `yaml-node`
    only when its premise holds (T1): a YAML-text sibling overload, or, for a constructor, the
    class's own static `from_yaml(text)` (`from_yaml`, C6 6a). When it does not, the overload
    falls through to `ACCEPTED_CPP_OVERLOADS` and the later rules."""
    if ov.absent_ok == "yaml-node":
        # A YAML sibling is declared on the class itself (a ctor, never inherited), so C++
        # ancestry cannot supply one.
        return _yaml_sibling(ov, sym, ps.overloads, NO_ANCESTORS) or (
            sym.kind is Kind.CONSTRUCTOR and _is_yaml_text_factory(from_yaml)
        )
    return ov.absent_ok is not None


def _whole_symbol_rule(sym: CppSymbol) -> str | None:
    """The rule accepting an entirely unbound symbol through its structural `absent_ok`: the
    symbol's own (fields), or the one `WHOLE_SYMBOL_ABSENT_OK` rule every overload carries.
    Overloads carrying two different rules stay a gap: no single justification covers them."""
    if sym.absent_ok in WHOLE_SYMBOL_ABSENT_OK:
        return sym.absent_ok
    rules = {ov.absent_ok for ov in sym.overloads}
    if sym.overloads and len(rules) == 1 and rules <= WHOLE_SYMBOL_ABSENT_OK:
        return next(iter(rules))
    return None


def _bound_as_method(name: str, sym: CppSymbol, py: PyApi, stub_roots: Sequence[Path]) -> bool:
    """A free function whose every overload takes one class `T` and is bound as `T.name` (T7)."""
    if "." in name or sym.kind is not Kind.FUNCTION or not sym.overloads:
        return False
    for ov in sym.overloads:
        if ov.arity != Arity(1, 1) or len(ov.params) != 1:
            return False
        classes = [k[1] for k in ov.params[0] or () if k is not None and k[0] == "class"]
        if not any(_py_member(f"{c}.{name}", py, stub_roots) for c in classes):
            return False
    return True


def _unbound_rule(
    module: str,
    name: str,
    sym: CppSymbol,
    cpp: Mapping[str, CppSymbol],
    py: PyApi,
    stub_roots: Sequence[Path],
) -> str | None:
    """The ACCEPTED rule for a C++ symbol Python does not bind, or None (a gap)."""
    owner, _, leaf = name.rpartition(".")
    instances = TEMPLATE_INSTANCES.get((module, name))
    if instances and all(_py_member(i, py, stub_roots) for i in instances):
        return TEMPLATE_INSTANCE_RULE
    if rule := ACCEPTED_SYMBOLS.get((module, name)) or _whole_symbol_rule(sym):
        return rule
    if sym.kind is Kind.OPERATOR and (module, owner) in EIGEN_VALUE_WRAPPERS:
        return "eigen-value-wrapper"
    if _bound_as_method(name, sym, py, stub_roots):
        return "free-function-as-method"
    if (
        owner
        and leaf in VECTOR_FORWARDING_MEMBERS
        and all(f"{owner}.{m}" in cpp for m in VECTOR_FORWARDER_MEMBERS)
    ):
        return "vector-forwarding-no-python-form"
    return None


def _uncovered_rule(
    module: str,
    name: str,
    ov: CppOverload,
    sym: CppSymbol,
    ps: PySymbol,
    ancestors: Ancestors,
    with_polys: Ancestors,
    from_yaml: PySymbol | None = None,
) -> str | None:
    """The ACCEPTED rule for a C++ overload no Python overload covers, or None (a gap).

    Order: a Poly alternative's overload covers it; the structural `absent_ok`
    (`_absent_ok_accepts`); `ACCEPTED_CPP_OVERLOADS`; a reference-wrapper view beside a covered
    sibling (R4); a derived-element container sibling.
    """
    if _cover(ov, ps.overloads, with_polys) is not None:
        return "poly-alternative-overloads"
    if _absent_ok_accepts(ov, sym, ps, from_yaml):
        return ov.absent_ok
    if rule := ACCEPTED_CPP_OVERLOADS.get((module, name, str(ov.arity))):
        return rule
    if ov.reference_wrapper_view and any(
        _cover(o, ps.overloads, ancestors) is not None for o in sym.overloads if o is not ov
    ):
        return "reference-wrapper-view"
    if _derived_element_sibling(ov, sym, ps.overloads, ancestors):
        return "derived-element-container"
    return None


def match(
    module: str,
    cpp: dict[str, CppSymbol],
    tu_names: frozenset[str],
    py: PyApi,
    stub_rel: str,
    stub_roots: Sequence[Path] = (SRC,),
    ancestors: Ancestors = NO_ANCESTORS,
    bound: BoundLookup = no_bound,
    bases: Bases = NO_BASES,
) -> ModuleReport:
    """Diff the audited C++ API against the stub; base classes resolve under `stub_roots`,
    C++ class ancestry (`audit_types.class_ancestors`) lets a subclass bind a base parameter.

    Classes are identified by qualified C++ name through `bound` (module -> its
    `bound_classes`): a class this module binds under another Python name is compared as that
    name, and a class another stub declares under its leaf is bound there only if that
    module's binding does not demonstrably bind a different class under it. `bases`
    (`audit_types.public_bases`) checks each bound class's stub bases (`_base_rows`).
    """
    gaps: list[Gap] = []
    accepted: list[Accepted] = []
    covered = 0
    fallbacks = 0
    with_polys = poly_ancestors(ancestors)
    renamed = {q: p for p, q in bound(module).items()}
    # Top-level C++ classes this module binds under another Python name: C++ -> Python name.
    alias = {
        name: renamed[sym.qualified]
        for name, sym in cpp.items()
        if sym.kind in (Kind.CLASS, Kind.ENUM)
        and "." not in name
        and renamed.get(sym.qualified, name) != name
    }
    unalias = {p: c for c, p in alias.items()}
    py = _unaliased_api(py, unalias)  # parameter types name classes as C++ does
    # Top-level classes this stub lacks but another extension binds: their members are
    # compared against that extension's stub (#232). An own rename wins over a leaf hit there.
    elsewhere: dict[str, PyApi] = {}
    for name, sym in cpp.items():
        if sym.kind is not Kind.CLASS or name in alias or name in py.symbols:
            continue
        found = _binding_stub(name, stub_rel, stub_roots)
        if found is not None and _same_class(bound(found[0]).get(name), sym.qualified):
            elsewhere[name] = found[1]
    # Python enumerator name -> its `kCamelCase` C++ enumerator (R15).
    k_aliases = {
        k_alias: name
        for name, sym in cpp.items()
        if sym.kind is Kind.ENUMERATOR and (k_alias := _k_alias(name)) is not None
    }
    for name, sym in cpp.items():
        top, dot, _ = name.partition(".")
        api = elsewhere.get(top, py) if dot else py
        if _ancestor_missing(name, cpp, api, alias):
            continue  # the missing class is the one row
        if _inherited_from_bound(sym, api, stub_roots, bound):
            continue  # the bound base audits it; a missing stub base edge is T5's row
        ps = _py_member(_renamed(name, alias), api, stub_roots)
        if ps is None:
            owner, _, leaf = name.rpartition(".")
            dunder = PROTOCOL_MEMBERS.get(leaf)
            if dunder and _py_member(f"{_renamed(owner, alias)}.{dunder}", api, stub_roots):
                covered += 1  # the dunder's own row names the rule
            elif rule := _unbound_rule(module, name, sym, cpp, api, stub_roots):
                accepted.append(Accepted(name, rule, sym.location))
            elif sym.kind is Kind.ENUMERATOR and _k_alias(name) in api.symbols:
                accepted.append(Accepted(name, "k-prefix-enumerator", sym.location))
            elif name in elsewhere:
                accepted.append(Accepted(name, "bound-in-other-module", sym.location))
            else:
                gaps.append(Gap(name, sym.kind, sym.location))
            continue
        ok = True
        # The class's own static `from_yaml`, never an inherited one (C6 6a).
        from_yaml = api.symbols.get(f"{_renamed(name, alias).rpartition('.')[0]}.from_yaml")
        for ov in sym.overloads:
            how = _cover(ov, ps.overloads, ancestors)
            if how is None and (
                rule := _uncovered_rule(module, name, ov, sym, ps, ancestors, with_polys, from_yaml)
            ):
                accepted.append(Accepted(name, rule, ov.location))
            elif how is None:
                gaps.append(Gap(name, sym.kind, ov.location, str(ov.arity)))
                ok = False
            else:
                fallbacks += not how.typed
                if how.rule != "exact":
                    accepted.append(Accepted(name, how.rule, ov.location))
        covered += ok

    deviations: list[Deviation] = []
    instance_of = {i for (m, _), insts in TEMPLATE_INSTANCES.items() if m == module for i in insts}
    for name, ps in py.symbols.items():
        where = f"{stub_rel}:{ps.line}"
        rule = ACCEPTED_SYMBOLS.get((module, name))
        owner, _, leaf = name.rpartition(".")
        cname = _renamed(name, unalias)  # the C++ name of a member of a renamed class
        # A member inherited from a bound base is that base's: reported there, not here.
        own = cpp.get(cname)
        if own is not None and _inherited_from_bound(own, py, stub_roots, bound):
            own = None
        if name in instance_of:
            accepted.append(Accepted(name, TEMPLATE_INSTANCE_RULE, where))
            continue
        if ps.kind is Kind.ENUMERATOR and name in k_aliases:
            accepted.append(Accepted(name, "k-prefix-enumerator", where))
            continue
        if ps.enum_alias:
            found = None  # a second name for an enum value, even if C++ has it unscoped
        elif (
            leaf == "__hash__"
            and ps.none_annotated
            and _py_member(f"{owner}.__eq__", py, stub_roots)
        ):
            accepted.append(Accepted(name, "value-equality-unhashable", where))
            continue
        elif ps.kind is Kind.PROTOCOL and own is None:
            proto = _protocol_rule(cname, cpp, tu_names)
            if proto:
                accepted.append(Accepted(name, proto, where))
                continue
            found = None
        elif ps.kind is Kind.PROTOCOL:
            found = True
        elif leaf == "__init__":
            found = "ctor"
        else:
            found = DUNDER_OPERATORS.get(leaf, leaf) in tu_names or None
        if found is None:
            if rule:
                accepted.append(Accepted(name, rule, where))
            elif _stands_for_yaml_ctor(leaf, ps, cpp.get(f"{_renamed(owner, unalias)}.__init__")):
                accepted.append(Accepted(name, "yaml-node", where))
            else:
                deviations.append(Deviation(name, ps.kind, where))
            continue
        if own is not None and own.overloads:
            for p in ps.overloads:
                at = f"{stub_rel}:{p.line}"
                if any(_cover(ov, [p], ancestors) is not None for ov in own.overloads):
                    continue
                if any(_cover(ov, [p], with_polys) is not None for ov in own.overloads):
                    accepted.append(Accepted(name, "poly-alternative-overloads", at))
                elif py_rule := ACCEPTED_PY_OVERLOADS.get((module, name, p.params)):
                    accepted.append(Accepted(name, py_rule, at))
                else:
                    deviations.append(Deviation(name, Kind.OVERLOAD, at, str(p.arity)))
        elif rule:
            accepted.append(Accepted(name, rule, where))

    base_gaps, base_deviations = _base_rows(module, cpp, py, stub_rel, stub_roots, bound, bases)
    gaps += base_gaps
    deviations += base_deviations
    return ModuleReport(
        module,
        covered,
        sorted(gaps),
        sorted(deviations),
        sorted(accepted),
        py.quoted,
        type_fallbacks=fallbacks,
    )


def _stub_base(expr: str, module: str) -> tuple[str, str]:
    """(module, class) a stub base expression names: `pkg.mod._mod.Outer.Inner` ->
    (`mod`, `Outer.Inner`); a bare name is a class of `module`."""
    parts = expr.split(".")
    private = [i for i, p in enumerate(parts) if p.startswith("_")]
    if not private:
        return module, expr
    i = private[-1]
    return parts[i].removeprefix("_"), ".".join(parts[i + 1 :])


def _base_rows(
    module: str,
    cpp: dict[str, CppSymbol],
    py: PyApi,
    stub_rel: str,
    stub_roots: Sequence[Path],
    bound: BoundLookup,
    bases: Bases,
) -> tuple[list[Gap], list[Deviation]]:
    """Stub base classes vs the bound C++ bases (T5), for each class the stub binds.

    Expected are the nearest *bound* C++ ancestors (an unbound intermediate is looked
    through); one the stub omits is a gap `D(B)` at the C++ class. A stub base that is no C++
    ancestor at all is a deviation `D(S)` at the stub class. Bases are compared by qualified
    class: a stub base resolves through `bound` of its module, and only a name the index does
    not know falls back to the leaf (positive evidence only, as for classes).
    """
    own = bound(module)
    renamed = {q: p for p, q in own.items()}

    def py_name(q: str) -> str:
        return renamed.get(q, q.rpartition("::")[2])

    def is_bound(q: str) -> bool:
        p = py_name(q)
        sym = py.symbols.get(p)
        if sym is not None and sym.kind is Kind.CLASS:
            return _same_class(own.get(p), q)
        found = _binding_stub(p, stub_rel, stub_roots)
        return found is not None and _same_class(bound(found[0]).get(p), q)

    def nearest(q: str, seen: set[str]) -> list[str]:
        out: list[str] = []
        for b in bases.get(q, ()):
            if b not in seen:
                seen.add(b)
                out += [b] if is_bound(b) else nearest(b, seen)
        return out

    def all_ancestors(q: str) -> set[str]:
        found: set[str] = set()
        todo = list(bases.get(q, ()))
        while todo:
            b = todo.pop()
            if b not in found:
                found.add(b)
                todo.extend(bases.get(b, ()))
        return found

    def names(expr: str, q: str) -> bool:
        mod, cls = _stub_base(expr, module)
        known = bound(mod).get(cls)
        return known == q if known is not None else cls == py_name(q)

    gaps: list[Gap] = []
    deviations: list[Deviation] = []
    for name, sym in cpp.items():
        top = name.partition(".")[0]
        pname = renamed.get(cpp[top].qualified, top) + name[len(top) :] if top in cpp else name
        ps = py.symbols.get(pname)
        if sym.kind is not Kind.CLASS or ps is None or ps.kind is not Kind.CLASS:
            continue
        stub = [e for e in ps.bases if e != "object" and e not in ENUM_BASES]
        for b in nearest(sym.qualified, set()):
            if not any(names(e, b) for e in stub):
                gaps.append(Gap(f"{pname}({py_name(b)})", Kind.BASE, sym.location))
        ancestors = all_ancestors(sym.qualified)
        for e in stub:
            if not any(names(e, a) for a in ancestors):
                where = f"{stub_rel}:{ps.line}"
                deviations.append(
                    Deviation(f"{pname}({_stub_base(e, module)[1]})", Kind.BASE, where)
                )
    return gaps, deviations


def unincluded_header_gaps(
    cpp_path: Path, headers: dict[Path, str], extra_include_dirs: Sequence[Path] = ()
) -> list[Gap]:
    """One `header` gap per header the binding never includes, if it declares any API.

    The headers are parsed in a synthetic TU that includes the binding first, so they see
    the binding's flags and macros. The row points at the header's first auditable
    declaration; a header of forward declarations or macros only gets no row. If that TU does
    not compile, each header is parsed alone, and one that does not compile on its own is an
    `unparseable header` row at its first error: it must not hide the other headers' rows.
    The headers that do parse alone must still parse together.

    Raises:
        HeaderParseError: the synthetic TU does not compile and no header fails on its own
            (headers that conflict only together), or the headers that parse alone do not
            parse together.
    """
    if not headers:
        return []
    try:
        return _header_gaps(cpp_path, headers, extra_include_dirs)
    except HeaderParseError:
        unparseable: list[Gap] = []
        parseable: dict[Path, str] = {}
        for h, spelling in sorted(headers.items(), key=lambda hs: hs[1]):
            try:
                _header_gaps(cpp_path, {h: spelling}, extra_include_dirs)
            except HeaderParseError as e:
                unparseable.append(Gap(spelling, Kind.UNPARSEABLE_HEADER, e.first_error))
            else:
                parseable[h] = spelling
        if not unparseable:
            raise  # no header is at fault alone: the combined error is the finding
    rows = _header_gaps(cpp_path, parseable, extra_include_dirs) if parseable else []
    return [*unparseable, *rows]


def _header_gaps(
    cpp_path: Path, headers: dict[Path, str], extra_include_dirs: Sequence[Path]
) -> list[Gap]:
    """`unincluded_header_gaps` for headers that parse together in one synthetic TU.

    Raises:
        HeaderParseError: the synthetic TU does not compile.
    """
    synthetic = cpp_path.with_name(f"{cpp_path.stem}_unincluded_headers.cpp")
    text = f'#include "{cpp_path.name}"\n' + "".join(
        f"#include <{spelling}>\n" for spelling in sorted(headers.values())
    )
    tu = parse_tu(synthetic, extra_include_dirs, unsaved=text)
    lines = own_header_lines(cpp_api(tu, frozenset(headers)))
    gaps = []
    for h, spelling in headers.items():
        found = lines.get(header_rel(h))
        if found:
            gaps.append(Gap(spelling, Kind.HEADER, f"{header_rel(h)}:{min(found)}"))
    return gaps


def audit_tu(
    module: str,
    cpp_path: Path,
    stub_path: Path,
    init_path: Path,
    prefix: tuple[str, ...],
    extra_include_dirs: Sequence[Path] = (),
    other_bindings: Sequence[Path] = (),
    stub_roots: Sequence[Path] = (SRC,),
    prefixes: Mapping[str, tuple[str, ...]] | None = None,
    bound: BoundLookup | None = None,
) -> ModuleReport:
    """Audit one translation unit against one stub and package `__init__.py`.

    Audits every header under `prefix` (the module's prefixes) that the module owns, except
    `UNAUDITED_HEADERS` and headers that the binding
    does not #include but one of `other_bindings` does: those are audited by `header_auditor`
    of their includers and listed in `ModuleReport.delegated`. Conversely, a header under
    another module's prefix (`prefixes`: module → prefixes, for ownership) that this binding
    #includes and its owner's binding does not is audited here when this module is its auditor.
    Stub base classes bound in other extensions resolve through stubs under `stub_roots`.
    Class identity (`match`'s `bound`) defaults to this binding's own `bound_classes` and
    `bound_index` for every other module.

    Raises:
        PrefixMismatchError: `prefixes` holds `module` with other prefixes than `prefix`.
    """
    include_dirs = (*extra_include_dirs, *INCLUDE_DIRS)
    if prefixes is not None and prefixes.get(module, prefix) != prefix:
        raise PrefixMismatchError(
            f"{module}: prefix {prefix} disagrees with prefixes[{module!r}] = {prefixes[module]}"
        )
    prefixes = {**(prefixes or {}), module: prefix}
    if bound is None:
        own = _bound_classes_of(cpp_path, tuple(extra_include_dirs))

        def bound(m: str) -> Mapping[str, str]:
            return own if m == module else bound_index(m)

    tu = parse_tu(cpp_path, extra_include_dirs)
    direct = audited_headers(tu, module, prefixes, include_dirs)
    other_includes = {binding_module(p): direct_include_spellings([p]) for p in other_bindings}
    owned_elsewhere = frozenset().union(*other_includes.values())
    audited: dict[Path, str] = {}
    delegated: dict[str, str] = {}
    for h, spelling in prefix_headers(module, prefixes, include_dirs).items():
        if h in direct:
            audited[h] = spelling
        elif is_unaudited(spelling, prefix):
            continue
        elif spelling in owned_elsewhere:
            delegated[spelling] = header_auditor(
                m for m, inc in other_includes.items() if spelling in inc
            )
        else:
            audited[h] = spelling
    audited |= foreign_headers(
        module, direct_include_spellings([cpp_path]), other_includes, prefixes, include_dirs
    )
    included = frozenset(Path(i.include.name).resolve() for i in tu.get_includes())
    report = match(
        module,
        cpp_api(tu, frozenset(audited) & included, prefixes),
        decl_names(tu),
        py_api(load_stub(stub_path), rel(stub_path)),
        rel(stub_path),
        stub_roots,
        audit_types.class_ancestors(tu),
        bound,
        audit_types.public_bases(tu),
    )
    unincluded = {h: s for h, s in audited.items() if h not in included}
    report.gaps = sorted(
        [*report.gaps, *unincluded_header_gaps(cpp_path, unincluded, extra_include_dirs)]
    )
    report.deviations = sorted([*report.deviations, *init_findings(init_path)])
    report.accepted = sorted([*report.accepted, *init_accepted(init_path)])
    report.delegated = dict(sorted(delegated.items()))
    return report


def audit_module(module: str) -> ModuleReport:
    """Audit one binding module by short name.

    Raises:
        UnknownModuleError: see `resolve_module`.
        UnauditedModuleError: see `resolve_module`.
        StubMissingError: see `resolve_module`.
        HeaderParseError: see `parse_tu`.
    """
    resolve_module(module)
    return audit_tu(
        module,
        binding_source(module),
        committed_stub(module),
        STUB_ROOT / module / "__init__.py",
        AUDITED_HEADER_PREFIX[module],
        other_bindings=[binding_source(m) for m in binding_modules() if m != module],
        prefixes=owner_prefixes(),
    )
