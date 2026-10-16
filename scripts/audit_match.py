"""Diff a binding's C++ API against its stub; audit one module."""

from __future__ import annotations

import functools
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

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
    cpp_api,
    decl_names,
    direct_include_spellings,
    foreign_headers,
    header_rel,
    parse_tu,
    prefix_headers,
)
from audit_stub import (
    DUNDER_OPERATORS,
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
from audit_types import NO_ANCESTORS, Ancestors, TypeKey


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


# C++ members a Python protocol dunder covers (container-protocol, iterator-pair rules): a
# class that binds the dunder needs no binding under the C++ name.
PROTOCOL_MEMBERS = {
    "size": "__len__",
    "begin": "__iter__",
    "end": "__iter__",
    "cbegin": "__iter__",
    "cend": "__iter__",
}
# Python protocol dunders accepted by rule: dunder -> (ACCEPTED key, C++ members, keyed as in
# `cpp_api`, the owner must declare). An owner with no C++ class (a bound std container
# typedef) is checked against the TU's declaration names instead.
PROTOCOL_RULES = {
    "__len__": ("container-protocol", ("size",)),
    "__setitem__": ("container-protocol", ("__getitem__",)),
    "__iter__": ("iterator-pair", ("begin", "end")),
    "__repr__": ("presentation-dunder", ()),
    "__str__": ("presentation-dunder", ()),
}
# Deviations accepted by rule; each is printed with its reason in the report.
ACCEPTED = {
    "out-param": "A non-const lvalue-reference out-param (or a by-value `Eigen::Ref` of a "
    "non-const matrix) is returned in a tuple with the result (Phase A precedent: "
    "checkTrajectory), or alone when the C++ returns `void`.",
    "stringstream": "A `std::stringstream&` parameter the C++ writes into is returned as `str`.",
    "byte-buffer": "A `(const uint8_t* bytes, size_t len)` pair is one Python `bytes` argument: "
    "the binding passes the buffer and its size (`createMeshFromBytes`, `BytesResource`).",
    "scalar-last-quaternion": "Quaterniond takes (x, y, z, w), the project-wide scalar-last "
    "order; Eigen's constructor is (w, x, y, z).",
    "container-protocol": "`size()` is bound as `__len__` and `operator[]` as "
    "`__getitem__`/`__setitem__`, the Python container protocol.",
    "iterator-pair": "A `begin()`/`end()` pair (and `cbegin`/`cend`) is bound as `__iter__`.",
    "stream-insertion": "A free `operator<<(std::ostream&, const T&)` is bound as `T.__str__` "
    "(`T` a class or an enum).",
    "presentation-dunder": "`__repr__` and `__str__` are Python presentation and need no C++ "
    "counterpart.",
    "serialization-default-ctor": "The default constructor of a class that befriends cereal "
    "`serialize` and declares another public constructor exists for deserialization only.",
    "value-equality-unhashable": "`__hash__ = None` next to a bound `__eq__` makes a mutable "
    "value type unhashable, as Python does for any class that defines `__eq__`; it needs no "
    "C++ counterpart.",
    "eigen-template-instance": "`Eigen::Hyperplane<double, 3>` and "
    "`Eigen::ParametrizedLine<double, 3>` have no Eigen typedef; the class takes Eigen's own "
    "`…3d` naming (`Vector3d`, `Quaterniond`).",
    "quaternion-rpy": "`Quaterniond.from_rpy`/`to_rpy` convert roll-pitch-yaw in the ROS/tf2 "
    "convention, which Eigen spells as three composed `AngleAxisd` rotations; `to_rpy` returns "
    "tf2's canonical ranges, which `eulerAngles` does not guarantee.",
    "eigen-default-precision": "`EIGEN_DEFAULT_PREC` re-exports "
    "`Eigen::NumTraits<double>::dummy_precision()` (1e-12) so Python compares with Eigen's "
    "own default tolerance instead of a duplicated literal.",
    "template-instance-name": "Python cannot spell a C++ template argument, so each instance of "
    "a function template gets its own function named after the type (`createMeshFromPath<ConvexMesh>` "
    "→ `createConvexMeshFromPath`, `Serialization::toArchiveStringXML<Environment>` → "
    "`environment_to_xml`).",
    "copied-vector-field-mutator": "A `std::vector` field converts to a fresh Python list on each "
    "access, so `obj.field.append(x)` edits a copy; `add…`/`clear…` methods mutate the C++ vector.",
    "boost-graph-property-tag": "Boost Graph property tags (`vertex_link_t`, …) select the "
    "properties `SceneGraph` stores; Python reaches the properties through `SceneGraph` methods.",
    "bound-in-other-module": "The class is bound in another extension module, which defines its "
    "Python type once for every module; its members are compared against that module's stub.",
    "yaml-node": "An overload taking a `YAML::Node` has no Python form (no Python type converts "
    "to one); the sibling overloads taking a config file path or YAML text load the same config.",
    "derived-element-container": "An overload taking a container of a derived class only upcasts "
    "into its base-element sibling (`CompoundMesh(vector<ConvexMesh>)` → "
    "`CompoundMesh(vector<PolygonMesh>)`, compound_mesh.cpp:59-63); Python passes a list of any "
    "subclass to the bound base-element overload.",
    "module-getattr": "A package `__init__.py` defines a PEP 562 module `__getattr__` to re-export "
    "a name lazily (an eager import would be an import cycle); it is an attribute-lookup hook, "
    "not API.",
}
# Python and C++ names accepted by a named rule: (module, name) -> ACCEPTED key.
ACCEPTED_SYMBOLS = {
    ("tesseract_common", "Quaterniond.__init__"): "scalar-last-quaternion",
    ("tesseract_common", "Quaterniond.from_xyzw"): "scalar-last-quaternion",
    ("tesseract_common", "Hyperplane3d"): "eigen-template-instance",
    ("tesseract_common", "ParametrizedLine3d"): "eigen-template-instance",
    ("tesseract_common", "Quaterniond.from_rpy"): "quaternion-rpy",
    ("tesseract_common", "Quaterniond.to_rpy"): "quaternion-rpy",
    ("tesseract_common", "EIGEN_DEFAULT_PREC"): "eigen-default-precision",
    **{
        ("tesseract_geometry", f"create{t}MeshFrom{s}"): "template-instance-name"
        for t in ("Convex", "SDF")
        for s in ("Path", "Resource", "Bytes")
    },
    **{
        ("tesseract_serialization", f"{t}_{d}_{f}"): "template-instance-name"
        for t in ("composite_instruction", "environment", "scene_state")
        for d in ("to", "from")
        for f in ("xml", "file", "binary")
    },
    **{
        ("tesseract_scene_graph", f"Link.{op}{field}"): "copied-vector-field-mutator"
        for op in ("add", "clear")
        for field in ("Visual", "Collision")
    },
    **{
        ("tesseract_scene_graph", tag): "boost-graph-property-tag"
        for tag in ("vertex_link_t", "edge_joint_t", "graph_root_t", "property_kind")
    },
}
# A C++ member template bound once per instance, under a name Python can spell
# (`InstructionPoly::as<MoveInstructionPoly>()` → `asMoveInstruction`): (module, C++ dotted
# name) -> the Python dotted names of its instances. The C++ row is accepted only when every
# listed instance is bound; each instance is accepted on the Python side (template-instance-name).
TEMPLATE_INSTANCES: dict[tuple[str, str], tuple[str, ...]] = {}
# The rule `TEMPLATE_INSTANCES` entries are accepted under.
TEMPLATE_INSTANCE_RULE = "template-instance-name"
# A C++ overload accepted unbound: (module, dotted name, `str(Arity)`) -> ACCEPTED rule. For a
# symbol Python binds, where the other overloads stay checked (T4).
ACCEPTED_CPP_OVERLOADS: dict[tuple[str, str, str], str] = {}
# A Python-only overload accepted: (module, dotted name, `PyOverload.params`) -> ACCEPTED rule.
# Two overloads of one arity differ by their annotations (G2: `setWarmStart(bool)`).
ACCEPTED_PY_OVERLOADS: dict[tuple[str, str, tuple[str, ...]], str] = {}
# A type-erased Poly -> the bound classes it wraps, which C++ converts into it through the
# Poly's converting constructors (R8). A Python overload typed by an alternative stands for
# the Poly-typed C++ parameter.
POLY_ALTERNATIVES: dict[str, tuple[str, ...]] = {}


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


def _binding_stub(name: str, stub_rel: str, stub_roots: Sequence[Path]) -> PyApi | None:
    """The other extension's stub under `stub_roots` declaring top-level class `name`, or None."""
    if "." in name:
        return None
    return next(
        (
            api
            for root in stub_roots
            for stub in _stubs_under(root)
            if rel(stub) != stub_rel and name in (api := _stub_py_api(stub)).symbols
        ),
        None,
    )


@functools.cache
def _stubs_under(root: Path) -> tuple[Path, ...]:
    """Every extension stub (`_<module>.pyi`) below `root`, listed once per audit run."""
    return tuple(sorted(root.rglob("_*.pyi")))


def _ancestor_missing(name: str, cpp: dict[str, CppSymbol], py: PyApi) -> bool:
    parts = name.split(".")
    owners = (".".join(parts[:i]) for i in range(1, len(parts)))
    return any(o in cpp and o not in py.symbols for o in owners)


def _unbound_rule(
    module: str,
    name: str,
    sym: CppSymbol,
    cpp: Mapping[str, CppSymbol],
    py: PyApi,
    stub_roots: Sequence[Path],
) -> str | None:
    """The ACCEPTED rule for a C++ symbol Python does not bind, or None (a gap)."""
    instances = TEMPLATE_INSTANCES.get((module, name))
    if instances and all(_py_member(i, py, stub_roots) for i in instances):
        return TEMPLATE_INSTANCE_RULE
    return ACCEPTED_SYMBOLS.get((module, name))


def match(
    module: str,
    cpp: dict[str, CppSymbol],
    tu_names: frozenset[str],
    py: PyApi,
    stub_rel: str,
    stub_roots: Sequence[Path] = (SRC,),
    ancestors: Ancestors = NO_ANCESTORS,
) -> ModuleReport:
    """Diff the audited C++ API against the stub; base classes resolve under `stub_roots`,
    C++ class ancestry (`audit_types.class_ancestors`) lets a subclass bind a base parameter."""
    gaps: list[Gap] = []
    accepted: list[Accepted] = []
    covered = 0
    fallbacks = 0
    # Top-level classes this stub lacks but another extension binds: their members are
    # compared against that extension's stub (#232).
    elsewhere = {
        name: api
        for name, sym in cpp.items()
        if sym.kind is Kind.CLASS
        and name not in py.symbols
        and (api := _binding_stub(name, stub_rel, stub_roots)) is not None
    }
    for name, sym in cpp.items():
        top, dot, _ = name.partition(".")
        api = elsewhere.get(top, py) if dot else py
        if _ancestor_missing(name, cpp, api):
            continue  # the missing class is the one row
        ps = _py_member(name, api, stub_roots)
        if ps is None:
            owner, _, leaf = name.rpartition(".")
            dunder = PROTOCOL_MEMBERS.get(leaf)
            if dunder and _py_member(f"{owner}.{dunder}", api, stub_roots):
                covered += 1  # the dunder's own row names the rule
            elif rule := _unbound_rule(module, name, sym, cpp, api, stub_roots):
                accepted.append(Accepted(name, rule, sym.location))
            elif name in elsewhere:
                accepted.append(Accepted(name, "bound-in-other-module", sym.location))
            else:
                gaps.append(Gap(name, sym.kind, sym.location))
            continue
        ok = True
        for ov in sym.overloads:
            how = _cover(ov, ps.overloads, ancestors)
            if how is None and ov.absent_ok:
                accepted.append(Accepted(name, ov.absent_ok, ov.location))
            elif how is None and _derived_element_sibling(ov, sym, ps.overloads, ancestors):
                accepted.append(Accepted(name, "derived-element-container", ov.location))
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
        if name in instance_of:
            accepted.append(Accepted(name, TEMPLATE_INSTANCE_RULE, where))
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
        elif ps.kind is Kind.PROTOCOL and name not in cpp:
            proto = _protocol_rule(name, cpp, tu_names)
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
            else:
                deviations.append(Deviation(name, ps.kind, where))
            continue
        if name in cpp and cpp[name].overloads:
            for p in ps.overloads:
                if not any(_cover(ov, [p], ancestors) is not None for ov in cpp[name].overloads):
                    deviations.append(
                        Deviation(name, Kind.OVERLOAD, f"{stub_rel}:{p.line}", str(p.arity))
                    )
        elif rule:
            accepted.append(Accepted(name, rule, where))

    return ModuleReport(
        module,
        covered,
        sorted(gaps),
        sorted(deviations),
        sorted(accepted),
        py.quoted,
        type_fallbacks=fallbacks,
    )


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
    lines: dict[str, list[int]] = {}
    for sym in cpp_api(tu, frozenset(headers)).values():
        path, line = sym.location.rsplit(":", 1)
        lines.setdefault(path, []).append(int(line))
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
) -> ModuleReport:
    """Audit one translation unit against one stub and package `__init__.py`.

    Audits every header under `prefix` (the module's prefixes) that the module owns, except
    `UNAUDITED_HEADERS` and headers that the binding
    does not #include but one of `other_bindings` does: those are audited by `header_auditor`
    of their includers and listed in `ModuleReport.delegated`. Conversely, a header under
    another module's prefix (`prefixes`: module → prefixes, for ownership) that this binding
    #includes and its owner's binding does not is audited here when this module is its auditor.
    Stub base classes bound in other extensions resolve through stubs under `stub_roots`.

    Raises:
        PrefixMismatchError: `prefixes` holds `module` with other prefixes than `prefix`.
    """
    include_dirs = (*extra_include_dirs, *INCLUDE_DIRS)
    if prefixes is not None and prefixes.get(module, prefix) != prefix:
        raise PrefixMismatchError(
            f"{module}: prefix {prefix} disagrees with prefixes[{module!r}] = {prefixes[module]}"
        )
    prefixes = {**(prefixes or {}), module: prefix}
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
        cpp_api(tu, frozenset(audited) & included),
        decl_names(tu),
        py_api(load_stub(stub_path), rel(stub_path)),
        rel(stub_path),
        stub_roots,
        audit_types.class_ancestors(tu),
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
