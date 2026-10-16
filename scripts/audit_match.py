"""Audit each binding's C++ API against its committed Python stub.

For each extension module, libclang parses `src/<module>/<module>_bindings.cpp`
(exactly the headers and macros the binding sees) and `ast` parses the
committed `_<module>.pyi` and the package `__init__.py`. Every header under the
module's prefix is audited: one the binding never includes is parsed in a
synthetic TU (the binding plus that header) and reported as one `header` row.
No extension module is imported. Gaps (C++ without Python), deviations (Python without C++) and
accepted deviations go to `docs/developer/binding-api-audit.md`.

Usage:
    pixi run -e audit audit-bindings [MODULE ...] [--json PATH]
"""

from __future__ import annotations

import argparse
import ast
import functools
import json
import subprocess
import sys
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from pathlib import Path

import audit_types
import clang.cindex as ci
import tomllib
from audit_ownership import (
    AUDITED_HEADER_PREFIX,
    REPO_ROOT,
    SRC,
    STUB_ROOT,
    THIRD_PARTY_HEADERS,
    UNAUDITED_HEADERS,
    UNAUDITED_MODULES,
    Coverage,
    UnauditedModuleError,
    UnknownModuleError,
    binding_modules,
    binding_source,
    check_coverage,
    committed_stub,
    coverage,
    header_auditor,
    is_unaudited,
    owner_prefixes,
)
from audit_parse import (
    INCLUDE_DIRS,
    OPERATOR_DUNDERS,
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
from audit_types import NO_ANCESTORS, Ancestors, TypeKey


class PrefixMismatchError(ValueError):
    """`audit_tu` got a module prefix that disagrees with that module's `prefixes` entry."""


class StubMissingError(FileNotFoundError):
    """The module has no committed stub."""


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


def load_stub(path: Path) -> ast.Module:
    """Parse a committed stub.

    Raises:
        StubMissingError: `path` does not exist.
    """
    if not path.is_file():
        raise StubMissingError(f"no committed stub {path}; run `pixi run stubs`")
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


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
ENUM_BASES = frozenset({"enum.Enum", "enum.IntEnum", "enum.Flag", "enum.IntFlag"})
DUNDER_OPERATORS = {v: k for k, v in OPERATOR_DUNDERS.items()}
NOT_AN_OVERLOAD = "—"  # arity column of rows that are not about one overload
TRY_IMPORT = "try: import … except ImportError"


@dataclass(frozen=True)
class PyOverload:
    arity: Arity
    returns: str
    line: int
    params: tuple[str, ...] = ()  # parameter annotations, `self` excluded, keyword-only last
    keys: tuple[TypeKey, ...] = ()  # `audit_types.py_key` of each of `params`


@dataclass
class PySymbol:
    name: str
    kind: Kind
    line: int
    overloads: list[PyOverload] = field(default_factory=list)
    bases: tuple[str, ...] = ()  # stub base-class expressions, for inherited members
    enum_alias: bool = False  # module constant whose value is a stub enum member (Note 1)
    none_annotated: bool = False  # `name: None`, how nanobind spells `__hash__ = None`


@dataclass(frozen=True, order=True)
class QuotedType:
    name: str
    annotation: str
    location: str


@dataclass(frozen=True, order=True)
class Deviation:
    name: str
    kind: Kind
    location: str
    arity: str = NOT_AN_OVERLOAD


@dataclass
class PyApi:
    symbols: dict[str, PySymbol]
    quoted: list[QuotedType]


def rel(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT).as_posix()


def _decorators(node: ast.FunctionDef) -> set[str]:
    return {ast.unparse(d) for d in node.decorator_list}


def _is_setter(node: ast.FunctionDef) -> bool:
    return any(d.endswith(".setter") for d in _decorators(node))


def _py_arity(node: ast.FunctionDef, bound: bool) -> Arity:
    a = node.args
    positional = [*a.posonlyargs, *a.args][1 if bound else 0 :]
    required_kw = sum(d is None for d in a.kw_defaults)
    lo = len(positional) - len(a.defaults) + required_kw
    hi = None if a.vararg else len(positional) + len(a.kwonlyargs)
    return Arity(lo, hi)


def _py_param_nodes(node: ast.FunctionDef, bound: bool) -> list[ast.arg]:
    """Positional parameters (`self` excluded), then keyword-only ones: nanobind's
    `nb::kw_only()` makes trailing C++ parameters keyword-only, in C++ order."""
    a = node.args
    return [*a.posonlyargs, *a.args][1 if bound else 0 :] + a.kwonlyargs


def _function_kind(node: ast.FunctionDef, in_class: bool) -> Kind:
    if "property" in _decorators(node):
        return Kind.FIELD
    if node.name == "__init__":
        return Kind.CONSTRUCTOR
    if node.name.startswith("__") and node.name.endswith("__"):
        return Kind.OPERATOR if node.name in DUNDER_OPERATORS else Kind.PROTOCOL
    return Kind.METHOD if in_class else Kind.FUNCTION


def _annotations(node: ast.AST) -> list[ast.expr]:
    if isinstance(node, ast.AnnAssign):
        return [node.annotation]
    a = node.args
    args = [*a.posonlyargs, *a.args, *a.kwonlyargs, a.vararg, a.kwarg]
    return [x.annotation for x in args if x is not None and x.annotation is not None] + (
        [node.returns] if node.returns is not None else []
    )


def _quoted_names(annotation: ast.expr) -> list[str]:
    """String constants inside an annotation that spell a C++ name.

    nanobind emits an unbound type as its quoted, fully qualified C++ name, so it
    always contains `::`; other annotation strings (`order="C"`) do not.
    """
    return [
        n.value
        for n in ast.walk(annotation)
        if isinstance(n, ast.Constant) and isinstance(n.value, str) and "::" in n.value
    ]


def _enum_member(value: ast.expr | None, symbols: dict[str, PySymbol]) -> bool:
    """Whether `value` spells `E.member` for an enum `E` the stub declared earlier."""
    if not (isinstance(value, ast.Attribute) and isinstance(value.value, ast.Name)):
        return False
    owner = symbols.get(value.value.id)
    return owner is not None and owner.kind is Kind.ENUM


def py_api(tree: ast.Module, stub_rel: str) -> PyApi:
    """Names, kinds and overload arities a stub declares, plus quoted C++ annotations."""
    symbols: dict[str, PySymbol] = {}
    quoted: list[QuotedType] = []

    def visit(body: list[ast.stmt], prefix: str, in_enum: bool) -> None:
        for node in body:
            if isinstance(node, ast.ClassDef):
                is_enum = any(ast.unparse(b) in ENUM_BASES for b in node.bases)
                name = prefix + node.name
                symbols[name] = PySymbol(
                    name,
                    Kind.ENUM if is_enum else Kind.CLASS,
                    node.lineno,
                    bases=tuple(ast.unparse(b) for b in node.bases),
                )
                visit(node.body, f"{name}.", is_enum)
                continue
            if isinstance(node, ast.FunctionDef):
                if _is_setter(node):
                    continue
                name = prefix + node.name
                kind = _function_kind(node, bool(prefix))
                sym = symbols.setdefault(name, PySymbol(name, kind, node.lineno))
                if kind is not Kind.FIELD:
                    bound = bool(prefix) and "staticmethod" not in _decorators(node)
                    returns = ast.unparse(node.returns) if node.returns else ""
                    args = _py_param_nodes(node, bound)
                    sym.overloads.append(
                        PyOverload(
                            _py_arity(node, bound),
                            returns,
                            node.lineno,
                            tuple(ast.unparse(p.annotation) if p.annotation else "" for p in args),
                            tuple(audit_types.py_key(p.annotation) for p in args),
                        )
                    )
            elif isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                for t in targets:
                    if isinstance(t, ast.Name):
                        kind = (
                            Kind.ENUMERATOR if in_enum else Kind.FIELD if prefix else Kind.CONSTANT
                        )
                        name = prefix + t.id
                        alias = kind is Kind.CONSTANT and _enum_member(node.value, symbols)
                        none = (
                            isinstance(node, ast.AnnAssign)
                            and ast.unparse(node.annotation) == "None"
                        )
                        symbols[name] = PySymbol(
                            name, kind, node.lineno, enum_alias=alias, none_annotated=none
                        )
            else:
                continue
            for ann in _annotations(node) if not isinstance(node, ast.Assign) else []:
                for q in _quoted_names(ann):
                    owner = name if isinstance(node, ast.FunctionDef) else prefix + node.target.id
                    quoted.append(QuotedType(owner, q, f"{stub_rel}:{node.lineno}"))

    visit(tree.body, "", False)
    return PyApi(symbols, sorted(set(quoted)))


def _catches_import_error(handler: ast.ExceptHandler) -> bool:
    if handler.type is None:
        return False
    names = handler.type.elts if isinstance(handler.type, ast.Tuple) else [handler.type]
    return any(ast.unparse(n) in {"ImportError", "ModuleNotFoundError"} for n in names)


# Module-level hooks a package `__init__.py` may define, accepted by rule: name -> ACCEPTED key.
INIT_HOOKS = {"__getattr__": "module-getattr"}


def init_findings(path: Path) -> list[Deviation]:
    """Classes/functions a package `__init__.py` defines, and fail-loud violations."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    where = rel(path)
    rows = [
        Deviation(
            n.name,
            Kind.CLASS if isinstance(n, ast.ClassDef) else Kind.FUNCTION,
            f"{where}:{n.lineno}",
        )
        for n in tree.body
        if isinstance(n, ast.ClassDef)
        or (isinstance(n, ast.FunctionDef) and n.name not in INIT_HOOKS)
    ]
    rows += [
        Deviation(TRY_IMPORT, Kind.FAIL_LOUD, f"{where}:{n.lineno}")
        for n in ast.walk(tree)
        if isinstance(n, ast.Try) and any(_catches_import_error(h) for h in n.handlers)
    ]
    return sorted(rows)


def init_accepted(path: Path) -> list[Accepted]:
    """The module-level hooks (`INIT_HOOKS`) a package `__init__.py` defines."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return sorted(
        Accepted(n.name, INIT_HOOKS[n.name], f"{rel(path)}:{n.lineno}")
        for n in tree.body
        if isinstance(n, ast.FunctionDef) and n.name in INIT_HOOKS
    )


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


@dataclass(frozen=True, order=True)
class Gap:
    symbol: str
    kind: Kind
    location: str
    arity: str = NOT_AN_OVERLOAD


@dataclass(frozen=True, order=True)
class Accepted:
    symbol: str
    rule: str
    location: str


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
            elif rule := ACCEPTED_SYMBOLS.get((module, name)):
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
    for name, ps in py.symbols.items():
        where = f"{stub_rel}:{ps.line}"
        rule = ACCEPTED_SYMBOLS.get((module, name))
        owner, _, leaf = name.rpartition(".")
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


REPORT_PATH = REPO_ROOT / "docs" / "developer" / "binding-api-audit.md"
LIMITATION = (
    "Python member names are checked against every declaration in the TU, so a "
    "Python-only member that shares a common C++ name (`size`, `clear`) is not flagged. "
    "Overloads are matched by arity and parameter type: each C++ parameter type and each stub "
    "annotation map to a key (`scripts/audit_types.py`), and a bound overload covers a C++ one "
    "only where the keys agree. A type the mapping does not know (a template parameter, "
    "`object`, a quoted C++ name) leaves that position to arity alone; the `type fallbacks` "
    "column counts the C++ overloads covered that way. Overloads that differ only in `const` "
    "of `this` or `&`/`&&` of a parameter count as one. An overload taking a raw buffer (a "
    "pointer to a non-`char` scalar) has no Python form and is reported whatever the arity, "
    "unless it is a `(const uint8_t*, size_t)` pair bound as one `bytes` parameter. "
    "A header the binding never includes that does not compile on its own is listed as "
    "`unparseable header`, at its first error, and its API is not checked."
)


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()


def libclang_version() -> str:
    """`clang version 22.1.8`: python-clang registers no restype for this call, so set it."""
    fn = ci.conf.lib.clang_getClangVersion
    fn.restype = ci._CXString
    return ci._CXString.from_result(fn())


def provenance(modules: Sequence[str]) -> dict[str, str]:
    """Package pin, libclang version and the stubs' git SHA (no timestamps: deterministic)."""
    pins = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    stubs = [rel(committed_stub(m)) for m in modules]
    sha = _git("log", "-1", "--format=%h", "--", *stubs)
    dirty = " (uncommitted stub changes)" if _git("status", "--porcelain", "--", *stubs) else ""
    return {
        "tesseract-robotics": pins["tool"]["pixi"]["dependencies"]["tesseract-robotics"],
        "libclang": libclang_version(),
        "stubs": sha + dirty,
    }


def _table(header: tuple[str, ...], rows: list[tuple[str, ...]]) -> list[str]:
    if not rows:
        return ["None.", ""]
    return [
        "| " + " | ".join(header) + " |",
        "|" + "---|" * len(header),
        *("| " + " | ".join(r) + " |" for r in rows),
        "",
    ]


def render_markdown(reports: list[ModuleReport], prov: dict[str, str], cov: Coverage) -> str:
    """The mkdocs report page: provenance, coverage, conclusion, then per-module tables."""
    out = [
        "# Binding API audit",
        "",
        "Generated by `pixi run -e audit audit-bindings`; do not edit by hand.",
        f"tesseract-robotics `{prov['tesseract-robotics']}` · libclang `{prov['libclang']}` · "
        f"stubs `{prov['stubs']}`.",
        "",
        f"{len(cov.audited)} of {cov.total} binding modules audited.",
        "",
        "Not audited:",
        "",
        *_table(("module", "reason"), list(cov.unaudited.items())),
        "| module | covered | gaps | deviations | accepted | quoted types | type fallbacks |",
        "|---|---|---|---|---|---|---|",
        *(
            f"| {r.module} | {r.covered} | {len(r.gaps)} | {len(r.deviations)} | "
            f"{len(r.accepted)} | {len(r.quoted)} | {r.type_fallbacks} |"
            for r in reports
        ),
        "",
        "Quoted C++ names in the stubs of audited modules (unbound type, or a missing "
        "`nb::module_::import_`):",
        "",
        *_table(
            ("module", "Python name", "annotation", "stub:line"),
            [
                (r.module, f"`{q.name}`", f"`{q.annotation}`", q.location)
                for r in reports
                for q in r.quoted
            ],
        ),
        '!!! note "Accepted deviation rules"',
        *(f"    - `{k}`: {v}" for k, v in sorted(ACCEPTED.items())),
        "",
        '!!! note "Unaudited headers"',
        *(f"    - `{k}`: {v}" for k, v in UNAUDITED_HEADERS.items()),
        "    - A header its prefix owner's binding does not #include, but another binding does, is "
        "audited with the first such module by name; each module's section lists those headers.",
        "",
        '!!! note "Third-party headers (never audited, even when #included directly)"',
        *(f"    - `{k}`: {v}" for k, v in THIRD_PARTY_HEADERS.items()),
        "",
        f'!!! warning "Limitations"\n    {LIMITATION}',
        "",
    ]
    for r in reports:
        out += [f"## {r.module}", "", "### Gaps", ""]
        out += _table(
            ("C++ symbol", "kind", "header:line", "missing overload (arity)"),
            [(f"`{g.symbol}`", g.kind.value, g.location, g.arity) for g in r.gaps],
        )
        out += ["### Deviations", ""]
        out += _table(
            ("Python name", "kind", "stub:line", "arity"),
            [(f"`{d.name}`", d.kind.value, d.location, d.arity) for d in r.deviations],
        )
        out += ["### Accepted", ""]
        out += _table(
            ("symbol", "rule", "location"),
            [(f"`{a.symbol}`", a.rule, a.location) for a in r.accepted],
        )
        out += ["### Headers audited with another module", ""]
        out += _table(
            ("header", "audited with", "status"),
            [
                (
                    f"`{h}`",
                    m,
                    "audited" if m in cov.audited else f"not yet: {cov.unaudited[m]}",
                )
                for h, m in r.delegated.items()
            ],
        )
    return "\n".join(out).rstrip() + "\n"


def to_json(reports: list[ModuleReport], prov: dict[str, str], cov: Coverage) -> str:
    """Same data as the page, for issue drafting."""
    modules = {
        r.module: {
            "covered": r.covered,
            "gaps": [asdict(g) for g in r.gaps],
            "deviations": [asdict(d) for d in r.deviations],
            "accepted": [asdict(a) for a in r.accepted],
            "quoted": [asdict(q) for q in r.quoted],
            "delegated": r.delegated,
            "type_fallbacks": r.type_fallbacks,
        }
        for r in reports
    }
    data = {
        "provenance": prov,
        "coverage": {"audited": list(cov.audited), "unaudited": dict(cov.unaudited)},
        "modules": modules,
    }
    return json.dumps(data, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "modules", nargs="*", default=sorted(AUDITED_HEADER_PREFIX),
        help="short module names (default: every module with an AUDITED_HEADER_PREFIX entry)",
    )  # fmt: skip
    parser.add_argument("--json", type=Path, help="also write the findings as JSON")
    args = parser.parse_args(argv)

    check_coverage(binding_modules())
    modules = sorted(resolve_module(m) for m in args.modules)
    reports = [audit_module(m) for m in modules]
    prov = provenance(modules)
    cov = coverage()
    REPORT_PATH.write_text(render_markdown(reports, prov, cov), encoding="utf-8")
    if args.json:
        args.json.write_text(to_json(reports, prov, cov), encoding="utf-8")
    print(f"{len(reports)} modules → {REPORT_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
