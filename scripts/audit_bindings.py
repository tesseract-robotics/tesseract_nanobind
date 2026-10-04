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
import fnmatch
import json
import os
import re
import subprocess
import sys
import sysconfig
from collections.abc import Sequence
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path

import clang.cindex as ci
import nanobind
import tomllib

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
STUB_ROOT = SRC / "tesseract_robotics"

CONDA_PREFIX = Path(os.environ["CONDA_PREFIX"])
CONDA_INCLUDE = CONDA_PREFIX / "include"
# Major version of the `clang-22` package in the pixi `audit` feature (pyproject.toml).
LIBCLANG_MAJOR = 22
# Builtin headers (stdarg.h, stddef.h) shipped by `clang-22`; libclang does not find
# them on its own when loaded from the conda prefix.
CLANG_RESOURCE_DIR = CONDA_PREFIX / "lib" / "clang" / str(LIBCLANG_MAJOR)
# Matches CMakeLists.txt `set(CMAKE_CXX_STANDARD 17)`.
CXX_STD = "c++17"
# Header search path of the binding build: tesseract, yaml-cpp and console_bridge
# from the conda prefix; Eigen under include/eigen3; tesseract_nb.h from src/;
# nanobind and Python headers from their packages.
INCLUDE_DIRS = (
    CONDA_INCLUDE,
    CONDA_INCLUDE / "eigen3",
    SRC,
    Path(nanobind.include_dir()),
    Path(sysconfig.get_paths()["include"]),
)
PARSE_ARGS = (
    "-x",
    "c++",
    f"-std={CXX_STD}",
    "-fsyntax-only",
    "-resource-dir",
    str(CLANG_RESOURCE_DIR),
)
# Declarations only: binding bodies (the NB_MODULE block) carry no API. The detailed
# processing record keeps the main file's #include directives as cursors: get_includes()
# reports a header only at its first inclusion, which may be transitive.
PARSE_OPTIONS = (
    ci.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES
    | ci.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD
)
# The first 20 diagnostics locate a missing -I or define; the count reports the rest.
MAX_REPORTED_DIAGNOSTICS = 20


class HeaderParseError(RuntimeError):
    """libclang reported an error; a partial AST would silently under-report."""


def parse_tu(
    cpp: Path, extra_include_dirs: Sequence[Path] = (), unsaved: str | None = None
) -> ci.TranslationUnit:
    """Parse one binding translation unit.

    Args:
        cpp: The binding `.cpp`, or the name of a synthetic TU.
        extra_include_dirs: Searched before `INCLUDE_DIRS` (test fixtures).
        unsaved: Source text of `cpp` that exists only in memory (a synthetic TU).

    Returns:
        The translation unit, free of error diagnostics.

    Raises:
        HeaderParseError: any diagnostic of severity >= error.
    """
    args = [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]
    files = [(str(cpp), unsaved)] if unsaved is not None else None
    tu = ci.Index.create().parse(str(cpp), args=args, unsaved_files=files, options=PARSE_OPTIONS)
    errors = [d for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        shown = "\n".join(str(d) for d in errors[:MAX_REPORTED_DIAGNOSTICS])
        raise HeaderParseError(f"{cpp}: {len(errors)} error diagnostics\n{shown}")
    return tu


# One extension module per binding TU: src/<module>/<module>_bindings.cpp.
# tests/scripts/test_generate_stubs.py pins this set == generate_stubs.discover_modules().
BINDING_GLOB = "*/*_bindings.cpp"

# Directly included headers under this prefix (relative to an include dir) are a module's
# audited API; every other declaration in the TU only resolves Python names. Explicit per
# module because binding TUs also include other components' headers (spec amendment A2).
AUDITED_HEADER_PREFIX = {
    "tesseract_collision": "tesseract/collision/",
    "tesseract_common": "tesseract/common/",
    "tesseract_environment": "tesseract/environment/",
}
# Headers under a prefix that are not Python API, as fnmatch patterns on the path below the
# prefix (`*` also matches `/`). A header the binding #includes directly is audited anyway.
UNAUDITED_HEADERS = {
    "test_suite/*": "gtest and Google Benchmark sources installed for plugin authors' tests.",
    "*_impl.hpp": "cereal implementation fragment, valid only after its `cereal_serialization.h`.",
    "bullet/*": "Bullet backend internals, loaded as a contact manager plugin; Python reaches "
    "them through the `DiscreteContactManager`/`ContinuousContactManager` interfaces.",
    "fcl/*": "FCL backend internals, loaded as a contact manager plugin; Python reaches them "
    "through the `DiscreteContactManager` interface.",
    "vhacd/VHACD.h": "Vendored third-party V-HACD library (namespace `VHACD`).",
}
# A header another binding TU #includes directly (`<…>` form) is audited with that module.
INCLUDE_DIRECTIVE = re.compile(r"^\s*#\s*include\s*<([^>]+)>", re.MULTILINE)
HEADER_SUFFIXES = frozenset({".h", ".hpp"})


class UnknownModuleError(LookupError):
    """The requested module is not an auditable binding module."""


class StubMissingError(FileNotFoundError):
    """The module has no committed stub."""


def binding_modules() -> list[str]:
    """Short names of all binding modules, sorted."""
    return sorted(p.parent.name for p in SRC.glob(BINDING_GLOB))


def binding_source(module: str) -> Path:
    """`tesseract_collision` → `src/tesseract_collision/tesseract_collision_bindings.cpp`."""
    return SRC / module / f"{module}_bindings.cpp"


def committed_stub(module: str) -> Path:
    """`tesseract_collision` → `src/tesseract_robotics/tesseract_collision/_tesseract_collision.pyi`."""
    return STUB_ROOT / module / f"_{module}.pyi"


def resolve_module(module: str) -> str:
    """Validate a short module name for auditing.

    Raises:
        UnknownModuleError: not a binding module, or no `AUDITED_HEADER_PREFIX` entry yet.
        StubMissingError: no committed stub.
    """
    known = binding_modules()
    if module not in known:
        raise UnknownModuleError(f"{module!r} is not a binding module; known: {', '.join(known)}")
    if module not in AUDITED_HEADER_PREFIX:
        raise UnknownModuleError(f"{module!r} has no AUDITED_HEADER_PREFIX entry yet")
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


# libclang spells this parameter's pointee type so; the spec accepts it bound as a `str` return.
STRINGSTREAM = "std::stringstream"
# cereal serialization hooks, member or free function: not API.
CEREAL_HOOKS = frozenset({"serialize", "load", "save"})
# EIGEN_MAKE_ALIGNED_OPERATOR_NEW expands to these in every aligned tesseract type: memory
# management, not API (spec amendment A4).
ALLOCATION_OPERATORS = frozenset(
    {"operator new", "operator new[]", "operator delete", "operator delete[]"}
)
# C++ operators a binding exposes as Python dunders. Any other operator is a gap.
OPERATOR_DUNDERS = {
    "operator==": "__eq__",
    "operator!=": "__ne__",
    "operator[]": "__getitem__",
    "operator<": "__lt__",
    "operator bool": "__bool__",
    "operator()": "__call__",
    "operator*": "__mul__",  # Eigen's is a member template; libclang still spells it so (M13)
}
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
# `operator<<(std::ostream&, const T&)` is bound as `T.__str__`; libclang spells the stream
# parameter's canonical declaration so.
OSTREAM_DECL = "basic_ostream"
# Namespaces that audited headers reopen to specialise library templates
# (`std::hash<LinkNamesPair>` in tesseract/common/types.h): not module API.
FOREIGN_NAMESPACES = frozenset({"std"})
RECORD_KINDS = frozenset(
    {ci.CursorKind.CLASS_DECL, ci.CursorKind.STRUCT_DECL, ci.CursorKind.CLASS_TEMPLATE}
)
FUNCTION_KINDS = frozenset({ci.CursorKind.FUNCTION_DECL, ci.CursorKind.FUNCTION_TEMPLATE})
METHOD_KINDS = frozenset(
    {ci.CursorKind.CXX_METHOD, ci.CursorKind.FUNCTION_TEMPLATE, ci.CursorKind.CONVERSION_FUNCTION}
)
FIELD_KINDS = frozenset({ci.CursorKind.FIELD_DECL, ci.CursorKind.VAR_DECL})


class Kind(str, Enum):
    """What a report row refers to."""

    CLASS = "class"
    ENUM = "enum"
    FUNCTION = "function"
    METHOD = "method"
    CONSTRUCTOR = "constructor"
    FIELD = "field"
    ENUMERATOR = "enumerator"
    OPERATOR = "operator"
    OVERLOAD = "overload"
    CONSTANT = "constant"
    PROTOCOL = "protocol"
    FAIL_LOUD = "fail-loud"
    HEADER = "header"


@dataclass(frozen=True)
class Arity:
    """Accepted positional-argument counts; `hi is None` means unbounded (`*args`)."""

    lo: int
    hi: int | None

    def overlaps(self, other: Arity) -> bool:
        return (self.hi is None or other.lo <= self.hi) and (
            other.hi is None or self.lo <= other.hi
        )

    def reduced(self, k: int) -> Arity:
        """Arity after `k` out-params move to the return value."""
        return Arity(max(0, self.lo - k), None if self.hi is None else self.hi - k)

    def __str__(self) -> str:
        if self.hi is None:
            return f"{self.lo}+"
        return str(self.lo) if self.lo == self.hi else f"{self.lo}-{self.hi}"


@dataclass(frozen=True)
class CppOverload:
    arity: Arity
    out_params: tuple[str, ...]  # pointee type spellings
    location: str
    returns_void: bool = False
    mapped: str | None = None  # ACCEPTED rule when bound under its mapped Python name
    absent_ok: str | None = None  # ACCEPTED rule under which leaving it unbound is accepted


@dataclass
class CppSymbol:
    name: str
    kind: Kind
    location: str
    overloads: list[CppOverload] = field(default_factory=list)


def header_rel(path: Path) -> str:
    """A header path relative to the conda include dir or the repo."""
    path = path.resolve()
    base = CONDA_INCLUDE if path.is_relative_to(CONDA_INCLUDE) else REPO_ROOT
    return path.relative_to(base).as_posix()


def location(cursor: ci.Cursor) -> str:
    """`header:line`, relative to the conda include dir or the repo."""
    return f"{header_rel(Path(cursor.location.file.name))}:{cursor.location.line}"


def audited_headers(
    tu: ci.TranslationUnit, prefix: str, include_dirs: Sequence[Path]
) -> frozenset[Path]:
    """Headers the TU's main file `#include`s itself and that live under `prefix`.

    Read from the main file's inclusion directives, not `get_includes()` depth: a header
    that an earlier include already pulled in is reported there only at that depth.
    """
    roots = [(d / prefix).resolve() for d in include_dirs]
    main = Path(tu.spelling).resolve()
    direct = (
        Path(c.get_included_file().name).resolve()
        for c in tu.cursor.get_children()
        if c.kind == ci.CursorKind.INCLUSION_DIRECTIVE
        and Path(c.location.file.name).resolve() == main
    )
    return frozenset(h for h in direct if any(h.is_relative_to(r) for r in roots))


def prefix_headers(prefix: str, include_dirs: Sequence[Path]) -> dict[Path, str]:
    """Every header under `prefix` in the include dirs: resolved path → `#include` spelling."""
    found: dict[Path, str] = {}
    for d in include_dirs:
        root = d / prefix
        if not root.is_dir():
            continue
        for h in sorted(root.rglob("*")):
            if h.suffix in HEADER_SUFFIXES:
                found.setdefault(h.resolve(), h.relative_to(d).as_posix())
    return found


def direct_include_spellings(sources: Sequence[Path]) -> frozenset[str]:
    """`<…>` spellings the files #include, read as text (other bindings are not parsed)."""
    return frozenset(
        m for s in sources for m in INCLUDE_DIRECTIVE.findall(s.read_text(encoding="utf-8"))
    )


def is_unaudited(spelling: str, prefix: str) -> bool:
    below = spelling.removeprefix(prefix)
    return any(fnmatch.fnmatchcase(below, pattern) for pattern in UNAUDITED_HEADERS)


def _is_out_param(parm: ci.Cursor) -> bool:
    """Non-const lvalue reference to a non-abstract type (spec amendment A3)."""
    t = parm.type
    if t.kind != ci.TypeKind.LVALUEREFERENCE or t.get_pointee().is_const_qualified():
        return False
    decl = t.get_pointee().get_canonical().get_declaration()
    return not (decl.kind in RECORD_KINDS and decl.is_abstract_record())


def _has_default(parm: ci.Cursor) -> bool:
    return any(c.kind.is_expression() for c in parm.get_children())


def _record_path(decl: ci.Cursor) -> str:
    """Dotted name of a record within its namespace (`Outer.Inner`)."""
    parts = [decl.spelling]
    parent = decl.semantic_parent
    while parent is not None and parent.kind in RECORD_KINDS:
        parts.append(parent.spelling)
        parent = parent.semantic_parent
    return ".".join(reversed(parts))


def _stream_insertion_owner(c: ci.Cursor) -> str | None:
    """`T` of a free `operator<<(std::ostream&, const T&)` with `T` a class, else None."""
    if c.spelling != "operator<<":
        return None
    params = [p for p in c.get_children() if p.kind == ci.CursorKind.PARM_DECL]
    if len(params) != 2:
        return None
    stream, value = (p.type.get_pointee().get_canonical().get_declaration() for p in params)
    if stream.spelling != OSTREAM_DECL or value.kind not in RECORD_KINDS:
        return None
    return _record_path(value)


def cpp_api(tu: ci.TranslationUnit, headers: frozenset[Path]) -> dict[str, CppSymbol]:
    """The audited C++ API: public, non-deprecated declarations in `headers`.

    Keys are dotted names within the namespace; operators that a binding maps to a
    dunder are keyed by the dunder, constructors by `__init__`.
    """
    symbols: dict[str, CppSymbol] = {}
    seen: set[str] = set()

    def first_time(c: ci.Cursor) -> bool:
        usr = c.get_usr()
        if usr in seen:
            return False
        seen.add(usr)
        return True

    def add(name: str, kind: Kind, c: ci.Cursor) -> CppSymbol:
        return symbols.setdefault(name, CppSymbol(name, kind, location(c)))

    def add_callable(
        name: str,
        kind: Kind,
        c: ci.Cursor,
        mapped: str | None = None,
        nullary_absent_ok: str | None = None,
    ) -> None:
        if not first_time(c):
            return
        if mapped is not None:
            # A mapped operator's operands become `self` and the result: nothing positional.
            ov = CppOverload(Arity(0, 0), (), location(c), mapped=mapped)
            add(name, kind, c).overloads.append(ov)
            return
        params = [p for p in c.get_children() if p.kind == ci.CursorKind.PARM_DECL]
        n_default = sum(_has_default(p) for p in params)
        out = tuple(p.type.get_pointee().spelling for p in params if _is_out_param(p))
        arity = Arity(len(params) - n_default, len(params))
        void = c.result_type.kind == ci.TypeKind.VOID
        absent_ok = nullary_absent_ok if arity.hi == 0 else None
        ov = CppOverload(arity, out, location(c), void, absent_ok=absent_ok)
        add(name, kind, c).overloads.append(ov)

    def add_enum(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.ENUM, c)
        for e in c.get_children():
            if e.kind == ci.CursorKind.ENUM_CONSTANT_DECL:
                add(f"{name}.{e.spelling}", Kind.ENUMERATOR, e)

    def add_record(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.CLASS, c)
        # An abstract class cannot be constructed from Python: no __init__ to audit.
        abstract = c.is_abstract_record()
        declares_ctor = False
        befriends_cereal = False
        ctors: list[ci.Cursor] = []
        for m in c.get_children():
            if m.kind == ci.CursorKind.CONSTRUCTOR:
                declares_ctor = True
            if m.kind == ci.CursorKind.FRIEND_DECL:
                befriends_cereal |= any(f.spelling in CEREAL_HOOKS for f in m.get_children())
            if m.access_specifier != ci.AccessSpecifier.PUBLIC:
                continue
            if m.availability == ci.AvailabilityKind.DEPRECATED:
                continue
            if m.kind == ci.CursorKind.CONSTRUCTOR:
                if not (
                    abstract
                    or m.is_copy_constructor()
                    or m.is_move_constructor()
                    or m.is_deleted_method()
                ):
                    ctors.append(m)
            elif m.kind in METHOD_KINDS:
                if (
                    m.spelling in CEREAL_HOOKS
                    or m.spelling in ALLOCATION_OPERATORS
                    or m.is_deleted_method()
                    or m.is_copy_assignment_operator_method()
                    or m.is_move_assignment_operator_method()
                ):
                    continue
                if m.spelling.startswith("operator"):
                    dunder = OPERATOR_DUNDERS.get(m.spelling, m.spelling)
                    add_callable(f"{name}.{dunder}", Kind.OPERATOR, m)
                else:
                    add_callable(f"{name}.{m.spelling}", Kind.METHOD, m)
            elif m.kind in FIELD_KINDS:
                add(f"{name}.{m.spelling}", Kind.FIELD, m)
            elif m.kind in RECORD_KINDS and m.is_definition():
                add_record(f"{name}.", m)
            elif m.kind == ci.CursorKind.ENUM_DECL and m.is_definition():
                add_enum(f"{name}.", m)
        # A cereal-befriending class with another public ctor has its default ctor for
        # deserialization only (serialization-default-ctor).
        serial = "serialization-default-ctor" if befriends_cereal and len(ctors) > 1 else None
        for m in ctors:
            add_callable(f"{name}.__init__", Kind.CONSTRUCTOR, m, nullary_absent_ok=serial)
        if not (declares_ctor or abstract):
            # The compiler declares an implicit default constructor.
            add(f"{name}.__init__", Kind.CONSTRUCTOR, c).overloads.append(
                CppOverload(Arity(0, 0), (), location(c))
            )

    def visit(scope: ci.Cursor) -> None:
        for c in scope.get_children():
            if c.kind == ci.CursorKind.NAMESPACE:
                if c.spelling not in FOREIGN_NAMESPACES:
                    visit(c)
                continue
            if c.location.file is None or Path(c.location.file.name).resolve() not in headers:
                continue
            if c.availability == ci.AvailabilityKind.DEPRECATED:
                continue
            if c.kind in RECORD_KINDS and c.is_definition() and first_time(c):
                add_record("", c)
            elif c.kind == ci.CursorKind.ENUM_DECL and c.is_definition() and first_time(c):
                add_enum("", c)
            elif c.kind in FUNCTION_KINDS and c.spelling not in CEREAL_HOOKS:
                owner = _stream_insertion_owner(c)
                if owner is not None:
                    add_callable(f"{owner}.__str__", Kind.OPERATOR, c, mapped="stream-insertion")
                else:
                    add_callable(c.spelling, Kind.FUNCTION, c)

    visit(tu.cursor)
    return symbols


def decl_names(tu: ci.TranslationUnit) -> frozenset[str]:
    """Spelling of every declaration in the TU: what a Python name may resolve to."""
    return frozenset(c.spelling for c in tu.cursor.walk_preorder() if c.kind.is_declaration())


ENUM_BASES = frozenset({"enum.Enum", "enum.IntEnum", "enum.Flag", "enum.IntFlag"})
DUNDER_OPERATORS = {v: k for k, v in OPERATOR_DUNDERS.items()}
NOT_AN_OVERLOAD = "—"  # arity column of rows that are not about one overload
TRY_IMPORT = "try: import … except ImportError"


@dataclass(frozen=True)
class PyOverload:
    arity: Arity
    returns: str
    line: int


@dataclass
class PySymbol:
    name: str
    kind: Kind
    line: int
    overloads: list[PyOverload] = field(default_factory=list)
    bases: tuple[str, ...] = ()  # stub base-class expressions, for inherited members
    enum_alias: bool = False  # module constant whose value is a stub enum member (Note 1)


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
                    sym.overloads.append(PyOverload(_py_arity(node, bound), returns, node.lineno))
            elif isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                for t in targets:
                    if isinstance(t, ast.Name):
                        kind = (
                            Kind.ENUMERATOR if in_enum else Kind.FIELD if prefix else Kind.CONSTANT
                        )
                        name = prefix + t.id
                        alias = kind is Kind.CONSTANT and _enum_member(node.value, symbols)
                        symbols[name] = PySymbol(name, kind, node.lineno, enum_alias=alias)
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
        if isinstance(n, (ast.ClassDef, ast.FunctionDef))
    ]
    rows += [
        Deviation(TRY_IMPORT, Kind.FAIL_LOUD, f"{where}:{n.lineno}")
        for n in ast.walk(tree)
        if isinstance(n, ast.Try) and any(_catches_import_error(h) for h in n.handlers)
    ]
    return sorted(rows)


# Deviations accepted by rule; each is printed with its reason in the report.
ACCEPTED = {
    "out-param": "A non-const lvalue-reference out-param is returned in a tuple with the result "
    "(Phase A precedent: checkTrajectory), or alone when the C++ returns `void`.",
    "stringstream": "A `std::stringstream&` parameter the C++ writes into is returned as `str`.",
    "scalar-last-quaternion": "Quaterniond takes (x, y, z, w), the project-wide scalar-last "
    "order; Eigen's constructor is (w, x, y, z).",
    "container-protocol": "`size()` is bound as `__len__` and `operator[]` as "
    "`__getitem__`/`__setitem__`, the Python container protocol.",
    "iterator-pair": "A `begin()`/`end()` pair (and `cbegin`/`cend`) is bound as `__iter__`.",
    "stream-insertion": "A free `operator<<(std::ostream&, const T&)` is bound as `T.__str__`.",
    "presentation-dunder": "`__repr__` and `__str__` are Python presentation and need no C++ "
    "counterpart.",
    "serialization-default-ctor": "The default constructor of a class that befriends cereal "
    "`serialize` and declares another public constructor exists for deserialization only.",
    "eigen-template-instance": "`Eigen::Hyperplane<double, 3>` and "
    "`Eigen::ParametrizedLine<double, 3>` have no Eigen typedef; the class takes Eigen's own "
    "`…3d` naming (`Vector3d`, `Quaterniond`).",
    "quaternion-rpy": "`Quaterniond.from_rpy`/`to_rpy` convert roll-pitch-yaw in the ROS/tf2 "
    "convention, which Eigen spells as three composed `AngleAxisd` rotations; `to_rpy` returns "
    "tf2's canonical ranges, which `eulerAngles` does not guarantee.",
    "eigen-default-precision": "`EIGEN_DEFAULT_PREC` re-exports "
    "`Eigen::NumTraits<double>::dummy_precision()` (1e-12) so Python compares with Eigen's "
    "own default tolerance instead of a duplicated literal.",
}
# Python names accepted by a named rule: (module, python name) -> ACCEPTED key.
ACCEPTED_SYMBOLS = {
    ("tesseract_common", "Quaterniond.__init__"): "scalar-last-quaternion",
    ("tesseract_common", "Quaterniond.from_xyzw"): "scalar-last-quaternion",
    ("tesseract_common", "Hyperplane3d"): "eigen-template-instance",
    ("tesseract_common", "ParametrizedLine3d"): "eigen-template-instance",
    ("tesseract_common", "Quaterniond.from_rpy"): "quaternion-rpy",
    ("tesseract_common", "Quaterniond.to_rpy"): "quaternion-rpy",
    ("tesseract_common", "EIGEN_DEFAULT_PREC"): "eigen-default-precision",
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


def _cover(ov: CppOverload, pys: list[PyOverload]) -> str | None:
    """How a C++ overload is covered by Python overloads: "exact", an ACCEPTED rule, or None.

    Out-param rules are tried first: `checkTrajectory`'s 5-parameter C++ overload would
    otherwise pair by raw arity with the wrong 5-argument Python overload.
    """
    if ov.out_params:
        reduced = ov.arity.reduced(len(ov.out_params))
        fits = [p for p in pys if p.arity.overlaps(reduced)]
        if set(ov.out_params) == {STRINGSTREAM} and any(p.returns == "str" for p in fits):
            return "stringstream"
        if any(p.returns.startswith("tuple[") for p in fits):
            return "out-param"
        if ov.returns_void and len(ov.out_params) == 1 and fits:
            return "out-param"  # nothing else to return: the out-param is the result
    if any(p.arity.overlaps(ov.arity) for p in pys):
        return ov.mapped or "exact"
    return None


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


def _py_member(name: str, py: PyApi) -> PySymbol | None:
    """The stub symbol for `Class.member`, looked up through the stub's base classes.

    nanobind subclasses inherit bound members, so a C++ override needs no own binding.
    Only bases declared in the same stub resolve; others cannot be checked here.
    """
    if name in py.symbols:
        return py.symbols[name]
    owner, _, member = name.rpartition(".")
    cls = py.symbols.get(owner)
    if cls is None:
        return None
    for base in cls.bases:
        found = _py_member(f"{base}.{member}", py)
        if found is not None:
            return found
    return None


def _ancestor_missing(name: str, cpp: dict[str, CppSymbol], py: PyApi) -> bool:
    parts = name.split(".")
    owners = (".".join(parts[:i]) for i in range(1, len(parts)))
    return any(o in cpp and o not in py.symbols for o in owners)


def match(
    module: str, cpp: dict[str, CppSymbol], tu_names: frozenset[str], py: PyApi, stub_rel: str
) -> ModuleReport:
    """Diff the audited C++ API against the stub."""
    gaps: list[Gap] = []
    accepted: list[Accepted] = []
    covered = 0
    for name, sym in cpp.items():
        if _ancestor_missing(name, cpp, py):
            continue  # the missing class is the one row
        ps = _py_member(name, py)
        if ps is None:
            owner, _, leaf = name.rpartition(".")
            dunder = PROTOCOL_MEMBERS.get(leaf)
            if dunder and _py_member(f"{owner}.{dunder}", py):
                covered += 1  # the dunder's own row names the rule
            else:
                gaps.append(Gap(name, sym.kind, sym.location))
            continue
        ok = True
        for ov in sym.overloads:
            how = _cover(ov, ps.overloads)
            if how is None and ov.absent_ok:
                accepted.append(Accepted(name, ov.absent_ok, ov.location))
            elif how is None:
                gaps.append(Gap(name, sym.kind, ov.location, str(ov.arity)))
                ok = False
            elif how != "exact":
                accepted.append(Accepted(name, how, ov.location))
        covered += ok

    deviations: list[Deviation] = []
    for name, ps in py.symbols.items():
        where = f"{stub_rel}:{ps.line}"
        rule = ACCEPTED_SYMBOLS.get((module, name))
        leaf = name.rpartition(".")[2]
        if ps.enum_alias:
            found = None  # a second name for an enum value, even if C++ has it unscoped
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
                if not any(_cover(ov, [p]) for ov in cpp[name].overloads):
                    deviations.append(
                        Deviation(name, Kind.OVERLOAD, f"{stub_rel}:{p.line}", str(p.arity))
                    )
        elif rule:
            accepted.append(Accepted(name, rule, where))

    return ModuleReport(
        module, covered, sorted(gaps), sorted(deviations), sorted(accepted), py.quoted
    )


def unincluded_header_gaps(
    cpp_path: Path, headers: dict[Path, str], extra_include_dirs: Sequence[Path] = ()
) -> list[Gap]:
    """One `header` gap per header the binding never includes, if it declares any API.

    The headers are parsed in a synthetic TU that includes the binding first, so they see
    the binding's flags and macros. The row points at the header's first auditable
    declaration; a header of forward declarations or macros only gets no row.
    """
    if not headers:
        return []
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
    prefix: str,
    extra_include_dirs: Sequence[Path] = (),
    other_bindings: Sequence[Path] = (),
) -> ModuleReport:
    """Audit one translation unit against one stub and package `__init__.py`.

    Audits every header under `prefix`, except `UNAUDITED_HEADERS` and headers that one of
    `other_bindings` #includes directly; the binding's own direct includes always count.
    """
    include_dirs = (*extra_include_dirs, *INCLUDE_DIRS)
    tu = parse_tu(cpp_path, extra_include_dirs)
    direct = audited_headers(tu, prefix, include_dirs)
    owned_elsewhere = direct_include_spellings(other_bindings)
    audited = {
        h: spelling
        for h, spelling in prefix_headers(prefix, include_dirs).items()
        if h in direct or not (is_unaudited(spelling, prefix) or spelling in owned_elsewhere)
    }
    included = frozenset(Path(i.include.name).resolve() for i in tu.get_includes())
    report = match(
        module,
        cpp_api(tu, frozenset(audited) & included),
        decl_names(tu),
        py_api(load_stub(stub_path), rel(stub_path)),
        rel(stub_path),
    )
    unincluded = {h: s for h, s in audited.items() if h not in included}
    report.gaps = sorted(
        [*report.gaps, *unincluded_header_gaps(cpp_path, unincluded, extra_include_dirs)]
    )
    report.deviations = sorted([*report.deviations, *init_findings(init_path)])
    return report


def audit_module(module: str) -> ModuleReport:
    """Audit one binding module by short name.

    Raises:
        UnknownModuleError: see `resolve_module`.
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
    )


REPORT_PATH = REPO_ROOT / "docs" / "developer" / "binding-api-audit.md"
LIMITATION = (
    "Python member names are checked against every declaration in the TU, so a "
    "Python-only member that shares a common C++ name (`size`, `clear`) is not flagged. "
    "Parameter types are not compared beyond quoted C++ names. "
    "Overloads are matched by arity only: a C++ overload is never reported while a bound "
    "overload takes the same number of arguments (e.g. `Environment::init(commands)` hidden "
    "by `init(scene_graph)`)."
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


def render_markdown(reports: list[ModuleReport], prov: dict[str, str]) -> str:
    """The mkdocs report page: provenance, conclusion, then per-module tables."""
    out = [
        "# Binding API audit",
        "",
        "Generated by `pixi run -e audit audit-bindings`; do not edit by hand.",
        f"tesseract-robotics `{prov['tesseract-robotics']}` · libclang `{prov['libclang']}` · "
        f"stubs `{prov['stubs']}`.",
        "",
        "| module | covered | gaps | deviations | accepted | quoted types |",
        "|---|---|---|---|---|---|",
        *(
            f"| {r.module} | {r.covered} | {len(r.gaps)} | {len(r.deviations)} | "
            f"{len(r.accepted)} | {len(r.quoted)} |"
            for r in reports
        ),
        "",
        "Quoted C++ names in stubs (unbound type, or a missing `nb::module_::import_`):",
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
        "    - Headers another binding #includes directly are audited with that module.",
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
    return "\n".join(out).rstrip() + "\n"


def to_json(reports: list[ModuleReport], prov: dict[str, str]) -> str:
    """Same data as the page, for issue drafting."""
    modules = {
        r.module: {
            "covered": r.covered,
            "gaps": [asdict(g) for g in r.gaps],
            "deviations": [asdict(d) for d in r.deviations],
            "accepted": [asdict(a) for a in r.accepted],
            "quoted": [asdict(q) for q in r.quoted],
        }
        for r in reports
    }
    return json.dumps({"provenance": prov, "modules": modules}, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "modules", nargs="*", default=sorted(AUDITED_HEADER_PREFIX),
        help="short module names (default: every module with an AUDITED_HEADER_PREFIX entry)",
    )  # fmt: skip
    parser.add_argument("--json", type=Path, help="also write the findings as JSON")
    args = parser.parse_args(argv)

    modules = sorted(resolve_module(m) for m in args.modules)
    reports = [audit_module(m) for m in modules]
    prov = provenance(modules)
    REPORT_PATH.write_text(render_markdown(reports, prov), encoding="utf-8")
    if args.json:
        args.json.write_text(to_json(reports, prov), encoding="utf-8")
    print(f"{len(reports)} modules → {REPORT_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
