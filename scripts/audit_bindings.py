"""Audit each binding's C++ API against its committed Python stub.

For each extension module, libclang parses `src/<module>/<module>_bindings.cpp`
(exactly the headers and macros the binding sees) and `ast` parses the
committed `_<module>.pyi` and the package `__init__.py`. No extension module is
imported. Gaps (C++ without Python), deviations (Python without C++) and
accepted deviations go to `docs/developer/binding-api-audit.md`.

Usage:
    pixi run -e audit audit-bindings [MODULE ...] [--json PATH]
"""

from __future__ import annotations

import ast
import os
import sysconfig
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import clang.cindex as ci
import nanobind

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
# Declarations only: binding bodies (the NB_MODULE block) carry no API.
PARSE_OPTIONS = ci.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES
# The first 20 diagnostics locate a missing -I or define; the count reports the rest.
MAX_REPORTED_DIAGNOSTICS = 20


class HeaderParseError(RuntimeError):
    """libclang reported an error; a partial AST would silently under-report."""


def parse_tu(cpp: Path, extra_include_dirs: Sequence[Path] = ()) -> ci.TranslationUnit:
    """Parse one binding translation unit.

    Args:
        cpp: The binding `.cpp`.
        extra_include_dirs: Searched before `INCLUDE_DIRS` (test fixtures).

    Returns:
        The translation unit, free of error diagnostics.

    Raises:
        HeaderParseError: any diagnostic of severity >= error.
    """
    args = [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]
    tu = ci.Index.create().parse(str(cpp), args=args, options=PARSE_OPTIONS)
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
# cereal serialization hooks: not API.
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
}
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


@dataclass
class CppSymbol:
    name: str
    kind: Kind
    location: str
    overloads: list[CppOverload] = field(default_factory=list)


def location(cursor: ci.Cursor) -> str:
    """`header:line`, relative to the conda include dir or the repo."""
    path = Path(cursor.location.file.name).resolve()
    base = CONDA_INCLUDE if path.is_relative_to(CONDA_INCLUDE) else REPO_ROOT
    return f"{path.relative_to(base).as_posix()}:{cursor.location.line}"


def audited_headers(
    tu: ci.TranslationUnit, prefix: str, include_dirs: Sequence[Path]
) -> frozenset[Path]:
    """Headers the TU's main file includes directly and that live under `prefix`."""
    roots = [(d / prefix).resolve() for d in include_dirs]
    direct = (Path(i.include.name).resolve() for i in tu.get_includes() if i.depth == 1)
    return frozenset(h for h in direct if any(h.is_relative_to(r) for r in roots))


def _is_out_param(parm: ci.Cursor) -> bool:
    """Non-const lvalue reference to a non-abstract type (spec amendment A3)."""
    t = parm.type
    if t.kind != ci.TypeKind.LVALUEREFERENCE or t.get_pointee().is_const_qualified():
        return False
    decl = t.get_pointee().get_canonical().get_declaration()
    return not (decl.kind in RECORD_KINDS and decl.is_abstract_record())


def _has_default(parm: ci.Cursor) -> bool:
    return any(c.kind.is_expression() for c in parm.get_children())


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

    def add_callable(name: str, kind: Kind, c: ci.Cursor) -> None:
        if not first_time(c):
            return
        params = [p for p in c.get_children() if p.kind == ci.CursorKind.PARM_DECL]
        n_default = sum(_has_default(p) for p in params)
        out = tuple(p.type.get_pointee().spelling for p in params if _is_out_param(p))
        arity = Arity(len(params) - n_default, len(params))
        add(name, kind, c).overloads.append(CppOverload(arity, out, location(c)))

    def add_enum(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.ENUM, c)
        for e in c.get_children():
            if e.kind == ci.CursorKind.ENUM_CONSTANT_DECL:
                add(f"{name}.{e.spelling}", Kind.ENUMERATOR, e)

    def add_record(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.CLASS, c)
        declares_ctor = False
        for m in c.get_children():
            if m.kind == ci.CursorKind.CONSTRUCTOR:
                declares_ctor = True
            if m.access_specifier != ci.AccessSpecifier.PUBLIC:
                continue
            if m.availability == ci.AvailabilityKind.DEPRECATED:
                continue
            if m.kind == ci.CursorKind.CONSTRUCTOR:
                if not (
                    m.is_copy_constructor() or m.is_move_constructor() or m.is_deleted_method()
                ):
                    add_callable(f"{name}.__init__", Kind.CONSTRUCTOR, m)
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
        if not declares_ctor:
            # The compiler declares an implicit default constructor.
            add(f"{name}.__init__", Kind.CONSTRUCTOR, c).overloads.append(
                CppOverload(Arity(0, 0), (), location(c))
            )

    def visit(scope: ci.Cursor) -> None:
        for c in scope.get_children():
            if c.kind == ci.CursorKind.NAMESPACE:
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
            elif c.kind in FUNCTION_KINDS:
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


def py_api(tree: ast.Module, stub_rel: str) -> PyApi:
    """Names, kinds and overload arities a stub declares, plus quoted C++ annotations."""
    symbols: dict[str, PySymbol] = {}
    quoted: list[QuotedType] = []

    def visit(body: list[ast.stmt], prefix: str, in_enum: bool) -> None:
        for node in body:
            if isinstance(node, ast.ClassDef):
                is_enum = any(ast.unparse(b) in ENUM_BASES for b in node.bases)
                name = prefix + node.name
                symbols[name] = PySymbol(name, Kind.ENUM if is_enum else Kind.CLASS, node.lineno)
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
                        symbols[name] = PySymbol(name, kind, node.lineno)
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
