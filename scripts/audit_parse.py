"""libclang: parse a binding translation unit, select the headers it audits, and read their C++ API."""

from __future__ import annotations

import os
import sysconfig
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import audit_types
import clang.cindex as ci
import nanobind
from audit_ownership import (
    HEADER_SUFFIXES,
    INCLUDE_DIRECTIVE,
    REPO_ROOT,
    SRC,
    header_auditor,
    header_owner,
    is_third_party,
    is_unaudited,
    owning_prefix,
)
from audit_types import RECORD_KINDS, TypeKey, record_path

CONDA_PREFIX = Path(os.environ["CONDA_PREFIX"])
CONDA_INCLUDE = CONDA_PREFIX / "include"
# Major version of the `clang-22` package in the pixi `audit` feature (pyproject.toml).
LIBCLANG_MAJOR = 22
# Builtin headers (stdarg.h, stddef.h) shipped by `clang-22`; libclang does not find
# them on its own when loaded from the conda prefix.
CLANG_RESOURCE_DIR = CONDA_PREFIX / "lib" / "clang" / str(LIBCLANG_MAJOR)
# Matches CMakeLists.txt `set(CMAKE_CXX_STANDARD 17)`.
CXX_STD = "c++17"


class IncludeDirError(FileNotFoundError):
    """A versioned include dir of the binding build is missing or ambiguous."""


def versioned_include(name: str, include: Path) -> Path:
    """The one `include/<name>-<version>` dir (OMPL installs under `include/ompl-1.7`).

    Raises:
        IncludeDirError: no such dir, or more than one (an upstream bump left two).
    """
    found = sorted(d for d in include.glob(f"{name}-*") if d.is_dir())
    if len(found) != 1:
        raise IncludeDirError(
            f"expected one {include / name}-<version> dir, found {len(found)}: "
            f"{[d.name for d in found]}"
        )
    return found[0]


# Header search path of the binding build: tesseract, yaml-cpp and console_bridge
# from the conda prefix; Eigen under include/eigen3; OMPL under include/ompl-<version> and
# OSQP under include/osqp (the ompl and osqp CMake targets export both); tesseract_nb.h from
# src/; nanobind and Python headers from their packages.
INCLUDE_DIRS = (
    CONDA_INCLUDE,
    CONDA_INCLUDE / "eigen3",
    versioned_include("ompl", CONDA_INCLUDE),
    CONDA_INCLUDE / "osqp",
    SRC,
    Path(nanobind.include_dir()),
    Path(sysconfig.get_paths()["include"]),
)
PARSE_ARGS = (
    "-x",
    "c++",
    f"-std={CXX_STD}",
    # osqp-eigen's CMake target exports this for OSQP >= 1.0; its headers need it.
    "-DOSQP_EIGEN_OSQP_IS_V1",
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
# `bound_classes` reads the NB_MODULE body, which PARSE_OPTIONS skips: same record, all bodies.
BODY_PARSE_OPTIONS = ci.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD
# The first 20 diagnostics locate a missing -I or define; the count reports the rest.
MAX_REPORTED_DIAGNOSTICS = 20


class HeaderParseError(RuntimeError):
    """libclang reported an error; a partial AST would silently under-report.

    Attributes:
        first_error: `file:line` of the first error, relative like `location`.
    """

    def __init__(self, message: str, first_error: str) -> None:
        super().__init__(message)
        self.first_error = first_error


def parse_tu(
    cpp: Path,
    extra_include_dirs: Sequence[Path] = (),
    unsaved: str | None = None,
    *,
    bodies: bool = False,
) -> ci.TranslationUnit:
    """Parse one binding translation unit.

    Args:
        cpp: The binding `.cpp`, or the name of a synthetic TU.
        extra_include_dirs: Searched before `INCLUDE_DIRS` (test fixtures).
        unsaved: Source text of `cpp` that exists only in memory (a synthetic TU).
        bodies: Keep function bodies (`BODY_PARSE_OPTIONS`), for `bound_classes`.

    Returns:
        The translation unit, free of error diagnostics.

    Raises:
        HeaderParseError: any diagnostic of severity >= error.
    """
    args = [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]
    files = [(str(cpp), unsaved)] if unsaved is not None else None
    options = BODY_PARSE_OPTIONS if bodies else PARSE_OPTIONS
    tu = ci.Index.create().parse(str(cpp), args=args, unsaved_files=files, options=options)
    errors = [d for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        shown = "\n".join(str(d) for d in errors[:MAX_REPORTED_DIAGNOSTICS])
        first = errors[0].location
        raise HeaderParseError(
            f"{cpp}: {len(errors)} error diagnostics\n{shown}",
            f"{header_rel(Path(first.file.name))}:{first.line}",
        )
    return tu


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
# `operator<<(std::ostream&, const T&)` is bound as `T.__str__`; libclang spells the stream
# parameter's canonical declaration so.
OSTREAM_DECL = "basic_ostream"
# Namespaces that audited headers reopen to specialise library templates
# (`std::hash<LinkNamesPair>` in tesseract/common/types.h, `YAML::convert<PluginInfo>` in
# tesseract/common/yaml_extensions.h): not module API.
FOREIGN_NAMESPACES = frozenset({"std", "YAML"})
FUNCTION_KINDS = frozenset({ci.CursorKind.FUNCTION_DECL, ci.CursorKind.FUNCTION_TEMPLATE})
METHOD_KINDS = frozenset(
    {ci.CursorKind.CXX_METHOD, ci.CursorKind.FUNCTION_TEMPLATE, ci.CursorKind.CONVERSION_FUNCTION}
)
FIELD_KINDS = frozenset({ci.CursorKind.FIELD_DECL, ci.CursorKind.VAR_DECL})
# Pointees of a raw-buffer parameter: every scalar except plain `char` (CHAR_S / CHAR_U by
# platform), whose `const char*` nanobind binds as `str`.
RAW_BUFFER_POINTEES = frozenset(
    {
        ci.TypeKind.VOID,
        ci.TypeKind.BOOL,
        ci.TypeKind.UCHAR,
        ci.TypeKind.SCHAR,
        ci.TypeKind.SHORT,
        ci.TypeKind.USHORT,
        ci.TypeKind.INT,
        ci.TypeKind.UINT,
        ci.TypeKind.LONG,
        ci.TypeKind.ULONG,
        ci.TypeKind.LONGLONG,
        ci.TypeKind.ULONGLONG,
        ci.TypeKind.FLOAT,
        ci.TypeKind.DOUBLE,
        ci.TypeKind.LONGDOUBLE,
    }
)


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
    UNPARSEABLE_HEADER = "unparseable header"
    BASE = "base"  # a stub base class vs the bound C++ bases (T5)


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
    raw_buffer: bool = False  # takes a pointer to a non-char scalar: never covered by arity alone
    # Python positions of (const uint8_t*, size_t) pairs, each counted as one `bytes` parameter
    byte_buffers: tuple[int, ...] = ()
    # Keys a stub annotation may take, per parameter (a byte-buffer pair as one `bytes`);
    # empty: no parameter types to compare (`audit_types`, #247)
    params: tuple[frozenset[TypeKey] | None, ...] = ()
    out_positions: tuple[int, ...] = ()  # indices into `params` of the out-params

    def returned_out_params(self) -> tuple[frozenset[TypeKey] | None, ...]:
        """`params` of the Python form that returns the out-params."""
        return tuple(k for i, k in enumerate(self.params) if i not in self.out_positions)


@dataclass
class CppSymbol:
    name: str
    kind: Kind
    location: str
    overloads: list[CppOverload] = field(default_factory=list)
    qualified: str = ""  # `audit_types.qualified_path`, for Kind.CLASS and Kind.ENUM
    inherited_from: str | None = None  # `record_path` of the base declaring an inherited member


def header_rel(path: Path) -> str:
    """A header path relative to the conda include dir or the repo."""
    path = path.resolve()
    base = CONDA_INCLUDE if path.is_relative_to(CONDA_INCLUDE) else REPO_ROOT
    return path.relative_to(base).as_posix()


def location(cursor: ci.Cursor) -> str:
    """`header:line`, relative to the conda include dir or the repo."""
    return f"{header_rel(Path(cursor.location.file.name))}:{cursor.location.line}"


def audited_headers(
    tu: ci.TranslationUnit,
    module: str,
    prefixes: Mapping[str, tuple[str, ...]],
    include_dirs: Sequence[Path],
) -> frozenset[Path]:
    """Headers the TU's main file `#include`s itself that `module` owns (`owning_prefix`),
    except `THIRD_PARTY_HEADERS`.

    Read from the main file's inclusion directives, not `get_includes()` depth: a header
    that an earlier include already pulled in is reported there only at that depth.
    """
    roots = [((d / p).resolve(), p) for d in include_dirs for p in prefixes[module]]
    main = Path(tu.spelling).resolve()
    direct = (
        Path(c.get_included_file().name).resolve()
        for c in tu.cursor.get_children()
        if c.kind == ci.CursorKind.INCLUSION_DIRECTIVE
        and Path(c.location.file.name).resolve() == main
    )
    return frozenset(
        h
        for h in direct
        for r, p in roots
        if h.is_relative_to(r) and _owns(module, p + h.relative_to(r).as_posix(), prefixes)
    )


def _owns(module: str, spelling: str, prefixes: Mapping[str, tuple[str, ...]]) -> bool:
    """`module` owns the header and it is not a `THIRD_PARTY_HEADERS` entry."""
    owner = owning_prefix(spelling, prefixes)
    return (
        owner is not None
        and owner[0] == module
        and not is_third_party(spelling.removeprefix(owner[1]))
    )


def prefix_headers(
    module: str, prefixes: Mapping[str, tuple[str, ...]], include_dirs: Sequence[Path]
) -> dict[Path, str]:
    """Every header under `module`'s prefixes in the include dirs that `module` owns (a header
    under a longer prefix of another module is that module's): resolved path → spelling."""
    found: dict[Path, str] = {}
    for d in include_dirs:
        for prefix in prefixes[module]:
            root = d / prefix
            if not root.is_dir():
                continue
            for h in sorted(root.rglob("*")):
                spelling = h.relative_to(d).as_posix()
                if h.suffix in HEADER_SUFFIXES and header_owner(spelling, prefixes) == module:
                    found.setdefault(h.resolve(), spelling)
    return found


def direct_include_spellings(sources: Sequence[Path]) -> frozenset[str]:
    """`<…>` spellings the files #include, read as text (other bindings are not parsed)."""
    return frozenset(
        m for s in sources for m in INCLUDE_DIRECTIVE.findall(s.read_text(encoding="utf-8"))
    )


def binding_module(path: Path) -> str:
    """`src/tesseract_srdf/tesseract_srdf_bindings.cpp` → `tesseract_srdf`."""
    return path.stem.removesuffix("_bindings")


def foreign_headers(
    module: str,
    own_includes: frozenset[str],
    other_includes: Mapping[str, frozenset[str]],
    prefixes: Mapping[str, tuple[str, ...]],
    include_dirs: Sequence[Path],
) -> dict[Path, str]:
    """Headers under another module's prefix that this binding #includes directly, that the
    owner's binding does not, and that this module audits (`header_auditor`)."""
    found: dict[Path, str] = {}
    for spelling in own_includes:
        owner = header_owner(spelling, prefixes)
        if owner is None or owner == module or spelling in other_includes.get(owner, frozenset()):
            continue
        if is_unaudited(spelling, prefixes[owner]):
            continue
        includers = [module, *(m for m, inc in other_includes.items() if spelling in inc)]
        if header_auditor(includers) != module:
            continue
        for d in include_dirs:
            if (d / spelling).is_file():
                found[(d / spelling).resolve()] = spelling
                break
    return found


def _is_eigen_ref_of_non_const(t: ci.Type) -> bool:
    """A by-value `Eigen::Ref<T>` with non-const T: a writable view, like a non-const `T&`."""
    canonical = t.get_canonical()
    decl = canonical.get_declaration()
    if (
        decl.spelling != "Ref"
        or decl.semantic_parent is None
        or decl.semantic_parent.spelling != "Eigen"
    ):
        return False
    return not canonical.get_template_argument_type(0).is_const_qualified()


def _is_out_param(parm: ci.Cursor) -> bool:
    """Non-const lvalue reference to a non-abstract type (spec amendment A3), or a by-value
    `Eigen::Ref` of a non-const matrix (gh-213: `numericalJacobian`'s `jacobian`)."""
    t = parm.type
    if t.kind != ci.TypeKind.LVALUEREFERENCE:
        return _is_eigen_ref_of_non_const(t)
    if t.get_pointee().is_const_qualified():
        return False
    decl = t.get_pointee().get_canonical().get_declaration()
    return not (decl.kind in RECORD_KINDS and decl.is_abstract_record())


def _out_param_type(parm: ci.Cursor) -> str:
    """The written type: the pointee of a reference, the `Eigen::Ref` itself otherwise."""
    t = parm.type
    return t.get_pointee().spelling if t.kind == ci.TypeKind.LVALUEREFERENCE else t.spelling


def _is_raw_buffer(parm: ci.Cursor) -> bool:
    """A pointer to a scalar other than plain `char` (`const uint8_t*`, `double*`, `void*`).

    Such a parameter has no positional Python form (nanobind maps only `const char*`, to
    `str`), so a Python overload that merely takes as many arguments cannot be binding it.
    """
    t = parm.type.get_canonical()
    return t.kind == ci.TypeKind.POINTER and t.get_pointee().kind in RAW_BUFFER_POINTEES


# The spellings of a byte count that follows a byte pointer (`createMeshFromBytes(url, bytes,
# bytes_len)`, `BytesResource(url, bytes, bytes_len)`).
SIZE_SPELLINGS = frozenset({"size_t", "std::size_t"})


def _is_byte_pointer(parm: ci.Cursor) -> bool:
    """`const uint8_t*` (any spelling of a pointer to `const unsigned char`)."""
    t = parm.type.get_canonical()
    if t.kind != ci.TypeKind.POINTER:
        return False
    pointee = t.get_pointee()
    return pointee.kind == ci.TypeKind.UCHAR and pointee.is_const_qualified()


def _byte_buffer_pairs(params: list[ci.Cursor]) -> tuple[int, ...]:
    """Python positions of each `(const uint8_t* bytes, size_t len)` pair: one `bytes` argument."""
    positions: list[int] = []
    i = 0
    while i < len(params):
        if (
            _is_byte_pointer(params[i])
            and i + 1 < len(params)
            and params[i + 1].type.spelling in SIZE_SPELLINGS
        ):
            positions.append(i - len(positions))
            i += 2
        else:
            i += 1
    return tuple(positions)


def _has_default(parm: ci.Cursor) -> bool:
    return any(c.kind.is_expression() for c in parm.get_children())


# Canonical spelling of yaml-cpp's node type, which has no Python form.
YAML_NODE = "YAML::Node"


def _is_yaml_node(parm: ci.Cursor) -> bool:
    t = parm.type.get_canonical()
    if t.kind in (ci.TypeKind.LVALUEREFERENCE, ci.TypeKind.RVALUEREFERENCE):
        t = t.get_pointee().get_canonical()
    return t.spelling.removeprefix("const ") == YAML_NODE


# The key of a `(const uint8_t*, size_t)` pair: one Python `bytes` argument.
BYTES_KEYS = frozenset({("bytes",)})


def _same_python_form(a: CppOverload, b: CppOverload) -> bool:
    """Overloads that differ only in `const` of `this` or `&`/`&&` of a parameter: their
    parameter keys are equal and fully known, so one Python overload binds both."""
    same = (a.arity, a.out_params, a.params, a.out_positions) == (
        b.arity,
        b.out_params,
        b.params,
        b.out_positions,
    )
    return same and all(audit_types.is_known(k) for k in a.params)


def _stream_insertion_owner(c: ci.Cursor) -> str | None:
    """`T` of a free `operator<<(std::ostream&, const T&)` with `T` a class or enum, else None."""
    if c.spelling != "operator<<":
        return None
    params = [p for p in c.get_children() if p.kind == ci.CursorKind.PARM_DECL]
    if len(params) != 2:
        return None
    stream, value = (p.type.get_pointee().get_canonical().get_declaration() for p in params)
    if stream.spelling != OSTREAM_DECL or (
        value.kind not in RECORD_KINDS and value.kind != ci.CursorKind.ENUM_DECL
    ):
        return None
    return record_path(value)


def _befriends_cereal(record: ci.Cursor) -> bool:
    """The class, or a base of it, declares a cereal hook (`serialize`, …) as a friend."""
    for m in record.get_children():
        if m.kind == ci.CursorKind.FRIEND_DECL and any(
            f.spelling in CEREAL_HOOKS for f in m.get_children()
        ):
            return True
        if m.kind == ci.CursorKind.CXX_BASE_SPECIFIER and m.referenced is not None:
            if _befriends_cereal(m.referenced):
                return True
    return False


# Header roots of the libraries this project owns (spec D8): only a base declared under one
# of them contributes inherited members (A), not std, Eigen, ifopt or OMPL bases.
OWNED_ROOTS = ("tesseract/", "trajopt", "descartes_light/")


def _owned_headers(tu: ci.TranslationUnit) -> frozenset[Path]:
    """Headers the TU includes, from any file, by a spelling under `OWNED_ROOTS`."""
    return frozenset(
        Path(c.get_included_file().name).resolve()
        for c in tu.cursor.get_children()
        if c.kind == ci.CursorKind.INCLUSION_DIRECTIVE and c.spelling.startswith(OWNED_ROOTS)
    )


def _is_ctor(m: ci.Cursor, record: ci.Cursor) -> bool:
    """`m` constructs `record`. A constructor template (`template <class It> T(It, It)`) is a
    FUNCTION_TEMPLATE spelled like the class; it is a constructor, not a method named after it."""
    return m.kind == ci.CursorKind.CONSTRUCTOR or (
        m.kind == ci.CursorKind.FUNCTION_TEMPLATE and m.spelling == record.spelling
    )


def _member(m: ci.Cursor, record: ci.Cursor) -> tuple[str, Kind] | None:
    """Leaf name and kind of a public, non-deprecated method, operator or field of `record`
    that the audit compares, else None: the one member filter for a class's own members and
    the members it inherits. Constructors and nested types are not members here."""
    if m.access_specifier != ci.AccessSpecifier.PUBLIC:
        return None
    if m.availability == ci.AvailabilityKind.DEPRECATED:
        return None
    if m.kind in FIELD_KINDS:
        return m.spelling, Kind.FIELD
    if (
        m.kind not in METHOD_KINDS
        or _is_ctor(m, record)
        or m.spelling in CEREAL_HOOKS
        or m.spelling in ALLOCATION_OPERATORS
        or m.is_deleted_method()
        or m.is_copy_assignment_operator_method()
        or m.is_move_assignment_operator_method()
    ):
        return None
    kind = Kind.OPERATOR if m.spelling.startswith("operator") else Kind.METHOD
    return OPERATOR_DUNDERS.get(m.spelling, m.spelling), kind


def _declared_leaves(record: ci.Cursor) -> frozenset[str]:
    """Member names `record` declares itself, at any access: C++ name hiding, each hides every
    base member of that name. A using-declaration hides nothing: it re-exposes the base's
    member, which stays in the inherited walk and is reported if unbound."""
    return frozenset(
        OPERATOR_DUNDERS.get(m.spelling, m.spelling)
        for m in record.get_children()
        if m.kind in METHOD_KINDS or m.kind in FIELD_KINDS
    )


def _base_definition(base: ci.Cursor) -> ci.Cursor | None:
    """The class a `CXX_BASE_SPECIFIER` names; a template-instance base through `referenced`."""
    definition = base.type.get_declaration().get_definition()
    return definition if definition is not None else base.referenced


def _overload(c: ci.Cursor, nullary_absent_ok: str | None = None) -> CppOverload:
    """The Python-facing form of one C++ function, method or constructor."""
    params = [p for p in c.get_children() if p.kind == ci.CursorKind.PARM_DECL]
    n_default = sum(_has_default(p) for p in params)
    out = tuple(_out_param_type(p) for p in params if _is_out_param(p))
    pairs = _byte_buffer_pairs(params)
    paired = {pos + k + d for k, pos in enumerate(pairs) for d in (0, 1)}
    n = len(params) - len(pairs)
    arity = Arity(n - n_default, n)
    void = c.result_type.kind == ci.TypeKind.VOID
    absent_ok = nullary_absent_ok if arity.hi == 0 else None
    if any(_is_yaml_node(p) for p in params):
        absent_ok = "yaml-node"
    raw = any(_is_raw_buffer(p) for i, p in enumerate(params) if i not in paired)
    pointers = {pos + k for k, pos in enumerate(pairs)}
    python_params = [(i, p) for i, p in enumerate(params) if i in pointers or i not in paired]
    keys = tuple(
        BYTES_KEYS if i in pointers else audit_types.cpp_keys(p.type) for i, p in python_params
    )
    out_positions = tuple(j for j, (_, p) in enumerate(python_params) if _is_out_param(p))
    return CppOverload(
        arity,
        out,
        location(c),
        void,
        absent_ok=absent_ok,
        raw_buffer=raw,
        byte_buffers=pairs,
        params=keys,
        out_positions=out_positions,
    )


def cpp_api(tu: ci.TranslationUnit, headers: frozenset[Path]) -> dict[str, CppSymbol]:
    """The audited C++ API: public, non-deprecated declarations in `headers`.

    Keys are dotted names within the namespace; operators that a binding maps to a
    dunder are keyed by the dunder, constructors by `__init__`. A class also holds the public
    members it inherits through public bases under `OWNED_ROOTS` (`inherited_from` set).
    """
    symbols: dict[str, CppSymbol] = {}
    seen: set[str] = set()
    owned = _owned_headers(tu)
    # Inherited overloads dedupe per (derived class, USR), never through `seen`: the base's
    # own record must keep its members, and each derived class its inherited copy.
    inherited: set[tuple[str, str]] = set()

    def first_time(c: ci.Cursor) -> bool:
        usr = c.get_usr()
        if usr in seen:
            return False
        seen.add(usr)
        return True

    def add(name: str, kind: Kind, c: ci.Cursor) -> CppSymbol:
        return symbols.setdefault(name, CppSymbol(name, kind, location(c)))

    def add_overload(sym: CppSymbol, ov: CppOverload) -> None:
        if not any(_same_python_form(ov, other) for other in sym.overloads):
            sym.overloads.append(ov)

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
        add_overload(add(name, kind, c), _overload(c, nullary_absent_ok))

    def add_enum(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.ENUM, c).qualified = audit_types.qualified_path(c)
        for e in c.get_children():
            if e.kind == ci.CursorKind.ENUM_CONSTANT_DECL:
                add(f"{name}.{e.spelling}", Kind.ENUMERATOR, e)

    def add_record(prefix: str, c: ci.Cursor) -> None:
        name = prefix + c.spelling
        add(name, Kind.CLASS, c).qualified = audit_types.qualified_path(c)
        # An abstract class cannot be constructed from Python: no __init__ to audit.
        abstract = c.is_abstract_record()
        declares_ctor = False
        befriends_cereal = False
        ctors: list[ci.Cursor] = []
        for m in c.get_children():
            is_ctor = _is_ctor(m, c)
            if is_ctor:
                declares_ctor = True
            if m.kind == ci.CursorKind.FRIEND_DECL:
                befriends_cereal |= any(f.spelling in CEREAL_HOOKS for f in m.get_children())
            if m.kind == ci.CursorKind.CXX_BASE_SPECIFIER and m.referenced is not None:
                # A subclass serialized through its base (`cereal::base_class`) has no friend
                # of its own: `Mesh` and `SDFMesh` through `PolygonMesh`.
                befriends_cereal |= _befriends_cereal(m.referenced)
            if m.access_specifier != ci.AccessSpecifier.PUBLIC:
                continue
            if m.availability == ci.AvailabilityKind.DEPRECATED:
                continue
            if is_ctor:
                if not (
                    abstract
                    or m.is_copy_constructor()
                    or m.is_move_constructor()
                    or m.is_deleted_method()
                ):
                    ctors.append(m)
            elif (member := _member(m, c)) is not None:
                leaf, kind = member
                if kind is Kind.FIELD:
                    add(f"{name}.{leaf}", kind, m)
                else:
                    add_callable(f"{name}.{leaf}", kind, m)
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
        add_inherited(name, c, _declared_leaves(c))

    def add_inherited(name: str, record: ci.Cursor, hidden: frozenset[str]) -> None:
        """Register as `name.leaf` the members `record`'s public bases under an owned root
        declare and `hidden` does not name, then recurse into their bases (A)."""
        for b in record.get_children():
            if b.kind != ci.CursorKind.CXX_BASE_SPECIFIER:
                continue
            if b.access_specifier != ci.AccessSpecifier.PUBLIC:
                continue
            base = _base_definition(b)
            if base is None or base.location.file is None:
                continue
            if Path(base.location.file.name).resolve() not in owned:
                continue
            for m in base.get_children():
                member = _member(m, base)
                if member is None or member[0] in hidden or (name, m.get_usr()) in inherited:
                    continue
                inherited.add((name, m.get_usr()))
                leaf, kind = member
                sym = symbols.setdefault(
                    f"{name}.{leaf}",
                    CppSymbol(
                        f"{name}.{leaf}", kind, location(m), inherited_from=record_path(base)
                    ),
                )
                if kind is not Kind.FIELD:
                    add_overload(sym, _overload(m))
            add_inherited(name, base, hidden | _declared_leaves(base))

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


def own_header_lines(cpp: Mapping[str, CppSymbol]) -> dict[str, list[int]]:
    """Header path → lines of the symbols it declares. An inherited symbol sits at its base's
    member, in another header, so it says nothing about where its class's header starts."""
    lines: dict[str, list[int]] = {}
    for sym in cpp.values():
        if sym.inherited_from is None:
            path, line = sym.location.rsplit(":", 1)
            lines.setdefault(path, []).append(int(line))
    return lines


def decl_names(tu: ci.TranslationUnit) -> frozenset[str]:
    """Spelling of every declaration in the TU: what a Python name may resolve to."""
    return frozenset(c.spelling for c in tu.cursor.walk_preorder() if c.kind.is_declaration())


# Canonical spellings of the nanobind constructions that bind a C++ type to a Python name.
NB_CLASS_TEMPLATES = ("nanobind::class_<", "nanobind::enum_<")
NB_CLASS_CTORS = frozenset({"class_", "enum_"})
NB_MODULE_SCOPE = "nanobind::module_"
# NB_MODULE(name, m) defines `PyInit_<name>`; a TU parsed without bodies only declares it.
NB_MODULE_INIT_PREFIX = "PyInit_"


class SkippedBodiesError(ValueError):
    """`bound_classes` got a TU parsed without function bodies: NB_MODULE's body is empty."""


def _in_main_file(c: ci.Cursor, main: Path) -> bool:
    return c.location.file is not None and Path(c.location.file.name).resolve() == main


def _scope_type(arg: ci.Cursor) -> str:
    """Canonical type of the scope a construction binds into, under nanobind's `handle`
    conversion: `nanobind::module_` for a top-level class, a `class_<…>` for a nested one."""
    for c in arg.walk_preorder():
        spelling = c.type.get_canonical().spelling.removeprefix("const ")
        if spelling.startswith((NB_MODULE_SCOPE, *NB_CLASS_TEMPLATES)):
            return spelling
    return ""


def bound_classes(tu: ci.TranslationUnit) -> dict[str, str]:
    """Top-level Python class name -> qualified C++ class, as the binding's `nb::class_` and
    `nb::enum_` constructions in the TU's main file say (`TOTGCompositeProfile` ->
    `tesseract::time_parameterization::TimeOptimalTrajectoryGenerationCompositeProfile`).

    A template instance maps to its template (`Planner<double>` -> `…::Planner`), as
    `cpp_api` keys it. Nested classes (bound into a `class_`, not the module) are skipped.

    Raises:
        SkippedBodiesError: `tu` was parsed without function bodies (`parse_tu(bodies=True)`).
    """
    main = Path(tu.spelling).resolve()
    out: dict[str, str] = {}
    # Walk only the main file's top-level declarations: the headers' bodies are most of a TU.
    own = (
        c
        for top in tu.cursor.get_children()
        if _in_main_file(top, main)
        for c in top.walk_preorder()
    )
    for c in own:
        if (
            c.kind == ci.CursorKind.FUNCTION_DECL
            and c.spelling.startswith(NB_MODULE_INIT_PREFIX)
            and not c.is_definition()
            and c.get_definition() is None
        ):
            raise SkippedBodiesError(f"{tu.spelling}: parsed without bodies; NB_MODULE is empty")
        if c.kind != ci.CursorKind.CALL_EXPR or c.spelling not in NB_CLASS_CTORS:
            continue
        t = c.type.get_canonical()
        if not t.spelling.startswith(NB_CLASS_TEMPLATES):
            continue
        args = list(c.get_arguments())
        if len(args) < 2 or not _scope_type(args[0]).startswith(NB_MODULE_SCOPE):
            continue
        literal = next(
            (a for a in args[1].walk_preorder() if a.kind == ci.CursorKind.STRING_LITERAL), None
        )
        decl = t.get_template_argument_type(0).get_canonical().get_declaration()
        if literal is not None and decl.kind != ci.CursorKind.NO_DECL_FOUND:
            out[literal.spelling.strip('"')] = audit_types.qualified_path(decl)
    return out
