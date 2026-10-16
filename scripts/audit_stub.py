"""`ast`: read a committed stub and a package `__init__.py` into the Python API the audit compares."""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from pathlib import Path

import audit_types
from audit_ownership import REPO_ROOT
from audit_parse import OPERATOR_DUNDERS, Arity, Kind
from audit_types import TypeKey


class StubMissingError(FileNotFoundError):
    """The module has no committed stub."""


def load_stub(path: Path) -> ast.Module:
    """Parse a committed stub.

    Raises:
        StubMissingError: `path` does not exist.
    """
    if not path.is_file():
        raise StubMissingError(f"no committed stub {path}; run `pixi run stubs`")
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


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


@dataclass(frozen=True, order=True)
class Accepted:
    symbol: str
    rule: str
    location: str


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
