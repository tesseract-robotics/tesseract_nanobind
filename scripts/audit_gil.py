"""Check every GIL release in the bindings against scripts/nogil_entry_points.yaml (#124).

A binding releases the GIL when a `.def` / `m.def` call passes
`nb::call_guard<nb::gil_scoped_release>()`, or when the function it binds declares an
`nb::gil_scoped_release` in its body, directly or in a function, function template or
local lambda of the same binding TU that it calls. Any other call that passes the
call_guard (e.g. `bind_value_equality(cls, guard)`) counts under the callee's name.

libclang parses each binding TU with function bodies (the API audit skips them) and
the result is compared with the list, in both directions:

- a listed entry point that no longer releases the GIL fails;
- a release on a binding that is not listed fails.

An entry names the Python module, the Python class (`module` for module-level
functions) and the Python method name. It covers every overload of that name that
releases the GIL. Each entry carries the reason the release is needed: the list is
the documentation (docs/developer/threading.md embeds it).

Usage:
    pixi run -e audit audit-gil [MODULE ...]
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import clang.cindex as ci
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ENTRY_POINTS = REPO_ROOT / "scripts" / "nogil_entry_points.yaml"

# The scope of a module-level `m.def`.
MODULE_SCOPE = "module"
RELEASE_TYPE = "nanobind::gil_scoped_release"
GUARD_TYPE = f"nanobind::call_guard<{RELEASE_TYPE}>"
CLASS_PREFIX = "nanobind::class_<"
MODULE_TYPE = "nanobind::module_"
DEF_METHODS = frozenset(
    {"def", "def_static", "def_prop_ro", "def_prop_rw", "def_prop_ro_static", "def_prop_rw_static"}
)
# `nb::self == nb::self` and friends, named as Python names them.
OPERATOR_NAMES = {
    "==": "__eq__",
    "!=": "__ne__",
    "<": "__lt__",
    "<=": "__le__",
    ">": "__gt__",
    ">=": "__ge__",
    "+": "__add__",
    "-": "__sub__",
    "*": "__mul__",
    "/": "__truediv__",
}
# Descending into these would leave the binding: a release inside them is not the binding's.
OPAQUE_KINDS = frozenset({ci.CursorKind.CLASS_DECL, ci.CursorKind.STRUCT_DECL})
FOLLOWED_KINDS = frozenset(
    {
        ci.CursorKind.FUNCTION_DECL,
        ci.CursorKind.FUNCTION_TEMPLATE,
        ci.CursorKind.CXX_METHOD,
        ci.CursorKind.VAR_DECL,
        ci.CursorKind.LAMBDA_EXPR,
    }
)


class ParseError(RuntimeError):
    """libclang reported an error: a partial AST would silently miss releases."""


class EntryPointsError(ValueError):
    """nogil_entry_points.yaml is malformed."""


@dataclass(frozen=True, order=True)
class Release:
    """One bound name that releases the GIL: Python module, class (or `module`) and name."""

    module: str
    scope: str
    name: str

    def __str__(self) -> str:
        return f"{self.module}: {self.scope}.{self.name}"


def parse(cpp: Path, args: Sequence[str], unsaved: str | None = None) -> ci.TranslationUnit:
    """Parse a binding TU with function bodies; raise on any error diagnostic."""
    files = [(str(cpp), unsaved)] if unsaved is not None else None
    tu = ci.Index.create().parse(str(cpp), args=list(args), unsaved_files=files)
    errors = [d for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        shown = "\n".join(str(d) for d in errors[:20])
        raise ParseError(f"{cpp}: {len(errors)} error diagnostics\n{shown}")
    return tu


def _in_main_file(cursor: ci.Cursor, tu: ci.TranslationUnit) -> bool:
    loc = cursor.location
    return loc.file is not None and loc.file.name == tu.spelling


def _canonical(t: ci.Type) -> ci.Type:
    t = t.get_canonical()
    if t.kind in (ci.TypeKind.LVALUEREFERENCE, ci.TypeKind.RVALUEREFERENCE):
        t = t.get_pointee().get_canonical()
    return t


def _strip(expr: ci.Cursor) -> ci.Cursor:
    """Through the implicit casts and temporaries libclang reports as UNEXPOSED_EXPR."""
    while expr.kind == ci.CursorKind.UNEXPOSED_EXPR and len(list(expr.get_children())) == 1:
        expr = next(expr.get_children())
    return expr


def _string_literal(expr: ci.Cursor) -> str | None:
    tokens = list(expr.get_tokens())
    if len(tokens) == 1 and tokens[0].spelling.startswith('"'):
        return tokens[0].spelling[1:-1]
    return None


class Scanner:
    """Finds the GIL releases of one binding TU."""

    def __init__(self, tu: ci.TranslationUnit, module: str) -> None:
        self.tu = tu
        self.module = module
        self._releases_memo: dict[str, bool] = {}
        self._definitions = self._index_definitions()
        self.python_names = self._class_names()

    # --- indexes -------------------------------------------------------------

    def _index_definitions(self) -> dict[str, list[ci.Cursor]]:
        """Main-file function (template) definitions by name, for calls libclang resolves to an
        implicit template instantiation, whose cursor has no body."""
        defs: dict[str, list[ci.Cursor]] = {}
        for c in self.tu.cursor.walk_preorder():
            if (
                c.kind in (ci.CursorKind.FUNCTION_DECL, ci.CursorKind.FUNCTION_TEMPLATE)
                and c.is_definition()
                and _in_main_file(c, self.tu)
            ):
                defs.setdefault(c.spelling, []).append(c)
        return defs

    def _class_names(self) -> dict[str, str]:
        """C++ type spelling -> Python class name, from every `nb::class_<T>(scope, "Name")`."""
        names: dict[str, str] = {}
        for c in self.tu.cursor.walk_preorder():
            if c.kind not in (ci.CursorKind.CALL_EXPR, ci.CursorKind.UNEXPOSED_EXPR):
                continue
            t = _canonical(c.type)
            if not t.spelling.startswith(CLASS_PREFIX) or not _in_main_file(c, self.tu):
                continue
            if c.referenced is None or c.referenced.kind != ci.CursorKind.CONSTRUCTOR:
                continue
            args = list(c.get_arguments())
            if len(args) >= 2 and (name := _string_literal(_strip(args[1]))) is not None:
                names.setdefault(_canonical(t.get_template_argument_type(0)).spelling, name)
        return names

    # --- release detection ---------------------------------------------------

    def _key(self, c: ci.Cursor) -> str:
        loc = c.extent.start
        return f"{loc.file}:{loc.line}:{loc.column}:{c.kind}"

    def releases(self, node: ci.Cursor) -> bool:
        """`node` declares a gil_scoped_release, or reaches one in this TU through a call,
        a function reference or a local lambda variable."""
        key = self._key(node)
        if key in self._releases_memo:
            return self._releases_memo[key]
        self._releases_memo[key] = False  # recursion guard
        found = False
        stack = [node]
        while stack and not found:
            c = stack.pop()
            if c.kind in OPAQUE_KINDS:
                continue  # a local struct's methods are not this binding's body
            if c.kind == ci.CursorKind.VAR_DECL and _canonical(c.type).spelling == RELEASE_TYPE:
                found = True
            elif c.kind in (ci.CursorKind.CALL_EXPR, ci.CursorKind.DECL_REF_EXPR) and any(
                self.releases(t) for t in self._targets(c)
            ):
                found = True
            stack.extend(c.get_children())
        self._releases_memo[key] = found
        return found

    def _targets(self, ref: ci.Cursor) -> Iterable[ci.Cursor]:
        """The main-file definitions a call or reference reaches."""
        target = ref.referenced
        if (
            target is None
            or target.kind not in FOLLOWED_KINDS
            or not _in_main_file(target, self.tu)
        ):
            return ()  # headers (tesseract, nanobind) are not the binding
        if target.kind == ci.CursorKind.VAR_DECL:
            # A local lambda (`auto f = [](...) {...};`): follow its initializer only.
            return (target,) if "lambda" in target.type.spelling else ()
        definition = target.get_definition() or target
        if any(True for _ in definition.get_children()):
            return (definition,)
        # An implicit instantiation of a main-file template: libclang gives no body.
        return tuple(self._definitions.get(target.spelling, ()))

    def _guarded(self, call: ci.Cursor) -> bool:
        return any(GUARD_TYPE in _canonical(a.type).spelling for a in call.get_arguments())

    # --- bound names ---------------------------------------------------------

    def _scope(self, receiver: ci.Cursor) -> str | None:
        t = _canonical(receiver.type)
        if t.spelling == MODULE_TYPE:
            return MODULE_SCOPE
        if t.spelling.startswith(CLASS_PREFIX):
            cpp = _canonical(t.get_template_argument_type(0)).spelling
            return self.python_names.get(cpp, cpp)
        return None

    def _bound_name(self, call: ci.Cursor, args: list[ci.Cursor]) -> str:
        if not args:
            return "<unnamed>"
        first = _strip(args[0])
        if (name := _string_literal(first)) is not None:
            return name
        spelling = _canonical(first.type).spelling
        if "nanobind::detail::init" in spelling:
            return "__init__"
        tokens = [t.spelling for t in first.get_tokens()]
        if "self" in tokens:
            for tok in tokens:
                if tok in OPERATOR_NAMES:
                    return OPERATOR_NAMES[tok]
        return "<unnamed>"

    def scan(self) -> dict[Release, str]:
        """Every release in the TU, with the `file:line` of the first binding that has it."""
        found: dict[Release, str] = {}
        for call in self.tu.cursor.walk_preorder():
            if call.kind != ci.CursorKind.CALL_EXPR or not _in_main_file(call, self.tu):
                continue
            children = list(call.get_children())
            if not children:
                continue
            callee = _strip(children[0])
            args = list(call.get_arguments())
            release: Release | None = None
            if callee.kind == ci.CursorKind.MEMBER_REF_EXPR and call.spelling in DEF_METHODS:
                receivers = list(callee.get_children())
                scope = self._scope(receivers[0]) if receivers else None
                if scope is None:
                    continue
                bound = args[1:] if args and _string_literal(_strip(args[0])) is not None else args
                if self._guarded(call) or any(self.releases(a) for a in bound):
                    release = Release(self.module, scope, self._bound_name(call, args))
            elif self._guarded(call) and args:
                # A binding helper given the guard: bind_value_equality(cls, guard).
                scope = self._scope(_strip(args[0]))
                if scope is not None:
                    release = Release(self.module, scope, call.spelling)
            if release is not None:
                # The `.def` token, not the start of the chain the call is part of.
                loc = (
                    callee.location
                    if callee.kind == ci.CursorKind.MEMBER_REF_EXPR
                    else call.location
                )
                found.setdefault(release, f"{Path(loc.file.name).name}:{loc.line}")
        return found


def load_entry_points(path: Path) -> dict[Release, str]:
    """`{module: {scope: {name: reason}}}` -> `{Release: reason}`."""
    data = yaml.safe_load(path.read_text()) or {}
    out: dict[Release, str] = {}
    if not isinstance(data, Mapping):
        raise EntryPointsError(f"{path}: expected a mapping of modules")
    for module, scopes in data.items():
        if not isinstance(scopes, Mapping):
            raise EntryPointsError(f"{path}: {module}: expected a mapping of classes")
        for scope, names in scopes.items():
            if not isinstance(names, Mapping):
                raise EntryPointsError(f"{path}: {module}.{scope}: expected a mapping of names")
            for name, reason in names.items():
                if not isinstance(reason, str) or not reason.strip():
                    raise EntryPointsError(f"{path}: {module}: {scope}.{name}: needs a reason")
                out[Release(str(module), str(scope), str(name))] = reason
    return out


def compare(
    found: Mapping[Release, str], listed: Mapping[Release, str], modules: Iterable[str]
) -> list[str]:
    """Problems, one line each; empty when the bindings and the list agree."""
    scope = set(modules)
    problems = [
        f"{where}: {r} releases the GIL but is not in {ENTRY_POINTS.name}"
        for r, where in sorted(found.items())
        if r not in listed
    ]
    problems += [
        f"{r} is in {ENTRY_POINTS.name} but no longer releases the GIL"
        for r in sorted(listed)
        if r.module in scope and r not in found
    ]
    return problems


def main(argv: list[str] | None = None) -> int:
    import audit_bindings  # needs the audit env (CONDA_PREFIX); not at import, for the tests
    from audit_ownership import binding_modules, binding_source

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("modules", nargs="*", help="Default: every binding module.")
    opts = parser.parse_args(argv)
    modules = opts.modules or binding_modules()
    args = [
        *audit_bindings.PARSE_ARGS,
        *(f"-I{d}" for d in audit_bindings.INCLUDE_DIRS),
    ]
    found: dict[Release, str] = {}
    for module in modules:
        found.update(Scanner(parse(binding_source(module), args), module).scan())
    problems = compare(found, load_entry_points(ENTRY_POINTS), modules)
    for line in problems:
        print(line, file=sys.stderr)
    if not problems:
        print(f"{len(found)} GIL releases, all listed in {ENTRY_POINTS.name}")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
