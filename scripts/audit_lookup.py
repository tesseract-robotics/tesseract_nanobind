"""Find a C++ symbol's Python form: stub members through base classes, classes bound in
another extension, and classes a binding renames."""

from __future__ import annotations

import functools
from collections.abc import Callable, Mapping, Sequence
from dataclasses import replace
from pathlib import Path
from types import MappingProxyType

from audit_ownership import SRC
from audit_parse import CppSymbol, Kind
from audit_stub import PyApi, PySymbol, load_stub, py_api, rel
from audit_types import TypeKey


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


def _stub_base(expr: str, module: str) -> tuple[str, str]:
    """(module, class) a stub base expression names: `pkg.mod._mod.Outer.Inner` ->
    (`mod`, `Outer.Inner`); a bare name is a class of `module`."""
    parts = expr.split(".")
    private = [i for i, p in enumerate(parts) if p.startswith("_")]
    if not private:
        return module, expr
    i = private[-1]
    return parts[i].removeprefix("_"), ".".join(parts[i + 1 :])
