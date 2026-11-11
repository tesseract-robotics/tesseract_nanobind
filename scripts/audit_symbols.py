"""The C++ API of a translation unit as plain data: symbols, overloads, and their JSON form.

`audit_parse.extract_api` reads these from libclang; `audit_facts.api_facts` stores them in
the fact cache. A stored symbol must not depend on what other modules bind, so the one rule
that does (`unbound-dependency-type`, R13) is kept unresolved as a `Dependency` and applied
after the fetch by `resolve_bound`.

Needs no libclang and imports no audit script at runtime.
"""

from __future__ import annotations

import dataclasses
import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from audit_types import TypeKey

UNBOUND_DEPENDENCY = "unbound-dependency-type"


class SymbolShapeError(RuntimeError):
    """A stored API is valid JSON but not what `encode_symbols` writes."""


class Kind(StrEnum):
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
class Dependency:
    """The types with no Python form that a signature or field names (R13), unresolved.

    Attributes:
        always: One of them is a template instance or has no qualified name: no binding
            gives it a Python form.
        unless_bound: Qualified classes; the dependency holds unless some module binds
            every one of them.
    """

    always: bool = False
    unless_bound: frozenset[str] = frozenset()

    def holds(self, is_bound: Callable[[str], bool]) -> bool:
        """Whether a named type has no Python form. `is_bound` is not called when `always`
        or when no class is named: its first call reads every binding's bound classes."""
        return self.always or any(not is_bound(q) for q in sorted(self.unless_bound))

    def __or__(self, other: Dependency) -> Dependency:
        return Dependency(self.always or other.always, self.unless_bound | other.unless_bound)


NO_DEPENDENCY = Dependency()


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
    # takes a `std::vector<std::reference_wrapper<T>>`: a view a sibling overload builds (R4)
    reference_wrapper_view: bool = False
    yaml_position: int | None = None  # index into `params` of the first `YAML::Node` parameter
    names: tuple[str, ...] = ()  # C++ parameter names, parallel to `params`
    # Unresolved R13: when it holds, `absent_ok` is `UNBOUND_DEPENDENCY` (`resolve_bound`);
    # `absent_ok` above is the rule that applies when it does not.
    dependency: Dependency = NO_DEPENDENCY

    def returned_out_params(self) -> tuple[frozenset[TypeKey] | None, ...]:
        """`params` of the Python form that returns the out-params."""
        return tuple(k for i, k in enumerate(self.params) if i not in self.out_positions)

    def returned_out_names(self) -> tuple[str, ...]:
        """`names` of the Python form that returns the out-params."""
        return tuple(n for i, n in enumerate(self.names) if i not in self.out_positions)


@dataclass
class CppSymbol:
    name: str
    kind: Kind
    location: str
    overloads: list[CppOverload] = field(default_factory=list)
    qualified: str = ""  # `audit_types.qualified_path`, for Kind.CLASS and Kind.ENUM
    inherited_from: str | None = None  # `record_path` of the base declaring an inherited member
    # ACCEPTED rule under which leaving the whole symbol unbound is accepted, for symbols
    # without overloads (fields); read through `WHOLE_SYMBOL_ABSENT_OK`
    absent_ok: str | None = None
    dependency: Dependency = NO_DEPENDENCY  # unresolved R13 of a field's type, as on an overload


def _resolved(
    absent_ok: str | None, dep: Dependency, is_bound: Callable[[str], bool]
) -> str | None:
    return UNBOUND_DEPENDENCY if dep != NO_DEPENDENCY and dep.holds(is_bound) else absent_ok


def resolve_bound(
    symbols: Mapping[str, CppSymbol], is_bound: Callable[[str], bool]
) -> dict[str, CppSymbol]:
    """`symbols` with every `Dependency` decided: the cross-module half of `cpp_api`.

    Args:
        symbols: An extracted API (`audit_parse.extract_api`), in its order.
        is_bound: Whether some module binds a qualified class.

    Returns:
        New symbols, same order, `absent_ok` final and no dependency left.
    """
    out: dict[str, CppSymbol] = {}
    for name, sym in symbols.items():
        out[name] = dataclasses.replace(
            sym,
            overloads=[
                dataclasses.replace(
                    ov,
                    absent_ok=_resolved(ov.absent_ok, ov.dependency, is_bound),
                    dependency=NO_DEPENDENCY,
                )
                for ov in sym.overloads
            ],
            absent_ok=_resolved(sym.absent_ok, sym.dependency, is_bound),
            dependency=NO_DEPENDENCY,
        )
    return out


def _encode_key(key: object) -> object:
    """A `TypeKey` (nested tuples of `str` and `None`) as nested lists."""
    return [_encode_key(k) for k in key] if isinstance(key, tuple) else key


def _decode_key(value: object) -> object:
    return tuple(_decode_key(v) for v in value) if isinstance(value, list) else value


def _encode_keys(keys: frozenset[TypeKey] | None) -> object:
    if keys is None:
        return None
    return sorted((_encode_key(k) for k in keys), key=json.dumps)


def _encode_dependency(dep: Dependency) -> object:
    return [dep.always, sorted(dep.unless_bound)]


def _decode_dependency(value: object) -> Dependency:
    always, unless = value  # type: ignore[misc]
    return Dependency(bool(always), frozenset(unless))


def encode_symbols(symbols: Mapping[str, CppSymbol]) -> list[object]:
    """`symbols` as JSON data. A list, not an object: the cache writes objects with sorted
    keys, and the order of `symbols` is the order `match` reads them in."""
    return [
        {
            "name": s.name,
            "kind": s.kind.value,
            "location": s.location,
            "qualified": s.qualified,
            "inherited_from": s.inherited_from,
            "absent_ok": s.absent_ok,
            "dependency": _encode_dependency(s.dependency),
            "overloads": [
                {
                    "arity": [ov.arity.lo, ov.arity.hi],
                    "out_params": list(ov.out_params),
                    "location": ov.location,
                    "returns_void": ov.returns_void,
                    "mapped": ov.mapped,
                    "absent_ok": ov.absent_ok,
                    "raw_buffer": ov.raw_buffer,
                    "byte_buffers": list(ov.byte_buffers),
                    "params": [_encode_keys(p) for p in ov.params],
                    "out_positions": list(ov.out_positions),
                    "reference_wrapper_view": ov.reference_wrapper_view,
                    "yaml_position": ov.yaml_position,
                    "names": list(ov.names),
                    "dependency": _encode_dependency(ov.dependency),
                }
                for ov in s.overloads
            ],
        }
        for s in symbols.values()
    ]


def _decode_overload(o: Mapping[str, object]) -> CppOverload:
    lo, hi = o["arity"]  # type: ignore[misc]
    return CppOverload(
        arity=Arity(lo, hi),
        out_params=tuple(o["out_params"]),  # type: ignore[arg-type]
        location=o["location"],  # type: ignore[arg-type]
        returns_void=o["returns_void"],  # type: ignore[arg-type]
        mapped=o["mapped"],  # type: ignore[arg-type]
        absent_ok=o["absent_ok"],  # type: ignore[arg-type]
        raw_buffer=o["raw_buffer"],  # type: ignore[arg-type]
        byte_buffers=tuple(o["byte_buffers"]),  # type: ignore[arg-type]
        params=tuple(
            None if p is None else frozenset(_decode_key(k) for k in p)  # type: ignore[misc]
            for p in o["params"]  # type: ignore[attr-defined]
        ),
        out_positions=tuple(o["out_positions"]),  # type: ignore[arg-type]
        reference_wrapper_view=o["reference_wrapper_view"],  # type: ignore[arg-type]
        yaml_position=o["yaml_position"],  # type: ignore[arg-type]
        names=tuple(o["names"]),  # type: ignore[arg-type]
        dependency=_decode_dependency(o["dependency"]),
    )


def decode_symbols(value: object) -> dict[str, CppSymbol]:
    """The symbols `encode_symbols` wrote, in their order.

    Raises:
        SymbolShapeError: `value` is not what `encode_symbols` writes.
    """
    try:
        out: dict[str, CppSymbol] = {}
        for s in value:  # type: ignore[attr-defined]
            out[s["name"]] = CppSymbol(
                name=s["name"],
                kind=Kind(s["kind"]),
                location=s["location"],
                overloads=[_decode_overload(o) for o in s["overloads"]],
                qualified=s["qualified"],
                inherited_from=s["inherited_from"],
                absent_ok=s["absent_ok"],
                dependency=_decode_dependency(s["dependency"]),
            )
        return out
    except (KeyError, TypeError, ValueError) as e:
        raise SymbolShapeError(f"not a stored API: {e!r}") from e
