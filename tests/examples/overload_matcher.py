"""Which nanobind overload a Python call dispatches to, decided conservatively.

nanobind tries a function's overloads in `__nb_signature__` order, twice: a
first pass with implicit conversions disabled, then a second pass with them
enabled. The first overload whose arguments all load wins. This module mirrors
that from the signature text and the argument values, with one deliberate
difference: a call is matched to an overload only when exactly one overload
provably accepts it in the deciding pass and every other overload provably
rejects it. Anything less (two acceptors, or a parameter type whose caster
this module does not model) is ambiguous, never a guess, so it can never
produce false coverage.

The caster rules below are read from nanobind's sources (2.12):
`load_f64` / `load_int` (src/common.cpp), the `bool` caster (nb_cast.h),
`seq_get` (src/common.cpp), and the dict / set / pair / tuple / path /
function casters (include/nanobind/stl/).
"""

from __future__ import annotations

import datetime
import enum
import functools
import importlib
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

# The prefix of a fully qualified annotation that names a bound tesseract type.
BOUND_TYPE_PREFIX = "tesseract_robotics."
# Bound types that accept other types in nanobind's converting pass, registered with
# `nb::implicitly_convertible<Source, Target>()`; a contract test reads the binding
# sources and pins this set. Every other bound type takes only its own instances.
IMPLICIT_CONVERSION_TARGETS = frozenset({"InstructionPoly"})
# Integers outside this range may not fit the C++ parameter (int32 vs int64);
# the signature text does not say which, so such a value is not decided.
INT32_MIN, INT32_MAX = -(2**31), 2**31 - 1
# `ndarray[...]` spellings: the memory-order flags and the free-extent marker.
NDARRAY_ORDER_C, NDARRAY_ORDER_F = "C", "F"
NDARRAY_ANY_EXTENT = "*"
# What `tuple(x)` raises for a non-iterable or broken sequence; `seq_get` treats them as "no".
SEQUENCE_TUPLE_ERRORS = (TypeError, ValueError, IndexError, KeyError, AttributeError)


class SignatureParseError(Exception):
    """A `__nb_signature__` entry does not have the `def name(params) -> ret` shape."""


class Verdict(enum.Enum):
    """Whether an argument (or a whole call) loads into a parameter (or an overload)."""

    YES = "yes"
    NO = "no"
    UNKNOWN = "unknown"


class ParamKind(enum.Enum):
    POSITIONAL_ONLY = "positional_only"
    POSITIONAL_OR_KEYWORD = "positional_or_keyword"
    KEYWORD_ONLY = "keyword_only"
    VAR_POSITIONAL = "var_positional"
    VAR_KEYWORD = "var_keyword"


@dataclass(frozen=True)
class Param:
    """One parameter of a signature; `annotation` is None for an unannotated `self`."""

    name: str
    annotation: str | None
    has_default: bool
    kind: ParamKind


@dataclass(frozen=True)
class Match:
    """The outcome of matching one call against a function's overloads.

    Attributes:
        index: The overload the call dispatches to, or None if not decided.
        candidates: Overloads that accept, or may accept, the call in the
            deciding pass; empty when no overload accepts it at all.
        fully_bound: The call passed an argument for every parameter of
            `index` (`self` and variadic parameters excluded).
    """

    index: int | None
    candidates: tuple[int, ...]
    fully_bound: bool

    @property
    def ambiguous(self) -> bool:
        return self.index is None and bool(self.candidates)


def _combine_all(verdicts: list[Verdict]) -> Verdict:
    if Verdict.NO in verdicts:
        return Verdict.NO
    return Verdict.UNKNOWN if Verdict.UNKNOWN in verdicts else Verdict.YES


def _combine_any(verdicts: list[Verdict]) -> Verdict:
    if Verdict.YES in verdicts:
        return Verdict.YES
    return Verdict.UNKNOWN if Verdict.UNKNOWN in verdicts else Verdict.NO


def split_top_level(text: str, sep: str) -> list[str]:
    """Split `text` at `sep` occurrences outside brackets and parentheses."""
    parts, depth, start = [], 0, 0
    i = 0
    while i < len(text):
        ch = text[i]
        if ch in "[(":
            depth += 1
        elif ch in "])":
            depth -= 1
        elif depth == 0 and text.startswith(sep, i):
            parts.append(text[start:i].strip())
            start = i + len(sep)
            i = start
            continue
        i += 1
    tail = text[start:].strip()
    if tail:
        parts.append(tail)
    return parts


@functools.cache
def parse_signature(text: str) -> tuple[Param, ...]:
    """Parameters of one `__nb_signature__` text.

    Raises:
        SignatureParseError: `text` is not `def name(params) -> ret`.
    """
    if not text.startswith("def ") or "(" not in text:
        raise SignatureParseError(f"not a nanobind signature: {text!r}")
    open_at = text.index("(")
    depth = 0
    for close_at in range(open_at, len(text)):
        depth += text[close_at] in "[("
        depth -= text[close_at] in "])"
        if depth == 0:
            break
    else:
        raise SignatureParseError(f"unbalanced parameter list: {text!r}")

    params: list[Param] = []
    kind = ParamKind.POSITIONAL_OR_KEYWORD
    for part in split_top_level(text[open_at + 1 : close_at], ","):
        if part == "/":
            params = [
                Param(p.name, p.annotation, p.has_default, ParamKind.POSITIONAL_ONLY)
                for p in params
            ]
            continue
        if part == "*":
            kind = ParamKind.KEYWORD_ONLY
            continue
        decl, has_default = part, False
        pieces = split_top_level(part, " = ")
        if len(pieces) == 2:
            decl, has_default = pieces[0], True
        name, _, annotation = (s.strip() for s in decl.partition(":"))
        if name.startswith("**"):
            params.append(Param(name[2:], annotation or None, False, ParamKind.VAR_KEYWORD))
        elif name.startswith("*"):
            params.append(Param(name[1:], annotation or None, False, ParamKind.VAR_POSITIONAL))
            kind = ParamKind.KEYWORD_ONLY
        else:
            params.append(Param(name, annotation or None, has_default, kind))
    return tuple(params)


@functools.cache
def _resolve(dotted: str) -> Any:
    """The object a fully qualified name refers to, or None if it does not import."""
    parts = dotted.split(".")
    for split in range(len(parts) - 1, 0, -1):
        try:
            obj: Any = importlib.import_module(".".join(parts[:split]))
        except ImportError:
            continue
        for attr in parts[split:]:
            obj = getattr(obj, attr, None)
            if obj is None:
                return None
        return obj
    return None


def _check_float(value: object, convert: bool) -> Verdict:
    if type(value) is float:
        return Verdict.YES
    if not convert:
        return Verdict.NO
    has_number = hasattr(type(value), "__float__") or hasattr(type(value), "__index__")
    return Verdict.YES if has_number and not isinstance(value, (str, bytes)) else Verdict.NO


def _check_int(value: Any, convert: bool) -> Verdict:
    if type(value) is int:
        return Verdict.YES if INT32_MIN <= value <= INT32_MAX else Verdict.UNKNOWN
    if not convert or isinstance(value, float):
        return Verdict.NO
    if isinstance(value, (str, bytes)):
        # PyNumber_Long parses a numeric string exactly as int() does.
        try:
            return _check_int(int(value), convert=False)
        except ValueError:
            return Verdict.NO
    has_int = hasattr(type(value), "__index__") or hasattr(type(value), "__int__")
    return Verdict.YES if has_int else Verdict.NO


def _ndarray_spec(args: str) -> tuple[str | None, list[str] | None, str | None]:
    dtype = shape = order = None
    for item in split_top_level(args, ","):
        key, _, val = item.partition("=")
        if key == "dtype":
            dtype = val
        elif key == "shape":
            shape = [s.strip() for s in val.strip("()").split(",") if s.strip()]
        elif key == "order":
            order = val.strip("'\"")
    return dtype, shape, order


def _shape_fits(arr: np.ndarray, shape: list[str] | None) -> bool:
    if shape is None:
        return True
    if arr.ndim != len(shape):
        return False
    return all(s == NDARRAY_ANY_EXTENT or int(s) == n for s, n in zip(shape, arr.shape))


def _check_ndarray(args: str, value: object, convert: bool) -> Verdict:
    dtype, shape, order = _ndarray_spec(args)
    if not convert:
        if not isinstance(value, np.ndarray):
            return Verdict.NO
        if dtype is not None and value.dtype != np.dtype(dtype):
            return Verdict.NO
        if order == NDARRAY_ORDER_C and not value.flags.c_contiguous:
            return Verdict.NO
        if order == NDARRAY_ORDER_F and not value.flags.f_contiguous:
            return Verdict.NO
        return Verdict.YES if _shape_fits(value, shape) else Verdict.NO
    try:
        arr = np.asarray(value, dtype=dtype)
    except (TypeError, ValueError):
        return Verdict.NO
    return Verdict.YES if _shape_fits(arr, shape) else Verdict.NO


def _sequence_items(value: Any) -> list[Any] | None:
    """Items as `seq_get` sees them, or None where it fails.

    Exact str / bytes are refused; a tuple or list is read directly; anything
    else that `PySequence_Check` accepts (not a dict, has `__getitem__`, e.g. a
    bound `VectorVector3d` or an ndarray) goes through `PySequence_Tuple`.
    """
    if type(value) in (str, bytes):
        return None
    if isinstance(value, (list, tuple)):
        return list(value)
    if isinstance(value, dict) or not hasattr(type(value), "__getitem__"):
        return None
    try:
        return list(tuple(value))
    except SEQUENCE_TUPLE_ERRORS:
        return None  # seq_get clears the error and rejects the argument


def _check_bound_type(dotted: str, value: object, convert: bool) -> Verdict:
    cls = _resolve(dotted)
    if not isinstance(cls, type):
        return Verdict.UNKNOWN
    if isinstance(value, cls):
        return Verdict.YES
    if not convert or value is None:
        return Verdict.NO
    if issubclass(cls, enum.Enum):
        # nanobind enums may accept an integer when converting.
        return Verdict.UNKNOWN if isinstance(value, int) else Verdict.NO
    # Which source types a target converts from is not visible from Python.
    return Verdict.UNKNOWN if cls.__name__ in IMPLICIT_CONVERSION_TARGETS else Verdict.NO


def check(annotation: str, value: Any, convert: bool) -> Verdict:
    """Whether `value` loads into a parameter annotated `annotation` in one pass.

    Args:
        annotation: Fully qualified annotation text from `__nb_signature__`.
        value: The argument.
        convert: False for nanobind's first pass, True for the second.

    Returns:
        YES or NO when the caster's behaviour is modelled; UNKNOWN otherwise.
    """
    alternatives = split_top_level(annotation, "|")
    if len(alternatives) > 1:
        return _combine_any([check(a, value, convert) for a in alternatives])
    ann = alternatives[0]
    head, _, rest = ann.partition("[")
    args = rest[:-1] if rest.endswith("]") else ""
    if head.startswith(BOUND_TYPE_PREFIX):
        return _check_bound_type(head, value, convert)
    name = head.rsplit(".", 1)[-1]

    if name == "None":
        return Verdict.YES if value is None else Verdict.NO
    if name in ("object", "Any"):
        return Verdict.YES
    if name == "float":
        return _check_float(value, convert)
    if name == "int":
        return _check_int(value, convert)
    if name == "bool":
        return Verdict.YES if value is True or value is False else Verdict.NO
    if name == "str":
        return Verdict.YES if isinstance(value, str) else Verdict.NO
    if name == "bytes":
        return Verdict.YES if isinstance(value, bytes) else Verdict.NO
    if name == "PathLike":
        # PyOS_FSPath: str, bytes or os.PathLike, in either pass.
        return (
            Verdict.YES
            if isinstance(value, (str, bytes)) or hasattr(value, "__fspath__")
            else Verdict.NO
        )
    if name == "timedelta":
        return Verdict.YES if isinstance(value, datetime.timedelta) else Verdict.UNKNOWN
    if name == "Callable":
        if value is None:
            return Verdict.YES if convert else Verdict.NO
        return Verdict.YES if callable(value) else Verdict.NO
    if name == "ndarray":
        return _check_ndarray(args, value, convert)
    if name == "Sequence":
        items = _sequence_items(value)
        if items is None:
            return Verdict.NO
        return _combine_all([check(args, item, convert) for item in items])
    if name == "tuple":
        types = split_top_level(args, ",")
        items = _sequence_items(value)
        if items is None or len(items) != len(types):
            return Verdict.NO
        return _combine_all([check(t, item, convert) for t, item in zip(types, items)])
    if name == "Mapping":
        if not isinstance(value, Mapping) and not hasattr(value, "items"):
            return Verdict.NO
        key_ann, val_ann = split_top_level(args, ",")
        verdicts = []
        for k, v in value.items():
            verdicts += [check(key_ann, k, convert), check(val_ann, v, convert)]
        return _combine_all(verdicts)
    if name == "Set":
        try:
            iterator = iter(value)
        except TypeError:
            return Verdict.NO
        if iterator is value:
            return (
                Verdict.UNKNOWN
            )  # a one-shot iterator: reading it would consume the real argument
        items = list(iterator)
        return _combine_all([check(args, item, convert) for item in items])
    return Verdict.UNKNOWN


def _bind(
    params: tuple[Param, ...], args: tuple[object, ...], kwargs: Mapping[str, object]
) -> dict[str, object] | None:
    """Parameter name → argument, as nanobind binds them; None if the call does not fit."""
    bound: dict[str, object] = {}
    positional = [
        p for p in params if p.kind in (ParamKind.POSITIONAL_ONLY, ParamKind.POSITIONAL_OR_KEYWORD)
    ]
    has_var_pos = any(p.kind is ParamKind.VAR_POSITIONAL for p in params)
    has_var_kw = any(p.kind is ParamKind.VAR_KEYWORD for p in params)
    if len(args) > len(positional) and not has_var_pos:
        return None
    for p, a in zip(positional, args):
        bound[p.name] = a
    by_name = {
        p.name: p
        for p in params
        if p.kind in (ParamKind.POSITIONAL_OR_KEYWORD, ParamKind.KEYWORD_ONLY)
    }
    for key, val in kwargs.items():
        if key in bound:
            return None
        if key not in by_name:
            if has_var_kw:
                continue
            return None
        bound[key] = val
    for p in params:
        if p.kind in (ParamKind.VAR_POSITIONAL, ParamKind.VAR_KEYWORD):
            continue
        if p.name not in bound and not p.has_default:
            return None
    return bound


def _call_verdict(
    params: tuple[Param, ...], args: tuple[object, ...], kwargs: Mapping[str, object], convert: bool
) -> Verdict:
    bound = _bind(params, args, kwargs)
    if bound is None:
        return Verdict.NO
    has_var = any(p.kind in (ParamKind.VAR_POSITIONAL, ParamKind.VAR_KEYWORD) for p in params)
    named = {p.name for p in params}
    extra = len(args) > len(
        [
            p
            for p in params
            if p.kind in (ParamKind.POSITIONAL_ONLY, ParamKind.POSITIONAL_OR_KEYWORD)
        ]
    )
    if has_var and (extra or any(k not in named for k in kwargs)):
        return Verdict.UNKNOWN  # variadic arguments are not modelled
    verdicts = []
    for p in params:
        if p.name not in bound or p.annotation is None:
            continue  # defaulted, or the unannotated `self`
        verdicts.append(check(p.annotation, bound[p.name], convert))
    return _combine_all(verdicts)


def _fully_bound(
    params: tuple[Param, ...], args: tuple[object, ...], kwargs: Mapping[str, object]
) -> bool:
    bound = _bind(params, args, kwargs) or {}
    return all(
        p.name in bound
        for p in params
        if p.annotation is not None
        and p.kind not in (ParamKind.VAR_POSITIONAL, ParamKind.VAR_KEYWORD)
    )


def match_call(
    signatures: Sequence[str], args: tuple[object, ...], kwargs: Mapping[str, object]
) -> Match:
    """Match one call against a function's overloads.

    Args:
        signatures: The signature texts, in `__nb_signature__` (dispatch) order.
        args: Positional arguments, `self` first for a method.
        kwargs: Keyword arguments.

    Returns:
        The match. Unique acceptance in the deciding pass gives `index`;
        several acceptors (or any undecided overload) give `candidates` only.

    Raises:
        SignatureParseError: a signature text does not parse.
    """
    params = [parse_signature(s) for s in signatures]
    for convert in (False, True):
        verdicts = [_call_verdict(p, args, kwargs, convert) for p in params]
        if all(v is Verdict.NO for v in verdicts):
            continue
        candidates = tuple(i for i, v in enumerate(verdicts) if v is not Verdict.NO)
        accepted = [i for i, v in enumerate(verdicts) if v is Verdict.YES]
        if len(candidates) == 1 and accepted:
            index = accepted[0]
            return Match(index, candidates, _fully_bound(params[index], args, kwargs))
        return Match(None, candidates, False)
    return Match(None, (), False)
