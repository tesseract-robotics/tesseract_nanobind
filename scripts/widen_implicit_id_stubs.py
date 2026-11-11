"""Widen id parameters in generated stubs to the types nanobind implicitly converts.

The bindings declare `str -> LinkId`, `str -> JointId` and `tuple[str, str] -> LinkIdPair`
implicitly convertible (src/bindings/common.cpp), so `Link("base_link")` works at
runtime. nanobind's stubgen does not record implicit conversions and annotates the
parameter as `LinkId`, which makes every type checker reject the call. This rewrites
parameter annotations only; return types stay exact (a getter really returns a `LinkId`).

Usage:
    python scripts/widen_implicit_id_stubs.py STUB.pyi [STUB.pyi ...]
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

_QUAL = r"(?:tesseract_robotics\.common\._common\.)?"
# An id type name not already widened (`LinkId | str`) and not part of a longer name.
# Mapping/dict keys are left to `_widen_mappings`: keys are invariant, so
# `Mapping[LinkId | str, X]` would reject both a `dict[str, X]` (what the bindings hand out,
# src/tesseract_nb.h name_keyed_dict_caster) and a `dict[LinkId, X]`; a parameter becomes
# `Mapping[LinkId, X] | Mapping[str, X]`.
_NOT_KEY = r"(?<!Mapping\[)(?<!dict\[)"
_ID = re.compile(rf"{_NOT_KEY}(?<![\w.])({_QUAL}(?:LinkId|JointId))(?![\w]| \| str)")
_PAIR = re.compile(rf"{_NOT_KEY}(?<![\w.])({_QUAL}LinkIdPair)(?![\w]| \| tuple)")
# `Mapping[<id>, ` opening a parameter's id-keyed mapping, with the str form of that key.
_MAPPING = re.compile(rf"Mapping\[({_QUAL}(LinkIdPair|LinkId|JointId)), ")
_KEY_AS_STR = {"LinkIdPair": "tuple[str, str]", "LinkId": "str", "JointId": "str"}
_DEF = re.compile(r"^\s*def \w+\(")


def _param_span(text: str, start: int) -> int | None:
    """Index of the parenthesis closing the parameter list that opens at `start`, or None
    if `text` ends first (the `def` continues on the next line)."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] in "([":
            depth += 1
        elif text[i] in ")]":
            depth -= 1
            if depth == 0:
                return i
    return None


def _widen_mappings(params: str) -> str:
    """`Mapping[<id>, V]` → `Mapping[<id>, V] | Mapping[<str key>, V]`, inner mappings first.

    A mapping already followed by its str arm is left alone, so widening is idempotent.
    """
    out, pos = [], 0
    for m in _MAPPING.finditer(params):
        if m.start() < pos:
            continue  # inside a mapping already rewritten
        value_start = m.end()
        close = _param_span("[" + params[value_start:], 0)
        if close is None:
            raise ValueError(f"unbalanced Mapping value: {params!r}")
        value_end = value_start + close - 1
        value = _widen_mappings(params[value_start:value_end])
        key_as_str = _KEY_AS_STR[m.group(2)]
        whole = f"Mapping[{m.group(1)}, {value}]"
        tail = params[value_end + 1 :]
        out.append(params[pos : m.start()])
        if tail.startswith(f" | Mapping[{key_as_str}, "):
            out.append(whole)
        else:
            out.append(f"{whole} | Mapping[{key_as_str}, {value}]")
        pos = value_end + 1
    out.append(params[pos:])
    return "".join(out)


def widen_def(text: str) -> str:
    """Widen the id annotations inside one `def`'s parameter list (one or more lines)."""
    open_paren = _DEF.match(text).end() - 1
    close_paren = _param_span(text, open_paren)
    if close_paren is None:
        raise ValueError(f"unbalanced parameter list: {text!r}")
    params = _widen_mappings(text[open_paren:close_paren])
    params = _PAIR.sub(r"\1 | tuple[str, str]", params)
    params = _ID.sub(r"\1 | str", params)
    return text[:open_paren] + params + text[close_paren:]


def widen(text: str) -> str:
    """Widen every parameter annotation in a stub."""
    out = []
    pending = ""  # a `def` whose parameter list has not closed yet
    for line in text.splitlines(keepends=True):
        if pending:
            pending += line
        elif _DEF.match(line):
            pending = line
        else:
            out.append(line)
            continue
        if _param_span(pending, _DEF.match(pending).end() - 1) is None:
            continue
        out.append(widen_def(pending))
        pending = ""
    if pending:
        raise ValueError(f"unbalanced parameter list at end of stub: {pending!r}")
    return "".join(out)


def main(paths: list[str]) -> None:
    for path in map(Path, paths):
        text = path.read_text()
        widened = widen(text)
        if widened != text:
            path.write_text(widened)


if __name__ == "__main__":
    main(sys.argv[1:])
