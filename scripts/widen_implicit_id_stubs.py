"""Widen id parameters in generated stubs to the types nanobind implicitly converts.

The bindings declare `str -> LinkId`, `str -> JointId` and `tuple[str, str] -> LinkIdPair`
implicitly convertible (tesseract_common_bindings.cpp), so `Link("base_link")` works at
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

_QUAL = r"(?:tesseract_robotics\.tesseract_common\._tesseract_common\.)?"
# An id type name not already widened (`LinkId | str`) and not part of a longer name.
# Mapping/dict keys stay exact: keys are invariant, so `Mapping[LinkId | str, X]` would
# reject the `dict[LinkId, X]` the bindings hand out (SceneState.link_transforms).
_NOT_KEY = r"(?<!Mapping\[)(?<!dict\[)"
_ID = re.compile(rf"{_NOT_KEY}(?<![\w.])({_QUAL}(?:LinkId|JointId))(?![\w]| \| str)")
_PAIR = re.compile(rf"{_NOT_KEY}(?<![\w.])({_QUAL}LinkIdPair)(?![\w]| \| tuple)")
# The id classes' own dunders (`__eq__(self, arg: LinkId)`) already have a `str` overload.
_ID_CLASSES = re.compile(r"^class (LinkId|JointId|LinkIdPair)\b")
_CLASS = re.compile(r"^class \w+")
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


def widen_def(text: str) -> str:
    """Widen the id annotations inside one `def`'s parameter list (one or more lines)."""
    open_paren = _DEF.match(text).end() - 1
    close_paren = _param_span(text, open_paren)
    if close_paren is None:
        raise ValueError(f"unbalanced parameter list: {text!r}")
    params = text[open_paren:close_paren]
    params = _PAIR.sub(r"\1 | tuple[str, str]", params)
    params = _ID.sub(r"\1 | str", params)
    return text[:open_paren] + params + text[close_paren:]


def widen(text: str) -> str:
    """Widen every parameter annotation in a stub, except inside the id classes."""
    out = []
    in_id_class = False
    pending = ""  # a `def` whose parameter list has not closed yet
    for line in text.splitlines(keepends=True):
        if pending:
            pending += line
        elif _DEF.match(line):
            pending = line
        else:
            if _CLASS.match(line):
                in_id_class = _ID_CLASSES.match(line) is not None
            out.append(line)
            continue
        if pending.startswith("def "):
            in_id_class = False
        if _param_span(pending, _DEF.match(pending).end() - 1) is None:
            continue
        out.append(pending if in_id_class else widen_def(pending))
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
