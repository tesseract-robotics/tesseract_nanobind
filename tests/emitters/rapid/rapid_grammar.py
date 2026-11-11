"""A structural checker for emitted RAPID modules (parse-back contract).

Independent of the writer: it re-reads the text and enforces the RAPID kernel
rules (3HAC050946, modules, routines, data declarations) that a golden file
alone would happily lock in if they were broken:

- exactly one `MODULE` ... `ENDMODULE`, never nested;
- `PROC` ... `ENDPROC` only directly inside the module (routines do not nest);
- `WHILE ... DO` / `FOR ... DO` blocks only inside a routine, properly closed;
- `PERS` declarations only at module level; `VAR` / `CONST` at module level or
  at the top of a routine, before its first statement;
- every declaration is `<storage> <data type> <name> := <value>;`;
- every statement ends with `;` and every identifier respects 32 characters.
"""

from __future__ import annotations

import re

_DECL = re.compile(r"(PERS|VAR|CONST) ([a-z]+) ([A-Za-z][A-Za-z0-9_]{0,31}) := .+;")
_DATA_TYPES = frozenset(
    {"robtarget", "jointtarget", "tooldata", "wobjdata", "zonedata", "speeddata", "pos", "bool"}
)


class RapidGrammarError(AssertionError):
    pass


def check_module(text: str) -> None:
    stack: list[str] = []
    started: list[bool] = []
    modules = 0
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("!"):
            continue
        head = line.split()[0]
        if head == "MODULE":
            if stack:
                raise RapidGrammarError(f"nested MODULE: {line}")
            modules += 1
            stack.append("MODULE")
        elif head == "ENDMODULE":
            if stack != ["MODULE"]:
                raise RapidGrammarError(f"ENDMODULE with open {stack}")
            stack.pop()
        elif head == "PROC":
            if stack != ["MODULE"] or not re.fullmatch(r"PROC [A-Za-z]\w{0,31}\(\)", line):
                raise RapidGrammarError(f"PROC not directly in MODULE or malformed: {line}")
            stack.append("PROC")
            started.append(False)
        elif head == "ENDPROC":
            if stack[-1:] != ["PROC"]:
                raise RapidGrammarError(f"ENDPROC closes {stack}")
            stack.pop()
            started.pop()
        elif head in ("WHILE", "FOR"):
            if "PROC" not in stack or not line.endswith(" DO"):
                raise RapidGrammarError(f"{head} outside a routine or without DO: {line}")
            started[-1] = True
            stack.append(head)
        elif head in ("ENDWHILE", "ENDFOR"):
            if stack[-1:] != [head[3:]]:
                raise RapidGrammarError(f"{head} closes {stack}")
            stack.pop()
        elif head in ("PERS", "VAR", "CONST"):
            m = _DECL.fullmatch(line)
            if m is None or m.group(2) not in _DATA_TYPES:
                raise RapidGrammarError(f"declaration without data type or malformed: {line}")
            if stack == ["MODULE"]:
                continue
            if head == "PERS" or stack[-1:] != ["PROC"] or started[-1]:
                raise RapidGrammarError(f"declaration not allowed here ({stack}): {line}")
        else:
            if "PROC" not in stack:
                raise RapidGrammarError(f"statement outside a routine: {line}")
            if not line.endswith(";"):
                raise RapidGrammarError(f"statement without ';': {line}")
            started[-1] = True
    if stack or modules != 1:
        raise RapidGrammarError(f"unbalanced module: open {stack}, {modules} modules")
