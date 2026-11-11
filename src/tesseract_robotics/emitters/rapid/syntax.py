"""RAPID lexical rules: identifiers, string literals, booleans, storage classes.

Sources (ABB, RobotWare 6):

- Technical reference manual, RAPID kernel, 3HAC050946-001, sections 2.3 to 2.10:
  an identifier is `<letter> {<letter> | <digit> | '_'}`, at most 32 characters,
  case-insensitive, and never one of the reserved words below; a backslash or a
  double quote inside a string literal is written twice; a comment runs from `!`
  to the end of the line and can never contain a newline.
- Technical reference manual, RAPID Instructions, Functions and Data types,
  3HAC050917-001, `string` data type: 0 to 80 characters, counting the doubled
  quotation marks and backslashes.

Positions are millimetres, quaternions scalar-first `[q1, q2, q3, q4]` =
`[w, x, y, z]`, joint angles degrees. The Python surface of this package is SI
with scalar-last quaternions `[qx, qy, qz, qw]`; conversion happens only in the
formatters, through `core.units`.
"""

from __future__ import annotations

import re
from enum import IntEnum, StrEnum
from typing import Final

from ..core.errors import UnemittableTextError
from ..core.naming import IdentifierRules
from ..core.text import single_line

BRAND: Final[str] = "RAPID"

#: Reserved words, kernel manual section 2.4 (compared upper-cased).
_RESERVED: Final[frozenset[str]] = frozenset(
    """
    ALIAS AND BACKWARD CASE CONNECT CONST DEFAULT DIV DO ELSE ELSEIF ENDFOR
    ENDFUNC ENDIF ENDMODULE ENDPROC ENDRECORD ENDTEST ENDTRAP ENDWHILE ERROR EXIT
    FALSE FOR FROM FUNC GOTO IF INOUT LOCAL MOD MODULE NOSTEPIN NOT NOVIEW OR PERS
    PROC RAISE READONLY RECORD RETRY RETURN STEP SYSMODULE TEST THEN TO TRAP TRUE
    TRYNEXT UNDO VAR VIEWONLY WHILE WITH XOR
    """.split()
)

#: RAPID identifier rules (kernel manual sections 2.3 and 2.4).
RAPID_IDENTIFIERS: Final[IdentifierRules] = IdentifierRules(
    brand=BRAND,
    pattern=re.compile(r"[A-Za-z][A-Za-z0-9_]*"),
    pattern_text="a letter followed by letters, digits or '_'",
    max_len=32,
    reserved=_RESERVED,
    case_insensitive=True,
)

#: Maximum `string` length, counting doubled quotes and backslashes.
STRING_MAX_LEN: Final[int] = 80


class RapidType(StrEnum):
    """RAPID data-declaration storage classes.

    `CONST`: value fixed at declaration; cannot be assigned at runtime.
    `VAR`: runtime-mutable; reset to its initial value when the program restarts.
    `PERS`: persistent across program runs; the only class a `tooldata` or
    `wobjdata` argument of a move instruction accepts.
    """

    CONST = "CONST"
    VAR = "VAR"
    PERS = "PERS"


class MotionType(IntEnum):
    """RAPID configuration-monitoring selector used by `Conf` (ConfL vs ConfJ)."""

    LINEAR = 1
    JOINT = 2


def rapid_bool(value: bool) -> str:
    """Python bool to RAPID bool literal (`TRUE` / `FALSE`)."""
    return "TRUE" if value else "FALSE"


def string_literal(text: str) -> str:
    """RAPID string literal for `text`, quoted and escaped.

    Raises:
        UnemittableTextError: `text` contains a line break, or its escaped form
            exceeds `STRING_MAX_LEN`.
    """
    single_line(text, brand=BRAND)
    escaped = text.replace("\\", "\\\\").replace('"', '""')
    if len(escaped) > STRING_MAX_LEN:
        raise UnemittableTextError(
            text, BRAND, f"{len(escaped)} characters once escaped, maximum is {STRING_MAX_LEN}"
        )
    return f'"{escaped}"'


def comment_text(text: str) -> str:
    """Validate `text` as a RAPID comment body (no line breaks)."""
    return single_line(text, brand=BRAND)
