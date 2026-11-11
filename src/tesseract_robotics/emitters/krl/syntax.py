"""KRL lexical rules, the frame binding, and comment text.

Source: KUKA System Software 8.3, Operating and Programming Instructions for
System Integrators (KSS 8.3 SI V4), section 11.3.2: names "can have a maximum
length of 24 characters", "can consist of letters (A-Z), numbers (0-9) and the
signs '_' and '$'", "must not begin with a number" and "must not be keywords";
user names must not begin with `$` (reserved for system variables). Keywords
are case-insensitive. Up to 16 tool frames `TOOL_DATA[1..16]` and 32 base
frames `BASE_DATA[1..32]` (sections on tool and base calibration).
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from enum import Enum
from typing import Final

from ..core.errors import InvalidFrameBindingError
from ..core.frames import FrameBinding
from ..core.naming import IdentifierRules
from ..core.text import single_line

BRAND: Final[str] = "KRL"

#: KRL keywords a program name must not equal (compared upper-cased). The KSS
#: manual forbids keywords without listing them in one place; this is the
#: statement, declaration and motion vocabulary of KRL.
_KEYWORDS: Final[frozenset[str]] = frozenset(
    """
    AND ANIN ANOUT B_AND B_EXOR B_NOT B_OR BOOL BRAKE C_DIS C_ORI C_PTP C_VEL CASE
    CHAR CIRC CIRC_REL CONST CONTINUE DECL DEF DEFAULT DEFDAT DEFFCT DELAY DO ELSE
    END ENDDAT ENDFCT ENDFOR ENDIF ENDLOOP ENDSWITCH ENDWHILE ENUM EXIT EXOR EXT
    EXTFCT FALSE FOR GLOBAL GOTO HALT IF IMPORT INT INTERRUPT IS LIN LIN_REL LOOP
    MAXIMUM MINIMUM NOT OR PRIO PTP PTP_REL PUBLIC REAL REPEAT RESUME RETURN SEC
    SIGNAL STRUC SWITCH THEN TO TRIGGER TRUE UNTIL WAIT WHEN WHILE
    """.split()
)

KRL_IDENTIFIERS: Final[IdentifierRules] = IdentifierRules(
    brand=BRAND,
    pattern=re.compile(r"[A-Za-z_][A-Za-z0-9_]*"),
    pattern_text="a letter or '_' followed by letters, digits or '_'",
    max_len=24,
    reserved=_KEYWORDS,
    case_insensitive=True,
)

#: Valid `TOOL_DATA` / `BASE_DATA` indices (KSS 8.3).
TOOL_INDICES: Final[range] = range(1, 17)
BASE_INDICES: Final[range] = range(1, 33)


class KrlNullFrame(Enum):
    """`$NULLFRAME`: the flange as `$TOOL`, the world frame as `$BASE`."""

    NULLFRAME = "$NULLFRAME"


#: A KRL frame handle: a `TOOL_DATA` / `BASE_DATA` index, or `$NULLFRAME`.
type KrlFrameRef = int | KrlNullFrame


def _check(handle: KrlFrameRef, valid: range, array: str, frame: str) -> KrlFrameRef:
    if isinstance(handle, KrlNullFrame):
        return handle
    if isinstance(handle, bool) or not isinstance(handle, int) or handle not in valid:
        raise InvalidFrameBindingError(
            f"KRL: {frame!r} binds to {array}[{handle}]; valid indices are "
            f"{valid.start}..{valid.stop - 1} or KrlNullFrame.NULLFRAME"
        )
    return handle


def krl_frames(
    *, bases: Mapping[str, KrlFrameRef], tools: Mapping[str, KrlFrameRef]
) -> FrameBinding[KrlFrameRef, KrlFrameRef]:
    """Bind tesseract frames to KRL `BASE_DATA` / `TOOL_DATA` entries.

    Args:
        bases: Working-frame link name to `BASE_DATA` index (1..32) or `$NULLFRAME`.
        tools: TCP link name to `TOOL_DATA` index (1..16) or `$NULLFRAME` (flange).

    Raises:
        InvalidFrameBindingError: an index is out of range.
    """
    return FrameBinding.build(
        work_frames={k: _check(v, BASE_INDICES, "BASE_DATA", k) for k, v in bases.items()},
        tools={k: _check(v, TOOL_INDICES, "TOOL_DATA", k) for k, v in tools.items()},
    )


def frame_expr(handle: KrlFrameRef, array: str) -> str:
    """KRL expression for a frame handle: `BASE_DATA[i]` or `$NULLFRAME`."""
    return handle.value if isinstance(handle, KrlNullFrame) else f"{array}[{handle}]"


def comment_text(text: str) -> str:
    """Validate `text` as a `;` line-comment body (no line breaks)."""
    return single_line(text, brand=BRAND)
