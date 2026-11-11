"""Fanuc TP (LS ASCII) lexical rules, frame binding and arm configuration.

Sources: Fanuc HandlingTool Operator's Manual (R-J3iB, B-81524EN/01), position
data sections:

- user frame `UF`: "0: The world coordinate system is used. 1 to 9: The user
  coordinate system"; tool frame `UT`: "0: The mechanical interface coordinate
  system is used. 1 to 10: The tool coordinate system";
- configuration: joint placement letters FLIP/NOFLIP (control point relative to
  the J4 control plane), UP/DOWN (elbow) and FRONT/BACK ("control point placed
  on the front side of the control plane on the J1-axis"), followed by the turn
  numbers of J1, J4 and J6: "1: 180 to 539 / 0: -179 to 179 / -1: -539 to -180".

The FLIP and UP letters are defined by the arm geometry of the specific robot
model, not by a published joint-value rule, so the caller supplies them through
a `FanucConfigResolver`; the emitter computes the turn numbers itself.

Program names: letter first, then upper-case letters, digits or `_`, at most 36
characters (the TP program-name field of current controllers).
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final, Protocol

from ..core.errors import InvalidFrameBindingError
from ..core.frames import FrameBinding
from ..core.naming import IdentifierRules
from ..core.text import single_line
from ..core.units import Radians, to_degrees

BRAND: Final[str] = "LS"

LS_IDENTIFIERS: Final[IdentifierRules] = IdentifierRules(
    brand=BRAND,
    pattern=re.compile(r"[A-Z][A-Z0-9_]*"),
    pattern_text="an upper-case letter followed by upper-case letters, digits or '_'",
    max_len=36,
    reserved=frozenset(),
    case_insensitive=False,
)

#: Valid user-frame and tool-frame numbers (B-81524EN/01).
USER_FRAMES: Final[range] = range(0, 10)
TOOL_FRAMES: Final[range] = range(0, 11)
#: Half a turn (deg): the turn-number boundary.
_HALF_TURN_DEG: Final[float] = 180.0
#: One full turn (deg).
_FULL_TURN_DEG: Final[float] = 360.0


def ls_frames(
    *, user_frames: Mapping[str, int], tools: Mapping[str, int]
) -> FrameBinding[int, int]:
    """Bind tesseract frames to Fanuc user-frame (`UF`) and tool-frame (`UT`) numbers.

    Args:
        user_frames: Working-frame link name to UF number (0 = world, 1..9).
        tools: TCP link name to UT number (0 = mechanical interface, 1..10).

    Raises:
        InvalidFrameBindingError: a number is out of range.
    """
    for label, mapping, valid in (("UF", user_frames, USER_FRAMES), ("UT", tools, TOOL_FRAMES)):
        for frame, number in mapping.items():
            if isinstance(number, bool) or not isinstance(number, int) or number not in valid:
                raise InvalidFrameBindingError(
                    f"LS: {frame!r} binds to {label} {number}; valid: "
                    f"{valid.start}..{valid.stop - 1}"
                )
    return FrameBinding.build(work_frames=user_frames, tools=tools)


@dataclass(frozen=True)
class FanucArmConfig:
    """The three joint-placement letters of a Fanuc Cartesian position.

    Attributes:
        flip: `F` (FLIP) when True, `N` (NOFLIP) when False.
        up: `U` (UP) when True, `D` (DOWN) when False.
        front: `T` (FRONT) when True, `B` (BACK) when False.
    """

    flip: bool
    up: bool
    front: bool

    def letters(self) -> str:
        return f"{'F' if self.flip else 'N'} {'U' if self.up else 'D'} {'T' if self.front else 'B'}"


class FanucConfigResolver(Protocol):
    """Joint placement of a six-axis joint vector (radians), for one robot model."""

    def __call__(self, joints: Sequence[float]) -> FanucArmConfig: ...


def turn_number(angle: Radians) -> int:
    """Fanuc turn number: 0 on (-180, 180) deg, k on [180(2k-1), 180(2k+1)) for k > 0.

    Mirrored for negative angles: -1 on (-540, -180], following the manual's
    table (`-1: -539 to -180`, `0: -179 to 179`, `1: 180 to 539`).
    """
    deg = to_degrees(angle)
    if abs(deg) < _HALF_TURN_DEG:
        return 0
    k = math.floor((abs(deg) + _HALF_TURN_DEG) / _FULL_TURN_DEG)
    return k if deg > 0 else -k


def comment_text(text: str) -> str:
    """Validate `text` as a `!` remark body: one line, no `;` (the line terminator)."""
    return single_line(text, brand=BRAND, forbidden=";")
